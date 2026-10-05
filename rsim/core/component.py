"""Component lifecycle and metered tasks; no deployment or hardware dependencies."""
from __future__ import annotations
import asyncio
from typing import Awaitable, Callable
from .errors import ComponentError
from .signal import Signal, as_signal
from .metronome import Metronome

class Component:
    """A running resource/computation, with zero or more named output Signals.

    dependencies are ownership edges (a DAG); inputs are data edges (may form
    feedback cycles). A Component itself has no data history or get/publish API.
    """
    process_local = False

    def __init__(self, *dependencies, inputs=(), key=None, placement=None):
        if any(not isinstance(item, Component) for item in dependencies):
            raise TypeError("dependencies must be Components; pass data through inputs")
        self.dependencies = tuple(dependencies)
        self.inputs = tuple(as_signal(item) for item in inputs)
        self.key, self.placement = key, placement
        self.outputs, self.sinks = {}, {}
        self.command_targets = ()
        self._tasks = set()
        self._failure = None
        self._runtime = None
        self._closed = True
        self._closing = False
        self._canonical = None
        self._binding = None
        self._ready = asyncio.Event()
        self._failed = asyncio.Event()
        self._services = {}
        self._runtime_peers = ()

    @property
    def children(self):
        """Compatibility spelling for explicit ownership dependencies."""
        return self.dependencies

    @children.setter
    def children(self, value):
        self.dependencies = tuple(value)

    def signal(self, name, *, history=32, clock=None):
        return Signal(self, name, history=history, clock=clock)

    def service(self, name, handler, *, hz, capacity=16):
        """Register an async request handler on this component's metered task loop."""
        if name in self._services or capacity < 1:
            raise ValueError("service names must be unique and capacity positive")
        queue = asyncio.Queue(maxsize=capacity)

        async def dispatch():
            args, kwargs, response = await queue.get()
            if response.cancelled():
                return
            try:
                result = await handler(*args, **kwargs)
                if not response.done():
                    response.set_result(result)
            except asyncio.CancelledError:
                if not response.done():
                    response.set_exception(ComponentError("service stopped"))
                raise
            except Exception as error:
                if not response.done():
                    response.set_exception(error)
                raise

        self.task(f"service:{name}", dispatch, hz=hz)
        self._services[name] = queue

    async def call(self, name, *args, timeout=None, **kwargs):
        if self._binding is not None:
            return await self._binding.call(name, *args, timeout=timeout, **kwargs)
        if self._canonical is not None:
            return await self._canonical.call(name, *args, timeout=timeout, **kwargs)
        if self._closed or self._closing or self._failure is not None:
            raise ComponentError("component is not available") from self._failure
        if name not in self._services:
            raise KeyError(name)
        queue = self._services[name]
        response = asyncio.get_running_loop().create_future()
        try:
            async with asyncio.timeout(timeout):
                await queue.put((args, kwargs, response))
                if self._closed or self._closing or self._failure is not None:
                    if response.done() and not response.cancelled():
                        response.exception()
                    raise ComponentError("component stopped while queuing request") from self._failure
                return await response
        finally:
            if not response.done():
                response.cancel()
            if self._closed or self._closing or self._failure is not None:
                self._fail_queue(queue)

    @staticmethod
    def _fail_queue(queue):
        while not queue.empty():
            _, _, response = queue.get_nowait()
            if not response.done():
                response.set_exception(ComponentError("component stopped"))

    def _fail_requests(self):
        for queue in self._services.values():
            self._fail_queue(queue)

    def configuration(self):
        return (type(self), tuple((name, sig.history_size, sig.clock)
                                 for name, sig in self.outputs.items()), self.placement)

    def __getstate__(self):
        if self._runtime is not None:
            raise ComponentError("cannot deploy an already running component")
        state = dict(self.__dict__)
        for name in ("_tasks", "_ready", "_failed", "_services", "_runtime_peers"):
            state.pop(name, None)
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self._tasks, self._services, self._runtime_peers = set(), {}, ()
        self._ready, self._failed = asyncio.Event(), asyncio.Event()

    async def open(self):
        pass

    async def close(self):
        pass

    def task(self, name: str, callback: Callable[[], Awaitable], *, hz: float):
        if self._runtime is None or self._closed:
            raise ComponentError("task registration requires an active runtime")
        metronome = Metronome(hz)
        async def run():
            while True:
                await metronome.tick()
                await callback()
        task = asyncio.create_task(run(), name=f"{type(self).__name__}:{name}")
        self._tasks.add(task)
        return task

    async def wait(self):
        """Wait for failure or closure; useful for components without outputs."""
        owner, seen = self, set()
        while owner._binding is not None or owner._canonical is not None:
            if owner in seen:
                raise ComponentError("cyclic component binding")
            seen.add(owner)
            owner = owner._binding or owner._canonical
        if owner._closed:
            raise ComponentError("component is closed")
        await owner._failed.wait()
        raise ComponentError("component stopped") from owner._failure

    async def _watchdog(self):
        try:
            metronome = Metronome(20)
            while True:
                await metronome.tick()
                for task in self._tasks:
                    if task is asyncio.current_task() or not task.done():
                        continue
                    if task.cancelled():
                        raise ComponentError(f"task unexpectedly cancelled: {task.get_name()}")
                    error = task.exception()
                    if error is not None:
                        raise error
                    raise ComponentError(f"task unexpectedly ended: {task.get_name()}")
                for peer in self._runtime_peers:
                    if peer._failure is not None:
                        raise ComponentError("required component failed") from peer._failure
        except asyncio.CancelledError:
            raise
        except BaseException as error:
            self._failure = error
            self._failed.set()
            for output in self.outputs.values():
                await output._notify()
            self._fail_requests()
            for task in self._tasks:
                if task is not asyncio.current_task():
                    task.cancel()
            await asyncio.gather(*(task for task in self._tasks
                                   if task is not asyncio.current_task()), return_exceptions=True)
            for sink in self.sinks.values():
                try:
                    await sink._safe()
                except Exception:
                    # Preserve the original failure; Runtime.close retries any
                    # hardware cleanup. Remote providers own independent TTLs.
                    pass


class PrimaryComponent(Component):
    """Explicit single-output convenience; all data state lives in output."""
    def __init__(self, *dependencies, history=32, output_name="output", clock=None, **kwargs):
        super().__init__(*dependencies, **kwargs)
        self.output = Signal(self, output_name, history=history, clock=clock)
        self.primary = self.output

    async def get(self, **kwargs):
        return await self.primary.get(**kwargs)

    async def publish(self, data, **kwargs):
        return await self.primary.publish(data, **kwargs)

    @property
    def _history(self):
        return self.primary._history

    @property
    def _sequence(self):
        return self.primary._sequence


class Sensor(PrimaryComponent):
    """Compatibility base for old single-output components. Prefer Component."""


class Reference(PrimaryComponent):
    """Legacy non-owning alias; new dataflow edges directly reference Signals."""
    def __init__(self, target):
        super().__init__(history=1)
        self.target = target
        self.output._target = as_signal(target)
