"""Component lifetime and graph scheduling, independent of data transport."""
from __future__ import annotations

import asyncio
import math
from typing import Awaitable, Callable

from .errors import ComponentError, HistoryMiss, SensorError
from .model import Frame
from .signal import Signal, as_signal


class Metronome:
    """Monotonic deadlines; skip missed ticks instead of catch-up bursts."""

    def __init__(self, hz: float):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("hz must be finite and positive")
        self.period = 1 / hz
        self.deadline = None
        self.missed = 0

    async def tick(self):
        loop = asyncio.get_running_loop()
        now = loop.time()
        if self.deadline is None:
            self.deadline = now
        else:
            self.deadline += self.period
            if self.deadline < now:
                skipped = math.ceil((now - self.deadline) / self.period)
                self.missed += skipped
                self.deadline += skipped * self.period
        await asyncio.sleep(max(0, self.deadline - loop.time()))


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


class Runtime:
    """Own Components required by requested Components, Signals or sinks.

    Ownership DAGs determine lifetime order. Signal producers are discovered
    independently, so data feedback does not create an ownership cycle.
    """
    def __init__(self, *roots, placement=None):
        self.roots = self._original_roots = roots
        self.placement = placement or {}
        self._order, self._started, self._aliases = [], [], []
        self._original_dependencies = {}
        self._active = False
        self._binding_plan = None
        self._command_claims = {}

    def _release_bindings(self):
        for alias in self._aliases:
            alias._canonical = None
        self._aliases.clear()
        for component, dependencies in self._original_dependencies.items():
            component.dependencies = dependencies
            component._runtime_peers = ()
        self._original_dependencies.clear()
        self._command_claims.clear()
        self.roots = self._original_roots

    def _resolve(self):
        keyed, components, data_edges = {}, {}, {}
        self._order = []
        self._command_claims.clear()

        def visit(component):
            if not isinstance(component, Component):
                raise TypeError("Runtime roots must be Components, Signals or CommandSinks")
            if component._binding is not None:
                return visit(component._binding)
            if component._canonical is not None and component in self._aliases:
                return visit(component._canonical)
            if component.key is not None:
                prior = keyed.setdefault(component.key, component)
                if prior.configuration() != component.configuration():
                    raise ValueError(f"conflicting source configuration: {component.key}")
                if prior is not component:
                    if component._runtime is not None or (component._canonical is not None
                                                          and component not in self._aliases):
                        raise ComponentError("component alias already belongs to an active runtime")
                    if component not in self._aliases:
                        self._aliases.append(component)
                    component._canonical = prior
                component = prior
            if component in components:
                return component
            if component._runtime is not None:
                raise ComponentError("component already belongs to an active runtime")
            components[component] = None
            self._original_dependencies.setdefault(component, component.dependencies)
            component.dependencies = tuple(visit(child) for child in component.dependencies)
            data_edges[component] = tuple(visit(signal.producer) for signal in component.inputs)
            return component

        root_components = []
        for root in self.roots:
            owner = root if isinstance(root, Component) else getattr(root, "producer", None)
            root_components.append(visit(owner))

        visiting, visited = set(), set()
        def check_ownership(component):
            if component in visiting:
                raise ValueError("cyclic ownership dependency; use Signal inputs for feedback")
            if component in visited:
                return
            visiting.add(component)
            for child in component.dependencies:
                check_ownership(child)
            visiting.remove(component)
            visited.add(component)
        for component in components:
            check_ownership(component)

        visiting, visited = set(), set()
        def order(component):
            if component in visited or component in visiting:
                return
            visiting.add(component)
            for child in component.dependencies + data_edges[component]:
                order(child)
            visiting.remove(component)
            visited.add(component)
            self._order.append(component)
        for component in root_components:
            order(component)

        # Data cycles can contradict preferred input-first order. Hard resource
        # dependencies always take precedence; ready data producers are a tie
        # breaker, never a reason to open an owner before its dependency.
        remaining, self._order = self._order, []
        started = set()
        while remaining:
            ready = [item for item in remaining if set(item.dependencies) <= started]
            component = next((item for item in ready if set(data_edges[item]) <= started), ready[0])
            self._order.append(component)
            started.add(component)
            remaining.remove(component)

        for component in self._order:
            peers = component.dependencies + data_edges[component]
            if isinstance(component, Reference):
                target = component.target._canonical or component.target
                if target not in components or isinstance(target, Reference):
                    raise ComponentError("Reference target must be an owned component in this Runtime")
                peers += (target,)
            component._runtime_peers = tuple(peer for peer in peers if peer is not component)
            for sink in component.command_targets:
                sink = sink._resolved()
                if sink in self._command_claims and self._command_claims[sink] is not component:
                    raise ValueError("multiple command writers require an explicit CommandMux")
                self._command_claims[sink] = component

    async def __aenter__(self):
        if self._active:
            raise ComponentError("runtime already active")
        try:
            self._resolve()
            if self.placement or any(item.placement is not None for item in self._order):
                from .deployment import BindingPlan
                self._binding_plan = BindingPlan(self)
                self._binding_plan.prepare()
                self._resolve()
        except BaseException:
            if self._binding_plan is not None:
                self._binding_plan.release()
                self._binding_plan = None
            self._release_bindings()
            raise
        self._active = True
        try:
            # All outputs become readable before open callbacks run, so seeded
            # feedback and consumers waiting on later producers are possible.
            for component in self._order:
                component._runtime = self
                component._closed = False
                component._closing = False
                component._failure = None
                component._ready.clear()
                component._failed.clear()
                for output in component.outputs.values():
                    output._history.clear()
            for component in self._order:
                self._started.append(component)
                await component.open()
                for sink in component.sinks.values():
                    await sink._activate()
                component._ready.set()
                component._tasks.add(asyncio.create_task(
                    component._watchdog(), name=f"{type(component).__name__}:WatchDog"))
        except BaseException:
            await self.aclose()
            raise
        return self

    async def wait(self):
        if not self._active:
            raise ComponentError("runtime is not active")
        pending = [asyncio.create_task(component.wait()) for component in self._order]
        if not pending:
            raise ComponentError("runtime has no components")
        try:
            done, _ = await asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED)
            await next(iter(done))
        finally:
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)

    async def aclose(self):
        errors = []
        for component in reversed(self._started):
            component._closing = True
            component._ready.set()
            component._failed.set()
            for task in component._tasks:
                task.cancel()
            await asyncio.gather(*component._tasks, return_exceptions=True)
            component._tasks.clear()
            component._fail_requests()
            component._services.clear()
            for sink in component.sinks.values():
                try:
                    await sink._deactivate()
                except Exception as error:
                    errors.append(error)
            try:
                await component.close()
            except Exception as error:
                errors.append(error)
            finally:
                component._closed = True
                component._runtime = None
                for output in component.outputs.values():
                    await output._notify()
                    output._history.clear()
        # An open callback may fail before later components are started.
        for component in self._order:
            component._closed = True
            component._runtime = None
            component._failed.set()
            for output in component.outputs.values():
                await output._notify()
        self._started.clear()
        self._active = False
        if self._binding_plan is not None:
            self._binding_plan.release()
            self._binding_plan = None
        self._release_bindings()
        if errors:
            raise ExceptionGroup("component cleanup failed", errors)

    async def __aexit__(self, exc_type, exc, tb):
        cleanup = asyncio.create_task(self.aclose())
        try:
            await asyncio.shield(cleanup)
        except asyncio.CancelledError:
            await cleanup
            raise
