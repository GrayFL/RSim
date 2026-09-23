"""ROS-independent lifecycle, scheduling and bounded sensor history."""
from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass, field
import math
import time
from typing import Any, Awaitable, Callable


class SensorError(RuntimeError):
    pass


class HistoryMiss(LookupError):
    pass


@dataclass(frozen=True)
class Frame:
    data: Any
    stamp_ns: int
    clock: str
    received_ns: int = field(default_factory=time.time_ns)
    sequence: int = 0


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


class Sensor:
    """Override open/close and register async work using task()."""

    def __init__(self, *children: Sensor, key: str | None = None, history: int = 32):
        if history < 1:
            raise ValueError("history must be positive")
        self.children = tuple(children)
        self.key = key
        self._history = deque(maxlen=history)
        self._condition = asyncio.Condition()
        self._tasks: set[asyncio.Task] = set()
        self._failure: BaseException | None = None
        self._runtime: Runtime | None = None
        self._closed = True
        self._sequence = 0
        self._canonical: Sensor | None = None
        self._ready = asyncio.Event()
        self._services: dict[str, asyncio.Queue] = {}

    def service(self, name, handler, *, hz, capacity=16):
        """Register an async request handler on this sensor's metered task loop."""
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
                    response.set_exception(SensorError("service stopped"))
                raise
            except Exception as error:
                if not response.done():
                    response.set_exception(error)
                raise

        self.task(f"service:{name}", dispatch, hz=hz)
        self._services[name] = queue

    async def call(self, name, *args, timeout=None, **kwargs):
        if self._canonical is not None:
            return await self._canonical.call(name, *args, timeout=timeout, **kwargs)
        if self._closed or self._failure is not None:
            raise SensorError("sensor is not available") from self._failure
        if name not in self._services:
            raise KeyError(name)
        queue = self._services[name]
        response = asyncio.get_running_loop().create_future()
        try:
            async with asyncio.timeout(timeout):
                await queue.put((args, kwargs, response))
                if self._closed or self._failure is not None:
                    if response.done() and not response.cancelled():
                        response.exception()
                    raise SensorError("sensor stopped while queuing request") from self._failure
                return await response
        finally:
            if not response.done():
                response.cancel()
            if self._closed or self._failure is not None:
                self._fail_queue(queue)

    @staticmethod
    def _fail_queue(queue):
        while not queue.empty():
            _, _, response = queue.get_nowait()
            if not response.done():
                response.set_exception(SensorError("sensor stopped"))

    def _fail_requests(self):
        for queue in self._services.values():
            self._fail_queue(queue)

    def configuration(self):
        """Override for keyed sources; equal keys must mean equal settings."""
        return (type(self), self._history.maxlen)

    async def open(self):
        pass

    async def close(self):
        pass

    def task(self, name: str, callback: Callable[[], Awaitable], *, hz: float):
        if self._runtime is None or self._closed:
            raise SensorError("task registration requires an active runtime")
        metronome = Metronome(hz)

        async def run():
            while True:
                await metronome.tick()
                await callback()

        task = asyncio.create_task(run(), name=f"{type(self).__name__}:{name}")
        self._tasks.add(task)
        return task

    async def publish(self, data, *, stamp_ns: int, clock: str, received_ns=None):
        if self._closed:
            raise SensorError("sensor is closed")
        from .shared import current_store
        store = current_store.get()
        if store is not None:
            data = store.prepare(data)
        self._sequence += 1
        frame = Frame(data, stamp_ns, clock,
                      time.time_ns() if received_ns is None else received_ns,
                      self._sequence)
        async with self._condition:
            self._history.append(frame)
            self._condition.notify_all()
        return frame

    async def get(self, *, timestamp_ns: int | None = None, clock: str | None = None,
                  tolerance_ns: int = 0, after: int | None = None,
                  timeout: float | None = None) -> Frame:
        if self._canonical is not None:
            return await self._canonical.get(timestamp_ns=timestamp_ns, clock=clock,
                tolerance_ns=tolerance_ns, after=after, timeout=timeout)
        if tolerance_ns < 0:
            raise ValueError("tolerance_ns must be nonnegative")
        if timestamp_ns is not None and (clock is None or after is not None):
            raise ValueError("timestamp lookup requires clock and cannot use after")
        async with asyncio.timeout(timeout):
            async with self._condition:
                while True:
                    if self._failure is not None:
                        raise SensorError("sensor task failed") from self._failure
                    if self._closed:
                        raise SensorError("sensor is closed")
                    if timestamp_ns is not None:
                        frames = [f for f in self._history if f.clock == clock]
                        if frames:
                            frame = min(frames, key=lambda f: abs(f.stamp_ns - timestamp_ns))
                            if abs(frame.stamp_ns - timestamp_ns) <= tolerance_ns:
                                return frame
                        raise HistoryMiss("no retained sample within the requested clock/tolerance")
                    if self._history and (after is None or self._history[-1].sequence > after):
                        return self._history[-1]
                    await self._condition.wait()

    async def _watchdog(self):
        async def inspect():
            for task in self._tasks:
                if task is asyncio.current_task():
                    continue
                if task.done():
                    if task.cancelled():
                        raise SensorError(f"task unexpectedly cancelled: {task.get_name()}")
                    error = task.exception()
                    if error is not None:
                        raise error
                    raise SensorError(f"task unexpectedly ended: {task.get_name()}")
            for child in self.children:
                if child._failure is not None:
                    raise SensorError("child failed") from child._failure
            if isinstance(self, Reference) and self.target._failure is not None:
                raise SensorError("referenced sensor failed") from self.target._failure

        try:
            metronome = Metronome(20)
            while True:
                await metronome.tick()
                await inspect()
        except asyncio.CancelledError:
            raise
        except BaseException as error:
            async with self._condition:
                self._failure = error
                self._condition.notify_all()
            self._fail_requests()
            for task in self._tasks:
                if task is not asyncio.current_task():
                    task.cancel()


class Reference(Sensor):
    """Non-owning edge. Target must be owned elsewhere in the same Runtime.

    Reference cycles are allowed; feedback computations must supply a seed or
    avoid waiting on one another's first frame. Ownership cycles are rejected.
    """
    def __init__(self, target):
        super().__init__(history=1)
        self.target = target

    async def get(self, *, timeout=None, **kwargs):
        if self._closed:
            raise SensorError("reference is closed")
        target = self.target._canonical or self.target
        async with asyncio.timeout(timeout):
            await target._ready.wait()
            return await target.get(**kwargs)


class Runtime:
    """Own a shared sensor DAG on the caller's existing asyncio loop."""

    def __init__(self, *roots: Sensor):
        self.roots = roots
        self._order: list[Sensor] = []
        self._started: list[Sensor] = []
        self._active = False
        self._aliases: list[Sensor] = []
        self._original_children: dict[Sensor, tuple] = {}
        self._original_roots = roots

    def _release_bindings(self):
        for alias in self._aliases:
            alias._canonical = None
        self._aliases.clear()
        for sensor, children in self._original_children.items():
            sensor.children = children
        self._original_children.clear()
        self.roots = self._original_roots

    def _resolve(self):
        keyed = {}
        visiting = set()
        visited = set()
        self._order = []

        def visit(sensor):
            if sensor.key is not None:
                prior = keyed.setdefault(sensor.key, sensor)
                if prior.configuration() != sensor.configuration():
                    raise ValueError(f"conflicting source configuration: {sensor.key}")
                if prior is not sensor:
                    if sensor._runtime is not None or (sensor._canonical is not None
                                                      and sensor not in self._aliases):
                        raise SensorError("source alias already belongs to an active runtime")
                    if sensor not in self._aliases:
                        self._aliases.append(sensor)
                    sensor._canonical = prior
                sensor = prior
            if sensor in visiting:
                raise ValueError("cyclic data dependency; use shared sources instead")
            if sensor in visited:
                return sensor
            if sensor._runtime is not None or sensor._canonical is not None:
                raise SensorError("sensor already belongs to an active runtime")
            visiting.add(sensor)
            self._original_children[sensor] = sensor.children
            sensor.children = tuple(visit(child) for child in sensor.children)
            visiting.remove(sensor)
            visited.add(sensor)
            self._order.append(sensor)
            return sensor

        self.roots = tuple(visit(root) for root in self.roots)
        for sensor in self._order:
            if isinstance(sensor, Reference):
                target = sensor.target._canonical or sensor.target
                if target not in visited or isinstance(target, Reference):
                    raise ValueError("Reference target must be an owned sensor in this Runtime")

    async def __aenter__(self):
        if self._active:
            raise SensorError("runtime already active")
        try:
            self._resolve()
        except BaseException:
            self._release_bindings()
            raise
        self._active = True
        try:
            for sensor in self._order:
                sensor._ready.clear()
            for sensor in self._order:
                sensor._runtime = self
                sensor._closed = False
                sensor._failure = None
                sensor._history.clear()
                self._started.append(sensor)
                await sensor.open()
                sensor._ready.set()
                sensor._tasks.add(asyncio.create_task(
                    sensor._watchdog(), name=f"{type(sensor).__name__}:WatchDog"))
        except BaseException:
            await self.aclose()
            raise
        return self

    async def aclose(self):
        errors = []
        for sensor in reversed(self._started):
            async with sensor._condition:
                sensor._closed = True
                sensor._ready.set()
                sensor._condition.notify_all()
            for task in sensor._tasks:
                task.cancel()
            await asyncio.gather(*sensor._tasks, return_exceptions=True)
            sensor._tasks.clear()
            sensor._fail_requests()
            sensor._services.clear()
            try:
                await sensor.close()
            except Exception as error:
                errors.append(error)
            finally:
                sensor._runtime = None
                sensor._history.clear()
        self._started.clear()
        self._active = False
        self._release_bindings()
        if errors:
            raise ExceptionGroup("sensor cleanup failed", errors)

    async def __aexit__(self, exc_type, exc, tb):
        # Shield cleanup from caller cancellation, but do not leave it running
        # unobserved when cancellation happens during __aexit__.
        cleanup = asyncio.create_task(self.aclose())
        try:
            await asyncio.shield(cleanup)
        except asyncio.CancelledError:
            await cleanup
            raise
