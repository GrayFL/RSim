"""Thread ownership, RSim streams and command deadlines for an injected backend."""
from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import math
import threading
import time

from rsim.core.component import Component
from rsim.core.commands import CommandRejected, CommandSink
from .protocol import CommandSpec, Observation


class MuJoCoSource(Component):
    def __init__(self, backend_factory, *, outputs, commands=None, hz=50.,
                 speedup=1., motion_enabled=False):
        super().__init__()
        if not callable(backend_factory):
            raise TypeError('backend_factory must construct a project-owned SimulationBackend')
        if not all(math.isfinite(v) and v > 0 for v in (hz, speedup)):
            raise ValueError('hz and speedup must be finite and positive')
        self.backend_factory = backend_factory
        self.hz, self.speedup = hz, speedup
        self.motion_enabled = motion_enabled
        self.command_specs = dict(commands or {})
        self.backend = None
        self._executor = self._command_lock = None
        self._commands = {}
        names = list(outputs) + list(self.command_specs)
        if len(names) != len(set(names)) or any(not name.isidentifier() or hasattr(self, name) for name in names):
            raise ValueError('port names must be unique identifiers, without Component attribute conflicts')
        for name, history in outputs.items():
            setattr(self, name, self.signal(name, history=history))
        for name, spec in self.command_specs.items():
            if not isinstance(spec, CommandSpec):
                raise TypeError('commands must contain CommandSpec values')
            if not spec.is_safe(spec.fallback):
                raise ValueError('command fallback must satisfy is_safe')
            setattr(self, name, CommandSink(self, name, partial(self._apply, name),
                    fallback=spec.fallback, safe=partial(self._safe, name), hz=200))
            self._commands[name] = (spec.fallback, 0)

    async def _work(self, callback, *args):
        return await asyncio.get_running_loop().run_in_executor(self._executor, callback, *args)

    def _initialize(self):
        self.backend = self.backend_factory()
        self.backend.open()

    async def open(self):
        self._command_lock = threading.Lock()
        self._access = asyncio.Lock()
        self.clock = f'mujoco:{self._instance_id}'
        self._offset_ns = self._last_stamp_ns = self._episode = 0
        for port in self.outputs.values():
            port.clock = self.clock
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='rsim-mujoco')
        try:
            await self._work(self._initialize)
            async with self._access:
                await self._publish(await self._work(self._sample, self.demand_snapshot(), True))
            self.task('simulation', self._tick, hz=self.hz*self.speedup)
        except BaseException:
            await self.close()
            raise

    def _apply(self, name, envelope):
        spec = self.command_specs[name]
        if spec.validate:
            spec.validate(envelope.value)
        if not self.motion_enabled and not spec.is_safe(envelope.value):
            raise CommandRejected('active commands require motion_enabled=True')
        with self._command_lock:
            self._commands[name] = (envelope.value, envelope.deadline_ns)

    def _safe(self, name, value):
        if self._command_lock is None:
            self._commands[name] = (self.command_specs[name].fallback, 0)
        else:
            with self._command_lock:
                self._commands[name] = (self.command_specs[name].fallback, 0)

    @property
    def current_commands(self):
        now = time.monotonic_ns()
        def snapshot():
            return {name: value if now < deadline else self.command_specs[name].fallback
                    for name, (value, deadline) in self._commands.items()}
        if self._command_lock is None:
            return snapshot()
        with self._command_lock:
            return snapshot()

    def _sample(self, demand, force=False):
        return self.backend.sample(demand, force=force)

    def _advance(self, demand):
        # Check host time on the owner thread immediately before physical action.
        # speedup and a stalled event loop never extend a command's lease.
        self.backend.step(1/self.hz, self.current_commands)
        return self._sample(demand)

    async def _publish(self, observation):
        if not isinstance(observation, Observation) or not math.isfinite(observation.time):
            raise ValueError('backend must return Observation with a finite time')
        stamp = self._offset_ns + round(observation.time*1e9)
        if stamp < self._last_stamp_ns:
            raise ValueError('backend time moved backwards without reset')
        self._last_stamp_ns = stamp
        for name, value in observation.values.items():
            if name not in self.outputs:
                raise ValueError(f'backend returned undeclared output: {name}')
            metadata = dict(observation.metadata, episode=self._episode)
            metadata.update(observation.port_metadata.get(name, {}))
            await self.outputs[name].publish(value, stamp_ns=stamp, clock=self.clock, metadata=metadata)

    async def _tick(self):
        async with self._access:
            await self._publish(await self._work(self._advance, self.demand_snapshot()))

    async def inspect(self, callback, *args):
        """Tooling access to the backend, serialized on its owner thread."""
        if self._executor is None:
            raise RuntimeError('MuJoCo source is not running')
        async with self._access:
            return await self._work(callback, self.backend, *args)

    async def reset(self, *args, **kwargs):
        for name, spec in self.command_specs.items():
            self._safe(name, spec.fallback)
        demand = self.demand_snapshot()
        def reset():
            self.backend.reset(*args, **kwargs)
            return self._sample(demand, True)
        async with self._access:
            observation = await self._work(reset)
            self._episode += 1
            self._offset_ns = self._last_stamp_ns + max(1, round(1e9/self.hz)) - round(observation.time*1e9)
            await self._publish(observation)

    async def close(self):
        if self._executor is None:
            return
        for name, spec in self.command_specs.items():
            self._safe(name, spec.fallback)
        def cleanup():
            if self.backend is not None:
                try:
                    self.backend.stop()
                finally:
                    self.backend.close()
        try:
            await self._work(cleanup)
        finally:
            self._executor.shutdown(wait=True)
            self._executor = None
