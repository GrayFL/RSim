"""Explicit command ownership, arbitration and provider-side validity checks."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
import inspect
import math
import time
from typing import Generic, TypeVar
import uuid

from .component import Component, PrimaryComponent
from .errors import ComponentError, PortNotBound
from .signal import as_signal

T = TypeVar("T")


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.linear_x) or not math.isfinite(self.angular_z):
            raise ValueError("velocity must be finite")


@dataclass(frozen=True)
class CommandEnvelope(Generic[T]):
    """Deadline uses the provider's monotonic nanosecond clock.

    Same-host processes share CLOCK_MONOTONIC. Network adapters must explicitly
    convert deadlines conservatively; sending this value across hosts unchanged
    is invalid. Epoch identifies a controller session; sequence is anti-replay.
    """
    value: T
    controller_id: str
    controller_epoch: str
    sequence: int
    deadline_ns: int


VelocityCommandEnvelope = CommandEnvelope


class CommandRejected(ValueError):
    def __init__(self, message, *, reason="invalid"):
        super().__init__(message)
        self.reason = reason


class CommandGuard:
    """State at the provider, independently checkable without a transport.

    Exclusive ownership expires with the accepted command. Retired sessions and
    sequence cursors are retained until this provider is destroyed, not reset
    when a command expires. A controller restart must generate a new epoch.
    """
    def __init__(self, *, max_ttl=.5):
        if not math.isfinite(max_ttl) or max_ttl <= 0:
            raise ValueError("max_ttl must be finite and positive")
        self.max_ttl_ns = int(max_ttl * 1e9)
        self.owner = None
        self.deadline_ns = 0
        self.sequences, self.retired = {}, set()

    def accept(self, command, *, now_ns=None):
        now = time.monotonic_ns() if now_ns is None else now_ns
        if (not isinstance(command, CommandEnvelope) or not isinstance(command.controller_id, str)
                or not command.controller_id or not isinstance(command.controller_epoch, str)
                or not command.controller_epoch or not isinstance(command.sequence, int)
                or command.sequence <= 0 or not isinstance(command.deadline_ns, int)):
            raise CommandRejected("invalid command envelope")
        if command.deadline_ns <= now:
            raise CommandRejected("command expired", reason="expired")
        if command.deadline_ns - now > self.max_ttl_ns:
            raise CommandRejected("command deadline exceeds provider TTL")
        identity = (command.controller_id, command.controller_epoch)
        if identity in self.retired:
            raise CommandRejected("retired controller epoch")
        if command.sequence <= self.sequences.get(identity, 0):
            raise CommandRejected("replayed or out-of-order command")
        if self.owner is not None and identity != self.owner:
            if now < self.deadline_ns:
                raise CommandRejected("command sink has an exclusive controller; use CommandMux")
            self.retired.add(self.owner)
        self.owner = identity
        self.sequences[identity] = command.sequence
        self.deadline_ns = command.deadline_ns

    def retire(self):
        """Closing a provider ends its lease and retires the previous session."""
        if self.owner is not None:
            self.retired.add(self.owner)
        self.owner, self.deadline_ns = None, 0


class CommandSink(Generic[T]):
    """Producer-owned write port. Feedback belongs on a separate Signal.

    apply receives a validated envelope, so the hardware adapter can revalidate
    it at the final provider. safe receives the configured fallback value.
    Both callbacks run on the provider Component's metered tasks.
    """
    def __init__(self, producer, name, apply, *, fallback, safe=None, max_ttl=.5,
                 hz=100):
        if not name or name in producer.sinks:
            raise ValueError("sink names must be nonempty and unique")
        self.producer, self.name, self.apply = producer, name, apply
        self.fallback, self.safe, self.hz = fallback, safe, hz
        self.guard = CommandGuard(max_ttl=max_ttl)
        self._lock = asyncio.Lock()
        self._active = self._armed = False
        self._bound = True
        self._epoch, self._sequence = uuid.uuid4().hex, 0
        producer.sinks[name] = self

    def _resolved(self):
        owner = self.producer._binding or self.producer._canonical
        if owner and self.name not in owner.sinks:
            raise PortNotBound(f"command port {self.name!r} was not requested")
        return owner.sinks[self.name]._resolved() if owner else self

    def __getstate__(self):
        state = dict(self.__dict__)
        state.pop("_lock")
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self._lock = asyncio.Lock()

    async def _activate(self):
        self._active, self._armed = True, False
        self._epoch, self._sequence = uuid.uuid4().hex, 0
        self.producer.service("command:" + self.name, self._accept, hz=self.hz, capacity=1)
        self.producer.task("deadman:" + self.name, self._expire, hz=self.hz)

    async def set(self, command: T | CommandEnvelope[T], *, ttl=None, _writer=None):
        actual = self._resolved()
        if actual is not self:
            return await actual.set(command, ttl=ttl, _writer=_writer)
        if not self._bound:
            raise PortNotBound(f"command port {self.name!r} was not requested")
        if not self._active or self.producer._closed or self.producer._closing:
            raise ComponentError("command provider is closed")
        claim = self.producer._runtime._command_claims.get(self)
        if claim is not None and claim is not _writer:
            raise CommandRejected("sink owned by Connect; send through its CommandMux")
        if not isinstance(command, CommandEnvelope):
            duration = self.guard.max_ttl_ns if ttl is None else int(ttl * 1e9)
            if duration <= 0 or duration > self.guard.max_ttl_ns:
                raise CommandRejected("ttl must be positive and at most provider max_ttl")
            self._sequence += 1
            command = CommandEnvelope(command, "direct:" + self.name, self._epoch,
                                      self._sequence, time.monotonic_ns() + duration)
        result = await self.producer.call("command:" + self.name, command)
        if isinstance(result, CommandRejected):
            raise result
        return result

    async def _invoke(self, callback, value):
        result = callback(value)
        return await result if inspect.isawaitable(result) else result

    async def _accept(self, command):
        async with self._lock:
            # A rejected request must not kill the metered service or clear a
            # valid controller's lease. Return the error as an ordinary response.
            armed = self._armed
            try:
                self.guard.accept(command)
                self._armed = True
                return await self._invoke(self.apply, command)
            except CommandRejected as error:
                self._armed = armed
                return error

    async def _safe(self):
        async with self._lock:
            if not self._armed:
                return
            if self.safe is not None:
                await self._invoke(self.safe, self.fallback)
            else:
                self._sequence += 1
                envelope = CommandEnvelope(self.fallback, "safe:" + self.name,
                                           self._epoch, self._sequence,
                                           time.monotonic_ns() + self.guard.max_ttl_ns)
                await self._invoke(self.apply, envelope)
            self._armed = False

    async def _expire(self):
        if self._armed and time.monotonic_ns() >= self.guard.deadline_ns:
            await self._safe()

    async def _deactivate(self):
        self._active = False
        try:
            await self._safe()
        finally:
            self.guard.retire()


class Connect(Component):
    """Forward a command Signal to one explicitly claimed sink."""
    def __init__(self, source, sink, *, ttl=.25, hz=100, controller_id=None):
        self.source, self.sink = as_signal(source), sink
        if not math.isfinite(ttl) or ttl <= 0:
            raise ValueError("ttl must be finite and positive")
        super().__init__(sink.producer, inputs=(self.source,))
        self.command_targets = (sink,)
        self.ttl, self.hz = ttl, hz
        self.controller_id = controller_id or "connect:" + uuid.uuid4().hex
        self.epoch, self.previous, self.sequence = None, 0, 0
        self.dropped = 0

    async def open(self):
        self.epoch, self.previous, self.sequence = uuid.uuid4().hex, 0, 0
        self.dropped = 0
        self.task("connect", self.forward, hz=self.hz)

    async def forward(self):
        frame = await self.source.get(after=self.previous)
        self.previous = frame.sequence
        command = frame.data
        if not isinstance(command, CommandEnvelope):
            self.sequence += 1
            age = max(0, time.time_ns() - frame.received_ns)
            command = CommandEnvelope(command, self.controller_id, self.epoch,
                                      self.sequence, time.monotonic_ns() + int(self.ttl * 1e9) - age)
        if command.deadline_ns <= time.monotonic_ns():
            self.dropped += 1
            return
        try:
            await self.sink.set(command, _writer=self)
        except CommandRejected as error:
            if error.reason != "expired":
                raise
            # A delayed command is rejected at the provider, but the next fresh
            # command must still be able to use the same healthy connection.
            self.dropped += 1


@dataclass(frozen=True)
class CommandInput:
    signal: object
    priority: int = 0
    timeout: float = .25


class CommandMux(PrimaryComponent):
    """Priority arbitration with optional manual selection and safe fallback.

    Equal priorities use input declaration order. override(name) exclusively
    selects that input; if stale it falls back rather than selecting a different
    controller. None restores priority arbitration. Cached commands never renew
    their source validity merely because the mux runs at a faster rate.
    """
    def __init__(self, *, fallback, hz=50, ttl=.25, history=32, **sources):
        if not sources or not 0 < ttl <= .5:
            raise ValueError("mux needs sources and ttl in (0, .5]")
        self.sources = {name: (value if isinstance(value, CommandInput) else CommandInput(value))
                        for name, value in sources.items()}
        for item in self.sources.values():
            if not math.isfinite(item.timeout) or item.timeout <= 0:
                raise ValueError("input timeout must be finite and positive")
        super().__init__(inputs=[as_signal(item.signal) for item in self.sources.values()],
                         history=history, clock="host:monotonic")
        self.fallback, self.hz, self.ttl = fallback, hz, ttl
        self.manual = None
        self.selected = None
        self._seen, self._values = {}, {}
        self._identity = "mux:" + uuid.uuid4().hex

    def override(self, name=None):
        if name is not None and name not in self.sources:
            raise KeyError(name)
        self.manual = name

    async def open(self):
        self._seen.clear()
        self._values.clear()
        self._epoch, self._command_sequence = uuid.uuid4().hex, 0
        self.task("arbitrate", self.arbitrate, hz=self.hz)

    async def arbitrate(self):
        now = time.monotonic_ns()
        for name, item in self.sources.items():
            frames = as_signal(item.signal).frames
            if not frames or frames[-1].sequence <= self._seen.get(name, 0):
                continue
            frame = frames[-1]
            self._seen[name] = frame.sequence
            deadline = now + int(item.timeout * 1e9) - max(0, time.time_ns() - frame.received_ns)
            value = frame.data
            if isinstance(value, CommandEnvelope):
                deadline = min(deadline, value.deadline_ns)
                value = value.value
            self._values[name] = (value, deadline)
        candidates = [name for name in self.sources if name in self._values
                      and self._values[name][1] > now
                      and (self.manual is None or self.manual == name)]
        selected = max(candidates, key=lambda name: self.sources[name].priority, default=None)
        self.selected = selected
        value, deadline = (self._values[selected] if selected is not None
                           else (self.fallback, now + int(self.ttl * 1e9)))
        self._command_sequence += 1
        envelope = CommandEnvelope(value, self._identity, self._epoch, self._command_sequence,
                                   min(deadline, now + int(self.ttl * 1e9)))
        await self.output.publish(envelope, stamp_ns=now, clock="host:monotonic")


Arbiter = CommandMux
