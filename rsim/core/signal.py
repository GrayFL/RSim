"""Time-indexed logical outputs. Signals own no tasks or resource lifecycle."""
from __future__ import annotations

import asyncio
from collections import deque
import time
from typing import Generic, TypeVar, TYPE_CHECKING

from .errors import ComponentError, HistoryMiss, PortNotBound
from .model import Frame, SampleId

if TYPE_CHECKING:
    from .component import Component

T = TypeVar("T")


class Signal(Generic[T]):
    def __init__(self, producer: Component, name: str, *, history=32, clock=None):
        if not isinstance(history, int) or history < 1:
            raise ValueError("history must be a positive integer")
        if not name or name in producer.outputs:
            raise ValueError("output names must be nonempty and unique within a component")
        self.producer, self.name = producer, name
        self.clock = getattr(clock, "name", clock)
        self._history = deque(maxlen=history)
        self._condition = asyncio.Condition()
        self._sequence = 0
        self._target = None
        # Installed by channel bindings at a placement boundary, never selected
        # by Signal itself. Unbound local publication preserves payload identity.
        self._prepare = None
        self._prepare_hooks = {}
        self._publication_sequence = 0
        self._bound = True
        self._port_error = None
        producer.outputs[name] = self

    def _resolved(self):
        return self._resolve_chain()[0]

    def _resolve_chain(self):
        signal = self
        seen, owners = set(), []
        while True:
            if id(signal) in seen:
                raise ComponentError("cyclic signal binding")
            seen.add(id(signal))
            owner = signal.producer._binding or signal.producer._canonical
            if owner is not None:
                if signal.name not in owner.outputs:
                    raise PortNotBound(f"port {signal.name!r} was not requested; add it to Runtime roots or inputs")
                signal = owner.outputs[signal.name]
            else:
                owners.append(signal.producer)
                if signal._target is not None:
                    signal = signal._target
                else:
                    return signal, owners

    @property
    def history_size(self):
        return self._history.maxlen

    def __getstate__(self):
        state = dict(self.__dict__)
        state.pop("_condition")
        state.update(_history=deque(maxlen=self.history_size), _sequence=0, _prepare=None,
                     _prepare_hooks={})
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self._condition = asyncio.Condition()

    @property
    def frames(self):
        """A snapshot of retained Frames; payloads are the original references."""
        return tuple(self._resolved()._history)

    def add_prepare_hook(self, key, prepare):
        self._prepare_hooks[key] = prepare

    def remove_prepare_hook(self, key):
        self._prepare_hooks.pop(key, None)

    async def publish(self, data: T, *, stamp_ns: int, clock, received_ns=None,
                      sample_id=None, metadata=None) -> Frame[T]:
        if self._resolved() is not self:
            raise ComponentError("only the canonical producer can publish this signal")
        owner = self.producer
        if owner._closed or owner._failure is not None:
            raise ComponentError("signal producer is not running") from owner._failure
        clock = getattr(clock, "name", clock)
        if not isinstance(clock, str) or not clock:
            raise ValueError("clock must identify a time domain")
        if self.clock is not None and clock != self.clock:
            raise ValueError(f"signal {self.name} requires clock {self.clock}, received {clock}")
        if not isinstance(stamp_ns, int):
            raise TypeError("stamp_ns must be integer nanoseconds")
        if self._prepare is not None:
            data = self._prepare(data)
        for prepare in dict.fromkeys(self._prepare_hooks.values()):
            data = prepare(data)
        self._sequence += 1
        self._publication_sequence += 1
        if sample_id is None:
            sample_id = SampleId(owner._instance_id, self.name, self._publication_sequence)
        frame = Frame(data, stamp_ns, clock, time.time_ns() if received_ns is None else received_ns,
                      self._sequence, sample_id, dict(metadata or {}))
        async with self._condition:
            self._history.append(frame)
            self._condition.notify_all()
        return frame

    async def get(self, *, timestamp_ns=None, clock=None, tolerance_ns=0, after=None,
                  timeout=None) -> Frame[T]:
        actual, owners = self._resolve_chain()
        if not actual._bound:
            raise PortNotBound(f"port {self.name!r} was not requested; add it to Runtime roots or inputs")
        clock = getattr(clock, "name", clock)
        if tolerance_ns < 0:
            raise ValueError("tolerance_ns must be nonnegative")
        if timestamp_ns is not None and (clock is None or after is not None):
            raise ValueError("timestamp lookup requires clock and cannot use after")
        async with asyncio.timeout(timeout):
            async with actual._condition:
                while True:
                    if not actual._bound:
                        raise PortNotBound(f"port {self.name!r} is no longer bound")
                    if actual._port_error is not None:
                        raise ComponentError(f"port {self.name!r} failed") from actual._port_error
                    for owner in owners:
                        if owner._failure is not None:
                            raise ComponentError("component task failed") from owner._failure
                        if owner._closed:
                            raise ComponentError("signal producer is closed")
                    if timestamp_ns is not None:
                        frames = [frame for frame in actual._history if frame.clock == clock]
                        if frames:
                            frame = min(frames, key=lambda f: abs(f.stamp_ns - timestamp_ns))
                            if abs(frame.stamp_ns - timestamp_ns) <= tolerance_ns:
                                return frame
                        raise HistoryMiss("no retained sample within the requested clock/tolerance")
                    if actual._history and (after is None or actual._history[-1].sequence > after):
                        return actual._history[-1]
                    await actual._condition.wait()

    async def _notify(self):
        actual = self._resolved()
        async with actual._condition:
            actual._condition.notify_all()


def as_signal(value) -> Signal:
    """Accept a Signal or an explicitly single-output component."""
    if isinstance(value, Signal):
        return value
    primary = getattr(value, "primary", None)
    if isinstance(primary, Signal):
        return primary
    raise TypeError("expected Signal or a component with an explicit primary output")
