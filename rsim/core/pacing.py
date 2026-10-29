"""Optional source-driven pacing. Source time never replaces host safety clocks."""
from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
from fractions import Fraction
import math
from typing import Protocol

from .metronome import Metronome
from .signal import as_signal


class Pacer(Protocol):
    async def tick(self): ...
    async def aclose(self): ...


class PacerClosed(RuntimeError):
    """The authoritative source stopped; waiting consumers must stop too."""


class PacerReset(RuntimeError):
    """An old source publication was invalidated by an explicit new episode."""


@dataclass(frozen=True)
class Tick:
    stamp_ns: int
    clock: str
    episode: int
    step_index: int
    session: str = ''


class RealTimePacer(Metronome):
    """Explicit host pacing; ordinary task(hz=...) still uses Metronome directly."""

    async def aclose(self):
        pass


class TickSource:
    """Broadcast caller-supplied environment steps to independent subscriptions.

    latest keeps one pending tick and counts skips. every uses a bounded queue
    and backpressures publication until each subscriber has space; delivery is
    not a compute/command ACK and does not establish lockstep causality.
    Create this object in its owner worker's open(), never in a shared recipe.
    """

    def __init__(self, *, clock):
        if not isinstance(clock, str) or not clock:
            raise ValueError('clock must be an explicit source time domain')
        self.clock = clock
        self._condition = asyncio.Condition()
        self._publish_lock = asyncio.Lock()
        self._subscriptions = set()
        self._closed = False
        self._error = None
        self.last_tick = None
        self._generation = None
        self._retired_sessions = set()

    def __getstate__(self):
        raise TypeError('TickSource is worker-local; construct it in Component.open')

    def subscribe(self, *, hz=None, policy='latest', capacity=1):
        if self._closed:
            raise PacerClosed('tick source is closed') from self._error
        subscription = TickSubscription(self, hz=hz, policy=policy, capacity=capacity)
        self._subscriptions.add(subscription)
        return subscription

    async def publish(self, *, stamp_ns, episode, step_index, session=''):
        if not all(isinstance(v, int) and not isinstance(v, bool) and v >= 0
                   for v in (stamp_ns, episode, step_index)):
            raise ValueError('stamp_ns, episode and step_index must be nonnegative integers')
        tick = Tick(stamp_ns, self.clock, episode, step_index, session)
        if not isinstance(session, str):
            raise TypeError('session must be a string')
        generation = (session, episode)
        # A reset is a control boundary: it must be able to invalidate a writer
        # blocked by an old every-step subscriber without waiting for that data.
        async with self._condition:
            current = self._generation
            if current != generation:
                if session in self._retired_sessions or (current is not None
                        and current[0] == session and episode < current[1]):
                    raise PacerReset('source publication belongs to a retired generation')
                if current is not None and current[0] != session:
                    self._retired_sessions.add(current[0])
                self._generation = generation
                self.last_tick = None
                for subscription in self._subscriptions:
                    subscription._reset(stamp_ns)
                self._condition.notify_all()
        async with self._publish_lock:
            async with self._condition:
                previous = self.last_tick
                changed = previous is None or (previous.session, previous.episode) != generation
                if previous is not None and not changed:
                    if step_index <= previous.step_index or stamp_ns < previous.stamp_ns:
                        raise ValueError('source steps must advance within an episode')
                while True:
                    if self._closed:
                        raise PacerClosed('tick source is closed') from self._error
                    if generation != self._generation:
                        raise PacerReset('source publication invalidated by reset')
                    due = [s for s in self._subscriptions if s._due(stamp_ns)]
                    if all(s.policy == 'latest' or len(s._pending) < s.capacity for s in due):
                        break
                    await self._condition.wait()
                self.last_tick = tick
                for subscription in due:
                    subscription._put(tick)
                self._condition.notify_all()
        return tick

    async def aclose(self, error=None):
        async with self._condition:
            self._closed, self._error = True, error
            for subscription in self._subscriptions:
                subscription._pending.clear()
            self._condition.notify_all()


class TickSubscription:
    def __init__(self, source, *, hz, policy, capacity):
        if policy not in ('latest', 'every') or not isinstance(capacity, int) or capacity < 1:
            raise ValueError('policy must be latest/every with positive capacity')
        if hz is not None and (not math.isfinite(hz) or hz <= 0):
            raise ValueError('hz must be finite and positive')
        self.source, self.policy = source, policy
        self.capacity = 1 if policy == 'latest' else capacity
        self.frequency = Fraction(str(hz)) if hz is not None else None
        self._pending = deque()
        self._closed = False
        self.last_tick = None
        self.skipped = 0
        self._reset(0)

    def _reset(self, anchor):
        self._pending.clear()
        self._anchor, self._index = anchor, 0

    def _due(self, stamp):
        frequency = self.frequency
        return frequency is None or ((stamp - self._anchor) * frequency.numerator
                                     >= self._index * 1_000_000_000 * frequency.denominator)

    def _put(self, tick):
        if self.policy == 'latest' and self._pending:
            self.skipped += len(self._pending)
            self._pending.clear()
        self._pending.append(tick)
        if self.frequency is not None:
            self._index = ((tick.stamp_ns - self._anchor) * self.frequency.numerator
                           // (1_000_000_000 * self.frequency.denominator)) + 1

    async def tick(self):
        async with self.source._condition:
            while not self._pending:
                if self._closed or self.source._closed:
                    raise PacerClosed('tick subscription is closed') from self.source._error
                await self.source._condition.wait()
            self.last_tick = self._pending.popleft()
            self.source._condition.notify_all()
            return self.last_tick

    async def aclose(self):
        async with self.source._condition:
            self._closed = True
            self._pending.clear()
            self.source._subscriptions.discard(self)
            self.source._condition.notify_all()


class SignalPacer:
    """Latest-only source events across existing Signal/placement boundaries.

    Bind in the consumer's open(). The Signal must also appear in its inputs
    to request transport and lifecycle ownership. No remote clock polling.
    """

    def __init__(self, signal):
        self.signal = as_signal(signal)
        self.previous = 0
        self.last_frame = self.last_tick = None
        self._waiter = None
        self._closed = False

    async def tick(self):
        if self._closed:
            raise PacerClosed('signal pacer is closed')
        if self._waiter is not None:
            raise RuntimeError('each task needs an independent SignalPacer')
        self._waiter = asyncio.create_task(self.signal.get(after=self.previous))
        try:
            frame = await self._waiter
        except asyncio.CancelledError:
            if self._closed:
                raise PacerClosed('signal pacer is closed') from None
            raise
        finally:
            self._waiter = None
        self.previous, self.last_frame = frame.sequence, frame
        self.last_tick = Tick(frame.stamp_ns, frame.clock, frame.metadata.get('episode', 0),
                              frame.metadata.get('step_index', frame.sequence), frame.metadata.get('session', ''))
        return self.last_tick

    async def aclose(self):
        self._closed = True
        if self._waiter is not None:
            self._waiter.cancel()
            await asyncio.gather(self._waiter, return_exceptions=True)
