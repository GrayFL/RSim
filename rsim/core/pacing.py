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
from .errors import HistoryMiss


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


class RatePacer:
    """Optional adjustable host cadence for a fixed-step environment source.

    This object never advances environment time. The owner advances its own
    fixed dt after tick(), and owns command retirement at pause/rate boundaries.
    Missed host deadlines are skipped, with no unbounded catch-up bursts.
    Bind in open(); ordinary task(hz=...) does not use this implementation.
    """

    def __init__(self, hz, *, rate=1., paused=False):
        self.hz, self.rate = hz, rate
        self._period = self._validate_period(hz, rate)
        self.paused, self.closed = bool(paused), False
        self._condition = asyncio.Condition()
        self._next_deadline = None
        self._remaining = 0.
        self._ticking = False
        self.deadline = None
        self.actual_start_ns = self.scheduled_deadline_ns = None
        self.missed = self.received = 0

    @staticmethod
    def _validate_period(hz, rate):
        if (isinstance(hz, bool) or isinstance(rate, bool)
                or not math.isfinite(hz) or not math.isfinite(rate) or hz <= 0 or rate <= 0):
            raise ValueError('hz and rate must be finite and positive')
        period = (1 / hz) / rate
        if not math.isfinite(period) or period <= 0:
            raise ValueError('hz and rate must yield a finite positive period')
        return period

    def __getstate__(self):
        raise TypeError('RatePacer is worker-local; construct it in Component.open')

    def _check_open(self):
        if self.closed:
            raise PacerClosed('rate pacer is closed')

    async def tick(self):
        if self._ticking:
            raise RuntimeError('each task needs an independent RatePacer')
        self._ticking = True
        try:
            async with self._condition:
                self._check_open()
                loop = asyncio.get_running_loop()
                now = loop.time()
                if self._next_deadline is None:
                    self._next_deadline = now
                elif not self.paused and self._next_deadline < now:
                    skipped = math.ceil((now - self._next_deadline) / self._period)
                    self.missed += skipped
                    self._next_deadline += skipped * self._period
                while True:
                    self._check_open()
                    if self.paused:
                        await self._condition.wait()
                        continue
                    remaining = self._next_deadline - loop.time()
                    if remaining > 0:
                        try:
                            await asyncio.wait_for(self._condition.wait(), remaining)
                        except TimeoutError:
                            pass
                        continue
                    self.deadline = self._next_deadline
                    self.scheduled_deadline_ns = round(self.deadline * 1e9)
                    self.actual_start_ns = round(loop.time() * 1e9)
                    self._next_deadline += self._period
                    self.received += 1
                    return
        finally:
            self._ticking = False

    async def pause(self):
        async with self._condition:
            self._check_open()
            if not self.paused:
                self._remaining = (0. if self._next_deadline is None else
                                   max(0., self._next_deadline - asyncio.get_running_loop().time()))
                self.paused = True
                self._condition.notify_all()

    async def resume(self):
        async with self._condition:
            self._check_open()
            if self.paused:
                self._next_deadline = asyncio.get_running_loop().time() + self._remaining
                self.paused = False
                self._condition.notify_all()

    async def set_rate(self, rate):
        period = self._validate_period(self.hz, rate)
        async with self._condition:
            self._check_open()
            if self.paused:
                self._remaining *= period / self._period
            elif self._next_deadline is not None:
                now = asyncio.get_running_loop().time()
                self._next_deadline = now + max(0., self._next_deadline - now) * period / self._period
            self._period, self.rate = period, rate
            self._condition.notify_all()

    async def aclose(self):
        async with self._condition:
            self.closed = True
            self._condition.notify_all()


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

    @property
    def generation(self):
        """Current (session, episode), including a reset invalidating a blocked writer."""
        return self._generation

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

    async def _wait_boundary(self, generation):
        async with self._condition:
            while not self._closed and self._generation == generation:
                await self._condition.wait()
            if self._closed:
                raise PacerClosed('tick source is closed') from self._error


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

    def __init__(self, signal, *, hz=None, deduplicate=False):
        if hz is not None and (isinstance(hz, bool) or not math.isfinite(hz) or hz <= 0):
            raise ValueError('hz must be finite and positive')
        self.signal = as_signal(signal)
        self.frequency = Fraction(str(hz)) if hz is not None else None
        self.deduplicate = deduplicate
        self.previous = 0
        self.last_frame = self.last_tick = None
        self.elapsed_ns = None
        self.received = self.delivered = self.skipped = self.filtered = self.duplicates = self.resets = 0
        self.source_skipped = 0
        self._observed_tick = self._publication = None
        self._retired_sessions = set()
        self._anchor = self._index = 0
        self._waiter = None
        self._ticking = False
        self._closed = False

    async def tick(self):
        if self._closed:
            raise PacerClosed('signal pacer is closed')
        if self._ticking:
            raise RuntimeError('each task needs an independent SignalPacer')
        self._ticking = True
        try:
            while True:
                if self._closed:
                    raise PacerClosed('signal pacer is closed')
                self._waiter = asyncio.create_task(self.signal.get(after=self.previous))
                try:
                    frame = await self._waiter
                finally:
                    self._waiter = None
                self.previous = frame.sequence
                self.received += 1
                sample = frame.sample_id
                session = frame.metadata.get('session', sample.producer_instance_id if sample else '')
                step = frame.metadata.get('step_index', sample.publication_sequence if sample else frame.sequence)
                tick = Tick(frame.stamp_ns, frame.clock, frame.metadata.get('episode', 0), step, session)
                if (not isinstance(session, str) or any(type(value) is not int or value < 0
                        for value in (tick.episode, tick.step_index))):
                    raise ValueError('source tick requires a string session and nonnegative integer episode/step')
                old = self._observed_tick
                changed = old is None or (old.session, old.episode) != (tick.session, tick.episode)
                strict = self.deduplicate or self.frequency is not None
                duplicate = (old is not None and not changed and tick.step_index == old.step_index
                             and tick.stamp_ns == old.stamp_ns)
                if changed:
                    if strict and (tick.session in self._retired_sessions or (old is not None
                            and tick.session == old.session and tick.episode < old.episode)):
                        raise PacerReset('signal tick belongs to a retired generation')
                    if old is not None:
                        self.resets += 1
                        if strict and old.session != tick.session:
                            self._retired_sessions.add(old.session)
                    self._anchor, self._index = tick.stamp_ns, 0
                    self._publication = None
                elif strict and (tick.clock != old.clock or tick.stamp_ns < old.stamp_ns
                                 or tick.step_index < old.step_index):
                    raise ValueError('source time/step must not regress within an episode')
                elif strict and tick.step_index == old.step_index:
                    if not duplicate:
                        raise ValueError('one source step cannot have two timestamps')
                if sample is not None:
                    publication = (sample.producer_instance_id, sample.canonical_port_id, sample.publication_sequence)
                else:
                    publication = (tick.session, self.signal.name, frame.sequence)
                previous = self._publication
                if previous is not None and publication[:2] == previous[:2]:
                    self.skipped += max(0, publication[2] - previous[2] - 1)
                self._publication, self._observed_tick = publication, tick
                if duplicate:
                    self.duplicates += 1
                    if self.deduplicate:
                        continue
                frequency = self.frequency
                if frequency is not None:
                    numerator = (tick.stamp_ns - self._anchor) * frequency.numerator
                    denominator = 1_000_000_000 * frequency.denominator
                    if numerator < self._index * denominator:
                        self.filtered += 1
                        continue
                    self._index = numerator // denominator + 1
                previous = self.last_tick
                same = previous is not None and (previous.session, previous.episode) == (tick.session, tick.episode)
                self.elapsed_ns = tick.stamp_ns - previous.stamp_ns if same else None
                if same:
                    self.source_skipped += max(0, tick.step_index - previous.step_index - 1)
                self.last_frame, self.last_tick = frame, tick
                self.delivered += 1
                return tick
        except asyncio.CancelledError:
            if self._closed:
                raise PacerClosed('signal pacer is closed') from None
            raise
        finally:
            self._ticking = False

    async def aclose(self):
        self._closed = True
        if self._waiter is not None:
            self._waiter.cancel()
            await asyncio.gather(self._waiter, return_exceptions=True)


class FrameTickPacer:
    """Match a local TickSubscription to the corresponding completed Signal frame.

    Delivery order of independent subscriptions is not a computation barrier.
    A downstream stage can wait here for its upstream stage's actual output.
    The caller must ensure co-location and enough Signal history for every's
    bounded queue. An irretrievably missing every frame is an explicit error.
    """

    def __init__(self, signal, subscription):
        if not isinstance(subscription, TickSubscription):
            raise TypeError('FrameTickPacer requires a worker-local TickSubscription')
        self.signal, self.subscription = as_signal(signal), subscription
        self.last_tick = self.last_frame = None
        self.elapsed_ns = None
        self.received = self.delivered = self.resets = 0
        self._discarded = 0
        self._waiter = None
        self._closed = self._ticking = False

    @property
    def skipped(self):
        return self.subscription.skipped + self._discarded

    async def _wait(self, awaitable):
        self._waiter = asyncio.create_task(awaitable)
        try:
            return await self._waiter
        finally:
            self._waiter = None

    @staticmethod
    def _identity(frame):
        metadata = frame.metadata
        return (metadata.get('session', ''), metadata.get('episode', 0),
                metadata.get('step_index', frame.sequence))

    async def _wait_frame(self, after, generation):
        # A reset/failed source must also release a consumer whose upstream
        # calculation never produced a matching frame.
        frame = asyncio.create_task(self.signal.get(after=after))
        boundary = asyncio.create_task(self.subscription.source._wait_boundary(generation))
        try:
            done, _ = await asyncio.wait((frame, boundary), return_when=asyncio.FIRST_COMPLETED)
            if boundary in done:
                await boundary
            else:
                await frame
        finally:
            for task in (frame, boundary):
                if not task.done():
                    task.cancel()
            await asyncio.gather(frame, boundary, return_exceptions=True)

    async def tick(self):
        if self._ticking:
            raise RuntimeError('each task needs an independent FrameTickPacer')
        self._ticking = True
        try:
            while not self._closed:
                tick = await self._wait(self.subscription.tick())
                self.received += 1
                identity = (tick.session, tick.episode, tick.step_index)
                while not self._closed:
                    if self.subscription.source.generation != identity[:2]:
                        self._discarded += 1
                        break  # A reset superseded an in-flight upstream computation.
                    frames = self.signal.frames
                    match = next((frame for frame in reversed(frames)
                                  if self._identity(frame) == identity
                                  and frame.stamp_ns == tick.stamp_ns and frame.clock == tick.clock), None)
                    if match is not None:
                        old = self.last_tick
                        same = old is not None and (old.session, old.episode) == identity[:2]
                        self.elapsed_ns = tick.stamp_ns - old.stamp_ns if same else None
                        if old is not None and not same:
                            self.resets += 1
                        self.last_tick, self.last_frame = tick, match
                        self.delivered += 1
                        return tick
                    latest = frames[-1] if frames else None
                    if latest is not None and self._identity(latest)[:2] == identity[:2]:
                        observed_step = self._identity(latest)[2]
                        if observed_step >= tick.step_index:
                            if self.subscription.policy == 'every':
                                raise HistoryMiss(f'no frame for delivered tick {identity!r}; '
                                                  'check upstream cadence and history capacity')
                            self._discarded += 1
                            break
                    await self._wait(self._wait_frame(latest.sequence if latest is not None else 0, identity[:2]))
            raise PacerClosed('frame tick pacer is closed')
        except asyncio.CancelledError:
            if self._closed:
                raise PacerClosed('frame tick pacer is closed') from None
            raise
        finally:
            self._ticking = False

    async def aclose(self):
        self._closed = True
        if self._waiter is not None:
            self._waiter.cancel()
            await asyncio.gather(self._waiter, return_exceptions=True)
        await self.subscription.aclose()
