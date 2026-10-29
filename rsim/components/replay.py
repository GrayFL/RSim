"""Replay recorded source timestamps with explicit pause, rate and seek episodes."""
import asyncio
from copy import deepcopy
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import uuid
from time import monotonic_ns

from rsim.core import Component, ComponentError, Tick, PacerClosed


@dataclass(frozen=True)
class ReplaySample:
    stamp_ns: int
    data: object
    metadata: dict = field(default_factory=dict)


class ReplayPacer:
    """One playback owner. Other consumers subscribe through its output Signal."""

    def __init__(self, stamps, *, clock, rate, paused=False):
        self.stamps, self.clock, self.rate = stamps, clock, rate
        self.position = self.episode = 0
        self.session = uuid.uuid4().hex
        self.last_tick = None
        self.paused, self.closed = paused, False
        self.condition = asyncio.Condition()
        self.anchor_host = asyncio.get_running_loop().time()
        self.anchor_stamp = stamps[0]
        self.pause_host = self.anchor_host if paused else None

    @property
    def finished(self):
        return self.position >= len(self.stamps)

    async def tick(self):
        async with self.condition:
            while True:
                if self.closed:
                    raise PacerClosed('replay stopped')
                if self.paused or self.finished:
                    await self.condition.wait()
                    continue
                stamp = self.stamps[self.position]
                remaining = (self.anchor_host + (stamp-self.anchor_stamp)/1e9/self.rate
                             - asyncio.get_running_loop().time())
                if remaining > 0:
                    try:
                        await asyncio.wait_for(self.condition.wait(), remaining)
                    except TimeoutError:
                        pass
                    continue
                self.last_tick = Tick(stamp, self.clock, self.episode, self.position, self.session)
                self.position += 1
                return self.last_tick

    async def pause(self):
        async with self.condition:
            if not self.paused:
                self.paused = True
                self.pause_host = asyncio.get_running_loop().time()
                self.condition.notify_all()

    async def resume(self):
        async with self.condition:
            if self.paused:
                self.anchor_host += asyncio.get_running_loop().time() - self.pause_host
                self.paused, self.pause_host = False, None
                self.condition.notify_all()

    async def set_rate(self, rate):
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('replay rate must be finite and positive')
        async with self.condition:
            now = self.pause_host if self.paused else asyncio.get_running_loop().time()
            self.anchor_stamp += (now - self.anchor_host) * self.rate * 1e9
            self.anchor_host, self.rate = now, rate
            self.condition.notify_all()

    async def seek(self, index):
        if not isinstance(index, int) or not 0 <= index < len(self.stamps):
            raise ValueError('replay index is outside the recording')
        async with self.condition:
            self.position, self.episode = index, self.episode + 1
            self.anchor_stamp = self.stamps[index]
            self.anchor_host = asyncio.get_running_loop().time()
            if self.paused:
                self.pause_host = self.anchor_host
            self.condition.notify_all()

    async def aclose(self):
        async with self.condition:
            self.closed = True
            self.condition.notify_all()


class ReplaySource(Component):
    """Replay one recorded episode; seek starts a new episode and clears history.

    Records are explicit immutable snapshots owned by this component. Native
    timestamps are never replaced by callback time. EOF idles until seek/close;
    cached data does not renew command leases. File input supports JSON data;
    array/image record formats can construct ReplaySample objects directly.
    """

    def __init__(self, records, *, clock, rate=1., paused=False, history=32):
        super().__init__()
        self.records = tuple(deepcopy(record) for record in records)
        if not self.records or any(not isinstance(r, ReplaySample) or not isinstance(r.stamp_ns, int)
                                   or r.stamp_ns < 0 for r in self.records):
            raise ValueError('replay needs samples with nonnegative integer timestamps')
        if any(a.stamp_ns >= b.stamp_ns for a, b in zip(self.records, self.records[1:])):
            raise ValueError('recorded timestamps must strictly increase within an episode')
        if not isinstance(clock, str) or not clock:
            raise ValueError('replay requires the source clock domain')
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('replay rate must be finite and positive')
        self.clock, self.rate, self.paused = clock, rate, paused
        self.output = self.signal('output', clock=clock, history=history)
        self.pacer = None

    @classmethod
    def from_jsonl(cls, path, **settings):
        """Explicit recording I/O; no repository scanning or implicit path changes."""
        with Path(path).open(encoding='utf-8') as stream:
            records = [ReplaySample(**json.loads(line)) for line in stream if line.strip()]
        return cls(records, **settings)

    async def open(self):
        self.pacer = ReplayPacer(tuple(record.stamp_ns for record in self.records), clock=self.clock,
                                 rate=self.rate, paused=self.paused)
        self.task('replay', self.publish_record, pacer=self.pacer)

    async def publish_record(self):
        tick = self.pacer.last_tick
        record = self.records[tick.step_index]
        metadata = dict(record.metadata, episode=tick.episode, session=tick.session, step_index=tick.step_index)
        if 'host_monotonic_ns' in metadata:
            metadata['recorded_host_monotonic_ns'] = metadata['host_monotonic_ns']
        # Replay is a new source session. Keep recorded source host time as
        # provenance, while freshness measures this publication. Command
        # envelopes in the payload retain their original expiry unchanged.
        metadata['host_monotonic_ns'] = monotonic_ns()
        await self.output.publish(record.data, stamp_ns=record.stamp_ns, clock=self.clock, metadata=metadata)

    def _active(self):
        if self.pacer is None or self._closed:
            raise ComponentError('replay controls require a running source')
        return self.pacer

    async def pause(self):
        await self._active().pause()

    async def resume(self):
        await self._active().resume()

    async def set_rate(self, rate):
        await self._active().set_rate(rate)

    async def seek(self, index):
        pacer = self._active()
        if not isinstance(index, int) or not 0 <= index < len(self.records):
            raise ValueError('replay index is outside the recording')
        await self.output.clear_history()
        await pacer.seek(index)

    async def close(self):
        if self.pacer is not None:
            await self.pacer.aclose()
