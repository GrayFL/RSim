"""Time joins, distinct from Bundle's independent latest snapshots."""
import asyncio
import inspect

from .clocks import clock_name
from .core import PrimaryComponent
from .errors import HistoryMiss
from .model import Frame
from .signal import as_signal


class Synchronizer(PrimaryComponent):
    """Join retained samples to one reference timestamp.

    Nearest selects the closest sample currently retained within tolerance.
    Interpolation is opt-in per input: callback(left.data, right.data, fraction).
    It requires a bracket, both ends within tolerance, and never extrapolates.
    Output data is a name -> Frame mapping, preserving original sample clocks;
    interpolated Frames use the target clock, timestamp and sequence=0 (derived).
    Missing matches wait at most wait_timeout, then drop that reference sample.
    """
    def __init__(self, *, tolerance_ns, reference=None, clock=None, transforms=None,
                 interpolate=None, wait_timeout=.2, hz=100, history=32, **sources):
        if not sources or tolerance_ns < 0 or wait_timeout < 0:
            raise ValueError("sources, nonnegative tolerance and wait_timeout required")
        self.sources = {name: as_signal(value) for name, value in sources.items()}
        self.reference = reference or next(iter(sources))
        if self.reference not in sources:
            raise ValueError("reference must name an input")
        self.transforms, self.interpolate = dict(transforms or {}), dict(interpolate or {})
        if (self.transforms.keys() | self.interpolate.keys()) - self.sources.keys():
            raise ValueError("transform/interpolation names must identify inputs")
        self.target_clock = clock_name(clock) if clock is not None else None
        self.tolerance_ns, self.wait_timeout, self.hz = tolerance_ns, wait_timeout, hz
        self.previous, self.dropped = 0, 0
        super().__init__(inputs=self.sources.values(), history=history, clock=self.target_clock)
        # Known mismatches fail while constructing the graph, unknown clocks are
        # checked again on every received frame.
        declared = []
        for name, source in self.sources.items():
            transform = self.transforms.get(name)
            if transform and source.clock and source.clock != transform.source:
                raise ValueError(f"transform source clock does not match {name}")
            declared.append(transform.target if transform else source.clock)
        known = {value for value in declared if value is not None}
        if self.target_clock:
            known.add(self.target_clock)
        if len(known) > 1:
            raise ValueError("different clock domains require explicit ClockTransforms")

    def _timestamp(self, name, frame, target):
        transform = self.transforms.get(name)
        if transform:
            if transform.target != target:
                raise ValueError("ClockTransform must target the join clock")
            return transform.convert(frame.stamp_ns, clock=frame.clock)
        if frame.clock != target:
            raise ValueError(f"input {name} clock {frame.clock} differs from join clock {target}")
        return frame.stamp_ns

    async def _sample(self, name, stamp_ns, clock):
        source = self.sources[name]
        while True:
            frames = source.frames
            candidates = sorted((self._timestamp(name, frame, clock), frame.sequence, frame)
                                for frame in frames)
            callback = self.interpolate.get(name)
            if candidates:
                nearest = min(candidates, key=lambda item: abs(item[0] - stamp_ns))
                if nearest[0] == stamp_ns or callback is None:
                    if abs(nearest[0] - stamp_ns) <= self.tolerance_ns:
                        return nearest[2]
                else:
                    left = [item for item in candidates if item[0] < stamp_ns]
                    right = [item for item in candidates if item[0] > stamp_ns]
                    if left and right:
                        a, b = left[-1], right[0]
                        if max(stamp_ns - a[0], b[0] - stamp_ns) <= self.tolerance_ns:
                            data = callback(a[2].data, b[2].data,
                                            (stamp_ns - a[0]) / (b[0] - a[0]))
                            if inspect.isawaitable(data):
                                data = await data
                            return Frame(data, stamp_ns, clock,
                                         max(a[2].received_ns, b[2].received_ns), 0)
            await source.get(after=frames[-1].sequence if frames else 0)

    async def join(self, timestamp_ns, *, clock, timeout=None):
        clock = clock_name(clock)
        if self.target_clock is not None and self.target_clock != clock:
            raise ValueError("join clock differs from the declared target")
        tasks = [asyncio.create_task(self._sample(name, timestamp_ns, clock))
                 for name in self.sources]
        try:
            async with asyncio.timeout(self.wait_timeout if timeout is None else timeout):
                values = await asyncio.gather(*tasks)
            return dict(zip(self.sources, values))
        except TimeoutError as error:
            raise HistoryMiss("time join has no matching sample within tolerance") from error
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def open(self):
        self.previous, self.dropped = 0, 0
        self.task("time-join", self.combine, hz=self.hz)

    async def combine(self):
        reference = await self.sources[self.reference].get(after=self.previous)
        self.previous = reference.sequence
        transform = self.transforms.get(self.reference)
        clock = self.target_clock or (transform.target if transform else reference.clock)
        timestamp = self._timestamp(self.reference, reference, clock)
        try:
            samples = await self.join(timestamp, clock=clock)
        except HistoryMiss:
            self.dropped += 1
            return
        await self.output.publish(samples, stamp_ns=timestamp, clock=clock,
                                  received_ns=max(frame.received_ns for frame in samples.values()))


TimeJoin = Synchronizer
