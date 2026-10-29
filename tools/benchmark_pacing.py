"""Repeatable software pacing measurements; run as python -m tools.benchmark_pacing."""
import argparse
import asyncio
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import numpy as np

from rsim.core import Component, Metronome
from rsim.runtime import Runtime, ProcessPlacement


def percentiles(values):
    return dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(values, (50, 95, 99)))))


class Pulse(Component):
    def __init__(self, hz, elements):
        super().__init__()
        self.hz, self.elements = hz, elements
        self.output = self.signal('output', history=2, clock='host:monotonic')

    async def open(self):
        self.payload = np.zeros(self.elements, dtype=np.float64)
        self.task('pulse', self.pulse, hz=self.hz)

    async def pulse(self):
        await self.output.publish(self.payload, stamp_ns=time.monotonic_ns(), clock='host:monotonic')


async def measure(hz, count, elements, seconds, placed, load):
    pulses = [Pulse(hz, elements) for _ in range(count)]
    intervals = [[] for _ in pulses]
    observed = [0] * count
    async def consume(index, signal):
        previous, stamp = 0, None
        while True:
            frame = await signal.get(after=previous, timeout=10)
            previous = frame.sequence
            observed[index] += 1
            if stamp is not None:
                intervals[index].append(abs((frame.stamp_ns - stamp) / 1e9 - 1 / hz))
            stamp = frame.stamp_ns
    async def busy():
        pace = Metronome(100)
        while True:
            await pace.tick()
            until = time.perf_counter() + .0005
            while time.perf_counter() < until:
                pass
    placement = {pulse: ProcessPlacement('pacing') for pulse in pulses} if placed else None
    async with Runtime(*(pulse.output for pulse in pulses), placement=placement):
        await asyncio.gather(*(pulse.output.get(timeout=15) for pulse in pulses))
        tasks = [asyncio.create_task(consume(i, pulse.output)) for i, pulse in enumerate(pulses)]
        if load:
            tasks.append(asyncio.create_task(busy()))
        start, cpu = time.perf_counter(), time.process_time()
        await asyncio.sleep(seconds)
        elapsed, cpu = time.perf_counter() - start, time.process_time() - cpu
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
    values = [value for group in intervals for value in group]
    return dict(hz=hz, components=count, elements=elements, processes=2 if placed else 1,
                load=load, elapsed_s=elapsed, parent_cpu_s=cpu, frames=sum(observed),
                observed_hz=sum(observed) / elapsed / count,
                interval_error_s=percentiles(values),
                note='Cross-process observation is latest-only; CPU covers parent only.')


async def throughput():
    component = Component()
    output = component.signal('output', history=2, clock='bench')
    payload = np.zeros(1024)
    async with Runtime(output):
        start, cpu = time.perf_counter(), time.process_time()
        for i in range(20000):
            frame = await output.publish(payload, stamp_ns=i, clock='bench')
            assert await output.get() is frame
        elapsed = time.perf_counter() - start
        return dict(operations=20000, seconds=elapsed, cpu_s=time.process_time() - cpu,
                    publish_get_per_s=20000 / elapsed)


async def main(args):
    rows, hot = [], []
    for repeat in range(args.repeat):
        hot.append(await throughput())
        for hz in (50, 100, 1000):
            for count, elements, placed, load in ((1, 1, False, False), (4, 1024, False, True),
                                                 (2, 8192, True, False)):
                row = await measure(hz, count, elements, args.seconds, placed, load)
                row['repeat'] = repeat + 1
                rows.append(row)
    report = dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
                  label=args.label, rows=rows, hot_path=hot,
                  median_publish_get_per_s=statistics.median(row['publish_get_per_s'] for row in hot))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--label', default='candidate')
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--seconds', type=float, default=1.5)
    args = parser.parse_args()
    if args.repeat < 1 or args.seconds <= 0:
        parser.error('repeat and seconds must be positive')
    asyncio.run(main(args))
