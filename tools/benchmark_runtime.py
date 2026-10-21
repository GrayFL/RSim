"""Reproducible same-platform performance evidence; never operates hardware."""
import argparse
import asyncio
import json
import importlib.metadata
import platform
from pathlib import Path
import sys
import time
import uuid

# Optional baseline checkout is explicit and recorded, never used by wheel gates.
if '--source' in sys.argv:
    sys.path.insert(0, sys.argv[sys.argv.index('--source') + 1])

import numpy as np
import psutil
import rsim
from rsim import Component, Runtime, SharedProvider, SharedComponent, CommandSink, VelocityCommand
from rsim.runtime.sharing import component_ports


WORKLOADS = {
    'small': dict(elements=1, consumers=1, late_s=0., delay_s=0.),
    'rgbd': dict(elements=640*480*4, consumers=1, late_s=0., delay_s=0.),
    'points': dict(elements=300_000, consumers=1, late_s=0., delay_s=0.),
    'fanout': dict(elements=300_000, consumers=3, late_s=0., delay_s=0.),
    'late': dict(elements=300_000, consumers=2, late_s=.3, delay_s=0.),
    'slow': dict(elements=300_000, consumers=1, late_s=0., delay_s=.1),
    'mapping-control': dict(elements=300_000, consumers=2, late_s=0., delay_s=.03),
}


class Source(Component):
    def __init__(self, elements, hz, history):
        super().__init__()
        self.elements, self.hz = elements, hz
        self.output = self.signal('output', history=history, clock='benchmark')
        self.feedback = self.signal('feedback', history=history, clock='benchmark')
        self.velocity = CommandSink(self, 'velocity', self.apply, safe=self.safe, fallback=VelocityCommand())
        self.current, self.fallback_ns = VelocityCommand(), None
    async def apply(self, envelope):
        self.current = envelope.value
    async def safe(self, value):
        self.current, self.fallback_ns = value, time.monotonic_ns()
    async def open(self):
        self.task('publish', self.publish, hz=self.hz)
    async def publish(self):
        now = time.monotonic_ns()
        array = np.full(self.elements, now % 123, dtype=np.float32)
        array.flags.writeable = False
        await self.output.publish(array, stamp_ns=now, clock='benchmark', metadata={'published_ns': now})
        await self.feedback.publish(dict(linear_x=self.current.linear_x, angular_z=self.current.angular_z,
                                         fallback_ns=self.fallback_ns), stamp_ns=now, clock='benchmark')


def percentiles(values):
    return dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(values, [50, 95, 99])))) if values else {}


async def measure(name, config, seconds, hz, history):
    source = Source(config['elements'], hz, history)
    provider = SharedProvider(source, key='benchmark:' + uuid.uuid4().hex)
    runtime = Runtime(provider, _root_ports=False)
    start = time.perf_counter()
    await runtime.__aenter__()
    startup = time.perf_counter() - start
    views = [SharedComponent(key=provider.component_key, ports=component_ports(source)) for _ in range(config['consumers'])]
    clients = []
    latencies, command_ack, skipped = [], [], 0
    first_frame = None
    try:
        for i, view in enumerate(views):
            if i and config['late_s']:
                await asyncio.sleep(config['late_s'])
            client = Runtime(view.output, view.feedback, view.velocity)
            await client.__aenter__()
            clients.append(client)
        initial = await asyncio.gather(*(view.output.get(timeout=5) for view in views))
        first_frame = time.perf_counter() - start
        cursors = [frame.sequence for frame in initial]
        publications = [frame.sample_id.publication_sequence for frame in initial]
        stop = time.perf_counter() + seconds
        while time.perf_counter() < stop:
            for index, view in enumerate(views):
                frame = await view.output.get(after=cursors[index], timeout=5)
                sequence = frame.sample_id.publication_sequence
                if publications[index]:
                    skipped += max(0, sequence - publications[index] - 1)
                cursors[index], publications[index] = frame.sequence, sequence
                latencies.append((time.monotonic_ns()-frame.metadata['published_ns']) / 1e6)
            sent = time.perf_counter()
            await views[0].velocity.set(VelocityCommand(), ttl=.2)
            command_ack.append((time.perf_counter()-sent)*1000)
            await asyncio.sleep(config['delay_s'])
        await asyncio.sleep(.22)  # finish the previous controller's .2s lease
        deadline = time.monotonic_ns() + 80_000_000
        from rsim.core import CommandEnvelope
        # This is an in-memory fixture, never a device command.
        await views[0].velocity.set(CommandEnvelope(VelocityCommand(.1), 'benchmark', 'ttl', 1, deadline))
        feedback = await views[0].feedback.get(timeout=2)
        while feedback.data['fallback_ns'] is None or feedback.data['fallback_ns'] < deadline:
            feedback = await views[0].feedback.get(after=feedback.sequence, timeout=2)
        from rsim.core import CommandRejected
        try:
            await views[0].velocity.set(CommandEnvelope(VelocityCommand(), 'benchmark', 'ttl', 2, time.monotonic_ns()-1))
        except CommandRejected as error:
            assert error.reason == 'expired'
            expired_commands = 1
        else:
            raise AssertionError('provider accepted an expired command')
        worker = psutil.Process(provider.manifest['pid'])
        metrics = dict(rss_bytes=worker.memory_info().rss,
                       handles=worker.num_handles() if hasattr(worker, 'num_handles') else worker.num_fds(),
                       disk_io=worker.io_counters()._asdict())
        manifest = await provider.connection.request('describe')
        result = dict(workload=name, config=dict(config, seconds=seconds, hz=hz, history=history),
                      startup_s=startup, first_frame_s=first_frame, latency_ms=percentiles(latencies),
                      command_ack_ms=percentiles(command_ack), observed_samples=len(latencies),
                      skipped_publications=skipped, software_fallback_delay_ms=(feedback.data['fallback_ns']-deadline)/1e6,
                      expired_commands=expired_commands,
                      consumer_queues=[dict(dropped=getattr(view.endpoints['output'], 'queue_drops', None),
                          evicted=getattr(view.endpoints['output'], 'evicted_descriptors', None)) for view in views],
                      resources=metrics, active_files=len(list(provider.directory.rglob('*.npy'))),
                      transport_stats=manifest.get('transport_stats', {}))
    finally:
        for client in reversed(clients):
            await client.aclose()
        close_start = time.perf_counter()
        await runtime.aclose()
        async with asyncio.timeout(15):
            while provider.directory.exists():
                await asyncio.sleep(.02)
    result['cleanup_s'] = time.perf_counter() - close_start
    return result


async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=5.)
    parser.add_argument('--hz', type=float, default=20.)
    parser.add_argument('--history', type=int, default=4)
    parser.add_argument('--workloads', default=','.join(WORKLOADS))
    args = parser.parse_args()
    results = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for name in args.workloads.split(','):
        results.append(await measure(name, WORKLOADS[name], args.seconds, args.hz, args.history))
        args.output.write_text(json.dumps(dict(python=sys.version, system=platform.system(),
            platform=platform.platform(), numpy=np.__version__, cyclonedds=importlib.metadata.version('cyclonedds'),
            rsim_file=rsim.__file__, workloads=results), indent=2), encoding='utf-8')


if __name__ == '__main__':
    asyncio.run(main())
