"""Local DDS/mmap benchmark: payload, process I/O and host disk I/O are separate.

No hardware, wheel build, cache purge or forced disk flush. See docs/benchmark.md.
"""
import argparse
import asyncio
from contextlib import AsyncExitStack
from datetime import datetime, timezone
from functools import partial
import gc
import importlib.metadata
import itertools
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
import uuid

INVOCATION_DIRECTORY = Path.cwd()
ROOT = Path(__file__).resolve().parents[1]
# Workers need the selected source before importing their -m entrypoint.
SOURCE = ROOT
if __name__ == '__main__':
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument('--source', type=Path)
    selected, _ = bootstrap.parse_known_args()
    if selected.source is not None:
        SOURCE = selected.source.expanduser().resolve()
        if not (SOURCE / 'rsim').is_dir():
            raise SystemExit('--source must contain rsim/')
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SOURCE))

import numpy as np
import psutil
import rsim
from rsim import Component, Runtime, SharedProvider, SharedComponent, CommandSink, VelocityCommand, TransportConfig
from rsim.platforms import get_platform
from rsim.platforms.files import remove_tree
from rsim.platforms.paths import new_directory, roots
from rsim.runtime.options import RuntimeOptions, RuntimePaths
from rsim.runtime.sharing import component_ports
from tools.benchmark_metrics import (ResourceSampler, counter_delta, host_snapshot,
                                     host_interval, transport_totals)

WORKLOADS = {
    'small': dict(elements=1, consumers=1, late_s=0., delay_s=0.),
    'rgbd': dict(elements=640*480*4, consumers=1, late_s=0., delay_s=0.),
    'points': dict(elements=300_000, consumers=1, late_s=0., delay_s=0.),
    'fanout': dict(elements=300_000, consumers=3, late_s=0., delay_s=0.),
    'late': dict(elements=300_000, consumers=2, late_s=.3, delay_s=0.),
    'slow': dict(elements=300_000, consumers=1, late_s=0., delay_s=.1),
    'mapping-control': dict(elements=300_000, consumers=2, late_s=0., delay_s=.03),
}
VARIANTS = {
    'file': dict(allocation='copy', file_policy='normal'),
    'temporary': dict(allocation='copy', file_policy='temporary'),
    'direct': dict(allocation='direct', file_policy='normal'),
    'direct-temporary': dict(allocation='direct', file_policy='temporary'),
}


class Source(Component):
    def __init__(self, elements, hz, history, variant, directory):
        super().__init__()
        self.elements, self.hz, self.variant, self.directory = elements, hz, variant, directory
        self.output = self.signal('output', history=history, clock='benchmark')
        self.feedback = self.signal('feedback', history=history, clock='benchmark')
        self.velocity = CommandSink(self, 'velocity', self.apply, safe=self.safe, fallback=VelocityCommand())
        self.current, self.fallback_ns, self.count = VelocityCommand(), None, 0
        self.store = self.original_open = None

    async def apply(self, envelope):
        self.current = envelope.value

    async def safe(self, value):
        self.current, self.fallback_ns = value, time.monotonic_ns()

    async def open(self):
        if self.variant['file_policy'] == 'temporary':
            native = get_platform().files
            self.original_open = native.open_shared
            # Only this dedicated benchmark worker opts into the cache hint.
            native.open_shared = partial(native.open_shared, temporary=True)
        if self.variant['allocation'] == 'direct':
            from rsim.transport.shared import SharedStore
            self.store = SharedStore(self.directory)
        self.task('publish', self.publish, hz=self.hz)

    async def publish(self):
        self.count += 1
        now = time.monotonic_ns()
        if self.store is None:
            array = np.empty(self.elements, dtype=np.float32)
        else:
            array = self.store.allocate((self.elements,), np.float32)
        array.fill(self.count % 123)
        array.flags.writeable = False
        await self.output.publish(array, stamp_ns=now, clock='benchmark',
            metadata={'published_ns': now, 'publication': self.count, 'value': self.count % 123})
        await self.feedback.publish(dict(linear_x=self.current.linear_x, angular_z=self.current.angular_z,
            fallback_ns=self.fallback_ns), stamp_ns=now, clock='benchmark')

    async def close(self):
        try:
            if self.store is not None:
                self.store.close()
        finally:
            if self.original_open is not None:
                get_platform().files.open_shared = self.original_open


def percentiles(values):
    return dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(values, [50, 95, 99])))) if values else {}


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return value


def nonnegative(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError('must be finite and nonnegative')
    return value


def positive_int(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError('must be positive')
    return number


def absolute_path(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else INVOCATION_DIRECTORY / path


def distribution_version(name):
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None  # Direct source imports need no installed distribution.


async def check_commands(view):
    from rsim.core import CommandEnvelope, CommandRejected
    await asyncio.sleep(.22)
    deadline = time.monotonic_ns() + 80_000_000
    await view.velocity.set(CommandEnvelope(VelocityCommand(.1), 'benchmark', 'ttl', 1, deadline))
    # Bounded even if feedback continues but fallback never happens.
    async with asyncio.timeout(5):
        feedback = await view.feedback.get(timeout=2)
        while feedback.data['fallback_ns'] is None or feedback.data['fallback_ns'] < deadline:
            feedback = await view.feedback.get(after=feedback.sequence, timeout=2)
    try:
        await view.velocity.set(CommandEnvelope(VelocityCommand(), 'benchmark', 'ttl', 2, time.monotonic_ns()-1))
    except CommandRejected as error:
        assert error.reason == 'expired'
    else:
        raise AssertionError('provider accepted an expired command')
    return (feedback.data['fallback_ns'] - deadline) / 1e6


async def measure(name, config, variant_name, repeat, args):
    variant = VARIANTS[variant_name]
    directory = new_directory(prefix='rsim-benchmark-', root=args.storage_root or roots()['storage_root'])
    options = RuntimeOptions(paths=RuntimePaths(storage_root=directory,
        registry_root=directory/'registry', log_root=directory/'logs'))
    transport = TransportConfig('cyclonedds', args.domain)
    source = Source(config['elements'], args.hz, args.history, variant, directory/'direct')
    provider = SharedProvider(source, key='benchmark:' + uuid.uuid4().hex, options=options, transport=transport)
    result = dict(workload=name, variant=variant_name, repeat=repeat,
        config=dict(config, seconds=args.seconds, hz=args.hz, history=args.history,
                    warmup_s=args.warmup_seconds, idle_s=args.idle_seconds, settle_s=args.settle_seconds,
                    sample_interval_s=args.sample_interval, read_mode=args.read_mode, **variant),
        storage=dict(root=str(directory.parent), filesystem=get_platform().files.filesystem_info(directory)),
        io={}, cleanup_complete=False)
    baseline_start = host_snapshot()
    provider_directory = None
    try:
        await asyncio.sleep(args.idle_seconds)
        result['io']['idle'] = host_interval(baseline_start, host_snapshot())
        whole_start = host_snapshot()
        started = time.perf_counter()
        async with AsyncExitStack() as stack:
            await stack.enter_async_context(Runtime(provider, _root_ports=False, options=options))
            provider_directory = provider.directory
            result['startup_s'] = time.perf_counter() - started
            views = []
            for index in range(config['consumers']):
                if index and config['late_s']:
                    await asyncio.sleep(config['late_s'])
                view = SharedComponent(key=provider.component_key, ports=component_ports(source),
                                       options=options, transport=transport)
                await stack.enter_async_context(Runtime(view.output, view.feedback, view.velocity, options=options))
                views.append(view)
            initial = await asyncio.gather(*(view.output.get(timeout=10) for view in views))
            result['first_frame_s'] = time.perf_counter() - started
            cursors = [frame.sequence for frame in initial]
            publications = [frame.metadata['publication'] for frame in initial]
            del initial  # Do not accidentally pin first-frame mappings throughout the run.
            latencies, reads, command_ack = [], [], []
            skipped, read_bytes, checksum = 0, 0, 0.

            async def consume(record):
                nonlocal skipped, read_bytes, checksum
                for index, view in enumerate(views):
                    frame = await view.output.get(after=cursors[index], timeout=10)
                    publication = frame.metadata['publication']
                    if record:
                        skipped += max(0, publication - publications[index] - 1)
                    cursors[index], publications[index] = frame.sequence, publication
                    ready_ns = time.monotonic_ns()
                    begin = time.perf_counter()
                    if args.read_mode == 'sum':
                        total = float(np.sum(frame.data, dtype=np.float64))
                        expected = frame.metadata['value'] * config['elements']
                        if total != expected:
                            raise AssertionError(f'payload mismatch: {total} != {expected}')
                        if record:
                            read_bytes += frame.data.nbytes
                            checksum += total
                    if record:
                        latencies.append((ready_ns - frame.metadata['published_ns']) / 1e6)
                        reads.append((time.perf_counter() - begin) * 1000)
                begin = time.perf_counter()
                await views[0].velocity.set(VelocityCommand(), ttl=.2)
                if record:
                    command_ack.append((time.perf_counter() - begin) * 1000)
                await asyncio.sleep(config['delay_s'])

            warmup_end = time.perf_counter() + args.warmup_seconds
            while time.perf_counter() < warmup_end:
                await consume(False)
            worker = psutil.Process(provider.manifest['pid'])
            processes = dict(worker=worker, consumers_and_launcher=psutil.Process())
            supervisor = worker.parent()
            if supervisor is not None and supervisor.pid != os.getpid():
                processes['supervisor'] = supervisor
            before_stats = transport_totals((await provider.connection.request('describe')).get('transport_stats', {}))
            publication_start = max(publications)
            async with ResourceSampler(processes, directory, args.sample_interval) as sampler:
                steady_start = host_snapshot()
                finish = time.perf_counter() + args.seconds
                while time.perf_counter() < finish:
                    await consume(True)
                steady_end = host_snapshot()
            manifest = await provider.connection.request('describe')
            result['io']['steady'] = host_interval(steady_start, steady_end)
            result['resources'] = sampler.report()
            result['resources'].update({key: result['resources']['processes']['worker']['after'].get(key)
                                        for key in ('rss_bytes', 'handles')})
            result['active_files'] = result['resources']['files_after']['paths']
            result['transport_stats'] = manifest.get('transport_stats', {})
            result['transport_delta'] = counter_delta(before_stats, transport_totals(result['transport_stats']))
            publications_seen = max(publications) - publication_start
            duration = result['io']['steady']['seconds']
            result.update(latency_ms=percentiles(latencies), payload_read_ms=percentiles(reads),
                command_ack_ms=percentiles(command_ack), observed_samples=len(latencies),
                skipped_publications=skipped, observed_publications=publications_seen,
                payload_bytes_per_publication=config['elements'] * np.dtype(np.float32).itemsize,
                observed_payload_bytes=publications_seen * config['elements'] * np.dtype(np.float32).itemsize,
                observed_publication_hz=publications_seen / duration,
                consumer_read_bytes=read_bytes, read_checksum=checksum,
                consumer_queues=[dict(dropped=getattr(view.endpoints['output'], 'queue_drops', None),
                    evicted=getattr(view.endpoints['output'], 'evicted_descriptors', None)) for view in views])
            result['software_fallback_delay_ms'] = await check_commands(views[0])
            result['expired_commands'] = 1
            close_start = time.perf_counter()
        # Independent supervisor cleanup can outlive the last client close.
        async with asyncio.timeout(15):
            while provider_directory.exists():
                await asyncio.sleep(.02)
        result['cleanup_s'] = time.perf_counter() - close_start
        views.clear()
        gc.collect()
        remove_tree(directory)
        result['cleanup_complete'] = True
        post_start = host_snapshot()
        await asyncio.sleep(args.settle_seconds)
        end = host_snapshot()
        result['io']['post_close'] = host_interval(post_start, end)
        result['io']['whole_run'] = host_interval(whole_start, end)
        return result
    finally:
        # AsyncExitStack handles partial client/provider startup as well as cancellation.
        if provider.directory is not None:
            async with asyncio.timeout(15):
                while provider.directory.exists():
                    await asyncio.sleep(.02)
        if directory.exists():
            remove_tree(directory)


def print_row(row, devices):
    mib = 1024**2
    steady = row['io']['steady']
    copied = row['transport_delta'].get('materialized_bytes')
    copies = f'{copied / mib:.1f} MiB' if copied is not None else 'unavailable'
    print(f"{row['workload']:16} {row['variant']:17} #{row['repeat']}  "
          f"payload={row['observed_payload_bytes']/mib:.1f} MiB  "
          f"read={row['consumer_read_bytes']/mib:.1f} MiB  skipped={row['skipped_publications']}  "
          f"copies={copies}  "
          f"files(peak)={row['resources']['sampled_peak_unique_file_bytes']/mib:.1f} MiB", flush=True)
    for name, counters in steady['disks'].items():
        if devices and name not in devices:
            continue
        written = counters.get('delta', {}).get('write_bytes')
        idle = row['io']['idle']['disks'].get(name, {}).get('write_bytes_per_s')
        tail = row['io']['post_close']['disks'].get(name, {}).get('delta', {}).get('write_bytes')
        if written is not None:
            print(f"  host {name}: {written/mib:.2f} MiB written, "
                  f"{counters['write_bytes_per_s']/mib:.2f} MiB/s; "
                  f"idle={idle/mib if idle is not None else float('nan'):.2f} MiB/s; "
                  f"post-close={tail/mib if tail is not None else float('nan'):.2f} MiB", flush=True)
        else:
            print(f"  host {name}: unavailable ({counters})", flush=True)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--source', type=absolute_path)
    result.add_argument('--output', type=absolute_path, required=True)
    result.add_argument('--storage-root', type=absolute_path, help='e.g. /dev/shm versus an existing local disk directory')
    result.add_argument('--seconds', type=positive, default=10.)
    result.add_argument('--warmup-seconds', type=nonnegative, default=2.)
    result.add_argument('--idle-seconds', type=positive, default=3.)
    result.add_argument('--settle-seconds', type=positive, default=5.)
    result.add_argument('--sample-interval', type=positive, default=.5)
    result.add_argument('--hz', type=positive, default=20.)
    result.add_argument('--elements', type=positive_int, help='override float32 element count for selected workloads')
    result.add_argument('--history', type=positive_int, default=4)
    result.add_argument('--workloads', default=','.join(WORKLOADS))
    result.add_argument('--variant', choices=VARIANTS, action='append', help='repeat to compare alternatives (default: file)')
    result.add_argument('--repeat', type=positive_int, default=1)
    result.add_argument('--read-mode', choices=('sum', 'metadata'), default='sum')
    result.add_argument('--disk', action='append', help='device to emphasize in terminal output; JSON always records every device')
    result.add_argument('--domain', type=int, default=79)
    return result


async def main():
    cli = parser()
    args = cli.parse_args()
    names = args.workloads.split(',')
    if any(name not in WORKLOADS for name in names) or len(names) != len(set(names)):
        cli.error('--workloads must contain distinct supported names: ' + ','.join(WORKLOADS))
    variants = args.variant or ['file']
    if len(variants) != len(set(variants)):
        cli.error('duplicate --variant')
    if platform.system() != 'Windows' and any('temporary' in name for name in variants):
        cli.error('temporary file hints are Windows-only; compare storage roots on Linux')
    if not 0 <= args.domain <= 232:
        cli.error('--domain must be between 0 and 232')
    if args.storage_root and not args.storage_root.is_dir():
        cli.error('--storage-root must be an existing local directory')
    disks = host_snapshot()['disks']
    if args.disk and not set(args.disk) <= disks.keys():
        cli.error('unknown --disk; available: ' + ', '.join(disks))
    os.chdir(SOURCE)
    os.environ['PYTHONPATH'] = os.pathsep.join((str(SOURCE), str(ROOT), os.environ.get('PYTHONPATH', '')))
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    report = dict(schema_version=2, complete=False, expected_runs=len(names) * len(variants) * args.repeat,
        started_at=datetime.now(timezone.utc).isoformat(), host=platform.node(),
        python=sys.version, system=platform.system(), platform=platform.platform(),
        numpy=np.__version__, psutil=psutil.__version__, cyclonedds=distribution_version('cyclonedds'),
        rsim_file=str(Path(rsim.__file__).resolve()), rsim_version=distribution_version('rsim'),
        measurement=dict(read_mode=args.read_mode, focus_disks=args.disk, client_processes=1,
            notes=['Logical payload and materialization bytes are not disk writes.',
                   'Process I/O can include non-disk traffic and omit mmap writeback; use deltas only.',
                   'Host device I/O includes other applications and filesystem metadata. It is not SSD NAND wear.',
                   'Do not sum overlapping Linux disks/partitions/device-mapper counters.',
                   'Idle is a context sample, not a reliable amount to subtract from active writes.',
                   'Post-close is a finite observation window, not proof that all dirty pages have been flushed.',
                   'RSS can double count shared pages and excludes some system cache. File bytes count live unique paths only.',
                   'Temporary-file hints and tmpfs can still incur paging under pressure.',
                   'Transport counters bracket describe requests, not exactly the consumer observation window.',
                   'Transport mapping_creations excludes the source-owned store used by direct variants.',
                   'Direct uses a source-owned SharedStore, with a new immutable mapping per publication; it is not a reused buffer.']),
        workloads=[])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    print('Device write counters are host-wide; they are not writes attributable solely to RSim.', flush=True)
    cases = list(itertools.product(names, variants))
    try:
        for repeat in range(1, args.repeat + 1):
            for name, variant in cases if repeat % 2 else reversed(cases):
                config = dict(WORKLOADS[name])
                if args.elements is not None:
                    config['elements'] = args.elements
                row = await measure(name, config, variant, repeat, args)
                report['workloads'].append(row)
                report['complete'] = len(report['workloads']) == report['expected_runs']
                print_row(row, args.disk)
                args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    except BaseException as error:
        report['complete'] = False
        report['error'] = f'{type(error).__name__}: {error}'
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
        raise


if __name__ == '__main__':
    asyncio.run(main())
