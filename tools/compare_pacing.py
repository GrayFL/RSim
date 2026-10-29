"""Paired, CPU-pinned pacing comparisons and explicitly scoped extension rates.

Use ``paired --baseline BASE --candidate CANDIDATE --output REPORT`` for the
nine-case matrix. Compare a baseline against itself first to expose host noise.
The benchmark file must match in both source trees; each subprocess imports rsim
from its own working directory and retains its raw report and command metadata.

To investigate the loaded 1000 Hz case, add ``--scenario local-load-1000
--seconds 5 --warmup-seconds 1 --repeat 5``. This calls the same measure function
with a longer observation window and a discarded warmup. It records source-file
hashes and GC collection counts; ``--disable-gc`` is a diagnostic variant only.
Keep earlier results when changing the measurement procedure. Median limits do
not establish confidence intervals, stable individual runs, or hardware latency.

``extensions`` measures synthetic TickSource fanout and ReplaySource publication;
it does not measure physics, rendering, recorder I/O or distributed lockstep.
``memory`` samples Linux RSS in an untraced pass, then traced peak and net retained
Python allocations in a separate pass. Retained counts are not allocation totals.
"""
from __future__ import annotations

import argparse
import asyncio
from collections import defaultdict
import gc
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time


def stats(values):
    values = list(values)
    middle = statistics.median(values)
    return dict(values=values, median=middle, minimum=min(values), maximum=max(values),
                mad=statistics.median(abs(value-middle) for value in values),
                stdev=statistics.pstdev(values))


def key(row):
    return tuple(row[name] for name in ('hz', 'components', 'elements', 'processes', 'load'))


def compare(paths_a, paths_b):
    if not paths_a or len(paths_a) != len(paths_b):
        raise ValueError('comparison requires equal, nonempty paired inputs')
    reports = [[json.loads(Path(path).read_text()) for path in paths] for paths in (paths_a, paths_b)]
    groups = []
    for side in reports:
        grouped = defaultdict(list)
        for report in side:
            for row in report['rows']:
                grouped[key(row)].append(row)
        groups.append(grouped)
    assert groups[0].keys() == groups[1].keys()
    metrics = {
        'observed_hz': (lambda row: row['observed_hz'], 5., -1),
        'parent_cpu_fraction': (lambda row: row['parent_cpu_s']/row['elapsed_s'], 5., 1),
        'p99_interval_error_s': (lambda row: row['interval_error_s']['p99'], 10., 1),
        'p95_interval_error_s': (lambda row: row['interval_error_s']['p95'], None, 1),
        'p50_interval_error_s': (lambda row: row['interval_error_s']['p50'], None, 1),
    }
    rows = []
    for case in groups[0]:
        result = dict(zip(('hz', 'components', 'elements', 'processes', 'load'), case))
        result['metrics'] = {}
        for name, (extract, threshold, direction) in metrics.items():
            a, b = ([extract(row) for row in group[case]] for group in groups)
            delta = statistics.median(b)-statistics.median(a)
            percentage = 100*delta/statistics.median(a)
            paired = [100*(right/left-1) for left, right in zip(a, b)]
            result['metrics'][name] = dict(
                a=stats(a), b=stats(b), absolute_delta=delta, median_change_pct=percentage,
                paired_change_pct=stats(paired), suggested_limit_pct=threshold,
                median_limit_exceeded=(percentage*direction > threshold) if threshold else None,
                all_pairs_exceed_limit=(min(change*direction for change in paired) > threshold) if threshold else None,
            )
        rows.append(result)
    micro = [[row['publish_get_per_s'] for report in side for row in report['hot_path']] for side in reports]
    return {
        'a_paths': list(map(str, paths_a)), 'b_paths': list(map(str, paths_b)), 'rows': rows,
        'micro_publish_get': {
            'a': stats(micro[0]), 'b': stats(micro[1]),
            'absolute_delta': statistics.median(micro[1])-statistics.median(micro[0]),
            'median_change_pct': 100*(statistics.median(micro[1])/statistics.median(micro[0])-1),
            'paired_change_pct': stats(100*(b/a-1) for a, b in zip(*micro)),
        },
        'limits': 'Suggested median throughput/parent-CPU 5%, p99 interval-error 10%; self-comparison noise must also be reviewed.',
        'limitations': [
            'Interval error is consecutive observed source timestamp spacing, not absolute callback wakeup latency.',
            'Cross-process Signals use latest-only observation; lower observed Hz can include transport skips.',
            'Per-scenario CPU is the parent process only; no total worker CPU or hardware real-time claim.',
            'Small repeated samples do not establish a statistical confidence interval; all raw values are retained.',
        ],
    }


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2) + '\n')


def run_pairs(args):
    root = args.output.resolve()
    root.parent.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    environment.pop('PYTHONPATH', None)
    environment.pop('PYTHONHOME', None)
    cpus = [int(value) for value in args.cpus.split(',')]
    if not set(cpus).issubset(os.sched_getaffinity(0)):
        raise ValueError('requested CPUs are outside the allowed affinity')
    repositories = {'a': args.baseline.resolve(), 'b': args.candidate.resolve()}
    metadata = dict(python=sys.executable, python_version=sys.version, platform=platform.platform(),
                    affinity=cpus, seconds_per_scenario=args.seconds, pairs=args.repeat,
                    environment={name: environment[name] for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')},
                    scenario=args.scenario, warmup_seconds=args.warmup_seconds,
                    gc_disabled=args.disable_gc,
                    runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    repositories={}, runs=[])
    for name, repository in repositories.items():
        source = subprocess.check_output([sys.executable, '-c', 'import rsim; print(rsim.__file__)'],
                                         cwd=repository, env=environment, text=True).strip()
        git_head = subprocess.run(['git', '-C', str(repository), 'rev-parse', 'HEAD'],
                                  text=True, capture_output=True)
        git_status = subprocess.run(['git', '-C', str(repository), 'status', '--short'],
                                    text=True, capture_output=True)
        metadata['repositories'][name] = dict(
            path=str(repository), imported_source=source,
            head=git_head.stdout.strip() if git_head.returncode == 0 else None,
            status=git_status.stdout if git_status.returncode == 0 else 'unversioned diagnostic source snapshot',
            benchmark_sha256=hashlib.sha256((repository/'tools/benchmark_pacing.py').read_bytes()).hexdigest(),
        )
    assert metadata['repositories']['a']['benchmark_sha256'] == metadata['repositories']['b']['benchmark_sha256']
    paths = {'a': [], 'b': []}
    for repeat in range(1, args.repeat+1):
        order = ('a', 'b') if repeat % 2 else ('b', 'a')
        for side in order:
            output = root.with_name(root.stem + f'-{side}-{repeat}.json')
            log = output.with_suffix('.log')
            command = ['taskset', '-c', args.cpus, sys.executable, '-m', 'tools.benchmark_pacing',
                       '--repeat', '1', '--seconds', str(args.seconds), '--label', f'{root.stem}-{side}-{repeat}',
                       '--output', str(output)]
            if args.scenario != 'all':
                command = ['taskset', '-c', args.cpus, sys.executable, '-c',
                           'import runpy,sys;runpy.run_path(sys.argv.pop(1),run_name="__main__")',
                           str(Path(__file__).resolve()), 'sample', '--repeat', '1',
                           '--seconds', str(args.seconds), '--warmup-seconds', str(args.warmup_seconds),
                           '--scenario', args.scenario, '--label', f'{root.stem}-{side}-{repeat}',
                           '--output', str(output)]
                if args.disable_gc:
                    command.append('--disable-gc')
            start = time.time()
            with log.open('w') as stream:
                subprocess.run(command, cwd=repositories[side], env=environment, stdout=stream,
                               stderr=subprocess.STDOUT, check=True, timeout=120)
            paths[side].append(output)
            metadata['runs'].append(dict(side=side, repeat=repeat, start_unix_s=start,
                                         wall_s=time.time()-start, command=command, log=str(log)))
            write(root.with_name(root.stem+'-metadata.json'), metadata)
            print(json.dumps({'completed': f'{side}-{repeat}', 'output': str(output),
                              'wall_s': time.time()-start}), flush=True)
    # Sort by pair number even when execution order is reversed.
    result = compare(sorted(paths['a']), sorted(paths['b']))
    result['metadata'] = metadata
    write(root, result)
    print(json.dumps({'comparison': str(root), 'micro_change_pct': result['micro_publish_get']['median_change_pct']}), flush=True)


async def sample(args):
    """Run the unchanged benchmark function with a focused scenario selection."""
    from tools.benchmark_pacing import measure, throughput
    import rsim
    if args.disable_gc:
        gc.disable()
    source_root = Path(rsim.__file__).resolve().parent
    source_hashes = {str(path.relative_to(source_root)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in sorted(source_root.rglob('*.py'))}
    rows, hot = [], []
    before = gc.get_stats()
    for repeat in range(args.repeat):
        hot.append(await throughput())
        if args.warmup_seconds:
            await measure(1000, 4, 1024, args.warmup_seconds, False, True)
        row = await measure(1000, 4, 1024, args.seconds, False, True)
        rows.append(dict(row, repeat=repeat+1))
    after = gc.get_stats()
    write(args.output, dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
                            label=args.label, rows=rows, hot_path=hot,
                            imported_source=str(source_root), source_sha256=source_hashes,
                            gc_enabled=gc.isenabled(), gc_threshold=gc.get_threshold(),
                            gc_delta=[{name: right[name]-left[name] for name in left}
                                      for left, right in zip(before, after)],
                            warmup_seconds=args.warmup_seconds))


async def tick_benchmark(subscribers, steps=20000):
    from rsim.core.pacing import TickSource
    source = TickSource(clock='benchmark:simulation')
    subscriptions = [source.subscribe(policy='every', capacity=8) for _ in range(subscribers)]
    counts = [0]*subscribers
    async def consume(index, subscription):
        for expected in range(steps):
            tick = await subscription.tick()
            assert tick.step_index == expected
            counts[index] += 1
    consumers = [asyncio.create_task(consume(index, sub)) for index, sub in enumerate(subscriptions)]
    start, cpu = time.perf_counter(), time.process_time()
    for index in range(steps):
        await source.publish(stamp_ns=index*2_000_000, episode=0, step_index=index)
    await asyncio.gather(*consumers)
    wall, cpu = time.perf_counter()-start, time.process_time()-cpu
    await source.aclose()
    return dict(case='TickSource every', subscribers=subscribers, steps=steps, wall_s=wall,
                cpu_s=cpu, steps_per_s=steps/wall, virtual_seconds=(steps-1)*.002,
                rtf=(steps-1)*.002/wall, received=counts,
                note='Bounded queue capacity 8; synthetic source has no physics, rendering or algorithm work.')


async def replay_benchmark(rate, steps=2000):
    from rsim.components.replay import ReplaySample, ReplaySource
    from rsim.runtime import Runtime
    source = ReplaySource([ReplaySample(index*1_000_000, index) for index in range(steps)],
                          clock='benchmark:recorded', rate=rate, paused=True, history=steps)
    observed = 0
    async def consume():
        nonlocal observed
        previous = 0
        while True:
            frame = await source.output.get(after=previous, timeout=10)
            previous = frame.sequence
            observed += 1
            if frame.data == steps-1:
                return
    async with Runtime(source.output):
        task = asyncio.create_task(consume())
        start, cpu = time.perf_counter(), time.process_time()
        await source.resume()
        await asyncio.wait_for(task, 10)
        wall, cpu = time.perf_counter()-start, time.process_time()-cpu
        published = len(source.output.frames)
        assert published == steps
        assert source.output.frames[-1].stamp_ns == (steps-1)*1_000_000
    return dict(case='ReplaySource', requested_rate=rate, steps=steps, published=published,
                observed_latest=observed, wall_s=wall, cpu_s=cpu, steps_per_s=steps/wall,
                virtual_seconds=(steps-1)*.001, rtf=(steps-1)*.001/wall,
                note='All records published; latest-only consumer may skip intermediate frames. No recorder I/O/physics included.')


async def extensions(args):
    cpus = {int(value) for value in args.cpus.split(',')}
    os.sched_setaffinity(0, cpus)
    rows = []
    for repeat in range(args.repeat):
        for subscribers in (1, 4):
            rows.append(dict(await tick_benchmark(subscribers), repeat=repeat+1))
        for rate in (1., 10., 100.):
            rows.append(dict(await replay_benchmark(rate), repeat=repeat+1))
    write(args.output, dict(python=sys.executable, affinity=sorted(cpus), rows=rows,
                           scope='Single-process source pacing/replay; no cross-process ACK barrier or robot simulation throughput claim.'))
    print(json.dumps({'extensions': str(args.output), 'rows': len(rows)}), flush=True)


async def memory_benchmark(operations):
    import resource
    import tracemalloc
    import numpy as np
    from rsim.core import Component
    from rsim.runtime import Runtime

    def rss():
        pages = int(Path('/proc/self/statm').read_text().split()[1])
        return pages * os.sysconf('SC_PAGE_SIZE')

    source = Component()
    output = source.signal('output', history=2, clock='bench')
    payload = np.zeros(1024)
    sequence = 0

    async def publish_get(count):
        nonlocal sequence
        for _ in range(count):
            frame = await output.publish(payload, stamp_ns=sequence, clock='bench')
            assert await output.get() is frame
            sequence += 1

    async with Runtime(output):
        await publish_get(1000)
        gc.collect()
        rss_before = rss()
        await publish_get(operations)
        rss_after = rss()
        # On Linux ru_maxrss is KiB. Capture it before tracing allocates its own
        # bookkeeping so the RSS result describes the uninstrumented pass.
        peak_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
        tracemalloc.start(1)
        await publish_get(1000)
        gc.collect()
        before = tracemalloc.take_snapshot()
        traced_before, _ = tracemalloc.get_traced_memory()
        tracemalloc.reset_peak()
        await publish_get(operations)
        traced_after, traced_peak = tracemalloc.get_traced_memory()
        after = tracemalloc.take_snapshot()
        deltas = after.compare_to(before, 'filename')
        rsim_deltas = [delta for delta in deltas if '/rsim/' in str(delta.traceback)]
        tracemalloc.stop()
    return dict(operations_per_pass=operations, payload_elements=1024, history=2,
                rss_before_bytes=rss_before, rss_after_bytes=rss_after,
                rss_delta_bytes=rss_after-rss_before,
                process_peak_rss_before_tracing_bytes=peak_rss,
                traced_current_delta_bytes=traced_after-traced_before,
                traced_peak_above_start_bytes=traced_peak-traced_before,
                net_retained_allocation_count=sum(delta.count_diff for delta in deltas),
                net_retained_bytes=sum(delta.size_diff for delta in deltas),
                rsim_net_retained_allocation_count=sum(delta.count_diff for delta in rsim_deltas),
                rsim_net_retained_bytes=sum(delta.size_diff for delta in rsim_deltas))


async def memory(args):
    import rsim
    os.sched_setaffinity(0, {int(value) for value in args.cpus.split(',')})
    rows = [dict(await memory_benchmark(args.operations), repeat=repeat+1)
            for repeat in range(args.repeat)]
    write(args.output, dict(python=sys.executable, platform=platform.platform(),
                           imported_source=rsim.__file__, affinity=sorted(os.sched_getaffinity(0)),
                           source_sha256={name: hashlib.sha256((Path(rsim.__file__).parent/name).read_bytes()).hexdigest()
                                          for name in ('core/component.py', 'core/signal.py', 'core/metronome.py')},
                           rows=rows, limitations=[
                               'Linux current RSS and process high-water mark; not worker totals or native/GPU allocation accounting.',
                               'Untraced RSS pass precedes a separate tracemalloc pass; no throughput or latency claim is made.',
                               'Snapshot differences count net live allocations only. Temporaries that were freed are not counted; total allocation calls remain unmeasured.',
                               'Python traced peak includes instrumentation bookkeeping; all source and tool allocations are included unless explicitly labelled rsim.',
                           ]))
    print(json.dumps({'memory': str(args.output), 'rows': len(rows)}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    paired = commands.add_parser('paired')
    paired.add_argument('--baseline', type=Path, required=True)
    paired.add_argument('--candidate', type=Path, required=True)
    paired.add_argument('--seconds', type=float, default=2.)
    focused = commands.add_parser('sample')
    focused.add_argument('--seconds', type=float, default=5.)
    focused.add_argument('--label', default='focused')
    for command in (paired, focused):
        command.add_argument('--scenario', choices=('all', 'local-load-1000'), default='all')
        command.add_argument('--warmup-seconds', type=float, default=0.)
        command.add_argument('--disable-gc', action='store_true')
    memory_command = commands.add_parser('memory')
    memory_command.add_argument('--operations', type=int, default=20000)
    for command in (paired, focused, commands.add_parser('extensions'), memory_command):
        command.add_argument('--output', type=Path, required=True)
        command.add_argument('--repeat', type=int, default=3)
        command.add_argument('--cpus', default='4,5,6,7')
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('repeat must be positive')
    if args.command in ('paired', 'sample'):
        if args.seconds <= 0:
            parser.error('seconds must be positive')
        if args.warmup_seconds < 0:
            parser.error('warmup seconds must be nonnegative')
        if args.command == 'paired':
            if args.scenario == 'all' and (args.warmup_seconds or args.disable_gc):
                parser.error('warmup and GC diagnostics require a focused scenario')
            run_pairs(args)
        elif args.scenario == 'local-load-1000':
            asyncio.run(sample(args))
        else:
            parser.error('sample requires a focused scenario')
    elif args.command == 'memory':
        if args.operations < 1:
            parser.error('operations must be positive')
        asyncio.run(memory(args))
    else:
        asyncio.run(extensions(args))


if __name__ == '__main__':
    main()
