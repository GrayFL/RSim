"""Opt-in benchmark instrumentation. Nothing in rsim imports this module.

Callbacks keep their original scheduling. The wrapper reads the actual
Metronome in Component.task's running coroutine, or the supplied RatePacer.
Source-driven callbacks have no host deadline and therefore no wake jitter.
"""
from __future__ import annotations

import asyncio
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import statistics
import time

import psutil

from rsim import Component, Runtime


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, default=str) + '\n')


def distribution(values):
    values = sorted(float(value) for value in values if value is not None)
    if not values:
        return dict(count=0, p50=None, p95=None, p99=None, max=None, mean=None, variance=None)
    def percentile(q):
        offset = (len(values)-1)*q
        lo, hi = math.floor(offset), math.ceil(offset)
        return values[lo] + (values[hi]-values[lo])*(offset-lo)
    return dict(count=len(values), p50=percentile(.5), p95=percentile(.95), p99=percentile(.99),
                max=max(values), mean=statistics.mean(values), variance=statistics.pvariance(values))


class ProcessTreeSampler:
    """External sampler; retains descendants even after reparenting.

    CPU is a sum of observed per-PID deltas, not the last parent's CPU reading.
    The first/last samples omit unobserved process lifetime and are labelled so.
    """
    def __init__(self, pid):
        self.root = psutil.Process(pid)
        self.known = {pid: self.root}
        self.samples = []

    def sample(self):
        now = time.monotonic_ns()
        try:
            self.known.update((p.pid, p) for p in self.root.children(recursive=True))
        except psutil.Error:
            pass
        rows = []
        for pid, process in tuple(self.known.items()):
            try:
                with process.oneshot():
                    cpu = process.cpu_times()
                    rows.append(dict(pid=pid, created_unix_s=process.create_time(),
                        cpu_s=cpu.user+cpu.system, rss_bytes=process.memory_info().rss,
                        name=process.name(), status=process.status()))
            except psutil.Error:
                pass
        self.samples.append(dict(host_ns=now, processes=rows))

    def report(self, start=None, end=None):
        return resource_window(self.samples, start, end)

    def living_pids(self):
        living = []
        for pid, process in self.known.items():
            try:
                if process.is_running() and process.status() != psutil.STATUS_ZOMBIE:
                    living.append(pid)
            except psutil.Error:
                pass  # Exit can race is_running/status; the process is gone.
        return living


def resource_window(samples, start=None, end=None):
    samples = [row for row in samples if (start is None or row['host_ns'] >= start)
               and (end is None or row['host_ns'] <= end)]
    grouped = defaultdict(list)
    for sample in samples:
        for row in sample['processes']:
            grouped[(row['pid'], row['created_unix_s'])].append(dict(row, host_ns=sample['host_ns']))
    processes = []
    for (pid, created), rows in grouped.items():
        duration = (rows[-1]['host_ns']-rows[0]['host_ns'])/1e9
        delta = rows[-1]['cpu_s']-rows[0]['cpu_s'] if len(rows)>1 else None
        processes.append(dict(pid=pid, created_unix_s=created, samples=len(rows),
            first_host_ns=rows[0]['host_ns'], last_host_ns=rows[-1]['host_ns'],
            observed_cpu_s=delta, observed_cpu_fraction=delta/duration if duration and delta is not None else None,
            peak_sampled_rss_bytes=max(row['rss_bytes'] for row in rows), last=rows[-1]))
    duration = (samples[-1]['host_ns']-samples[0]['host_ns'])/1e9 if len(samples)>1 else 0.
    total = sum(row['observed_cpu_s'] for row in processes if row['observed_cpu_s'] is not None)
    return dict(processes=processes, sample_count=len(samples), sampled_seconds=duration,
        observed_tree_cpu_s=total if processes else None,
        observed_tree_cpu_fraction=total/duration if duration else None,
        peak_sampled_sum_rss_bytes=max((sum(p['rss_bytes'] for p in row['processes']) for row in samples), default=None),
        cpu_scope='sum of first-to-last observed per-PID deltas inside the window; excludes unsampled lifetime',
        rss_scope='simultaneous sum; shared pages can be counted more than once',
        gpu_allocation_bytes=None, native_allocation_bytes=None,
        unavailable_reason='No portable allocator/native/GPU allocation accounting; RSS includes native resident memory')


_hooks = None


class Hooks:
    def __init__(self, directory):
        self.directory = str(directory)
        self.callbacks, self.commands, self.ipc = [], [], []
        self.created_ns = time.monotonic_ns()
        self.diagnostics = []

    def install(self):
        from rsim.core.commands import CommandSink
        from rsim.transport.backends.cyclone import Endpoint
        original_task, original_set, original_publish = Component.task, CommandSink.set, Endpoint.publish
        recorder = self

        def task(owner, name, callback, *, hz=None, pacer=None):
            previous_counters = dict(missed=0, skipped=0, source_skipped=0)
            async def measured():
                start = time.monotonic_ns()
                deadline = getattr(pacer, 'scheduled_deadline_ns', None)
                metronome = None
                if hz is not None:
                    coroutine = asyncio.current_task().get_coro()
                    frame = getattr(coroutine, 'cr_frame', None)
                    metronome = frame.f_locals.get('metronome') if frame is not None else None
                    if metronome is not None and metronome.deadline is not None:
                        deadline = round(metronome.deadline*1e9)
                tick = getattr(pacer, 'last_tick', None)
                row = dict(task=f'{type(owner).__name__}:{name}', instance=owner._instance_id,
                    started_ns=start, scheduled_deadline_ns=deadline,
                    jitter_ns=start-deadline if deadline is not None else None,
                    cumulative_missed=getattr(metronome if metronome is not None else pacer, 'missed', None),
                    source_stamp_ns=getattr(tick, 'stamp_ns', None),
                    source_step=getattr(tick, 'step_index', None),
                    source_episode=getattr(tick, 'episode', None),
                    source_session=getattr(tick, 'session', None),
                    skipped=getattr(pacer, 'skipped', None), source_skipped=getattr(pacer, 'source_skipped', None))
                for key, column in (('missed','cumulative_missed'),('skipped','skipped'),('source_skipped','source_skipped')):
                    value = row[column]
                    row[key+'_delta'] = max(0,value-previous_counters[key]) if value is not None else None
                    if value is not None:
                        previous_counters[key] = value
                try:
                    return await callback()
                except BaseException as error:
                    row['exception'] = type(error).__name__
                    raise
                finally:
                    row['finished_ns'] = time.monotonic_ns()
                    recorder.callbacks.append(row)
            return original_task(owner, name, measured, hz=hz, pacer=pacer)

        async def command(sink, value, **kwargs):
            # Aliases recurse to their actual sink; record only that actual call.
            if sink._resolved() is not sink:
                return await original_set(sink, value, **kwargs)
            payload = getattr(value, 'value', value)
            row = dict(port=sink.name, owner=type(sink.producer).__name__, started_ns=time.monotonic_ns(),
                       request_kind=payload.get('kind') if isinstance(payload, dict) else type(payload).__name__)
            try:
                result = await original_set(sink, value, **kwargs)
                row['result_kind'] = result.get('kind') if isinstance(result, dict) else type(result).__name__
                return result
            except BaseException as error:
                row['exception'] = type(error).__name__
                raise
            finally:
                row['finished_ns'] = time.monotonic_ns()
                recorder.commands.append(row)

        def publish(endpoint, text):
            result = original_publish(endpoint, text)
            recorder.ipc.append(dict(host_ns=time.monotonic_ns(), utf8_bytes=len(text.encode('utf-8'))))
            return result

        Component.task, CommandSink.set, Endpoint.publish = task, command, publish

    def flush(self):
        write_json(Path(self.directory)/f'process-{os.getpid()}.json', dict(pid=os.getpid(),
            created_ns=self.created_ns, flushed_ns=time.monotonic_ns(), callbacks=self.callbacks,
            commands=self.commands, ipc=self.ipc, diagnostics=self.diagnostics,
            ipc_scope='actual successful Cyclone Endpoint JSON UTF-8 publications; excludes CDR/network headers/retransmits',
            callback_scope='wall duration; service callbacks include queue wait and Connect includes input wait'))


def install_hooks(directory):
    global _hooks
    if _hooks is None:
        _hooks = Hooks(directory)
        _hooks.install()
    elif _hooks.directory != str(directory):
        raise RuntimeError('one measurement directory per benchmark process')
    return _hooks


class Probe(Component):
    """Benchmark-only dependency opens before and closes after a worker graph."""
    def __init__(self, directory):
        super().__init__()
        self.directory = str(directory)

    async def open(self):
        install_hooks(self.directory)

    async def close(self):
        _hooks.diagnostics.append(self._runtime.diagnostics())
        _hooks.flush()


class ObservedRuntime(Runtime):
    """Used only by a benchmark's explicit app-module Runtime substitution."""
    directory = None
    records = None
    source = None

    def __init__(self, *roots, placement=None, **kwargs):
        placement = dict(placement or {})
        groups = {}
        for component, target in list(placement.items()):
            if type(target).__name__ != 'ProcessPlacement':
                continue
            probe = groups.setdefault(target, Probe(type(self).directory))
            component.dependencies = (probe, *component.dependencies)
            placement[probe] = target
        super().__init__(*roots, placement=placement, **kwargs)
        self.measurement = dict(open_started_ns=None, ready_ns=None, close_started_ns=None, closed_ns=None)

    async def __aenter__(self):
        self.measurement['open_started_ns'] = time.monotonic_ns()
        result = await super().__aenter__()
        self.measurement['ready_ns'] = time.monotonic_ns()
        if type(self).source is not None:
            self.measurement['source_ready'] = await type(self).source.describe()
        type(self).records.append(self.measurement)
        return result

    async def __aexit__(self, *error):
        # Continuous physics starts during Runtime.open. Pause before close gives
        # an exact final source timestamp for the entire open-to-close interval.
        source = type(self).source
        if source is not None:
            self.measurement['source_final'] = await source.pause()
            self.measurement['source_paused_ns'] = time.monotonic_ns()
        self.measurement['diagnostics'] = self.diagnostics()
        self.measurement['close_started_ns'] = time.monotonic_ns()
        try:
            return await super().__aexit__(*error)
        finally:
            self.measurement['closed_ns'] = time.monotonic_ns()


def summarize_instrumentation(paths, start=None, end=None):
    callbacks, commands, ipc = defaultdict(list), defaultdict(list), []
    pids = []
    def within(row, field):
        return (start is None or row[field] >= start) and (end is None or row[field] <= end)
    for path in paths:
        report = json.loads(Path(path).read_text())
        pids.append(report['pid'])
        for row in report['callbacks']:
            if within(row, 'started_ns') and (end is None or row['finished_ns'] <= end):
                callbacks[row['task']].append(row)
        for row in report['commands']:
            if within(row, 'started_ns') and (end is None or row['finished_ns'] <= end):
                commands[(row['port'], row['request_kind'])].append(row)
        ipc.extend(row for row in report['ipc'] if within(row, 'host_ns'))
    return dict(instrumented_pids=sorted(set(pids)), callbacks={name: dict(
        jitter_ns=distribution(row['jitter_ns'] for row in rows),
        callback_wall_ns=distribution(row['finished_ns']-row['started_ns'] for row in rows),
        exceptions=sum('exception' in row for row in rows),
        missed_deadlines=sum(row.get('missed_delta') or 0 for row in rows),
        skipped_publications=sum(row.get('skipped_delta') or 0 for row in rows),
        skipped_source_steps=sum(row.get('source_skipped_delta') or 0 for row in rows),
        max_cumulative_missed=max((row['cumulative_missed'] for row in rows if row['cumulative_missed'] is not None), default=None),
        max_cumulative_source_skips=max((row['source_skipped'] for row in rows if row['source_skipped'] is not None), default=None))
        for name, rows in callbacks.items()}, commands={f'{port}:{kind}': dict(
            rtt_ns=distribution(row['finished_ns']-row['started_ns'] for row in rows),
            failed=sum('exception' in row for row in rows)) for (port, kind), rows in commands.items()},
        ipc_publications=len(ipc), ipc_json_utf8_bytes=sum(row['utf8_bytes'] for row in ipc))
