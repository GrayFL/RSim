"""Read-only OS observations for the local runtime benchmark.

Device counters belong to the whole host, not to an individual process. In
particular, process I/O is not a reliable measure of memory-map writeback.
"""
import asyncio
import os
from pathlib import Path
import stat
import time

import psutil


def counter_delta(before, after):
    """Unknown/reset counters stay unknown, rather than becoming zero writes."""
    return {key: (after[key] - value if key in after and after[key] >= value >= 0 else None)
            for key, value in before.items()}


def host_snapshot():
    errors = []
    try:
        disks = {key: value._asdict() for key, value in
                 (psutil.disk_io_counters(perdisk=True, nowrap=True) or {}).items()}
        if not disks:
            errors.append('disk device counters unavailable')
    except (OSError, psutil.Error) as error:
        disks = {}
        errors.append(str(error))
    memory = psutil.virtual_memory()._asdict()
    swap = psutil.swap_memory()._asdict()
    # psutil returns zero placeholders for swap traffic on Windows.
    if os.name == 'nt':
        swap['sin'] = swap['sout'] = None
    return dict(time_s=time.perf_counter(), disks=disks, memory=memory, swap=swap, errors=errors)


def host_interval(before, after):
    seconds = after['time_s'] - before['time_s']
    disks = {}
    for name in sorted(before['disks'].keys() | after['disks'].keys()):
        if name not in before['disks'] or name not in after['disks']:
            disks[name] = dict(error='device missing at one boundary')
            continue
        delta = counter_delta(before['disks'][name], after['disks'][name])
        disks[name] = dict(delta=delta, **{key + '_per_s': delta.get(key) / seconds
                            if delta.get(key) is not None and seconds > 0 else None
                            for key in ('read_bytes', 'write_bytes')})
    swap = {name: after['swap'][name] - before['swap'][name]
            if before['swap'].get(name) is not None and after['swap'].get(name) is not None
            and after['swap'][name] >= before['swap'][name] else None
            for name in ('sin', 'sout')}
    return dict(seconds=seconds, disks=disks, swap_io_delta=swap,
                memory_before=before['memory'], memory_after=after['memory'],
                swap_before=before['swap'], swap_after=after['swap'],
                errors=before['errors'] + after['errors'])


def process_snapshot(process):
    result = dict(pid=process.pid)
    try:
        result['rss_bytes'] = process.memory_info().rss
        result['handles'] = process.num_handles() if os.name == 'nt' else process.num_fds()
        result['io_counters'] = process.io_counters()._asdict()
    except (psutil.Error, OSError) as error:
        result['error'] = f'{type(error).__name__}: {error}'
    return result


def file_snapshot(directory):
    """Count live names and unique backing files, never sum hard links twice.

    Unlinked-but-retained mappings are absent here. st_size is a logical file
    length, not a disk-write or resident-memory measurement.
    """
    names, unique, temporary, raced = 0, {}, 0, 0
    for path in Path(directory).rglob('*.npy'):
        try:
            info = path.stat()
        except FileNotFoundError:
            raced += 1
            continue
        names += 1
        identity = (info.st_dev, info.st_ino)
        if identity not in unique:
            unique[identity] = info.st_size
            temporary += bool(getattr(info, 'st_file_attributes', 0)
                              & getattr(stat, 'FILE_ATTRIBUTE_TEMPORARY', 0x100))
    return dict(paths=names, unique_files=len(unique), unique_file_bytes=sum(unique.values()),
                temporary_unique_files=temporary, disappeared_during_scan=raced)


class ResourceSampler:
    def __init__(self, processes, directory, interval):
        self.processes, self.directory, self.interval = processes, directory, interval
        self.samples = []
        self.stopped = asyncio.Event()
        self.task = None

    def sample(self):
        self.samples.append(dict(time_s=time.perf_counter(),
            processes={name: process_snapshot(process) for name, process in self.processes.items()},
            files=file_snapshot(self.directory), memory_available_bytes=psutil.virtual_memory().available))

    async def run(self):
        while not self.stopped.is_set():
            try:
                await asyncio.wait_for(self.stopped.wait(), self.interval)
            except TimeoutError:
                self.sample()

    async def __aenter__(self):
        self.sample()
        self.task = asyncio.create_task(self.run())
        return self

    async def __aexit__(self, *error):
        self.stopped.set()
        await self.task
        self.sample()

    def report(self):
        first, last = self.samples[0], self.samples[-1]
        processes = {}
        for name in self.processes:
            before, after = first['processes'][name], last['processes'][name]
            values = [sample['processes'][name].get('rss_bytes') for sample in self.samples]
            processes[name] = dict(before=before, after=after,
                io_delta=counter_delta(before['io_counters'], after['io_counters'])
                    if 'io_counters' in before and 'io_counters' in after else None,
                sampled_peak_rss_bytes=max((v for v in values if v is not None), default=None))
        return dict(sample_interval_s=self.interval, sampled_seconds=last['time_s'] - first['time_s'],
            samples=len(self.samples), processes=processes,
            files_before=first['files'], files_after=last['files'],
            sampled_peak_unique_file_bytes=max(s['files']['unique_file_bytes'] for s in self.samples),
            sampled_peak_unique_files=max(s['files']['unique_files'] for s in self.samples),
            minimum_host_available_bytes=min(s['memory_available_bytes'] for s in self.samples))


def transport_totals(report):
    """Sum named cumulative counters; ignore directory sizes and gauge fields."""
    if not report:
        return {}
    keys = ('materialized_bytes', 'mapping_creations', 'hardlinks')
    totals = dict.fromkeys(keys, 0)
    def visit(value):
        if not isinstance(value, dict):
            return
        for key, item in value.items():
            if key in totals and isinstance(item, (int, float)):
                totals[key] += item
            elif isinstance(item, dict):
                visit(item)
    visit(report)
    return totals
