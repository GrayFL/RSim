"""Measurement semantics: neither cache hits nor missing counters imply zero disk I/O."""
import os

import numpy as np
import pytest

pytest.importorskip('psutil')
from tools.benchmark_metrics import counter_delta, file_snapshot, host_interval, transport_totals


def test_counter_deltas_do_not_turn_missing_or_reset_counters_into_zero():
    assert counter_delta({'write_bytes': 10, 'read_bytes': 20, 'other': -1, 'lost': 3},
                         {'write_bytes': 25, 'read_bytes': 4, 'other': -1}) == {
        'write_bytes': 15, 'read_bytes': None, 'other': None, 'lost': None}


def test_host_disks_remain_separate_and_missing_devices_are_reported():
    def snapshot(stamp, disks):
        return dict(time_s=stamp, disks=disks, memory={}, swap={'sin': None, 'sout': None}, errors=[])
    before = snapshot(10, {'disk': {'write_bytes': 100}, 'partition': {'write_bytes': 30}, 'gone': {}})
    after = snapshot(12, {'disk': {'write_bytes': 150}, 'partition': {'write_bytes': 50}})
    report = host_interval(before, after)
    assert report['disks']['disk']['write_bytes_per_s'] == 25
    assert report['disks']['partition']['delta']['write_bytes'] == 20
    assert 'error' in report['disks']['gone']
    assert report['swap_io_delta']['sout'] is None
    assert 'total' not in report['disks']


def test_live_file_sizes_deduplicate_hardlinks(tmp_path):
    source = tmp_path / 'one.npy'
    source.write_bytes(b'payload')
    os.link(source, tmp_path / 'two.npy')
    report = file_snapshot(tmp_path)
    assert report['paths'] == 2
    assert report['unique_files'] == 1
    assert report['unique_file_bytes'] == 7


def test_transport_counters_exclude_live_file_sizes_and_subscription_counts():
    report = {'local': {'allocator': {'materialized_bytes': 100, 'mapping_creations': 1},
              'ports': {'output': {'hardlinks': 4, 'subscriptions': 3, 'active_files': 20}}}}
    assert transport_totals(report) == dict(materialized_bytes=100, mapping_creations=1, hardlinks=4)
    assert transport_totals({}) == {}


@pytest.mark.skipif(os.name != 'nt', reason='Windows temporary file cache hint')
def test_temporary_hint_keeps_retained_mappings_and_hardlinks_working(tmp_path, monkeypatch):
    from functools import partial
    from rsim.platforms.windows import files
    from rsim.core import Frame
    from rsim.transport.shared import SharedStore, decode
    monkeypatch.setattr(files, 'open_shared', partial(files.open_shared, temporary=True))
    store = SharedStore(tmp_path / 'store')
    original = store.prepare(np.arange(16, dtype=np.float32))
    frame = decode(store.put(Frame(original, 1, 'test'))['data'], store.directory)
    paths = list(store.directory.rglob('*.npy'))
    assert paths and all(path.stat().st_file_attributes & 0x100 for path in paths)
    assert store.diagnostics()['hardlinks'] == 1
    store.close()
    np.testing.assert_array_equal(frame, np.arange(16))
    assert not frame.flags.writeable


def test_benchmark_arguments_reject_invalid_measurement_windows():
    from tools.benchmark_runtime import parser
    for arguments in (['--seconds', 'nan'], ['--seconds', '0'], ['--history', '0'],
                      ['--idle-seconds', '-1'], ['--sample-interval', 'inf']):
        with pytest.raises(SystemExit):
            parser().parse_args(['--output', 'unused.json', *arguments])


def test_summary_preserves_repeats_variants_and_unknown_disk_counters():
    from tools.compare_benchmarks import summarize_reports
    def row(variant, repeat, writes):
        disk = {'delta': {'write_bytes': writes}}
        return dict(workload='rgbd', variant=variant, repeat=repeat, cleanup_complete=True,
                    config={'history': 4}, storage={'root': '/dev/shm'},
                    io={stage: {'disks': {'disk.with.dots': disk}} for stage in
                        ('idle', 'steady', 'post_close', 'whole_run')})
    report = dict(schema_version=2, host='same', platform='same', python='same', measurement={},
                  workloads=[row('file', 1, 10), row('file', 2, 30), row('direct', 1, None)])
    summary = summarize_reports([('test', report)])
    normal, direct = summary['groups']
    assert normal['repeats'] == [1, 2]
    assert normal['disks']['disk.with.dots']['steady']['delta.write_bytes'] == dict(
        available=2, runs=2, median=20, minimum=10, maximum=30)
    assert direct['disks']['disk.with.dots']['steady']['delta.write_bytes']['median'] is None
    with pytest.raises(ValueError, match='unknown disk'):
        summarize_reports([('test', report)], ['missing'])
    report['workloads'].append(row('file', 1, 100))
    with pytest.raises(ValueError, match='duplicate repeat'):
        summarize_reports([('test', report)])


def test_summary_rejects_incomparable_or_incomplete_reports():
    from tools.compare_benchmarks import summarize_reports
    with pytest.raises(ValueError, match='schema_version'):
        summarize_reports([('old', {})])
    with pytest.raises(ValueError, match='incomplete benchmark'):
        summarize_reports([('failed', {'schema_version': 2, 'error': 'timeout'})])
    with pytest.raises(ValueError, match='incomplete benchmark'):
        summarize_reports([('running', {'schema_version': 2, 'complete': False})])
