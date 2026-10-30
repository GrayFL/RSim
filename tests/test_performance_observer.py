"""Measurement correctness: include worker CPU and use actual task deadlines."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

from tools.performance_observer import distribution, resource_window


def test_process_window_sums_worker_deltas_and_keeps_missing_lifetime_explicit():
    def process(pid, cpu, rss, created=10.):
        return dict(pid=pid, created_unix_s=created, cpu_s=cpu, rss_bytes=rss)
    samples = [dict(host_ns=1_000_000_000, processes=[process(1,10,100),process(2,2,200)]),
               dict(host_ns=2_000_000_000, processes=[process(1,10.1,110),process(2,2.8,250),process(3,4,300)]),
               dict(host_ns=3_000_000_000, processes=[process(1,10.2,120),process(3,4.3,320)])]
    report = resource_window(samples)
    assert report['observed_tree_cpu_s'] == pytest.approx(1.3)
    assert report['observed_tree_cpu_fraction'] == pytest.approx(.65)
    assert report['peak_sampled_sum_rss_bytes'] == 660
    assert report['gpu_allocation_bytes'] is None
    narrow = resource_window(samples,2_000_000_000,3_000_000_000)
    assert narrow['observed_tree_cpu_s'] == pytest.approx(.4)
    assert next(row for row in narrow['processes'] if row['pid']==2)['observed_cpu_s'] is None


def test_process_identity_does_not_join_reused_pid_and_statistics_keep_signed_jitter():
    samples = [dict(host_ns=i*1_000_000_000, processes=[dict(pid=2,created_unix_s=created,
                cpu_s=cpu,rss_bytes=1)]) for i,created,cpu in [(1,5.,7.),(2,6.,.2),(3,6.,.5)]]
    report = resource_window(samples)
    assert len(report['processes']) == 2
    assert report['observed_tree_cpu_s'] == pytest.approx(.3)
    assert distribution([-2,0,2])['mean'] == 0
    assert distribution([None])['max'] is None


def test_instrumented_default_task_records_true_deadlines_in_an_isolated_process(tmp_path):
    output = tmp_path/'result.json'
    raw = tmp_path/'raw'
    subprocess.run([sys.executable,'-m','tools.benchmark_matrix','sample','--case','core',
        '--output',str(output),'--raw',str(raw),'--seconds','.1','--warmup','.02'],
        cwd=Path(__file__).resolve().parents[1],check=True,capture_output=True,text=True,timeout=30)
    result = json.loads(output.read_text())
    assert result['valid']
    report, = (json.loads(path.read_text()) for path in raw.glob('process-*.json'))
    rows = [row for row in report['callbacks'] if row['task']=='Pulse:pulse']
    assert len(rows)>20
    assert all(row['scheduled_deadline_ns'] is not None for row in rows)
    assert all(row['jitter_ns']==row['started_ns']-row['scheduled_deadline_ns'] for row in rows)
    assert all(row['finished_ns']>=row['started_ns'] for row in rows)
