"""Summarize local benchmark reports and repeats; no fixed performance gate.

Example: python tools/compare_benchmarks.py assets/benchmark/windows.json --disk PhysicalDrive0
"""
import argparse
import json
from pathlib import Path
import statistics


def value(record, path):
    for part in path.split('.'):
        if not isinstance(record, dict):
            return None
        record = record.get(part)
    return record


def summarize(values):
    known = [item for item in values if item is not None]
    return dict(available=len(known), runs=len(values),
                median=statistics.median(known) if known else None,
                minimum=min(known) if known else None, maximum=max(known) if known else None)


def summarize_reports(reports, devices=None):
    """Keep variant/storage/config differences explicit; never collapse repeats."""
    output = []
    environment = None
    for label, report in reports:
        if report.get('schema_version') != 2:
            raise ValueError(f'{label}: requires schema_version=2; rerun the benchmark')
        if report.get('error'):
            raise ValueError(f'{label}: incomplete benchmark: {report["error"]}')
        if not report.get('complete', True):
            raise ValueError(f'{label}: incomplete benchmark; wait for every requested run')
        current = {key: report.get(key) for key in ('host', 'platform', 'python')}
        if environment is not None and current != environment:
            raise ValueError('reports must come from the same host, platform and Python environment')
        environment = current
        groups = {}
        for row in report['workloads']:
            if not row.get('cleanup_complete'):
                raise ValueError(f'{label}: incomplete cleanup for {row["workload"]}')
            key = (row['workload'], row['variant'], row['storage']['root'])
            groups.setdefault(key, []).append(row)
        if not groups:
            raise ValueError(f'{label}: contains no completed workloads')
        for (workload, variant, storage), rows in groups.items():
            if any(row['config'] != rows[0]['config'] for row in rows):
                raise ValueError(f'{label}/{workload}/{variant}: repeat configurations differ')
            repeats = [row['repeat'] for row in rows]
            if len(repeats) != len(set(repeats)):
                raise ValueError(f'{label}/{workload}/{variant}: duplicate repeat numbers')
            focus = devices or report['measurement'].get('focus_disks')
            available = set().union(*(row['io']['steady']['disks'] for row in rows))
            if focus and not set(focus) <= available:
                raise ValueError(f'{label}: unknown disk; available: {sorted(available)}')
            metrics = {metric: summarize([value(row, metric) for row in rows]) for metric in (
                'observed_publication_hz', 'observed_payload_bytes', 'consumer_read_bytes',
                'observed_samples', 'skipped_publications',
                'transport_delta.materialized_bytes', 'resources.sampled_peak_unique_file_bytes',
                'resources.minimum_host_available_bytes', 'latency_ms.p95', 'payload_read_ms.p95')}
            disks = {}
            for device in sorted(focus or available):
                # Index device names directly: a device name may contain dots.
                disks[device] = {stage: {
                    metric: summarize([value(row['io'][stage]['disks'].get(device, {}), metric)
                                       for row in rows])
                    for metric in ('delta.write_bytes', 'write_bytes_per_s', 'delta.read_bytes')}
                    for stage in ('idle', 'steady', 'post_close', 'whole_run')}
            output.append(dict(report=label, workload=workload, variant=variant, storage_root=storage,
                               config=rows[0]['config'], repeats=repeats, metrics=metrics, disks=disks))
    return dict(environment=environment, groups=output,
                notes=['Each summary is min/median/max across independent runs, not pooled frame percentiles.',
                       'Check config and storage_root before comparing groups.',
                       'Disk counters are host-wide. Idle writes are not subtracted; there is no pass/fail threshold.',
                       'Missing/reset counters stay unavailable; zero is only a measured zero.'])


def show(report):
    def number(metric, scale=1):
        median = metric['median']
        return f'{median / scale:.2f}' if median is not None else 'n/a'
    mib = 1024**2
    print('Medians across runs; host disk counters include other applications. Units: MiB and MiB/s.')
    for group in report['groups']:
        config, metrics = group['config'], group['metrics']
        print(f"\n{group['report']} / {group['workload']} / {group['variant']} / {len(group['repeats'])} run(s)"
              f" / history={config['history']} / {config['read_mode']} / {group['storage_root']}")
        print(f"  observed Hz={number(metrics['observed_publication_hz'])}  "
              f"payload={number(metrics['observed_payload_bytes'], mib)}  "
              f"read={number(metrics['consumer_read_bytes'], mib)}  "
              f"skipped={number(metrics['skipped_publications'])}  "
              f"copied={number(metrics['transport_delta.materialized_bytes'], mib)}  "
              f"live files peak={number(metrics['resources.sampled_peak_unique_file_bytes'], mib)}")
        for name, disk in group['disks'].items():
            print(f"  {name}: steady write={number(disk['steady']['delta.write_bytes'], mib)}  "
                  f"rate={number(disk['steady']['write_bytes_per_s'], mib)}  "
                  f"idle rate={number(disk['idle']['write_bytes_per_s'], mib)}  "
                  f"post-close write={number(disk['post_close']['delta.write_bytes'], mib)}  "
                  f"whole-run write={number(disk['whole_run']['delta.write_bytes'], mib)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', type=Path, nargs='+')
    parser.add_argument('--disk', action='append', help='device names; never sum disks and their partitions')
    parser.add_argument('--output', type=Path, help='optional JSON with per-group ranges and configuration')
    args = parser.parse_args()
    try:
        report = summarize_reports([(str(path), json.loads(path.read_text(encoding='utf-8')))
                                    for path in args.reports], args.disk)
    except (ValueError, KeyError) as error:
        parser.error(str(error))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    show(report)


if __name__ == '__main__':
    main()
