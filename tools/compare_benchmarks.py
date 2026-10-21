"""Compare fixed workloads against a same-platform baseline; emit JSON evidence."""
import argparse
import json
from pathlib import Path


def value(record, path):
    for part in path.split('.'):
        record = record[part]
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('current', type=Path)
    parser.add_argument('--thresholds', type=Path, default=Path(__file__).with_name('performance-thresholds.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    previous, current, limits = [json.loads(path.read_text(encoding='utf-8'))
                                  for path in (args.baseline, args.current, args.thresholds)]
    if previous['platform'] != current['platform'] or previous['python'] != current['python']:
        raise ValueError('baseline and current must use the same platform and Python environment')
    baseline = {row['workload']: row for row in previous['workloads']}
    checks = []
    if set(baseline) != {row['workload'] for row in current['workloads']}:
        raise ValueError('workload sets differ')
    for row in current['workloads']:
        old = baseline[row['workload']]
        if old['config'] != row['config']:
            raise ValueError('workload configuration differs: ' + row['workload'])
        def check(metric, bound):
            measured = value(row, metric)
            checks.append(dict(workload=row['workload'], metric=metric, measured=measured,
                               maximum=bound, passed=measured <= bound))
        for metric, (ratio, allowance) in limits['relative_with_absolute_allowance'].items():
            original = value(old, metric)
            check(metric, max(original * ratio, original + allowance))
        for metric, allowance in limits['absolute_growth'].items():
            check(metric, value(old, metric) + allowance)
        check('software_fallback_delay_ms', limits['software_fallback_delay_ms'])
    passed = all(row['passed'] for row in checks)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(passed=passed, thresholds=limits, checks=checks), indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=passed, failures=[row for row in checks if not row['passed']]), indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
