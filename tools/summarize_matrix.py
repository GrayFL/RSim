"""Derive tables and validity checks from benchmark_matrix's archived samples.

This is an offline reader. Run after all measurements have stopped, so JSON
parsing cannot compete with the measured workers. Input files stay unchanged.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics

from tools.performance_observer import distribution, write_json


def scalar_stats(values):
    values = [float(value) for value in values if value is not None]
    return dict(distribution(values), values=values,
                median=statistics.median(values) if values else None)


def raw_reports(path):
    return [json.loads(file.read_text()) for file in (path.parent/'instrumentation').glob('process-*.json')]


def validate_navigation(data):
    navigation = data.get('navigation')
    if navigation is None:
        return []
    errors = []
    locked = isinstance(navigation,dict)
    episodes = navigation['episodes'] if locked else navigation
    for index, episode in enumerate(episodes):
        if not episode['arrived'] or episode['collided']:
            errors.append(f'episode {index} did not arrive collision-free')
        last = episode['samples'][-1]
        if abs(last['speed'])>=.02 or abs(last['turn'])>=.02:
            errors.append(f'episode {index} did not stop')
        if locked:
            for i, trace in enumerate(episode['trace']):
                advance, accepted, context = trace['advanced'],trace['accepted'],trace['context']
                n = navigation['requested_substeps']
                dt_ns = round(navigation['physics_timestep']*1e9)
                if (not trace['ok'] or context['step_index']!=i
                        or advance['request_context']!=context or accepted['context']!=context
                        or advance['command_id']!=accepted['command_id']
                        or advance['actual_substeps']!=n or advance['requested_substeps']!=n
                        or advance['physical_substeps']!=(i+1)*n
                        or advance['stamp_ns']!=(i+1)*n*dt_ns
                        or advance['context']!={**context,'step_index':i+1}):
                    errors.append(f'episode {index} trace {i} inconsistent command/physics ACK')
                for stage in trace.get('stages',[]):
                    if stage['context']!=context or stage['finished_ns']<stage['started_ns']:
                        errors.append(f'episode {index} trace {i} inconsistent compute ACK')
        else:
            factory = data['factory']
            is_memory = data['case'].startswith('memory-')
            physical_ns = round((factory['dt'] if is_memory else factory['physics']['timestep'])*1e9)
            publication_ns = physical_ns if is_memory else round(1e9/factory['hz'])
            for left, right in zip(episode['samples'],episode['samples'][1:]):
                if (right['session']!=left['session'] or right['episode']!=left['episode']
                        or right['stamp_ns']-left['stamp_ns']!=(right['step_index']-left['step_index'])*publication_ns
                        or right['stamp_ns']<=left['stamp_ns']):
                    errors.append(f'episode {index} inconsistent source publication interval')
            final = data['runtime'][index]['source_final']
            if final['stamp_ns'] != final['physical_substeps']*physical_ns:
                errors.append(f'episode {index} inconsistent fixed physical dt')
    return errors


def counters(data, raw):
    components, stores = {}, {}
    snapshots = [row.get('diagnostics') for row in data.get('runtime',[])]
    snapshots += [snapshot for process in raw for snapshot in process.get('diagnostics',[])]
    for snapshot in snapshots:
        for row in (snapshot or {}).get('components',[]):
            key = row['instance_id']
            previous = components.setdefault(key,dict(queue_drops=0,evicted_descriptors=0,history_evictions=0,failure=None))
            for metric in ('queue_drops','evicted_descriptors'):
                previous[metric] = max(previous[metric],row.get(metric,0))
            previous['history_evictions'] = max(previous['history_evictions'],sum(row.get('history_evictions',{}).values()))
            previous['failure'] = previous['failure'] or row.get('failure')
            if 'store' in row:
                store = row['store']
                target = stores.setdefault(store['directory'],{})
                for metric in ('materialized_bytes','mapping_creations','hardlinks'):
                    if metric in store:
                        target[metric] = max(target.get(metric,0),store[metric])
    return {**{name:sum(row[name] for row in components.values()) for name in
               ('queue_drops','evicted_descriptors','history_evictions')},
            'provider_errors':[row['failure'] for row in components.values() if row['failure']],
            'store_counters':{name:sum(row.get(name,0) for row in stores.values()) for name in
                              ('materialized_bytes','mapping_creations','hardlinks')},
            'scope':'lifetime counters deduplicated by component instance/store directory; history evictions include normal bounded retention'}


def one_run(path):
    data = json.loads(path.read_text())
    raw = raw_reports(path)
    errors = validate_navigation(data)
    if not data.get('valid'):
        errors.append(data.get('error','invalid child result'))
    if not data.get('resource_coverage_complete'):
        errors.append('not every instrumented process has external resource samples')
    row = dict(path=str(path),case=data.get('case'),rate=data.get('rate'),valid=not errors,errors=errors,
        total_resources=data.get('resources'),counters=counters(data,raw),
        ipc_bytes=data.get('instrumentation',{}).get('ipc_json_utf8_bytes'))
    steady = data.get('steady',[])
    sample_time = sum(part['resources']['sampled_seconds'] for part in steady)
    cpu = sum(part['resources']['observed_tree_cpu_s'] or 0 for part in steady)
    row['steady_cpu_fraction'] = cpu/sample_time if sample_time else None
    row['steady_peak_rss_bytes'] = max((part['resources']['peak_sampled_sum_rss_bytes'] or 0 for part in steady),default=None)
    row['steady_window_seconds'] = sum((part['window'][1]-part['window'][0])/1e9 for part in steady)
    row['resource_window_scope'] = 'measured windows after warmup'
    if not steady:
        row['steady_cpu_fraction'] = data['resources']['observed_tree_cpu_fraction']
        row['steady_peak_rss_bytes'] = data['resources']['peak_sampled_sum_rss_bytes']
        row['resource_window_scope'] = 'whole child observation, including imports and artifact writes; no stationary window'
    row['open_s'] = [(r['ready_ns']-r['open_started_ns'])/1e9 for r in data.get('runtime',[])]
    row['close_s'] = [(r['closed_ns']-r['close_started_ns'])/1e9 for r in data.get('runtime',[])]
    windows = [part['window'] for part in steady]
    inside = lambda value: not windows or any(a<=value<=b for a,b in windows)
    command_groups, worker_commands, callback_groups = defaultdict(list), defaultdict(list), defaultdict(list)
    coordinator_pid = data['runtime'][0]['diagnostics']['pid'] if data.get('runtime') else None
    for report in raw:
        for sample in report['commands']:
            if inside(sample['started_ns']) and inside(sample['finished_ns']):
                target = command_groups if report['pid']==coordinator_pid else worker_commands
                key = (f"{sample['port']}:{sample['request_kind']}" if report['pid']==coordinator_pid
                       else f"{report['pid']}:{sample['owner']}:{sample['port']}:{sample['request_kind']}")
                target[key].append(sample)
        for sample in report['callbacks']:
            if inside(sample['started_ns']) and inside(sample['finished_ns']):
                callback_groups[sample['task']].append(sample)
    row['command_rtt_ms'] = {key:distribution((s['finished_ns']-s['started_ns'])/1e6 for s in samples)
                             for key,samples in command_groups.items()}
    row['worker_endpoint_call_ms'] = {key:distribution((s['finished_ns']-s['started_ns'])/1e6 for s in samples)
                                     for key,samples in worker_commands.items()}
    row['command_scope'] = 'coordinator call RTT; worker-local endpoint call durations are kept separately'
    row['callbacks'] = {key:dict(jitter_ms=distribution(s['jitter_ns']/1e6 if s['jitter_ns'] is not None else None for s in samples),
        wall_ms=distribution((s['finished_ns']-s['started_ns'])/1e6 for s in samples),
        missed_deadlines=sum(s.get('missed_delta') or 0 for s in samples)) for key,samples in callback_groups.items()}
    navigation = data.get('navigation')
    if isinstance(navigation,dict):
        episodes = navigation['episodes'][data['warmup_episodes']:]
        trace = [t for episode in episodes for t in episode['trace']]
        elapsed = sum(e['control_window_s'] for e in episodes)
        row.update(observed_rtf=sum(e['source_delta_s'] for e in episodes)/elapsed,
            complete_runtime_rtf=navigation['end_to_end_rtf'],control_steps_per_s=len(trace)/elapsed,
            step_scope='complete compute + accepted command + PhysicsAdvanced transaction',
            config_fingerprint=navigation['configuration_fingerprint'])
        phases, stage_elapsed, stage_rtt = defaultdict(list), defaultdict(list), defaultdict(list)
        for t in trace:
            for name, start, finish in t['timings']:
                phases[name].append((finish-start)/1e6)
            for stage,call in zip(t.get('stages',[]),t.get('stage_calls',[])):
                stage_elapsed[stage['task_id']].append((stage['finished_ns']-stage['started_ns'])/1e6)
                stage_rtt[stage['task_id']].append((call['finished_ns']-call['started_ns'])/1e6)
        row['coordinator_phase_ms'] = {key:distribution(values) for key,values in phases.items()}
        row['stage_elapsed_ms'] = {key:distribution(values) for key,values in stage_elapsed.items()}
        row['stage_rtt_ms'] = {key:distribution(values) for key,values in stage_rtt.items()}
    elif isinstance(navigation,list):
        episodes = navigation[data['warmup_episodes']:]
        elapsed = sum(e['observed_wall_s'] for e in episodes)
        local = [s for e in episodes for s in e['computation_samples']['local']]
        execution = [s for e in episodes for s in e['computation_samples']['execution']]
        count = sum(inside(s['completed_at']*1e9) for s in local)
        row.update(observed_rtf=sum(e['source_delta_s'] for e in episodes)/elapsed,
            complete_runtime_rtf=sum(e['complete_runtime_physical_s'] for e in episodes)/sum(e['complete_runtime_wall_s'] for e in episodes),
            control_steps_per_s=count/elapsed,step_scope='observed local computations; command ACK rates separate, continuous physics has no per-command ACK',
            config_fingerprint=episodes[0]['configuration_fingerprint'],
            local_compute_ms=distribution(s['compute_ms'] for s in local),
            execution_compute_ms=distribution(s['compute_ms'] for s in execution),
            consumed_source_dt_s=distribution(s.get('source_dt_s') for s in local),
            source_steps_skipped=[s.get('source_steps_skipped') for s in local[-1:]],
            command_drops=sum(e['command_drops'] for e in episodes))
    elif row['case']=='core':
        row['publish_get_per_s'] = data['micro']['publish_get_per_s']
    elif row['case']=='pacer':
        row['fanout'] = [dict(subscribers=f['subscribers'],steps_per_s=f['steps_per_s'],
            publish_wall_ns=distribution(f['publication_wall_ns'][f['warmup_steps']:]),skipped=f['skipped']) for f in data['fanout']]
        row['replay'] = [dict(rate=r['rate'],rtf=r['rtf']) for r in data['replay']]
        row['latest_coalesced'] = data['latest_coalesced']
    elif row['case']=='factory':
        groups = defaultdict(list)
        for s in data['config_build']:
            if not s['warmup']:
                groups[s['factory']].append(s)
        row['factories'] = {name:dict(parse_ms=distribution((s['parsed_ns']-s['started_ns'])/1e6 for s in samples),
            build_ms=distribution((s['built_ns']-s['parsed_ns'])/1e6 for s in samples),
            shared_ms=distribution((s['shared_ns']-s['built_ns'])/1e6 for s in samples)) for name,samples in groups.items()}
        row['spawn_ready_s'] = (data['worker_ready_ns']-data['spawn_started_ns'])/1e9
        row['generated_session_ms'] = distribution((s['closed_ns']-s['started_ns'])/1e6 for s in data['parameter_sessions'] if not s['warmup'])
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    manifest = json.loads((args.directory/'manifest.json').read_text())
    rows, groups = [], defaultdict(list)
    for run in manifest['runs']:
        path = Path(run['output'])
        if not path.is_absolute() and not path.exists():
            path = args.directory/path.parent.name/path.name
        row = one_run(path)
        row.update(side=run['side'],repeat=run['repeat'],exitcode=run['exitcode'],surviving_pids=run.get('surviving_pids'))
        rows.append(row)
        groups[f"{row['case']}:{row['rate']:g}:{row['side']}"].append(row)
    metrics = ('observed_rtf','complete_runtime_rtf','control_steps_per_s','steady_cpu_fraction',
               'steady_peak_rss_bytes','ipc_bytes','publish_get_per_s')
    summary = {key:dict(runs=len(values),failures=sum(not r['valid'] or bool(r['exitcode']) for r in values),
        metrics={name:scalar_stats(r.get(name) for r in values) for name in metrics}) for key,values in groups.items()}
    report = dict(runs=rows,summary=summary,manifest=str(args.directory/'manifest.json'),
        postprocessor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        valid=all(row['valid'] and row['exitcode']==0 and not row['surviving_pids'] for row in rows),
        note='Medians across repetitions; raw per-sample distributions and full source data remain archived. No uniform performance threshold applied.')
    write_json(args.output,report)
    lines=['# Archived performance matrix','',
        '| Case | Runs / invalid | Source RTF | Complete Runtime RTF | Steps/s | Tree CPU cores | Peak sum RSS MiB | IPC JSON MiB |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    def number(value,scale=1.):
        return '—' if value is None else f'{value/scale:.3f}'
    for key,group in summary.items():
        m=group['metrics']
        cells=[key,f"{group['runs']} / {group['failures']}"]
        cells += [number(m[name]['median'],scale) for name,scale in [('observed_rtf',1),('complete_runtime_rtf',1),
            ('control_steps_per_s',1),('steady_cpu_fraction',1),('steady_peak_rss_bytes',1024**2),('ipc_bytes',1024**2)]]
        lines.append('| '+' | '.join(cells)+' |')
    lines += ['', 'Continuous steps/s counts observed local computations; lockstep counts complete acknowledged transactions.',
        'CPU sums sampled coordinator + all descendants. RSS sums can double-count shared pages; IPC excludes framing and retransmissions.',
        'Pacer resource totals use the whole child observation; other rows use measured windows after warmup.',
        'Each result.json retains warmup and measured windows, detailed p50/p95/p99/max/mean/variance, misses, drops, command and stage RTTs.']
    args.output.with_suffix('.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(valid=report['valid'],runs=len(rows),output=str(args.output))))
    return 0 if report['valid'] else 1


if __name__=='__main__':
    raise SystemExit(main())
