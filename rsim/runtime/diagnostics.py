"""Serializable runtime snapshots; no executable recipes or session tokens."""
from dataclasses import asdict
import os
import sys


def runtime_report(runtime):
    from rsim.platforms.errors import cleanup_failures
    components = []
    for component in runtime._order:
        try:
            demand = component.demand_snapshot()
            ports = sorted(demand.ports)
        except Exception:
            ports = []
        entry = dict(type=type(component).__name__, instance_id=component._instance_id,
                     active=component._runtime is runtime and not component._closed,
                     demanded_ports=ports, failure=str(component._failure) if component._failure else None)
        entry['history_evictions'] = {name: port.history_evictions for name, port in component.outputs.items()}
        for metric in ('queue_drops', 'evicted_descriptors'):
            if hasattr(component, metric):
                entry[metric] = getattr(component, metric)
        for name in ('worker_pid', 'directory', 'log_path'):
            value = getattr(component, name, None)
            if value is not None:
                entry[name] = value if isinstance(value, (str, int)) else str(value)
        manifest = getattr(component, 'manifest', None)
        if manifest:
            entry['provider'] = {key: manifest[key] for key in ('instance_id', 'pid', 'source_leases',
                                                                'port_subscriptions', 'transport_stats') if key in manifest}
        bindings = getattr(component, 'bindings', None)
        if isinstance(bindings, dict):
            entry['subscriptions'] = sorted(bindings)
        store = getattr(component, 'store', None)
        if store is not None and hasattr(store, 'diagnostics'):
            entry['store'] = store.diagnostics()
        components.append(entry)
    plan = runtime._binding_plan
    placements = [] if plan is None else [dict(component=type(component).__name__,
        placement=getattr(placement, 'name', 'local')) for component, placement in plan.assignments.items()]
    return dict(pid=os.getpid(), python=sys.version, lifecycle=asdict(runtime.lifecycle),
                demand_generation=runtime.demand.generation, components=components, placements=placements,
                options=runtime.options.to_dict(), platform_cleanup_failures=list(cleanup_failures))
