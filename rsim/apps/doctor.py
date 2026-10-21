"""Explicit deployment probes. Required missing capabilities fail with exit 1."""
import argparse
import asyncio
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import uuid

from rsim.platforms import get_platform
from rsim.platforms.paths import roots
from rsim.platforms.probe import probe_storage
from rsim.runtime.options import RuntimeOptions, RuntimePaths


async def diagnose(*, require=(), options=None, deep=False, backend='cyclonedds', domain=197):
    options = (options or RuntimeOptions()).resolved()
    requested = set(require)
    known = {'process', 'shared-mapping', 'dds', 'render'}
    if requested - known:
        raise ValueError('unknown capabilities: ' + ', '.join(sorted(requested-known)))
    try:
        version = importlib.metadata.version('rsim')
    except importlib.metadata.PackageNotFoundError:
        version = 'uninstalled'
    commit = None
    repository = Path(__file__).resolve().parents[2]
    if (repository / '.git').exists():
        result = subprocess.run(['git', '-C', str(repository), 'rev-parse', 'HEAD'],
                                capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            commit = result.stdout.strip()
    report = dict(system=platform.platform(), python=sys.version, executable=sys.executable,
                  rsim_version=version, commit=commit, event_loop=type(asyncio.get_running_loop()).__name__,
                  required=sorted(requested), probes={}, errors=[])
    try:
        native = get_platform()
        report.update(platform=native.name, host_boot_scope=dict(boot_id=native.identity.boot_id(),
                      user_id=native.identity.user_id()), paths={name: str(path) for name, path in roots(options.paths).items()},
                      supervision=dict(managed_tree_cleanup=native.managed_tree_cleanup,
                                       os_enforced_tree_containment=native.os_enforced_tree_containment))
    except Exception as error:
        report['errors'].append(dict(capability='platform', error=f'{type(error).__name__}: {error}'))
        report['ok'] = False
        return report
    for capability in sorted(requested):
        try:
            if capability == 'shared-mapping':
                report['probes'][capability] = await asyncio.to_thread(probe_storage,
                    report['paths']['storage_root'], deep)
            elif capability == 'process':
                from rsim.runtime import Runtime, ProcessPlacement
                from rsim.components.synthetic import CounterArray
                from rsim.transport.descriptor import TransportConfig
                source = CounterArray(size=4)
                async with Runtime(source.output, options=options,
                                   placement={source: ProcessPlacement('doctor', TransportConfig(backend, domain))}) as runtime:
                    frame = await source.output.get(timeout=options.startup_timeout_s)
                    report['probes'][capability] = dict(shape=list(frame.data.shape),
                                                        workers=[host.worker_pid for host in runtime._binding_plan.hosts.values()])
            elif capability == 'dds':
                if backend == 'inprocess':
                    raise ValueError('inprocess cannot satisfy the DDS capability')
                from rsim.runtime import Runtime
                from rsim.transport.descriptor import DescriptorTransport, TransportConfig
                context = DescriptorTransport(TransportConfig(backend, domain))
                async with Runtime(context, options=options):
                    topic = '/rsim/doctor/p' + uuid.uuid4().hex
                    received = asyncio.Event()
                    subscriber = context.subscribe(topic, lambda message: received.set())
                    publisher = context.publisher(topic)
                    try:
                        async with asyncio.timeout(10):
                            while not received.is_set():
                                publisher.publish('probe')
                                await asyncio.sleep(.05)
                    finally:
                        publisher.close()
                        subscriber.close()
                report['probes'][capability] = dict(backend=backend, domain=domain, roundtrip=True)
            else:
                def render():
                    import mujoco
                    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos="0 0 3"/><geom type="plane" size="2 2 .1"/></worldbody></mujoco>')
                    data = mujoco.MjData(model)
                    with mujoco.Renderer(model, height=32, width=32) as renderer:
                        renderer.update_scene(data)
                        pixels = renderer.render()
                    return dict(mujoco=mujoco.__version__, shape=list(pixels.shape))
                report['probes'][capability] = await asyncio.to_thread(render)
        except Exception as error:
            import traceback
            report['errors'].append(dict(capability=capability, error=f'{type(error).__name__}: {error}',
                                         traceback=traceback.format_exc()))
    report['ok'] = not report['errors']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require', default='', help='comma-separated process,shared-mapping,dds,render')
    parser.add_argument('--deep', action='store_true', help='check retained mappings in another process')
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--backend', default='cyclonedds', choices=('cyclonedds', 'ros2', 'inprocess'))
    parser.add_argument('--domain', type=int, default=197)
    for name in ('registry', 'storage', 'log'):
        parser.add_argument('--' + name + '-root', type=Path)
    args = parser.parse_args()
    options = RuntimeOptions(paths=RuntimePaths(args.registry_root, args.storage_root, args.log_root))
    result = asyncio.run(diagnose(require=filter(None, args.require.split(',')), options=options,
                                  deep=args.deep, backend=args.backend, domain=args.domain))
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
