"""Run NAV/E acceptance with an engine-only provider and engine-free consumer."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid


PROVIDER = r'''
import asyncio, importlib.util, json, os, sys
from pathlib import Path
from rsim import Runtime, TransportConfig
from mujocosim.config import SensorConfig
from mujocosim.robots.differential_drive import DifferentialDrive, DifferentialDriveConfig
from mujocosim.rsim.drivers import MujocoChassisFactory
assert importlib.util.find_spec('pyrobo') is None
scene, cache, key, domain, ready, log = sys.argv[1:]
factory = MujocoChassisFactory(scene=scene, cache_root=cache, motion_enabled=True,
    spawn=(0., 0., 0.), robot=DifferentialDrive(DifferentialDriveConfig(wheel_track=.4)),
    sensors=SensorConfig(width=80, height=60))
provider = factory.shared(key=key, transport=TransportConfig(domain_id=int(domain)), log_path=log)
async def run():
    async with Runtime(provider.state):
        state = await provider.state.get(timeout=30)
        worker = provider.manifest['pid']
        cmdline = Path('/proc') / str(worker) / 'cmdline'
        evidence = dict(executable=sys.executable, sys_path=sys.path, pid=os.getpid(),
            worker_pid=worker, worker_command=cmdline.read_bytes().decode().split('\0')[:-1]
                if cmdline.exists() else None,
            manifest=provider.manifest, source_clock=state.clock,
            pyrobo_available=importlib.util.find_spec('pyrobo') is not None,
            fingerprint=factory.fingerprint())
        Path(ready).write_text(json.dumps(evidence, indent=2))
        await asyncio.Event().wait()
try:
    asyncio.run(run())
except KeyboardInterrupt:
    pass
'''


def run(args):
    output = args.output.absolute()
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, MUJOCO_GL='egl')
    environment.pop('PYTHONPATH', None)
    provider_python = str(args.provider_python.absolute())
    consumer_python = str(args.consumer_python.absolute())
    checks = ((provider_python, ('pyrobo',)), (consumer_python, ('mujoco', 'mujocosim')))
    for executable, forbidden in checks:
        code = ('import importlib.util; '
                f'assert all(importlib.util.find_spec(n) is None for n in {forbidden!r})')
        subprocess.run([executable, '-c', code], cwd=output, env=environment, check=True)
    reports = []
    for execution in args.execution:
        destination = output / execution
        destination.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='rsim-navigation-') as directory:
            directory = Path(directory)
            scene = directory / 'room.json'
            scene.write_text(json.dumps(dict(format='mujocosim-scene-v1', room=dict(size=[6, 6, 2.6]),
                                             spawn=[0, 0, 0], objects=[])))
            ready = directory / 'ready.json'
            key = 'navigation:' + uuid.uuid4().hex
            command = [provider_python, '-c', PROVIDER, str(scene), str(directory / 'cache'),
                       key, str(args.domain), str(ready), str(destination / 'worker.log')]
            with (destination / 'provider.log').open('w') as log:
                provider = subprocess.Popen(command, cwd=output, env=environment, stdout=log,
                                            stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 40
                    while not ready.exists():
                        if provider.poll() is not None:
                            raise RuntimeError(f'provider exited; see {destination / "provider.log"}')
                        if time.monotonic() > deadline:
                            raise TimeoutError('provider not ready within 40 seconds')
                        time.sleep(.05)
                    provider_info = json.loads(ready.read_text())
                    worker_command = provider_info['worker_command']
                    if worker_command is not None:
                        assert worker_command[0] == provider_python, worker_command
                    consumer_command = [consumer_python, '-m', 'pyrobo.apps.navigation_provider',
                        '--key', key, '--clock', 'simulation', '--domain', str(args.domain),
                        '--execution', execution, '--duration', '25',
                        '--output', str(destination / 'navigation.json')]
                    result = subprocess.run(consumer_command, cwd=output, env=environment,
                                            capture_output=True, text=True, timeout=65)
                    (destination / 'consumer.log').write_text(result.stdout + result.stderr)
                    (destination / 'provider.json').write_text(json.dumps(provider_info, indent=2))
                    result.check_returncode()
                    report = json.loads((destination / 'navigation.json').read_text())
                    assert report['arrived'] and not report['collided']
                    assert not report['environment']['mujoco_available']
                    assert not report['environment']['mujocosim_available']
                    reports.append(dict(execution=execution, arrived=True, collided=False,
                        provider_python=provider_python, consumer_python=consumer_python,
                        minimum_boundary_clearance_m=report['minimum_boundary_clearance_m'],
                        host_elapsed_s=report['elapsed_host_s'],
                        algorithm_pids=report['pids'], worker_pid=provider_info['worker_pid']))
                finally:
                    provider.send_signal(signal.SIGINT)
                    try:
                        provider.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        provider.kill()
                        provider.wait()
                        raise RuntimeError('provider did not cleanly stop')
    (output / 'result.json').write_text(json.dumps(reports, indent=2) + '\n')
    print(json.dumps(reports, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--provider-python', required=True, type=Path)
    parser.add_argument('--consumer-python', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=64)
    parser.add_argument('--execution', choices=('local', 'thread', 'process'), action='append')
    args = parser.parse_args()
    args.execution = args.execution or ['thread', 'process']
    run(args)
