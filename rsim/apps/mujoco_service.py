"""Serve MuJoCo physics in a shared worker and chassis control over Cyclone DDS."""
import argparse
import asyncio
import json
import os
from pathlib import Path

from rsim.runtime import Runtime
from rsim.runtime.sharing import SharedProvider
from rsim.transport.descriptor import TransportConfig
from rsim.drivers import MuJoCo, Chassis as Provider
from rsim.adapters.mujoco.config import SensorConfig


async def run(args):
    transport = TransportConfig('cyclonedds', args.domain)
    source = MuJoCo(args.scene, cache_root=args.cache_dir, motion_enabled=args.enable_motion,
                    sensors=SensorConfig(width=args.width, height=args.height))
    shared = SharedProvider(source, key='mujoco:' + args.name, interface_version='mujoco-v1',
                            provider_version=str((str(Path(args.scene).resolve()), args.width, args.height,
                                                  args.enable_motion)), transport=transport,
                            log_path=args.log)
    service = Provider(name=args.name, pose=shared.pose, velocity=shared.velocity, state=shared.state,
                       motion_enabled=args.enable_motion, transport=transport)
    async with Runtime(service) as runtime:
        await service.pose.get(timeout=30)
        status = dict(name=args.name, domain=args.domain, service_pid=os.getpid(),
                      physics_pid=shared.manifest['pid'], storage=str(shared.directory),
                      motion_enabled=args.enable_motion)
        if args.ready_file:
            path = Path(args.ready_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(status, indent=2), encoding='utf-8')
        print(json.dumps(status, indent=2), flush=True)
        if args.seconds is None:
            await runtime.wait()
        else:
            try:
                async with asyncio.timeout(args.seconds):
                    await runtime.wait()
            except TimeoutError:
                pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--name', default='mujoco')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--cache-dir', default='assets/mujoco/scenes')
    parser.add_argument('--log', default='assets/mujoco/provider.log')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--seconds', type=float, help='Optional service lifetime after ready')
    parser.add_argument('--ready-file', help='Write service and physics PIDs after startup')
    args = parser.parse_args(argv)
    if args.seconds is not None and (not __import__('math').isfinite(args.seconds) or args.seconds <= 0):
        parser.error('--seconds must be finite and positive')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
