"""A device-level motion routine that also works with a real chassis provider."""
import argparse
import asyncio
import json
import os
import sys
import uuid
from pathlib import Path

from rsim.runtime import Runtime
from rsim.devices import Chassis


async def exercise(chassis):
    """Backend-independent application: only the public device API is used."""
    start = await chassis.pose.get(timeout=3)
    moved = await chassis.move(.2, timeout=10)
    turned = await chassis.rotate(yaw_deg=30, timeout=10)
    returned = await chassis.move(-.1, timeout=10)
    await chassis.stop()
    return {'start': start.data.position.tolist(), 'moved': moved.position.tolist(),
            'turned_yaw_rad': float(turned.euler_rad[2]), 'final': returned.position.tolist()}


async def run(args):
    if args.connect:
        return await run_connected(args)
    # Only this deployment assembly knows the backend; exercise() does not.
    from rsim.drivers import MuJoCo, Chassis as Provider
    from rsim.transport.descriptor import TransportConfig
    source = MuJoCo(args.scene, cache_root=args.cache_dir, motion_enabled=True)
    transport = TransportConfig('inprocess')
    name = 'mujoco_motion_' + uuid.uuid4().hex
    provider = Provider(name=name, pose=source.pose, velocity=source.velocity, state=source.state,
                        transport=transport, motion_enabled=True)
    chassis = Chassis(name, transport=transport)
    async with Runtime(provider, chassis):
        result = await exercise(chassis)
        if args.output:
            output = Path(args.output)
            await source.inspect(lambda sim: sim.save_snapshot(output))
            (output / 'motion_summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2), flush=True)
    return result


async def run_connected(args):
    # This path imports neither the driver nor MuJoCo and can use another Python.
    from rsim.transport.descriptor import TransportConfig
    from rsim.runtime.sharing import SharedComponent, PortSpec
    from rsim.apps.mujoco_recording import snapshot
    transport = TransportConfig('cyclonedds', args.domain)
    chassis = Chassis(args.connect, transport=transport)
    sensors = SharedComponent(key='mujoco:' + args.connect, interface_version='mujoco-v1',
        ports={name: PortSpec(history_capacity=8) for name in ('image', 'depth', 'camera_info', 'scan')},
        transport=transport)
    # Warm dynamic RGBD before acquiring a motion lease.
    async with Runtime(chassis, sensors):
        await sensors.image.get(timeout=30)
        result = await exercise(chassis)
        result.update(client_pid=os.getpid(), physics_pid=sensors.manifest['pid'], python=sys.version,
                      transport='cyclonedds')
        if args.output:
            result.update(await snapshot(sensors, args.output))
            (Path(args.output) / 'motion_summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2), flush=True)
    return result


def main(argv=None, *, default_scene=None, default_cache='assets/mujoco/scenes',
         default_output='assets/mujoco/motion'):
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group()
    target.add_argument('--scene', default=default_scene)
    target.add_argument('--connect', help='Connect to an existing mujoco_service by name over DDS')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--cache-dir', default=default_cache)
    parser.add_argument('--output', default=default_output)
    args = parser.parse_args(argv)
    if not args.scene and not args.connect:
        parser.error('supply --scene or --connect')
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
