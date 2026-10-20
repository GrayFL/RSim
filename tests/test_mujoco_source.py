"""Real physics through RSim streams, command guards and unchanged device API."""
import asyncio
import math
import uuid

import numpy as np
import pytest

pytest.importorskip('mujoco')
pytest.importorskip('graphmap')
from rsim import Runtime
from rsim.core import VelocityCommand, CommandRejected
from rsim.drivers import MuJoCo, Chassis as Provider
from rsim.devices import Chassis
from rsim.components.motion import ChassisController
from rsim.adapters.mujoco.config import SensorConfig
from rsim.transport.descriptor import TransportConfig
from test_mujoco_physics import make_room


def source(tmp_path, **options):
    make_room(tmp_path)
    return MuJoCo(scene_dir=tmp_path, sensors=SensorConfig(width=80, height=60), **options)


def test_streams_encoder_odometry_reset_and_deadman(tmp_path):
    async def run():
        sim = source(tmp_path, motion_enabled=True)
        async with Runtime(sim):
            image = await sim.image.get(timeout=2)
            depth = await sim.depth.get(timeout=2)
            scan = await sim.scan.get(timeout=2)
            assert image.stamp_ns == depth.stamp_ns == scan.stamp_ns
            assert image.clock.startswith('mujoco:')
            assert image.data.pixels.shape == (60, 80, 3)
            assert not image.data.pixels.flags.writeable
            assert not depth.data.pixels.flags.writeable
            assert not scan.data['ranges'].flags.writeable
            saved = image.data.pixels.copy()
            cloud = await sim.points.get(timeout=2)
            assert cloud.data.points.dtype.names == ('x', 'y', 'z', 'r', 'g', 'b')
            await sim.velocity.set(VelocityCommand(.2, 0), ttl=.15)
            await asyncio.sleep(.3)
            state = await sim.state.get()
            assert state.data['wheel_targets'] == [0., 0.]
            odom = await sim.odom.get()
            assert odom.data['pose']['pose']['position']['x'] > .005
            imu = await sim.imu.get()
            assert imu.data['linear_acceleration']['z'] == pytest.approx(9.81, abs=.5)
            np.testing.assert_array_equal(image.data.pixels, saved)
            before = state.stamp_ns
            await sim.reset()
            assert (await sim.state.get()).stamp_ns > before
            assert (await sim.odom.get()).data['pose']['pose']['position']['x'] == 0
        assert sim.command == VelocityCommand()
        np.testing.assert_equal(sim.simulation.data.ctrl, [0., 0.])
        assert sim._executor is None
    asyncio.run(run())


def test_disabled_motion_and_no_demanded_renderer(tmp_path):
    async def run():
        sim = source(tmp_path)
        async with Runtime(sim.state):
            with pytest.raises(CommandRejected, match='motion_enabled'):
                await sim.velocity.set(VelocityCommand(.1, 0))
            assert not sim.image.frames
            assert not sim.scan.frames
            await sim.velocity.set(VelocityCommand())
    asyncio.run(run())


def test_existing_device_move_rotate_cancel_and_exclusive_owner(tmp_path):
    async def run():
        sim = source(tmp_path, motion_enabled=True)
        transport = TransportConfig('inprocess')
        name = 'mujoco_test_' + uuid.uuid4().hex
        server = Provider(name=name, pose=sim.pose, velocity=sim.velocity,
                          state=sim.state, motion_enabled=True, transport=transport)
        client = Chassis(name, transport=transport)
        other = Chassis(name, transport=transport)
        async with Runtime(server, client, other):
            start = (await client.pose.get(timeout=2)).data
            end = await client.move(.08, timeout=5)
            assert end.position[0]-start.position[0] == pytest.approx(.08, abs=.015)
            end = await client.rotate(yaw_deg=12, timeout=5)
            assert math.degrees(end.euler_rad[2]) == pytest.approx(12, abs=2)
            action = asyncio.create_task(client.move(1., timeout=10))
            await asyncio.sleep(.15)
            with pytest.raises(CommandRejected, match='another client'):
                await other.drive(VelocityCommand(.1, 0))
            action.cancel()
            with pytest.raises(asyncio.CancelledError):
                await action
            assert sim.command == VelocityCommand()
            assert server.owner is None
            await other.drive(VelocityCommand(.1, 0), ttl=.1)
            await asyncio.sleep(.2)
            assert sim.command == VelocityCommand()
            await other.stop()
        assert sim._executor is None
    asyncio.run(run())


def test_existing_planar_estimator_consumes_encoder_and_imu(tmp_path):
    async def run():
        sim = source(tmp_path, motion_enabled=True)
        control = ChassisController(sim, motion_enabled=True)
        async with Runtime(control):
            pose = await control.move(.05, timeout=5)
            assert pose.wrd_frame == 'odom'
            assert pose.position[0] == pytest.approx(.05, abs=.015)
            assert sim.command == VelocityCommand()
    asyncio.run(run())


def test_shared_source_late_sensor_subscription_and_remote_commands(tmp_path):
    pytest.importorskip('cyclonedds')
    from rsim.runtime.sharing import SharedProvider, SharedComponent, component_ports
    async def run():
        local = source(tmp_path, motion_enabled=True)
        key = 'mujoco_shared_' + uuid.uuid4().hex
        provider = SharedProvider(local, key=key)
        remote = SharedComponent(key=key, ports=component_ports(local))
        async with Runtime(provider, _root_ports=False):
            assert provider.bindings == {}
            async with Runtime(remote.image, remote.depth, remote.scan, remote.state, remote.velocity):
                rgb = await remote.image.get(timeout=15)
                assert isinstance(rgb.data.pixels, np.memmap)
                assert rgb.data.pixels.shape == (60, 80, 3)
                assert not rgb.data.pixels.flags.writeable
                scan = await remote.scan.get(timeout=3)
                assert len(scan.data['ranges']) == 360
                await remote.velocity.set(VelocityCommand(.1, 0), ttl=.15)
                await asyncio.sleep(.35)
                assert (await remote.state.get()).data['wheel_targets'] == [0., 0.]
                assert provider.manifest['pid'] != __import__('os').getpid()
        assert rgb.data.pixels.shape == (60, 80, 3)
    asyncio.run(run())
