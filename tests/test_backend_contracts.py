"""One application against memory/MuJoCo, with encoder/IMU estimated pose."""
import asyncio
import time
import uuid

import pytest
from rsim import Runtime, VelocityCommand
from rsim.core.contracts import validate_imu, validate_odometry
from rsim.devices.ports import ChassisPorts


async def application(client):
    pose = await client.pose.get(timeout=3)
    await client.move(0)
    await client.rotate(yaw_rad=0)
    await client.drive(VelocityCommand(), ttl=.1)
    await client.stop()
    return pose


@pytest.mark.parametrize('backend', ['memory', 'mujoco'])
def test_same_zero_control_application_and_si_contract(backend, tmp_path):
    pytest.importorskip('graphmap')
    from rsim.components.odometry import PlanarOdometry
    from rsim.devices import Chassis
    from rsim.drivers import Chassis as ChassisService
    from rsim.transport.descriptor import TransportConfig
    if backend == 'memory':
        from rsim.components.simulated_chassis import SimulatedChassis
        source = SimulatedChassis(noise=False)
        state = source.feedback
    else:
        pytest.importorskip('mujoco')
        from test_mujoco_physics import make_room
        from rsim.drivers import MuJoCo
        make_room(tmp_path)
        source = MuJoCo(scene_dir=tmp_path)
        state = source.state
    estimate = PlanarOdometry(source.odom, source.imu)
    ports = ChassisPorts(estimate.pose, source.velocity, state)
    async def run():
        name = 'contract_' + uuid.uuid4().hex
        config = TransportConfig('inprocess')
        service = ChassisService(name=name, pose=ports.pose, velocity=ports.velocity, state=ports.state, transport=config)
        client = Chassis(name, transport=config)
        async with Runtime(service, client):
            pose = await application(client)
            assert pose.data.wrd_frame == 'odom'
            assert (await estimate.pose.get()).metadata['pose_kind'] == 'estimate'
            imu = await source.imu.get()
            validate_imu(imu.data, gravity_removed=imu.metadata['gravity_removed'])
            validate_odometry((await source.odom.get()).data, pose_kind='estimate')
            if backend == 'mujoco':
                assert not source.image.frames
    asyncio.run(run())


def test_mujoco_reset_rejects_old_estimator_episode_and_paused_command(tmp_path):
    pytest.importorskip('graphmap')
    pytest.importorskip('mujoco')
    from test_mujoco_physics import make_room
    from rsim.drivers import MuJoCo
    from rsim.components.odometry import PlanarOdometry
    from rsim.core import ComponentError
    make_room(tmp_path)
    async def run():
        source = MuJoCo(scene_dir=tmp_path, motion_enabled=True, speedup=2.)
        estimate = PlanarOdometry(source.odom, source.imu)
        async with Runtime(estimate.pose, source.state):
            await estimate.pose.get(timeout=3)
            await source.velocity.set(VelocityCommand(.1), ttl=.05)
            # Pause the owner thread beyond command lifetime. Resumption must
            # validate host time before stepping, even at accelerated sim speed.
            await source.inspect(lambda sim: time.sleep(.12))
            state = await source.state.get()
            for _ in range(3):
                state = await source.state.get(after=state.sequence, timeout=1)
            assert state.data['wheel_targets'] == [0., 0.]
            before = (await source.odom.get()).metadata['episode']
            await source.reset()
            assert (await source.odom.get()).metadata['episode'] == before + 1
            with pytest.raises(ComponentError):
                await estimate.wait()
    asyncio.run(run())
