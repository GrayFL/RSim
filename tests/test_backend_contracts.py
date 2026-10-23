"""In-memory control and SI contract. Concrete MuJoCo robots live in MujocoSim."""
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


def test_same_zero_control_application_and_si_contract():
    pytest.importorskip('graphmap')
    from rsim.components.odometry import PlanarOdometry
    from rsim.devices import Chassis
    from rsim.drivers import Chassis as ChassisService
    from rsim.transport.descriptor import TransportConfig
    from rsim.components.simulated_chassis import SimulatedChassis
    source = SimulatedChassis(noise=False)
    state = source.feedback
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
    asyncio.run(run())
