import asyncio
import math
import os
import sys
import time
import uuid

import pytest

pytest.importorskip("cyclonedds")
pytest.importorskip("graphmap")
from rsim.core import VelocityCommand, CommandRejected
from rsim.runtime import Runtime
from rsim.runtime.service import RemoteError
from rsim.devices import Chassis
from rsim.drivers import Chassis as Provider
from rsim.transport.descriptor import TransportConfig


def setup(motion=True):
    name = "test_" + uuid.uuid4().hex
    transport = TransportConfig(domain_id=81)
    server = Provider(
        name=name, simulate=True, motion_enabled=motion, transport=transport
    )
    client = Chassis(name, transport=transport)
    return server, client


def test_dds_motion_and_cancellation_release_control():
    async def run():
        server, client = setup()
        other = Chassis(server.name, transport=client.bus.config)
        async with Runtime(server, client, other):
            pose = await client.move(0.04, timeout=5)
            assert pose.position[0] == pytest.approx(0.04, abs=0.013)
            pose = await client.rotate(yaw_deg=10, timeout=5)
            assert math.degrees(pose.euler_rad[2]) == pytest.approx(10, abs=1.5)
            action = asyncio.create_task(client.move(2))
            await asyncio.sleep(0.15)
            with pytest.raises(CommandRejected, match="another client"):
                await other.drive(VelocityCommand(0.05, 0))
            action.cancel()
            with pytest.raises(asyncio.CancelledError):
                await action
            assert server.controller.velocity.producer.command == VelocityCommand()
            assert server.owner is None
            await other.drive(VelocityCommand(0.05, 0))
            await other.stop()
            assert server.owner is None

    asyncio.run(run())


def test_zero_only_replay_and_expired_lease():
    async def run():
        server, client = setup(False)
        async with Runtime(server, client):
            await client.move(0)
            await client.rotate(yaw_rad=0)
            with pytest.raises(RemoteError, match="disabled"):
                await client.move(0.1)
            with pytest.raises(CommandRejected):
                await client.drive(VelocityCommand(0.01, 0))
            await client.drive(VelocityCommand())
            client.owns = False  # lost heartbeat without a graceful release
            await asyncio.sleep(0.4)
            assert server.owner is None
            packet = dict(
                session=client.session,
                sequence=client.sequence,
                deadline_ns=time.monotonic_ns() + 200_000_000,
                op="velocity",
                linear_x=0,
                angular_z=0,
            )
            with pytest.raises(RemoteError, match="replayed"):
                await server.handle(packet)
            packet["sequence"] += 1
            packet["deadline_ns"] = time.monotonic_ns() - 1
            with pytest.raises(RemoteError, match="expired"):
                await server.handle(packet)

    asyncio.run(run())


def test_manual_then_action_does_not_inherit_manual_deadman():
    async def run():
        server, client = setup()
        async with Runtime(server, client):
            await client.drive(VelocityCommand(0.03, 0), ttl=0.2)
            pose = await client.rotate(15, timeout=6)
            assert math.degrees(pose.euler_rad[2]) == pytest.approx(15, abs=1.5)

    asyncio.run(run())


def test_delayed_clock_sample_does_not_shorten_active_lease():
    async def run():
        server, client = setup()
        async with Runtime(server, client):
            request = client.request

            async def delayed(packet, **kw):
                result = await request(packet, **kw)
                if packet["op"] == "hello":
                    await asyncio.sleep(0.22)
                return result

            client.request = delayed
            pose = await client.rotate(15, timeout=5)
            assert math.degrees(pose.euler_rad[2]) == pytest.approx(15, abs=1.5)

    asyncio.run(run())


def test_service_task_failure_stops_inflight_action():
    async def run():
        server, client = setup()
        async with Runtime(server, client):
            action = asyncio.create_task(client.move(3))
            await asyncio.sleep(0.15)

            async def fail():
                raise ValueError("injected service failure")

            server.task("injected", fail, hz=100)
            await asyncio.sleep(0.2)
            assert server._failure is not None
            assert server.controller.velocity.producer.command == VelocityCommand()
            action.cancel()
            await asyncio.gather(action, return_exceptions=True)

    asyncio.run(run())


def test_repeated_telemetry_does_not_refresh_stopped_pose_source():
    from graphmap.pose import Pose
    from rsim.core import Component, CommandSink
    from rsim.components.motion import ChassisController
    from rsim.adapters.control_service import MotionService

    async def run():
        source = Component()
        pose = source.signal("pose")
        commands = []
        sink = CommandSink(
            source,
            "velocity",
            lambda env: commands.append(env.value),
            fallback=VelocityCommand(),
        )
        controller = ChassisController(
            pose=pose, velocity=sink, motion_enabled=True, pose_timeout=0.1
        )
        name = "test_" + uuid.uuid4().hex
        transport = TransportConfig(domain_id=81)
        server = MotionService(controller, name=name, transport=transport)
        client = Chassis(name, transport=transport)
        async with Runtime(server, client):
            await pose.publish(
                Pose(wrd_frame="odom", ego_frame="body"), stamp_ns=1, clock="test"
            )
            first = await client.pose.get(timeout=1)
            with pytest.raises(TimeoutError):
                await client.pose.get(after=first.sequence, timeout=0.2)
            assert (await client.pose.get()).received_ns == first.received_ns
            with pytest.raises(CommandRejected, match="stale"):
                await client.drive(VelocityCommand(0.05, 0))
            assert all(c == VelocityCommand() for c in commands)

    asyncio.run(run())


def test_peer_process_death_expires_motion():
    """Optionally select an independent interpreter with RSIM_TEST_CLIENT_PYTHON."""
    code = """
import sys
from importlib.abc import MetaPathFinder
sys.path[:] = [p for p in sys.path if '/opt/ros/' not in p]
class NoROS(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy','rospy','std_msgs','sensor_msgs','nav_msgs','ament_index_python'} or fullname.startswith('rsim.drivers'):
            raise AssertionError(fullname)
sys.meta_path.insert(0, NoROS())
import asyncio
from rsim.runtime import Runtime
from rsim.devices import Chassis
from rsim.transport.descriptor import TransportConfig
async def main():
    client = Chassis(sys.argv[1], transport=TransportConfig(domain_id=81))
    async with Runtime(client):
        print('ready', flush=True)
        await client.move(3)
asyncio.run(main())
"""

    async def run():
        server, _ = setup()
        source = server.controller.velocity.producer
        async with Runtime(server):
            child = await asyncio.create_subprocess_exec(
                os.environ.get("RSIM_TEST_CLIENT_PYTHON", sys.executable),
                "-c",
                code,
                server.name,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            try:
                async with asyncio.timeout(10):
                    assert await child.stdout.readline() == b"ready\n"
                    while source.command.linear_x == 0:
                        await asyncio.sleep(0.01)
                child.kill()
                await child.wait()
                async with asyncio.timeout(0.6):
                    while (
                        source.command != VelocityCommand() or server.owner is not None
                    ):
                        await asyncio.sleep(0.01)
                stopped = source.state.copy()
                await asyncio.sleep(0.15)
                assert (source.state == stopped).all()
            finally:
                if child.returncode is None:
                    child.kill()
                    await child.wait()

    asyncio.run(run())
