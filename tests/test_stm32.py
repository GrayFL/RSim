"""Exercise the real native driver against a PTY; never opens a hardware port."""
import asyncio
import os
import pty
import struct
import threading
import time
import uuid

import pytest

from rsim import Runtime, VelocityCommand, ProcessPlacement
from rsim.core.commands import CommandRejected
from rsim.drivers import STM32


class FakeBoard:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.running = self.feedback = True
        self.estop = False
        self.commands = []
        self.thread = threading.Thread(target=self.run, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.running = False
        self.thread.join()
        os.close(self.master)
        os.close(self.slave)

    def run(self):
        pending = bytearray()
        counter, next_frame = 0, 0
        while self.running:
            now = time.monotonic()
            if self.feedback and now >= next_frame:
                payload = bytearray(136)
                struct.pack_into('<H', payload, 0, counter % 65536)
                payload[4], payload[34] = 5, int(self.estop)
                struct.pack_into('<3f', payload, 88, 1., 2., .3)
                os.write(self.master, b'@@\x88\x00' + payload +
                         struct.pack('<H', sum(payload) % 65536) + b'##')
                counter += 1
                next_frame = now + .02
            try:
                pending.extend(os.read(self.master, 4096))
            except BlockingIOError:
                pass
            while len(pending) >= 72:
                if pending[:4] != b'@@\x40\x00':
                    del pending[0]
                    continue
                data = pending[:72]
                del pending[:72]
                assert data[-2:] == b'##'
                assert sum(data[4:68]) % 65536 == struct.unpack_from('<H', data, 68)[0]
                # All configuration/reset/LED flags must remain clear.
                assert data[7:28] == bytes(21)
                self.commands.append((time.monotonic(), struct.unpack_from('<f', data, 28)[0],
                                      struct.unpack_from('<f', data, 48)[0]))
            time.sleep(.001)


def native_available():
    pytest.importorskip('rsim_stm32.srv')
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    try:
        get_package_prefix('rsim_stm32')
    except PackageNotFoundError:
        pytest.skip('build and source the optional rsim_stm32 package')


def test_native_serial_watchdog_ignores_blocked_python_and_stale_feedback():
    native_available()

    async def run(board):
        chassis = STM32(board.port, namespace='/test_' + uuid.uuid4().hex, motion_enabled=True)
        async with Runtime(chassis):
            data = (await chassis.odom.get(timeout=5)).data
            assert data['pose']['pose']['position']['x'] == 1.
            state = await chassis.state.get(timeout=5)
            async with asyncio.timeout(5):
                while not state.data['normal']:
                    state = await chassis.state.get(after=state.sequence)
            started = time.monotonic()
            await chassis.velocity.set(VelocityCommand(angular_z=.12), ttl=.12)
            # The Python executor and command sink watchdog are both blocked.
            time.sleep(.3)
            segment = [row for row in board.commands if row[0] >= started]
            nonzero = [row for row in segment if row[2] > .1]
            assert nonzero
            assert nonzero[-1][0] < started + .15
            assert segment[-1][1:] == (0., 0.)
            assert max(b[0] - a[0] for a, b in zip(segment, segment[1:])) < .06
            await asyncio.sleep(.05)
            await chassis.velocity.set(VelocityCommand(angular_z=.12), ttl=.45)
            lost = time.monotonic()
            board.feedback = False
            await asyncio.sleep(.28)
            assert board.commands[-1][1:] == (0., 0.)
            assert max(row[0] for row in board.commands if row[2] > .1) < lost + .24
            board.feedback = True
            await asyncio.sleep(.1)
            assert board.commands[-1][1:] == (0., 0.)  # recovery cannot replay
            board.estop = True
            await asyncio.sleep(.05)
            with pytest.raises(CommandRejected):
                await chassis.velocity.set(VelocityCommand(angular_z=.1), ttl=.2)
        assert board.commands[-1][1:] == (0., 0.)

    with FakeBoard() as board:
        asyncio.run(run(board))


def test_native_defaults_to_zero_and_preserves_native_remaps():
    native_available()

    async def run(board):
        chassis = STM32(board.port, namespace='/test_' + uuid.uuid4().hex,
                        ros_args=['--ros-args', '-r', 'odom:=wheel', '-p', 'base_frame:=body'])
        assert chassis.topics['odom'].endswith('/wheel')
        async with Runtime(chassis):
            assert (await chassis.odom.get(timeout=5)).data['child_frame_id'] == 'body'
            await chassis.velocity.set(VelocityCommand(), ttl=.2)
            with pytest.raises(CommandRejected):
                await chassis.velocity.set(VelocityCommand(angular_z=.1), ttl=.2)
        assert board.commands and all(row[1:] == (0., 0.) for row in board.commands)

    with FakeBoard() as board:
        asyncio.run(run(board))


def test_cli_passes_native_parameters():
    pytest.importorskip('rclpy')
    from rsim.drivers.cli import _parse_args, _sensor_from_args
    args = _parse_args(['stm32', '--port', '/dev/example', '--baudrate', '460800',
                        '--ros-args', '-p', 'base_frame:=body', '-r', 'odom:=wheel'])
    chassis = _sensor_from_args(args)
    assert chassis.topics['odom'] == '/rsim/chassis/wheel'
    assert chassis.children[1].options.parameters['baudrate'] == 460800


def test_shared_chassis_rejects_conflicting_native_settings_and_shares_ros_context():
    pytest.importorskip('rclpy')
    from rsim.adapters.ros2.context import RosContext
    a = STM32('/dev/example', motion_enabled=False)
    b = STM32('/dev/example', motion_enabled=True)
    runtime = Runtime(a, b)
    try:
        with pytest.raises(ValueError, match='conflicting source configuration'):
            runtime._resolve()
    finally:
        runtime._release_bindings()
    context = RosContext()
    runtime = Runtime(a, context)
    try:
        runtime._resolve()
        assert len([item for item in runtime._order if isinstance(item, RosContext)]) == 1
    finally:
        runtime._release_bindings()


def test_native_provider_can_run_in_a_managed_child_process():
    native_available()

    async def run(board):
        chassis = STM32(board.port, namespace='/test_' + uuid.uuid4().hex, motion_enabled=True)
        async with Runtime(chassis, chassis.velocity, placement={chassis: ProcessPlacement('native-chassis-test')}):
            frame = await chassis.odom.get(timeout=10)
            assert frame.data['pose']['pose']['position']['x'] == 1.
            await chassis.velocity.set(VelocityCommand(angular_z=.1), ttl=.3)
            assert any(row[2] > .09 for row in board.commands)
            await chassis.velocity.set(VelocityCommand(), ttl=.3)
        assert board.commands[-1][1:] == (0., 0.)

    with FakeBoard() as board:
        asyncio.run(run(board))
