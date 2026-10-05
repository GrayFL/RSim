import asyncio
from binascii import crc_hqx
import math
import os
import pty
import struct

import pytest

pytest.importorskip("serial")
pytest.importorskip("graphmap.pose")

from rsim import ComponentError, Runtime
from rsim.imu import HipnucDecoder, SerialIMU, imu_message


def packet(payload):
    header = b"\x5a\xa5" + struct.pack("<H", len(payload))
    return header + struct.pack("<H", crc_hqx(payload, crc_hqx(header, 0))) + payload


LEGACY = (b"\x90\x01\xa0" + struct.pack("<3h", 100, -200, 1000) +
          b"\xb0" + struct.pack("<3h", 900, -450, 0) +
          b"\xc0" + struct.pack("<3h", 100, 200, 300) +
          b"\xd0" + struct.pack("<3h", 0, 0, 900) + b"\xf0" + struct.pack("<f", 0))


def test_legacy_crc_fragmentation_units_and_euler_order():
    from graphmap.pose import Pose
    decoder = HipnucDecoder()
    raw = packet(LEGACY)
    broken = bytearray(raw)
    broken[10] ^= 1
    result = []
    for byte in b"noise\x5a" + broken + raw + raw:
        result += decoder.feed(bytes([byte]))
    assert len(result) == 2 and decoder.crc_errors == 1
    assert result[0]["acceleration"] == pytest.approx([.98, -1.96, 9.8])
    assert result[0]["angular_velocity"] == pytest.approx([math.pi/2, -math.pi/4, 0])
    assert result[0]["magnetic_field"] == pytest.approx([1e-5, 2e-5, 3e-5])
    assert result[0]["orientation"] == pytest.approx(Pose(yaw=90).quat)
    # Pitch and roll have their own order/scale, not yaw's 0.1 degree units.
    tilted = decoder.feed(packet(b"\xd0" + struct.pack("<3h", 1000, 2000, -300)))[0]
    assert tilted["orientation"] == pytest.approx(Pose(roll=20, pitch=10, yaw=-30).quat)


def test_imusol_missing_data_invalid_and_bounded_buffer():
    decoder = HipnucDecoder()
    payload = struct.pack("<BB6xI16f", 0x91, 7, 1234,
                          0, 0, 1, 180, 0, 0, 10, 20, 30, 0, 0, 0, 1, 0, 0, 0)
    frame = decoder.feed(packet(payload))[0]
    assert frame["device_timestamp_ms"] == 1234
    assert frame["acceleration"] == [0, 0, 9.8]
    assert frame["angular_velocity"][0] == pytest.approx(math.pi)
    assert frame["orientation"] == [0, 0, 0, 1]
    only_acc = decoder.feed(packet(b"\xa0" + struct.pack("<3h", 0, 0, 1000)))[0]
    msg = imu_message(only_acc, "imu")
    assert msg["orientation_covariance"][0] == -1
    assert msg["angular_velocity_covariance"][0] == -1
    assert msg["linear_acceleration_covariance"][0] == 0
    assert "device_timestamp_ms" not in only_acc
    for bad in (b"\xd1" + struct.pack("<4f", 0, 0, 0, 0), b"\xa0\x01", b"\xff"):
        assert decoder.feed(packet(bad)) == []
    assert decoder.invalid_payloads == 3
    decoder.feed(b"\x5a\xa5\xff\xff" * 100000)
    assert len(decoder.buffer) <= 1030
    assert len(decoder.feed(packet(LEGACY))) == 1


def test_serial_async_pty_lock_timeout_and_cleanup():
    async def run():
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        source = SerialIMU(port, timeout=.15)
        try:
            async with Runtime(source):
                os.write(master, packet(LEGACY))
                frame = await source.imu.get(timeout=2)
                assert frame.clock == "host:monotonic"
                assert await source.imu.get(timestamp_ns=frame.stamp_ns, clock=frame.clock) is frame
                other = SerialIMU(port)
                with pytest.raises(ComponentError, match="physical device"):
                    async with Runtime(other):
                        pass
                with pytest.raises(ComponentError):
                    await source.imu.get(after=frame.sequence, timeout=1)
            assert source.serial is None and source._lease is None
            async with Runtime(SerialIMU(port)):
                pass  # The failed source released its port.
        finally:
            os.close(master)
            os.close(slave)
    asyncio.run(run())


@pytest.mark.parametrize("mode", ["serial", "ros2"])
def test_shared_provider_fanout_and_ros_parameters(mode, tmp_path):
    if mode == "ros2":
        pytest.importorskip("rclpy")
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
        try:
            get_package_prefix("rsim_hipnuc")
        except PackageNotFoundError:
            pytest.skip("optional ROS2 serial package not built")
    from rsim.drivers import Hipnuc
    from rsim.devices import Hipnuc as Client

    async def run():
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        async def send():
            while True:
                os.write(master, packet(LEGACY))
                await asyncio.sleep(.01)
        writer = asyncio.create_task(send())
        try:
            options = dict(parameters={"frame_id": "imu_test"})
            if mode == "ros2":
                options["ros_args"] = ["--ros-args", "-r", "imu/data:=/test/hipnuc"]
            provider = Hipnuc(port, mode=mode, history=16, log_path=tmp_path/"driver.log", **options)
            client = Client(port, history=16)
            async with Runtime(provider, client):
                a, b = await asyncio.gather(provider.get(timeout=15), client.get(timeout=15))
                assert a.data["header"]["frame_id"] == b.data["header"]["frame_id"] == "imu_test"
                assert a.data["linear_acceleration"]["z"] == pytest.approx(9.8)
                assert provider.worker_pid == client.worker_pid
                assert a.clock == ("host:monotonic" if mode == "serial" else "ros:system")
        finally:
            writer.cancel()
            await asyncio.gather(writer, return_exceptions=True)
            os.close(master)
            os.close(slave)
    asyncio.run(run())
