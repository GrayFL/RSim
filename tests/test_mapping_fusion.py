import asyncio
from types import SimpleNamespace

import numpy as np
import pytest

from rsim.components.synchronization import SampleHistory


def test_delayed_samples_interpolate_on_acquisition_time_without_extrapolation():
    history = SampleHistory(max_gap_s=.15)
    history.add(1_000_000_000, [1., 3.])
    assert history.at(1_050_000_000) is None
    history.add(1_100_000_000, [3., 5.])
    np.testing.assert_allclose(history.at(1_050_000_000), [2., 4.])
    assert not history.add(1_050_000_000, [10., 10.])
    history.add(1_300_000_000, [5., 7.])
    assert history.at(1_200_000_000) is None
    assert history.at(999_999_999) is None
    with pytest.raises(ValueError, match='finite'):
        history.add(1_400_000_000, [np.nan, 0.])


def fusion(tmp_path):
    from graphmap.pose import Pose
    from rsim.core import Component
    from rsim.adapters.ros2 import RosContext
    from rsim.adapters.ros2.mapping_fusion import MappingFusion
    from rsim.components.projection import PoseHistory
    scan = Component()
    scan.local, scan.odom_frame = '/test/scan2d', 'scan_odom'
    return MappingFusion(RosContext(), scan, prefix='/test', directory=tmp_path,
        frames=dict(odom='fused_odom', base='base', map='map'),
        mount=Pose(rotation=[90, 0, 0], degrees=True, wrd_frame='base', ego_frame='imu'),
        history=PoseHistory(), covariance_history=SampleHistory(),
        wheel_history=SampleHistory(), gyro_history=SampleHistory())


def stamp(message, ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)


def test_fusion_pairs_old_scan_with_interpolated_wheel_and_mounted_gyro(tmp_path):
    messages = pytest.importorskip('nav_msgs.msg')
    Imu = pytest.importorskip('sensor_msgs.msg').Imu
    c = fusion(tmp_path)
    published = []
    c.publisher = SimpleNamespace(publish=published.append)
    pose = messages.Odometry()
    stamp(pose, 1_050_000_000)
    pose.header.frame_id, pose.child_frame_id = 'scan_odom', 'base'
    pose.pose.pose.orientation.w = 1.
    pose.pose.pose.position.x = 5.
    pose.pose.covariance = np.diag([.001]*6).ravel().tolist()
    c.scan_pose(pose)  # Decades-old wall time still waits for acquisition brackets.
    asyncio.run(c.flush())
    assert not published and len(c.pending) == 1
    for t, speed in [(1_000_000_000, 1.), (1_100_000_000, 3.)]:
        wheel = messages.Odometry()
        stamp(wheel, t)
        wheel.child_frame_id = 'base'
        wheel.pose.pose.position.x = 999.  # Unrelated wheel origin must be ignored.
        wheel.twist.twist.linear.x = speed
        wheel.twist.covariance = np.diag([.001]*6).ravel().tolist()
        c.wheel(wheel)
        imu = Imu()
        stamp(imu, t)
        imu.header.frame_id = 'imu'
        imu.angular_velocity.y = speed  # Mount maps IMU +Y to body +Z.
        imu.angular_velocity_covariance = np.diag([.001]*3).ravel().tolist()
        c.imu(imu)
    asyncio.run(c.flush())
    out, = published
    assert out.header.stamp == pose.header.stamp and out.header.frame_id == 'fused_odom'
    assert out.pose.pose.position.x == 5.
    assert out.twist.twist.linear.x == pytest.approx(2.)
    assert out.twist.twist.angular.z == pytest.approx(2.)
    assert np.linalg.eigvalsh(np.array(out.twist.covariance).reshape(6, 6)).min() >= 0
    assert pose.header.frame_id == 'scan_odom'  # Input was not mutated.


def test_mapping_public_pose_uses_fusion_frame_while_colorizer_retains_lio_frame(tmp_path):
    from graphmap.pose import Pose
    from rsim.core import Component
    from rsim.adapters.ros2.mapping_output import MappingOutput
    Odometry = pytest.importorskip('nav_msgs.msg').Odometry
    inputs = Component()
    inputs.ros, inputs.prefix = None, '/test'
    observations = []
    output = MappingOutput(inputs, frames=dict(odom='fused', lio='lio', base='base', map='map'),
        T_base_imu=Pose(wrd_frame='base', ego_frame='imu'), database='test.db', assumptions={},
        mapping=SimpleNamespace(observe_pose=lambda t, p: observations.append(p)))
    native, filtered = Odometry(), Odometry()
    for msg in [native, filtered]:
        stamp(msg, 1_000_000_000)
        msg.pose.pose.orientation.w = 1.
    native.pose.pose.position.x, filtered.pose.pose.position.x = 10., 2.
    filtered.header.frame_id, filtered.child_frame_id = 'fused', 'base'
    output.fusion = SimpleNamespace(latest=filtered)
    output.odom_publisher = SimpleNamespace(publish=lambda msg: None)
    output.broadcaster = SimpleNamespace(sendTransform=lambda msg: None)
    output.odometry_queue.append(native)
    output._closed = False
    asyncio.run(output.convert())
    assert observations[0].wrd_frame == 'lio' and observations[0].position[0] == 10.
    assert output.latest['odometry'].data.wrd_frame == 'fused'
    assert output.latest['odometry'].data.position[0] == 2.


def test_rtab_rectification_leaves_raw_source_untouched():
    from rsim.adapters.ros2.laser_mapping import LaserMappingIO
    msgs = pytest.importorskip('sensor_msgs.msg')
    pytest.importorskip('cv2')
    image, info = msgs.Image(), msgs.CameraInfo()
    image.width = info.width = 8
    image.height = info.height = 6
    image.encoding, image.step = 'rgb8', 24
    data = np.arange(144, dtype=np.uint8).reshape(6, 8, 3)
    image.data = data.tobytes()
    info.k = [4., 0., 4., 0., 4., 3., 0., 0., 1.]
    info.d, info.distortion_model = [.2, 0., 0., 0., 0.], 'plumb_bob'
    io = object.__new__(LaserMappingIO)
    io.rectification = None
    rectified, calibration = io.rectify(image, info)
    assert bytes(image.data) == data.tobytes() and info.d[0] == .2
    assert not any(calibration.d)
    assert bytes(rectified.data) != bytes(image.data)
    np.testing.assert_array_equal(np.array(calibration.p).reshape(3, 4)[:, :3], np.array(info.k).reshape(3, 3))


def test_archive_thread_finishes_before_cancelled_task_allows_cleanup():
    import threading
    from rsim.adapters.ros2.laser_mapping import LaserMappingIO
    entered, release = threading.Event(), threading.Event()
    events = []

    def write():
        entered.set()
        release.wait(2)
        events.append('write-finished')

    async def run():
        io = object.__new__(LaserMappingIO)
        task = asyncio.create_task(io.run_thread(write))
        await asyncio.to_thread(entered.wait, 2)
        task.cancel()
        await asyncio.sleep(.01)
        assert not task.done()
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await task
        events.append('cleanup')
    asyncio.run(run())
    assert events == ['write-finished', 'cleanup']
