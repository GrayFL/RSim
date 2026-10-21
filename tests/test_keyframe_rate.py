"""Motion-dependent mapping throughput, without hardware or native processes."""
import asyncio
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from graphmap.pose import Pose
from rsim.components.keyframe_rate import MotionKeyframeRate
from rsim.components.projection import PoseHistory, PinholeCamera


EPOCH = 1_800_000_000 * 10**9


def pose(x=0., yaw=0.):
    return Pose(position=[x, 0, 0], rotation=[0, 0, yaw], degrees=False,
                wrd_frame='odom', ego_frame='base')


@pytest.mark.parametrize('velocity,angular,expected', [
    (0., 0., .1), (.009, .019, .1),  # Rest and small drift.
    (.155, 0., .55), (-.155, 0., .55),
    (0., .26, .55), (0., -.26, .55),
    (.155, .5, 1.), (.3, 0., 1.), (3., 5., 1.),
])
def test_translation_or_rotation_raises_rate_with_a_cap(velocity, angular, expected):
    history = PoseHistory()
    history.add(EPOCH, pose())
    history.add(EPOCH + 500_000_000, pose(velocity * .5, angular * .5))
    policy = MotionKeyframeRate(history, max_hz=1.)
    assert policy.frequency(EPOCH + 500_000_000) == pytest.approx(expected)
    assert policy.linear_speed == pytest.approx(abs(velocity))
    assert policy.angular_speed == pytest.approx(abs(angular))


def test_rotation_wrap_and_quaternion_sign_do_not_invent_fast_motion():
    history = PoseHistory()
    start, end = pose(yaw=math.radians(179)), pose(yaw=math.radians(-179))
    end = Pose(rotation=-end.quat, wrd_frame='odom', ego_frame='base')
    history.add(EPOCH, start)
    history.add(EPOCH + 500_000_000, end)
    policy = MotionKeyframeRate(history, max_hz=1.)
    assert .1 < policy.frequency(EPOCH + 500_000_000) < .25
    assert policy.angular_speed == pytest.approx(math.radians(4))


def test_missing_or_stale_pose_history_is_unknown_motion_not_rest():
    history = PoseHistory()
    policy = MotionKeyframeRate(history, max_hz=2.)
    assert policy.frequency(EPOCH) == 2.
    history.add(EPOCH, pose())
    history.add(EPOCH + 500_000_000, pose())
    assert policy.frequency(EPOCH + 500_000_000) == .1
    # No extrapolation of stale poses, including within a missing-data gap.
    assert policy.frequency(EPOCH + 600_000_000) == 2.
    assert not policy.diagnostics()['history_ready']
    assert policy.linear_speed is policy.angular_speed is None


@pytest.mark.parametrize('options', [
    {'min_hz': 0}, {'min_hz': 2}, {'max_hz': 0}, {'max_hz': float('nan')},
    {'min_hz': float('inf')}, {'window_s': 0}, {'window_s': 1e-12},
    {'angular_speed_rad_s': 0}, {'linear_speed_m_s': float('inf')},
    {'linear_deadband_m_s': -.01}, {'linear_deadband_m_s': .3},
    {'angular_deadband_rad_s': .5},
])
def test_invalid_rate_configuration_is_rejected(options):
    with pytest.raises(ValueError, match='keyframe rate'):
        MotionKeyframeRate(PoseHistory(), **{'max_hz': 1., **options})


def make_io(*, adaptive=True):
    pytest.importorskip('sensor_msgs.msg', reason='ROS message conversion integration')
    from sensor_msgs.msg import CameraInfo
    from rsim.adapters.ros2.laser_mapping import LaserMappingIO, stamp_ns
    from rsim.components.mapping import MapLedger

    history, submitted, projected = PoseHistory(capacity=4096), [], []
    policy = MotionKeyframeRate(history, max_hz=1.) if adaptive else None
    def colorize(xyz, **kwargs):
        projected.append(xyz)
        return xyz, np.array([[1, 2, 3, 255]], np.uint8), np.array([[0, 0]], np.int32)
    io = LaserMappingIO(ledger=MapLedger(), pose_history=history, keyframe_type=None,
        camera_type=PinholeCamera, colorize=colorize, T_base_imu=Pose(wrd_frame='base', ego_frame='imu'),
        camera_prefix='/camera', keyframe_hz=1., rate_policy=policy)
    io.info = CameraInfo(width=1, height=1, k=[1., 0., 0., 0., 1., 0., 0., 0., 1.])
    io.info.header.frame_id = 'optical'
    io.optical = Pose(wrd_frame='base', ego_frame='optical')
    io.owner = SimpleNamespace(frames={'odom': 'odom', 'base': 'base'}, latest={},
                               record_correction=lambda stamp: None)
    io.publish_inputs = lambda image, *args: submitted.append(stamp_ns(image))
    return io, submitted, projected


def packet(stamp):
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from geometry_msgs.msg import Pose as ROSPose
    image = Image(width=1, height=1, encoding='rgb8', step=3, data=b'\x01\x02\x03')
    image.header.frame_id = 'optical'
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 10**9)
    cloud = PointCloud2(width=1, height=1, point_step=16, row_step=16,
        data=np.array([1., 0., 1., 42.], dtype='<f4').tobytes(),
        fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(('x', 'y', 'z', 'intensity'))])
    cloud.header.stamp = image.header.stamp
    native_pose = ROSPose()
    native_pose.orientation.w = 1.
    return SimpleNamespace(cloud=cloud, pose=native_pose), image


async def feed(io, trajectory, seconds=20):
    # Pre-roll warms the speed window. Odometry remains 200 Hz, scans/RGB 10 Hz.
    for i in range(-100, int(seconds*200) + 1):
        stamp = EPOCH + i*5_000_000
        io.observe_pose(stamp, trajectory(i/200))
        if i >= 0 and i % 20 == 0:
            scan, image = packet(stamp)
            io.scan(scan)
            io.image(image)
            await io.submit_scan()


@pytest.mark.parametrize('adaptive,velocity,angular,expected', [
    (False, 0., 0., 21), (True, 0., 0., 3),
    (True, -.155, 0., 11), (True, .6, 0., 21),
    (True, .3, 0., 21), (True, 0., .5, 21),
    (True, 0., -.8, 21), (True, 0., .8, 21),
])
def test_only_selected_frames_are_projected_and_sent_to_rtab(adaptive, velocity, angular, expected):
    io, submitted, projected = make_io(adaptive=adaptive)
    asyncio.run(feed(io, lambda t: pose(velocity*t, angular*t)))
    assert len(submitted) == len(projected) == io.stats['submitted'] == expected
    assert min(np.diff(submitted)) >= 10**9
    assert len(io.pending) == expected  # No archive observations for skipped scans.
    assert not io.scans
    assert io.stats['rate_limited_scans'] == 201 - expected
    assert io.history.samples[-1][0] == EPOCH + 20*10**9
    assert len(io.history.samples) == 4096  # Full-rate pose history, not keyframe rate.
    state = io.diagnostics()
    assert state['keyframe_max_hz'] == 1.
    assert (state['keyframe_motion'] is not None) == adaptive


def test_acceleration_rechecks_the_deadline_and_stopping_returns_to_floor():
    io, submitted, _ = make_io()
    def trajectory(t):
        return pose(.6 * max(0., min(t-2., 2.)))  # Rest, move 2..4s, then rest.
    asyncio.run(feed(io, trajectory, seconds=16))
    seconds = [(t-EPOCH)/1e9 for t in submitted]
    assert seconds[0] == 0.
    assert 2. < seconds[1] <= 2.5  # Does not wait until the old 10s deadline.
    assert min(np.diff(submitted)) >= 10**9
    assert 13. <= seconds[-1] <= 14.5
    assert io.target_keyframe_hz == .1


def test_rgb_pairing_cannot_exceed_cap_and_a_skipped_queue_is_drained():
    from rsim.adapters.ros2.laser_mapping import stamp_ns
    io, submitted, _ = make_io(adaptive=False)
    async def run():
        # Candidate scans are one second apart, but RGB stamps only .96s apart.
        for offset, rgb_offset in [(0, 20_000_000), (10**9, -20_000_000)]:
            scan, _ = packet(EPOCH + offset)
            _, image = packet(EPOCH + offset + rgb_offset)
            io.observe_pose(EPOCH + offset + rgb_offset, pose())
            io.scan(scan)
            io.image(image)
            # Supply a later image so the bilateral nearest-RGB search can run.
            _, later = packet(EPOCH + offset + 100_000_000)
            io.image(later)
            await io.submit_scan()
        assert submitted == [EPOCH + 20_000_000]
        for offset in range(100_000_000, 900_000_000, 100_000_000):
            scan, image = packet(EPOCH + offset)
            io.scans.append((scan, time.monotonic()))
            io.image(image)
        # Keep the newest RGB at the tail for the wait-for-RGB check.
        io.images = type(io.images)(sorted(io.images, key=stamp_ns), maxlen=300)
        await io.submit_scan()
        assert not io.scans
        assert len(submitted) == 1
    asyncio.run(run())
