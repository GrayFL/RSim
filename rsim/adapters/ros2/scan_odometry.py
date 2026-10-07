"""Optional planar scan odometry with private wheel-motion TF."""
from collections import deque
from pathlib import Path
import time

import numpy as np

from rsim.core import Component
from .driver import Driver
from .mapping_input import pose_transform


def scan_interval(msg):
    """Acquisition bounds, including the last beam, in integer nanoseconds."""
    start = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    duration = (len(msg.ranges) - 1) * msg.time_increment
    if len(msg.ranges) < 2 or not np.isfinite(duration) or not 0 < duration < .5:
        raise ValueError('scan odometry requires per-beam timing shorter than 0.5 seconds')
    return start, start + round(duration * 1e9)


class ScanOdometry(Component):
    """Native RTAB ICP; wheel poses supply deskew/initial guess, never LIO.

    Output is prefix/scan2d/odom in the base frame. The private TF tree avoids
    claiming ownership of the application's global odometry transform.
    """
    def __init__(self, ros, *, prefix, mount, directory, parameters=None):
        from graphmap.pose import Pose
        import yaml
        self.ros, self.prefix = ros, prefix
        self.mount = Pose(**mount) if isinstance(mount, dict) else mount
        if (not np.isfinite(self.mount.matrix).all() or self.mount.scale != 1 or self.mount.wrd_frame != 'base_footprint'
                or not self.mount.ego_frame or self.mount.ego_frame == self.mount.wrd_frame):
            raise ValueError('scan mount must be SE(3), base_footprint -> scan frame')
        self.local = prefix + '/scan2d'
        self.guess_frame = prefix.strip('/').replace('/', '_') + '_wheel_guess'
        self.odom_frame = prefix.strip('/').replace('/', '_') + '_scan_odom'
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        config = {'Reg/Force3DoF': 'true', 'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '10', 'Icp/MaxCorrespondenceDistance': '0.2',
            'Icp/CorrespondenceRatio': '0.2', 'Icp/VoxelSize': '0.03',
            'OdomF2M/ScanMaxSize': '2000', 'Odom/ResetCountdown': '0',
            'scan_range_min': .45, 'scan_range_max': 12., **(parameters or {})}
        # These fields define the component's frame/transport contract.
        reserved = dict(frame_id=self.mount.wrd_frame, odom_frame_id=self.odom_frame,
            guess_frame_id=self.guess_frame, publish_tf=False, deskewing=True,
            wait_imu_to_init=False, wait_for_transform=.2, qos=1,
            topic_queue_size=100, always_process_most_recent_frame=False)
        conflicts = {k for k in reserved if k in (parameters or {}) and parameters[k] != reserved[k]}
        if conflicts:
            raise ValueError(f'scan component owns these parameters: {sorted(conflicts)}')
        config.update(reserved)
        parameter_file = directory / 'scan2d-parameters.yaml'
        parameter_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
        self.driver = Driver('rtabmap_odom', 'icp_odometry', key='mapping:scan2d:' + prefix,
            ros_args=['--params-file', str(parameter_file)], log_path=directory/'scan2d.log',
            remappings={'__ns': self.local, 'scan': self.local+'/scan',
                        '/tf': self.local+'/tf', '/tf_static': self.local+'/tf_static'})
        super().__init__(ros, self.driver)
        self.pending = deque()
        self.wheel_times = deque(maxlen=1000)
        self.last_scan = -1
        self.counts = dict(forwarded=0, uncovered=0, overflow=0, invalid=0, odometry=0)
        self.subscriptions, self.publishers = [], []
        self.wheel_frame = None
        self.last_output = None
        self.last_output_stamp = -1
        self.latest_pose = None

    async def open(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        node = self.ros.node
        self.tf = node.create_publisher(TFMessage, self.local+'/tf', QoSProfile(depth=100))
        self.static = node.create_publisher(TFMessage, self.local+'/tf_static',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan = node.create_publisher(LaserScan, self.local+'/scan', QoSProfile(depth=100))
        self.publishers = [self.tf, self.static, self.scan]
        self.static.publish(TFMessage(transforms=[pose_transform(self.mount)]))
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions = [node.create_subscription(Odometry, self.prefix+'/wheel_odom', self.wheel, qos),
            node.create_subscription(LaserScan, self.prefix+'/scan', self.receive_scan, qos),
            node.create_subscription(Odometry, self.local+'/odom', self.receive_odometry, QoSProfile(depth=100))]
        self.started = time.monotonic()
        self.last_output = None
        self.task('scan-odometry-health', self.health, hz=2)

    def wheel(self, msg):
        from graphmap.pose import Pose
        from tf2_msgs.msg import TFMessage
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        if (not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1) > .01
                or msg.child_frame_id != self.mount.wrd_frame or not msg.header.frame_id
                or (self.wheel_frame is not None and msg.header.frame_id != self.wheel_frame)
                or (self.wheel_times and stamp <= self.wheel_times[-1])):
            self.counts['invalid'] += 1
            return
        self.wheel_frame = msg.header.frame_id
        pose = Pose(position=values[:3], rotation=values[3:], wrd_frame=self.guess_frame,
                    ego_frame=self.mount.wrd_frame)
        self.tf.publish(TFMessage(transforms=[pose_transform(pose, msg.header.stamp)]))
        self.wheel_times.append(stamp)
        self.flush()

    def receive_scan(self, msg):
        try:
            start, end = scan_interval(msg)
            if msg.header.frame_id != self.mount.ego_frame or start <= self.last_scan:
                raise ValueError('scan frame or ordering mismatch')
        except ValueError:
            self.counts['invalid'] += 1
            return
        self.last_scan = start
        if len(self.pending) == 16:
            self.pending.popleft()
            self.counts['overflow'] += 1
        self.pending.append((start, end, msg))
        self.flush()

    def flush(self):
        while self.pending and self.wheel_times:
            start, end, msg = self.pending[0]
            if end > self.wheel_times[-1]:
                break
            self.pending.popleft()
            times = np.asarray(self.wheel_times, dtype=np.int64)
            lo = np.searchsorted(times, start, side='right') - 1
            hi = np.searchsorted(times, end, side='left')
            if lo < 0 or np.any(np.diff(times[lo:hi+1]) > 100_000_000):
                self.counts['uncovered'] += 1
                continue
            self.scan.publish(msg)
            self.counts['forwarded'] += 1

    def receive_odometry(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if (not np.isfinite(values).all() or stamp <= self.last_output_stamp
                or msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.mount.wrd_frame
                or abs(np.linalg.norm([q.x, q.y, q.z, q.w])-1) > .01):
            raise RuntimeError('scan odometry lost tracking or changed frames')
        covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
        if not np.isfinite(covariance).all() or (np.diag(covariance) < 0).any():
            raise RuntimeError('scan odometry returned invalid covariance')
        self.latest_pose = dict(stamp_ns=stamp, frame_id=self.odom_frame,
            child_frame_id=self.mount.wrd_frame, position=values[:3].tolist(),
            rotation=values[3:].tolist(), covariance=covariance.tolist())
        self.last_output_stamp = stamp
        self.counts['odometry'] += 1
        self.last_output = time.monotonic()

    async def health(self):
        now = time.monotonic()
        if now-self.started > 30 and now-(self.last_output or self.started) > 5:
            raise RuntimeError('scan odometry output stopped')

    def diagnostics(self):
        return dict(counts=dict(self.counts), pending=len(self.pending), pose=self.latest_pose,
                    stamp_age_s=None if self.last_output_stamp < 0 else (time.time_ns()-self.last_output_stamp)*1e-9,
                    output_age_s=None if self.last_output is None else time.monotonic()-self.last_output)

    async def close(self):
        for subscription in self.subscriptions:
            self.ros.node.destroy_subscription(subscription)
        for publisher in self.publishers:
            self.ros.node.destroy_publisher(publisher)
