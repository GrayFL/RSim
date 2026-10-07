"""Convert native SLAM output into ROS-independent RSim snapshots."""
from collections import deque
import time

import numpy as np
from graphmap.pose import Pose

from rsim.core import PrimaryComponent, Frame
from .mapping_input import pose_transform


def pose_from_ros(value, parent, child):
    translation = [getattr(value.position, axis) for axis in 'xyz']
    quaternion = [getattr(value.orientation, axis) for axis in 'xyzw']
    if not np.isfinite(translation + quaternion).all() or np.linalg.norm(quaternion) < 1e-6:
        raise ValueError('invalid mapping pose')
    return Pose(translation=translation, rotation=quaternion, wrd_frame=parent, ego_frame=child)


RGB_DTYPE = np.dtype([(name, '<f4') for name in 'xyz'] + [(name, 'u1') for name in 'rgb'])


def rgb_points(points, *, allocator=np.empty):
    points = points.reshape(-1)
    names = set(points.dtype.names or ())
    if not set('xyz') <= names or not ({'rgb'} <= names or {'rgba'} <= names or set('rgb') <= names):
        raise ValueError('mapping cloud must contain xyz and RGB colors')
    valid = np.ones(len(points), dtype=bool)
    for axis in 'xyz':
        valid &= np.isfinite(points[axis])
    points = points[valid]
    result = allocator((len(points),), RGB_DTYPE)
    for axis in 'xyz':
        result[axis] = points[axis]
    if set('rgb') <= names:
        for name in 'rgb':
            result[name] = points[name]
    else:
        packed = points['rgb' if 'rgb' in names else 'rgba']
        packed = packed.view(packed.dtype.byteorder + 'u4') if packed.dtype.kind == 'f' else packed
        for name, shift in zip('rgb', (16, 8, 0)):
            result[name] = (packed >> shift) & 255
    result.flags.writeable = False
    return result


class MappingOutput(PrimaryComponent):
    def __init__(self, inputs, *drivers, frames, T_base_imu, database, assumptions, allocator=np.empty, mapping=None):
        super().__init__(inputs, *drivers, history=3)
        self.ingress, self.ros, self.prefix = inputs, inputs.ros, inputs.prefix
        self.frames, self.T_base_imu = frames, T_base_imu
        self.database, self.assumptions = database, assumptions
        self.scan_odometry = None
        self.fusion = None
        self.allocator = allocator
        self.mapping = mapping
        self.odometry_queue = deque(maxlen=4096)
        self.correction = None
        self.latest = {}
        self.subscriptions = []
        self.sequence = dict.fromkeys(('odometry', 'pose', 'rgb_map', 'map', 'status'), 0)
        self.last_odom = None
        self.last_correction = None
        self.last_corrected_odometry = None
        self.last_lio_stamp = None
        self.last_odom_progress = self.last_correction_progress = None

    async def open(self):
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from tf2_ros import TransformBroadcaster
        node = self.ros.node
        self.broadcaster = TransformBroadcaster(node)
        self.odom_publisher = node.create_publisher(Odometry, self.prefix + '/odom', 20)
        self.subscriptions = [
            node.create_subscription(Odometry, self.prefix + '/lio/odom', lambda msg: self.odometry_queue.append(msg), 20),
            node.create_subscription(Odometry, self.prefix + '/lio/corrected_odom', self.corrected, 20),
            node.create_subscription(TFMessage, '/tf', self.transforms, 100),
        ]
        self.task('native-output', self.convert, hz=100)
        # Native TF follows IMU updates; shared snapshots have their own rate.
        # Avoid serializing diagnostics and descriptors at every IMU tick.
        self.task('mapping-snapshot', self.status, hz=20)
        if self.mapping is not None:
            await self.mapping.open(self)

    def corrected(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        self.last_corrected_odometry = max(self.last_corrected_odometry or stamp, stamp)
        self.record_correction(stamp)

    def record_correction(self, stamp):
        # Both native corrected odometry and atomic CloudPose carry the same
        # laser-corrected state. Independent DDS delivery must not move the
        # health timestamp backwards or make a fresh atomic scan look stale.
        if self.last_correction is None or stamp > self.last_correction:
            self.last_correction = stamp
            self.last_correction_progress = time.monotonic()

    def transforms(self, message):
        if self.mapping is not None:
            return  # The map graph supplies the correction alongside node poses.
        for tf in message.transforms:
            if tf.header.frame_id == self.frames['map'] and tf.child_frame_id == self.frames['odom']:
                q, t = tf.transform.rotation, tf.transform.translation
                self.correction = Pose(translation=[t.x, t.y, t.z], rotation=[q.x, q.y, q.z, q.w],
                    wrd_frame=self.frames['map'], ego_frame=self.frames['odom'])

    def frame(self, name, data, stamp_ns):
        self.sequence[name] += 1
        return Frame(data, stamp_ns, 'ros:system', sequence=self.sequence[name])

    async def convert(self):
        from nav_msgs.msg import Odometry
        while self.odometry_queue:
            msg = self.odometry_queue.popleft()
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if self.last_lio_stamp is not None and stamp <= self.last_lio_stamp:
                continue
            self.last_lio_stamp = stamp
            # Super-LIO odometry describes its IMU, not the chassis origin.
            T_odom_imu = pose_from_ros(msg.pose.pose, self.frames.get('lio', self.frames['odom']), self.T_base_imu.ego_frame)
            pose = T_odom_imu * ~self.T_base_imu
            if self.mapping is not None:
                self.mapping.observe_pose(stamp, pose)
            if self.fusion is not None:
                continue
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id, odom.child_frame_id = pose.wrd_frame, pose.ego_frame
            for axis, value in zip('xyz', pose.position):
                setattr(odom.pose.pose.position, axis, float(value))
            for axis, value in zip('xyzw', pose.quat):
                setattr(odom.pose.pose.orientation, axis, float(value))
            # Upstream publishes no covariance. This is a configured weight,
            # not an uncertainty estimate returned by its filter.
            odom.pose.covariance = np.diag([.01, .01, .01, .005, .005, .005]).ravel().tolist()
            self.publish_odometry(odom, pose)
        if self.fusion is not None and self.fusion.latest is not None:
            odom = self.fusion.latest
            stamp = odom.header.stamp.sec*10**9 + odom.header.stamp.nanosec
            if self.last_odom is None or stamp > self.last_odom:
                pose = pose_from_ros(odom.pose.pose, self.frames['odom'], self.frames['base'])
                self.publish_odometry(odom, pose)

    def publish_odometry(self, odom, pose):
        stamp = odom.header.stamp.sec*10**9 + odom.header.stamp.nanosec
        self.last_odom, self.last_odom_progress = stamp, time.monotonic()
        self.broadcaster.sendTransform(pose_transform(pose, odom.header.stamp))
        self.odom_publisher.publish(odom)
        self.latest['odometry'] = self.frame('odometry', pose, stamp)
        if self.correction is not None and 'rgb_map' in self.latest:
            self.latest['pose'] = self.frame('pose', self.correction * pose, stamp)

    async def status(self):
        stamp = time.time_ns()
        state = self.ingress.diagnostics()
        if self.scan_odometry is not None:
            state['scan_odometry'] = self.scan_odometry.diagnostics()
        if self.fusion is not None:
            state['fusion'] = self.fusion.diagnostics()
        if self.mapping is not None:
            state['mapping'] = self.mapping.diagnostics()
        state.update(map_ready='rgb_map' in self.latest, pose_ready='pose' in self.latest,
                     database=self.database, assumptions=self.assumptions,
                     last_odometry_ns=self.last_odom,
                     odometry_age_s=(stamp - self.last_odom) * 1e-9 if self.last_odom is not None else None,
                     lidar_correction_age_s=(stamp - self.last_correction) * 1e-9 if self.last_correction is not None else None,
                     corrected_odometry_age_s=(stamp - self.last_corrected_odometry) * 1e-9 if self.last_corrected_odometry is not None else None,
                     rgb_map_age_s=(stamp - self.latest['rgb_map'].stamp_ns) * 1e-9 if 'rgb_map' in self.latest else None,
                     map_points=len(self.latest['rgb_map'].data.points) if 'rgb_map' in self.latest else 0)
        self.latest['status'] = self.frame('status', state, stamp)
        await self.publish(dict(self.latest), stamp_ns=stamp, clock='ros:system')
        if self.last_odom_progress is not None and time.monotonic()-self.last_odom_progress > 5:
            raise RuntimeError('mapping odometry stopped progressing')
        if self.last_correction_progress is not None and time.monotonic()-self.last_correction_progress > 5:
            raise RuntimeError('Super-LIO lidar corrections stopped updating')

    async def close(self):
        if self.mapping is not None:
            await self.mapping.close()
        for subscription in self.subscriptions:
            self.ros.node.destroy_subscription(subscription)
        if hasattr(self, 'odom_publisher'):
            self.ros.node.destroy_publisher(self.odom_publisher)
            self.ros.node.destroy_publisher(self.broadcaster.pub_tf)
