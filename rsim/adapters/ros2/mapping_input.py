"""Timed mapping inputs; preserve point acquisition offsets across clock domains."""
import asyncio
from collections import deque
import time

import numpy as np

from rsim.core import Component, Image, PointCloud
from rsim.core.signal import as_signal
from .conversions import pointcloud_array
from .chassis import fill_ros2


class ClockAlignment:
    """Explicit fixed offset, or a frozen minimum reception-delay estimate.

    Reception alignment includes unknown transport latency; it is not clock
    synchronization. A source reset requires a fresh mapping session.
    """
    def __init__(self, *, offset_s=None, samples=20):
        if offset_s is not None and not np.isfinite(offset_s):
            raise ValueError("clock offset must be finite")
        if samples < 2:
            raise ValueError("clock estimation needs at least two samples")
        self.offset = offset_s
        self.estimated = offset_s is None
        self.samples = samples
        self.delays = []
        self.previous = None

    def observe(self, source_end, received):
        if not np.isfinite(source_end) or not np.isfinite(received):
            raise ValueError("nonfinite timestamp")
        if self.previous is not None and source_end <= self.previous:
            raise ValueError("mapping source clock moved backwards or repeated")
        self.previous = source_end
        if self.offset is None:
            self.delays.append(received - source_end)
            if len(self.delays) < self.samples:
                return None
            self.offset = min(self.delays)
        return source_end + self.offset

    def report(self):
        return dict(offset_s=self.offset, estimated=self.estimated,
                    calibration_samples=len(self.delays))


TIMED_DTYPE = np.dtype([(name, '<f4') for name in ('x', 'y', 'z', 'intensity', 'time')])


def timed_points(points, *, min_range=.3, max_range=50., stride=1):
    """Seyond absolute seconds -> sorted PointCloud2 relative-seconds schema."""
    points = points.reshape(-1)
    required = {'x', 'y', 'z', 'intensity', 'timestamp'}
    if not required <= set(points.dtype.names or ()):
        raise ValueError("Seyond mapping requires xyz, intensity and per-point timestamp")
    if stride < 1 or not 0 <= min_range < max_range:
        raise ValueError("invalid mapping cloud filter")
    ts = points['timestamp']
    if not len(points) or not np.isfinite(ts).all():
        raise ValueError("empty cloud or invalid per-point timing")
    start, end = float(ts.min()), float(ts.max())
    if not 0 < end - start < .5:
        raise ValueError("expected a timed scan shorter than half a second")
    valid = np.ones(len(points), dtype=bool)
    distance = np.zeros(len(points))
    for axis in 'xyz':
        valid &= np.isfinite(points[axis])
        distance += points[axis].astype('f8') ** 2
    valid &= (distance > min_range ** 2) & (distance < max_range ** 2)
    # Sort indices, not full vendor records (which contain unused fields).
    # Gather only the fields needed by Super-LIO after filtering/subsampling.
    selected = np.flatnonzero(valid)[::stride]
    selected = selected[np.argsort(ts[selected], kind='stable')]
    result = np.empty(len(selected), dtype=TIMED_DTYPE)
    for field in ('x', 'y', 'z', 'intensity'):
        result[field] = points[field][selected]
    result['time'] = ts[selected] - start
    return result, start, end


def ros_stamp(seconds):
    from builtin_interfaces.msg import Time
    nanoseconds = round(seconds * 1e9)
    return Time(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)


def pose_transform(pose, stamp=None):
    from geometry_msgs.msg import TransformStamped
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = pose.wrd_frame, pose.ego_frame
    if stamp is not None:
        msg.header.stamp = stamp
    for axis, value in zip('xyz', pose.position):
        setattr(msg.transform.translation, axis, float(value))
    for axis, value in zip('xyzw', pose.quat):
        setattr(msg.transform.rotation, axis, float(value))
    return msg


def rgb_message_fields(image):
    """Convert a shared RGB payload without changing acquisition time or pixels."""
    if not isinstance(image, Image) or image.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('mapping camera source requires an RGB/BGR Image')
    pixels = np.ascontiguousarray(image.pixels)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3 or min(pixels.shape[:2]) < 1:
        raise ValueError('mapping camera source requires uint8 HxWx3 pixels')
    return dict(header={'frame_id': image.frame_id}, height=pixels.shape[0], width=pixels.shape[1],
                encoding=image.encoding, is_bigendian=0, step=pixels.shape[1] * 3, data=pixels.tobytes())


class MappingInputs(Component):
    """Local ROS or optional ROS1 ingress, timed lidar and fixed sensor TF."""
    def __init__(self, ros, bridge, *, prefix, lidar_driver, camera_driver,
                 mounts, topics, timing, cloud_filter, history=512, sources=None, camera_prefix=None):
        self.ros, self.bridge, self.prefix = ros, bridge, prefix
        self.mounts, self.topics, self.cloud_filter = mounts, topics, cloud_filter
        sources = dict(sources or {})
        if set(sources) - {'imu', 'wheel', 'scan', 'lidar', 'camera', 'camera_info'}:
            raise ValueError('unknown injected mapping source')
        if ('camera' in sources) != ('camera_info' in sources):
            raise ValueError('camera source requires image and calibration Signals')
        self.extra_sources = {name: as_signal(value) for name, value in sources.items()
                              if name in ('lidar', 'camera', 'camera_info')}
        if any(as_signal(value).clock not in (None, 'ros:system') for name, value in sources.items()
               if name in ('imu', 'wheel', 'scan')):
            raise ValueError('injected chassis sources require ROS system time')
        if 'camera' in self.extra_sources:
            clock = self.extra_sources['camera'].clock
            if clock != self.extra_sources['camera_info'].clock:
                raise ValueError('camera and camera_info must use one source clock')
            if clock not in (None, 'ros:system') and 'camera' not in timing:
                raise ValueError('device camera clock requires explicit camera timing')
        self.camera_offset_ns = round(timing.get('camera', {}).get('offset_s', 0.) * 1e9)
        self.camera_prefix = camera_prefix
        self.remote = {
            'imu': bridge.topic(topics['imu'], 'sensor_msgs/Imu', hz=500, history=history),
            'wheel_odom': bridge.topic(topics['odom'], 'nav_msgs/Odometry', hz=100, history=64),
            'scan': bridge.topic(topics['scan'], 'sensor_msgs/LaserScan', hz=30, history=16),
        } if bridge is not None else {}
        if bridge is None:
            from .sensor import RosSensor
            if timing['chassis'].get('offset_s') != 0:
                raise ValueError('native ROS chassis timestamps require offset_s=0')
            self.remote = {name: RosSensor(topics[key], kind, ros=ros,
                clock='ros:system', hz=500, history=history,
                queue_size=history if name == 'imu' else 2,
                batch_size=32 if name == 'imu' else 1) for name, key, kind in
                (('imu', 'imu', 'imu'), ('wheel_odom', 'odom', 'odom'), ('scan', 'scan', 'scan'))}
        self.remote.update({('wheel_odom' if name == 'wheel' else name): as_signal(value)
                            for name, value in sources.items() if name in ('wheel', 'imu', 'scan')})
        super().__init__(ros, *(driver for driver in (lidar_driver, camera_driver) if driver is not None),
                         inputs=(*tuple(as_signal(source) for source in self.remote.values()), *self.extra_sources.values()))
        self.lidar_clock = ClockAlignment(**timing['lidar'])
        self.chassis_clock = ClockAlignment(**timing['chassis'])
        self.pending = deque(maxlen=4)
        self.last = dict.fromkeys((*self.remote, *self.extra_sources), 0)
        self.counts = dict.fromkeys(('lidar', *self.remote), 0)
        self.received = {}
        self.max_gap = {}
        self.source_stamps = {}
        self.max_source_gap = {}
        self.dropped = 0
        self.publishers = {}

    async def open(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan, PointCloud2, Image as RosImage, CameraInfo
        from rclpy.qos import qos_profile_sensor_data, QoSProfile
        from tf2_ros import StaticTransformBroadcaster
        self.ros = self.dependencies[0]
        node = self.ros.node
        self.types = {'imu': Imu, 'wheel_odom': Odometry, 'scan': LaserScan}
        for name, cls in dict(self.types, lidar=PointCloud2).items():
            self.publishers[name] = node.create_publisher(cls, self.prefix + '/' + name,
                QoSProfile(depth=4) if name == 'lidar' else qos_profile_sensor_data)
        if 'camera' in self.extra_sources:
            self.types.update(camera=RosImage, camera_info=CameraInfo)
            for name, suffix in (('camera', '/color/image_raw'), ('camera_info', '/color/camera_info')):
                self.publishers[name] = node.create_publisher(self.types[name], self.camera_prefix + suffix, qos_profile_sensor_data)
        self.static_tf = StaticTransformBroadcaster(node)
        self.static_tf.sendTransform([pose_transform(pose) for pose in self.mounts])
        def receive(msg):
            if len(self.pending) == self.pending.maxlen:
                self.dropped += 1
            self.pending.append((msg, time.time()))
            self.record_received('lidar')
        if 'lidar' not in self.extra_sources:
            self.subscription = node.create_subscription(PointCloud2, self.topics.get('points', self.prefix + '/raw/points'),
                                                           receive, QoSProfile(depth=4))
        # Seyond publishes reliably. Request retransmission for these multi-MB
        # clouds: best-effort fragment loss otherwise drops the entire scan.
        self.started = time.monotonic()
        self.task('lidar-time', self.lidar, hz=100)
        self.task('chassis-ingress', self.chassis, hz=500)
        if self.extra_sources:
            self.task('shared-mapping-inputs', self.shared_inputs, hz=100)
        self.task('input-health', self.health, hz=2)

    async def shared_inputs(self):
        for name, source in self.extra_sources.items():
            if source.frames:
                await source.get(timeout=.01)
            for frame in source.frames:
                if frame.sequence <= self.last[name]:
                    continue
                if source.clock is not None and frame.clock != source.clock:
                    raise ValueError('mapping source clock changed')
                self.last[name] = frame.sequence
                self.record_received(name)
                if name == 'lidar':
                    if not isinstance(frame.data, PointCloud) or frame.data.frame_id != self.topics['lidar_frame']:
                        raise ValueError('mapping lidar source requires PointCloud in its configured mount frame')
                    if len(self.pending) == self.pending.maxlen:
                        self.dropped += 1
                    self.pending.append((frame.data, frame.received_ns * 1e-9))
                else:
                    values = rgb_message_fields(frame.data) if name == 'camera' else frame.data
                    message = fill_ros2(self.types[name](), values)
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(frame.stamp_ns + self.camera_offset_ns, 10**9)
                    self.publishers[name].publish(message)
                    self.counts[name] = self.counts.get(name, 0) + 1

    async def lidar(self):
        if not self.pending:
            return
        from sensor_msgs.msg import PointCloud2, PointField
        msg, received = self.pending.popleft()
        points = msg.points if isinstance(msg, PointCloud) else pointcloud_array(msg).points
        data, start, end = await asyncio.to_thread(timed_points, points, **self.cloud_filter)
        mapped_end = self.lidar_clock.observe(end, received)
        if mapped_end is None or len(data) < 20:
            return
        cloud = PointCloud2()
        cloud.header.frame_id = self.topics['lidar_frame']
        cloud.header.stamp = ros_stamp(start + self.lidar_clock.offset)
        cloud.height, cloud.width = 1, len(data)
        cloud.is_dense = True
        cloud.point_step, cloud.row_step = data.dtype.itemsize, data.nbytes
        cloud.fields = [PointField(name=name, offset=data.dtype.fields[name][1],
                                   datatype=PointField.FLOAT32, count=1) for name in data.dtype.names]
        cloud.data = data.tobytes()
        self.publishers['lidar'].publish(cloud)
        self.counts['lidar'] += 1

    async def chassis(self):
        for name, source in self.remote.items():
            source = as_signal(source)
            if source.frames:
                await source.get(timeout=.01)
            for frame in source.frames:
                if frame.sequence <= self.last[name]:
                    continue
                self.last[name] = frame.sequence
                self.record_received(name)
                previous = self.source_stamps.get(name)
                if previous is not None:
                    self.max_source_gap[name] = max(self.max_source_gap.get(name, 0.),
                                                    (frame.stamp_ns-previous)*1e-9)
                self.source_stamps[name] = frame.stamp_ns
                source_time = frame.stamp_ns * 1e-9
                if name == 'imu':
                    target = self.chassis_clock.observe(source_time, frame.received_ns * 1e-9)
                    if target is None:
                        continue
                elif self.chassis_clock.offset is None:
                    continue
                target = source_time + self.chassis_clock.offset
                msg = fill_ros2(self.types[name](), frame.data)
                msg.header.stamp = ros_stamp(target)
                if name == 'imu':
                    # Use rate/acceleration only; do not fuse uncalibrated magnetic heading.
                    msg.header.frame_id = self.topics['imu_frame']
                self.publishers[name].publish(msg)
                self.counts[name] += 1

    async def health(self):
        if time.monotonic() - self.started < 30:
            return
        for name in ('lidar', 'imu'):
            if time.monotonic() - self.received.get(name, self.started) > 3:
                raise RuntimeError(f'mapping input stopped: {name}')

    def record_received(self, name):
        now = time.monotonic()
        if name in self.received:
            self.max_gap[name] = max(self.max_gap.get(name, 0.), now - self.received[name])
        self.received[name] = now

    def diagnostics(self):
        return dict(counts=dict(self.counts), lidar_clock=self.lidar_clock.report(),
                    chassis_clock=self.chassis_clock.report(), dropped_lidar=self.dropped,
                    input_age_s={key: time.monotonic() - value for key, value in self.received.items()},
                    max_input_gap_s=dict(self.max_gap), max_source_gap_s=dict(self.max_source_gap),
                    source_queue_drops={key: source.dropped for key, source in self.remote.items()
                                        if hasattr(source, 'dropped')})

    async def close(self):
        if hasattr(self, 'subscription'):
            self.ros.node.destroy_subscription(self.subscription)
        for publisher in self.publishers.values():
            self.ros.node.destroy_publisher(publisher)
        if hasattr(self, 'static_tf'):
            self.ros.node.destroy_publisher(self.static_tf.pub_tf)
