"""Join native Super-LIO scans with RGB and consume RTAB-Map node poses."""
import asyncio
from collections import deque
import copy
from pathlib import Path
import time

import numpy as np
from graphmap.pose import Pose

from rsim.core import PointCloud
from .conversions import pointcloud_array, image_array
from .mapping_output import pose_from_ros
from .mapping_input import pose_transform


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


class LaserMappingIO:
    """ROS wiring only; geometry/provenance algorithms are injected by driver."""
    def __init__(self, *, ledger, pose_history, keyframe_type, camera_type, colorize,
                 T_base_imu, camera_prefix, allocator=np.empty, max_rgb_dt_s=.06,
                 keyframe_hz=2., occlusion_cell=2, occlusion_tolerance_m=.05,
                 fusion=None, max_wait_s=5., scan_capacity=128, image_capacity=300, rectify_rgb=False):
        if max_rgb_dt_s <= 0 or keyframe_hz <= 0 or max_wait_s <= 0 or min(scan_capacity, image_capacity) < 2:
            raise ValueError('mapping synchronization bounds must be positive')
        self.ledger, self.history = ledger, pose_history
        self.keyframe_type, self.camera_type, self.colorize = keyframe_type, camera_type, colorize
        self.T_base_imu, self.camera_prefix, self.allocator = T_base_imu, camera_prefix, allocator
        self.fusion, self.max_wait_s = fusion, max_wait_s
        self.rectify_rgb = rectify_rgb
        self.rectification = None
        self.max_dt_ns, self.interval_ns = int(max_rgb_dt_s * 1e9), int(1e9/keyframe_hz)
        self.visibility = dict(occlusion_cell=occlusion_cell, occlusion_tolerance_m=occlusion_tolerance_m)
        self.scans, self.images, self.graphs = deque(maxlen=scan_capacity), deque(maxlen=image_capacity), deque(maxlen=32)
        self.pending = {}
        self.info = self.optical = None
        self.last_submitted = self.last_scan_received = None
        self.last_scan_stamp = None
        self.last_snapshot = 0.
        self.owner = None
        self.pending_correction = None
        self.graph_client = self.graph_future = None
        self.graph_generation = self.graph_completed = 0
        self.graph_requested = 0
        self.graph_wait_since = self.graph_received_at = None
        self.stats = dict(scans_received=0, rgb_received=0, submitted=0, keyframes=0,
                          unmatched_rgb=0, missing_pose=0, dropped_scans=0, loop_links=0,
                          graph_requests=0, graph_responses=0,
                          max_rgb_dt_s=0., max_graph_latency_s=0.)

    async def open(self, owner):
        from sensor_msgs.msg import Image, CameraInfo, PointCloud2
        from nav_msgs.msg import Odometry
        from super_lio.msg import CloudPose
        from rtabmap_msgs.msg import MapData
        from rtabmap_msgs.srv import GetMap
        from tf2_ros import Buffer, TransformListener
        from rclpy.qos import qos_profile_sensor_data
        self.owner = owner
        node, prefix = owner.ros.node, owner.prefix
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        self.graph_request_type = GetMap.Request
        self.graph_client = node.create_client(GetMap, prefix+'/rtabmap/get_map_data')
        self.subscriptions = [
            node.create_subscription(CloudPose, prefix+'/lio/cloud_pose', self.scan, 16),
            node.create_subscription(Image, self.camera_prefix+'/color/image_raw', self.image, qos_profile_sensor_data),
            node.create_subscription(CameraInfo, self.camera_prefix+'/color/camera_info', self.calibration, qos_profile_sensor_data),
            node.create_subscription(MapData, prefix+'/mapData', self.graph, 20),
        ]
        self.publishers = {
            'rgb': node.create_publisher(Image, prefix+'/keyframe/rgb', 4),
            'info': node.create_publisher(CameraInfo, prefix+'/keyframe/camera_info', 4),
            'scan': node.create_publisher(PointCloud2, prefix+'/keyframe/scan', 4),
            'odom': node.create_publisher(Odometry, prefix+'/keyframe/odom', 4),
        }
        owner.task('laser-rgb-map', self.process, hz=30)

    def scan(self, message):
        if len(self.scans) == self.scans.maxlen:
            self.stats['dropped_scans'] += 1
        self.scans.append((message, time.monotonic()))
        self.last_scan_received = time.monotonic()
        stamp = stamp_ns(message.cloud)
        self.last_scan_stamp = max(self.last_scan_stamp or stamp, stamp)
        self.owner.record_correction(stamp)
        self.stats['scans_received'] += 1

    def image(self, message):
        self.images.append(message)
        self.stats['rgb_received'] += 1

    def calibration(self, message):
        if self.info is not None and message.header.frame_id != self.info.header.frame_id:
            self.optical = None
        self.info = message

    def graph(self, message):
        # Streaming statistics can include a temporary localized signature and
        # edge which RTAB has already rejected. They only trigger a refresh;
        # get_map_data(global, optimized, graph_only) supplies the valid graph
        # and lightweight node metadata needed to acknowledge source stamps.
        self.graph_generation += 1
        if self.graph_wait_since is None:
            self.graph_wait_since = time.monotonic()

    def poll_graph(self):
        if self.graph_future is not None and self.graph_future.done():
            self.graphs.append(self.graph_future.result().data)
            self.graph_future = None
            self.graph_completed = self.graph_requested
            self.graph_received_at = time.monotonic()
            self.graph_wait_since = None
            self.stats['graph_responses'] += 1
        if self.graph_future is None and self.graph_generation > self.graph_completed:
            if self.graph_wait_since is None:
                self.graph_wait_since = time.monotonic()
            if self.graph_client.service_is_ready():
                request = self.graph_request_type(global_map=True, optimized=True, graph_only=True)
                self.graph_future = self.graph_client.call_async(request)
                self.graph_requested = self.graph_generation
                self.stats['graph_requests'] += 1
        if self.graph_wait_since is not None and time.monotonic()-self.graph_wait_since > self.max_wait_s:
            raise RuntimeError('RTAB-Map complete graph service stopped responding')

    def observe_pose(self, stamp, pose):
        self.history.add(stamp, pose)

    def diagnostics(self):
        return {**self.stats, 'geometry_source': 'super_lio_deskewed_laser',
                'color_source': 'd435_rgb', 'camera_depth_used': False,
                'map_revision': self.owner.latest['map'].data['revision'] if self.owner and 'map' in self.owner.latest else 0,
                'session_id': self.ledger.session_id,
                'cloud_pose_age_s': (time.time_ns()-self.last_scan_stamp)*1e-9 if self.last_scan_stamp is not None else None,
                'mapping_lag_s': (time.time_ns()-self.last_submitted)*1e-9 if self.last_submitted is not None else None,
                'pending_scans': len(self.scans),
                'pending_keyframes': len(self.pending), 'active_keyframes': len(self.ledger.active),
                'graph_response_age_s': None if self.graph_received_at is None else time.monotonic()-self.graph_received_at}

    def body_optical(self):
        if self.optical is not None:
            return self.optical
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            tf = self.buffer.lookup_transform(self.owner.frames['base'], self.info.header.frame_id, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        self.optical = Pose(translation=[t.x, t.y, t.z], rotation=[q.x, q.y, q.z, q.w],
            wrd_frame=self.owner.frames['base'], ego_frame=self.info.header.frame_id)
        return self.optical

    async def run_thread(self, function, *args, **kwargs):
        # asyncio cancellation does not stop a running thread. Join it before
        # Runtime starts close(), otherwise two archive writes can race.
        task = asyncio.create_task(asyncio.to_thread(function, *args, **kwargs))
        try:
            return await asyncio.shield(task)
        except asyncio.CancelledError:
            await task
            raise

    async def process(self):
        self.poll_graph()
        correction = None
        while self.graphs:
            message = self.graphs.popleft()
            if message.header.frame_id != self.owner.frames['map']:
                raise ValueError('RTAB-Map graph has unexpected frame')
            poses = {int(key): pose_from_ros(pose, self.owner.frames['map'], self.owner.frames['base'])
                for key, pose in zip(message.graph.poses_id, message.graph.poses) if key > 0}
            for node in message.nodes:
                if node.id <= 0 or node.id in self.ledger.keyframes or node.id not in poses:
                    continue
                stamp = round(node.stamp * 1e9)
                nearest = min(self.pending, key=lambda candidate: abs(candidate-stamp), default=None)
                if nearest is None or abs(nearest-stamp) > 1000:
                    raise RuntimeError(f'no retained laser observation for RTAB-Map node {node.id}')
                observation = self.pending.pop(nearest)
                frame = self.keyframe_type(node.id, **observation)
                self.ledger.add(frame, poses[node.id])
                self.stats['keyframes'] += 1
                self.stats['max_graph_latency_s'] = max(self.stats['max_graph_latency_s'], (time.time_ns()-stamp)*1e-9)
            if set(poses)-self.ledger.keyframes.keys():
                raise RuntimeError('complete RTAB graph is missing node source metadata')
            self.ledger.update_graph(poses, complete=True)
            self.stats['loop_links'] = sum(link.type in (1, 2, 3, 4) for link in message.graph.links)
            if poses:
                latest_id = max(poses, key=lambda key: self.ledger.keyframes[key].stamp_ns)
                correction = poses[latest_id] * ~self.ledger.keyframes[latest_id].odometry
        if correction is not None:
            self.pending_correction = correction
        if self.ledger.dirty and (time.monotonic() - self.last_snapshot > .4):
            snapshot = await self.run_thread(self.ledger.snapshot, allocator=self.allocator)
            await self.run_thread(self.ledger.save, Path(self.owner.database).with_suffix('.laser'))
            await self.publish_snapshot(snapshot)
            self.last_snapshot = time.monotonic()
        if not self.ledger.dirty and self.pending_correction is not None:
            self.owner.correction = self.pending_correction
            self.owner.correction_revision = dict(session_id=self.ledger.session_id, revision=self.ledger.revision)
            self.pending_correction = None
        await self.submit_scan()
        if self.last_scan_received is not None and time.monotonic() - self.last_scan_received > 5:
            raise RuntimeError('Super-LIO deskewed clouds stopped updating')

    async def submit_scan(self):
        if not self.scans or not self.images or self.info is None or self.body_optical() is None:
            return
        # Drain scans excluded by the selected keyframe rate together. Map
        # rebuilding must not turn intentional subsampling into a stale queue.
        while self.scans and self.last_submitted is not None and stamp_ns(self.scans[0][0].cloud) < self.last_submitted + self.interval_ns:
            self.scans.popleft()
        if not self.scans:
            return
        scan, arrival = self.scans[0]
        expired = time.monotonic()-arrival > self.max_wait_s
        scan_stamp = stamp_ns(scan.cloud)
        # Wait for one RGB sample beyond the scan time, so nearest is bilateral.
        if stamp_ns(self.images[-1]) < scan_stamp and not expired:
            return
        image = min(self.images, key=lambda value: abs(stamp_ns(value)-scan_stamp))
        info = self.info
        optical = self.optical
        if info.header.frame_id != image.header.frame_id:
            raise ValueError('RGB image and calibration have different optical frames')
        image_stamp = stamp_ns(image)
        if abs(image_stamp-scan_stamp) > self.max_dt_ns:
            self.scans.popleft()
            self.stats['unmatched_rgb'] += 1
            return
        image_pose = self.history.at(image_stamp)
        odom_pose = self.fusion.history.at(image_stamp) if self.fusion is not None else image_pose
        odom_cov = self.fusion.covariance_history.at(image_stamp) if self.fusion is not None else None
        if image_pose is None or odom_pose is None or (self.fusion is not None and odom_cov is None):
            if expired:
                self.scans.popleft()
                self.stats['missing_pose'] += 1
            return
        self.scans.popleft()
        if self.last_submitted is not None and image_stamp <= self.last_submitted:
            return
        points = pointcloud_array(scan.cloud).points.reshape(-1)
        xyz = np.column_stack([points[key] for key in 'xyz'])
        finite = np.isfinite(xyz).all(axis=1)
        xyz, intensity = xyz[finite], points['intensity'][finite]
        rgb = image_array(image).pixels
        if image.encoding == 'bgr8':
            rgb = rgb[..., ::-1]
        elif image.encoding != 'rgb8':
            raise ValueError('laser colorization requires rgb8 or bgr8')
        camera = self.camera_type(info.width, info.height, info.k, info.d, info.distortion_model)
        scan_pose = pose_from_ros(scan.pose, self.owner.frames.get('lio', self.owner.frames['odom']), self.T_base_imu.ego_frame)
        body, rgba, pixels = await asyncio.to_thread(self.colorize, xyz, scan_pose=scan_pose,
            image_pose=image_pose, body_to_optical=optical, camera=camera, rgb=rgb, **self.visibility)
        calibration = dict(width=info.width, height=info.height, matrix=list(info.k),
            distortion=list(info.d), model=info.distortion_model, frame_id=info.header.frame_id,
            body_to_optical=optical.matrix.tolist())
        observation = dict(stamp_ns=image_stamp, scan_stamp_ns=scan_stamp, xyz=body, rgba=rgba,
                           pixels=pixels, odometry=odom_pose, rgb=rgb, camera=calibration)
        self.pending[image_stamp] = observation
        # RTAB-Map rejects non-keyframes. Retain a bounded 30s acknowledgement window.
        self.pending = {key: value for key, value in self.pending.items() if image_stamp-key < 30*10**9}
        if self.rectify_rgb:
            image, info = await asyncio.to_thread(self.rectify, image, info)
        self.publish_inputs(image, info, body, intensity, odom_pose, odom_cov)
        self.last_submitted = image_stamp
        self.stats['submitted'] += 1
        self.stats['max_rgb_dt_s'] = max(self.stats['max_rgb_dt_s'], abs(image_stamp-scan_stamp)*1e-9)

    def rectify(self, image, info):
        """RTAB's laser depth projection uses a pinhole camera; archive stays raw."""
        import cv2
        key = (info.width, info.height, tuple(info.k), tuple(info.d), info.distortion_model)
        K = np.asarray(info.k).reshape(3, 3)
        if self.rectification is None or key != self.rectification[0]:
            method = cv2.fisheye.initUndistortRectifyMap if info.distortion_model == 'equidistant' else cv2.initUndistortRectifyMap
            maps = method(K, np.asarray(info.d), np.eye(3), K, (info.width, info.height), cv2.CV_32FC1)
            self.rectification = key, maps
        pixels = cv2.remap(image_array(image).pixels, *self.rectification[1], cv2.INTER_LINEAR)
        image, info = copy.deepcopy(image), copy.deepcopy(info)
        image.step, image.data = info.width*3, pixels.tobytes()
        info.d, info.distortion_model = [0.]*5, 'plumb_bob'
        info.r = np.eye(3).ravel().tolist()
        info.p = np.column_stack([K, np.zeros(3)]).ravel().tolist()
        return image, info

    def publish_inputs(self, image, calibration, xyz, intensity, pose, covariance=None):
        from sensor_msgs.msg import PointCloud2, PointField
        from nav_msgs.msg import Odometry
        data = np.empty(len(xyz), dtype=[(name, '<f4') for name in ('x', 'y', 'z', 'intensity')])
        for column, name in enumerate('xyz'):
            data[name] = xyz[:, column]
        data['intensity'] = intensity
        scan = PointCloud2()
        scan.header.stamp, scan.header.frame_id = image.header.stamp, self.owner.frames['base']
        scan.height, scan.width, scan.is_dense = 1, len(xyz), True
        scan.point_step, scan.row_step = data.dtype.itemsize, data.nbytes
        scan.fields = [PointField(name=name, offset=data.dtype.fields[name][1], datatype=PointField.FLOAT32, count=1)
                       for name in data.dtype.names]
        scan.data = data.tobytes()
        info = copy.deepcopy(calibration)
        info.header.stamp = image.header.stamp
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = image.header.stamp, pose.wrd_frame, pose.ego_frame
        for axis, value in zip('xyz', pose.position):
            setattr(odom.pose.pose.position, axis, float(value))
        for axis, value in zip('xyzw', pose.quat):
            setattr(odom.pose.pose.orientation, axis, float(value))
        if covariance is None:
            covariance = np.diag([.01, .01, .01, .005, .005, .005])
        odom.pose.covariance = np.asarray(covariance).ravel().tolist()
        # RTAB also looks up odometry TF even for exact synchronized messages.
        # Publish the same interpolated pose, including for delayed keyframes.
        self.owner.broadcaster.sendTransform(pose_transform(pose, image.header.stamp))
        for key, message in [('odom', odom), ('info', info), ('scan', scan), ('rgb', image)]:
            self.publishers[key].publish(message)

    async def publish_snapshot(self, snapshot):
        from graphmap.infopoints import InfoPoints
        info = InfoPoints(snapshot['infopoints'])
        cloud = self.allocator((len(info),), np.dtype([(key, '<f4') for key in 'xyz'] +
            [(key, 'u1') for key in ('r', 'g', 'b', 'color_valid')]))
        for i, key in enumerate('xyz'):
            cloud[key] = info.xyz[:, i]
        for i, key in enumerate('rgb'):
            cloud[key] = info.color[:, i]
        cloud['color_valid'] = (info.color[:, 3] > 0).astype(np.uint8)
        cloud.flags.writeable = False
        stamp = time.time_ns()
        metadata = dict(session_id=snapshot['session_id'], revision=snapshot['revision'])
        await self.owner.publish_port('map', snapshot, stamp, metadata=metadata)
        await self.owner.publish_port('rgb_map', PointCloud(cloud, snapshot['frame_id']), stamp, metadata=metadata)

    async def close(self):
        if self.owner is None:
            return
        self.listener.unregister()
        for sub in self.subscriptions:
            self.owner.ros.node.destroy_subscription(sub)
        for pub in self.publishers.values():
            self.owner.ros.node.destroy_publisher(pub)
        if self.graph_future is not None:
            self.graph_future.cancel()
        if self.graph_client is not None:
            self.owner.ros.node.destroy_client(self.graph_client)
        if self.ledger.keyframes:
            await asyncio.to_thread(self.ledger.save, Path(self.owner.database).with_suffix('.laser'))
