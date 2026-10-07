"""Acquisition-time wheel/gyro/2D odometry fusion using robot_localization."""
from collections import deque
import copy
from pathlib import Path
import time

import numpy as np

from rsim.core import Component
from .driver import Driver
from .mapping_output import pose_from_ros


def stamp_ns(message):
    return message.header.stamp.sec*10**9 + message.header.stamp.nanosec


def covariance(value, size):
    result = np.asarray(value, dtype=float).reshape(size, size)
    if (not np.isfinite(result).all() or not np.allclose(result, result.T, atol=1e-8)
            or np.linalg.eigvalsh(result).min() < -1e-8):
        raise ValueError('invalid fusion covariance')
    return result


class MappingFusion(Component):
    """Planar mapping odometry; its clock can lag actuator feedback.

    Each native measurement combines a scan pose with wheel velocity and gyro
    interpolated at that scan's timestamp. This keeps the filter on acquisition
    time and avoids feeding a fresh wheel measurement with an old scan pose.
    No LIO pose is fed back to either this estimator or the scan frontend.
    """
    def __init__(self, ros, scan, *, prefix, frames, mount, directory,
                 history, covariance_history, wheel_history, gyro_history,
                 parameters=None, max_wait_s=5., pending_capacity=256):
        import yaml
        if max_wait_s <= 0 or not np.isfinite(max_wait_s) or pending_capacity < 2:
            raise ValueError('invalid fusion buffering bounds')
        self.ros, self.scan = ros, scan
        self.prefix, self.frames, self.mount = prefix, frames, mount
        self.local = prefix+'/fusion'
        self.history, self.covariance_history = history, covariance_history
        self.wheel_history, self.gyro_history = wheel_history, gyro_history
        self.max_wait_s, self.pending_capacity = max_wait_s, pending_capacity
        self.pending = deque()
        self.counts = dict(measurements=0, invalid=0, missing_bracket=0, odometry=0)
        self.last_scan = self.last_output_stamp = -1
        self.last_output = None
        self.latest = None
        config = {'frequency': 50., 'sensor_timeout': 60., 'smooth_lagged_data': True,
                  'history_length': 10., 'predict_to_current_time': False,
                  'print_diagnostics': True, **(parameters or {})}
        # The scan frontend's local origin defines this odometry frame. Wheel
        # poses have a different origin and therefore only contribute velocity.
        contract = dict(two_d_mode=True, publish_tf=False, use_control=False,
            world_frame=frames['odom'], odom_frame=frames['odom'],
            map_frame=frames['map'], base_link_frame=frames['base'],
            odom0=self.local+'/measurement', odom0_differential=False, odom0_relative=False,
            odom0_config=[True, True, False, False, False, True,
                          True, True, False, False, False, True, False, False, False],
            odom0_queue_size=100, permit_corrected_publication=False)
        conflicts = {k for k in contract if k in (parameters or {}) and parameters[k] != contract[k]}
        if conflicts:
            raise ValueError(f'fusion component owns these parameters: {sorted(conflicts)}')
        config.update(contract)
        path = Path(directory)/'fusion-parameters.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
        self.driver = Driver('robot_localization', 'ekf_node', key='mapping:fusion:'+prefix,
            ros_args=['--params-file', str(path)], log_path=Path(directory)/'fusion.log',
            remappings={'__ns': self.local, 'odometry/filtered': self.local+'/odom'})
        super().__init__(ros, scan, self.driver)
        self.subscriptions = []

    async def open(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        node = self.ros.node
        sensor = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = node.create_publisher(Odometry, self.local+'/measurement', 100)
        self.subscriptions = [
            node.create_subscription(Odometry, self.prefix+'/wheel_odom', self.wheel, sensor),
            node.create_subscription(Imu, self.prefix+'/imu', self.imu, sensor),
            node.create_subscription(Odometry, self.scan.local+'/odom', self.scan_pose, 100),
            node.create_subscription(Odometry, self.local+'/odom', self.output, 100),
        ]
        self.started = time.monotonic()
        self.task('mapping-fusion-inputs', self.flush, hz=100)
        self.task('mapping-fusion-health', self.health, hz=2)

    def wheel(self, message):
        try:
            if message.child_frame_id != self.frames['base']:
                raise ValueError('wheel velocity must describe the base frame')
            linear = message.twist.twist.linear
            cov = covariance(message.twist.covariance, 6)[:2, :2] + np.eye(2)*.03**2
            if not self.wheel_history.add(stamp_ns(message), [linear.x, linear.y, *cov.ravel()]):
                raise ValueError('wheel stamps must increase')
        except ValueError:
            self.counts['invalid'] += 1

    def imu(self, message):
        try:
            if message.header.frame_id != self.mount.ego_frame:
                raise ValueError('IMU frame differs from its mount')
            angular = message.angular_velocity
            R = self.mount.matrix[:3, :3]
            omega = R @ np.array([angular.x, angular.y, angular.z])
            cov = R @ covariance(message.angular_velocity_covariance, 3) @ R.T
            if not self.gyro_history.add(stamp_ns(message), [omega[2], cov[2, 2]+.02**2]):
                raise ValueError('IMU stamps must increase')
        except ValueError:
            self.counts['invalid'] += 1

    def scan_pose(self, message):
        stamp = stamp_ns(message)
        try:
            if (stamp <= self.last_scan or message.header.frame_id != self.scan.odom_frame
                    or message.child_frame_id != self.frames['base']):
                raise ValueError('scan pose changed frames or ordering')
            pose = pose_from_ros(message.pose.pose, self.scan.odom_frame, self.frames['base'])
            cov = covariance(message.pose.covariance, 6)
            # Native initialization/lost-registration covariance is not a pose fix.
            if max(cov[0, 0], cov[1, 1], cov[5, 5]) >= 1e3:
                raise ValueError('scan pose has no valid registration')
            if abs(pose.position[2]) > 1e-3 or np.linalg.norm(pose.matrix[2, :2]) > .01:
                raise ValueError('mapping fusion currently requires planar scan odometry')
        except ValueError:
            self.counts['invalid'] += 1
            return
        self.last_scan = stamp
        if len(self.pending) >= self.pending_capacity:
            raise RuntimeError('mapping fusion queue overflow')
        self.pending.append((message, time.monotonic()))

    async def flush(self):
        while self.pending:
            message, arrival = self.pending[0]
            stamp = stamp_ns(message)
            wheel, gyro = self.wheel_history.at(stamp), self.gyro_history.at(stamp)
            if wheel is None or gyro is None:
                if time.monotonic()-arrival < self.max_wait_s:
                    return
                self.pending.popleft()
                self.counts['missing_bracket'] += 1
                continue
            self.pending.popleft()
            fused = copy.deepcopy(message)
            # Explicit coordinate convention: fused odometry uses the scan
            # frontend's origin. This is not an alignment to the wheel origin.
            fused.header.frame_id = self.frames['odom']
            cov = covariance(message.pose.covariance, 6).copy()
            cov += np.diag([.03**2, .03**2, 0., 0., 0., np.deg2rad(1.)**2])
            fused.pose.covariance = cov.ravel().tolist()
            fused.twist.twist.linear.x, fused.twist.twist.linear.y = map(float, wheel[:2])
            fused.twist.twist.angular.z = float(gyro[0])
            cov = np.zeros((6, 6))
            cov[:2, :2], cov[5, 5] = wheel[2:].reshape(2, 2), gyro[1]
            fused.twist.covariance = cov.ravel().tolist()
            self.publisher.publish(fused)
            self.counts['measurements'] += 1

    def output(self, message):
        stamp = stamp_ns(message)
        if stamp <= self.last_output_stamp:
            return
        if message.header.frame_id != self.frames['odom'] or message.child_frame_id != self.frames['base']:
            raise RuntimeError('mapping fusion changed frames')
        pose = pose_from_ros(message.pose.pose, self.frames['odom'], self.frames['base'])
        cov = covariance(message.pose.covariance, 6)
        self.history.add(stamp, pose)
        self.covariance_history.add(stamp, cov)
        self.latest = message
        self.last_output_stamp, self.last_output = stamp, time.monotonic()
        self.counts['odometry'] += 1

    async def health(self):
        now = time.monotonic()
        if now-self.started > 30 and now-(self.last_output or self.started) > 5:
            raise RuntimeError('mapping fusion output stopped progressing')

    def diagnostics(self):
        return dict(counts=dict(self.counts), pending=len(self.pending),
            source='wheel_velocity+imu_gyro+scan2d_pose', planar=True,
            odometry_age_s=None if self.last_output_stamp < 0 else (time.time_ns()-self.last_output_stamp)*1e-9,
            output_age_s=None if self.last_output is None else time.monotonic()-self.last_output)

    async def close(self):
        for sub in self.subscriptions:
            self.ros.node.destroy_subscription(sub)
        if hasattr(self, 'publisher'):
            self.ros.node.destroy_publisher(self.publisher)
