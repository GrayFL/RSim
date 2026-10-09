"""Reusable wheel/IMU/optional scan EKF with local ROS inputs and RSim outputs."""
from pathlib import Path
import copy
import time

import numpy as np
from graphmap.pose import Pose

from rsim.core import Component
from rsim.core.signal import as_signal
from .context import RosContext
from .sensor import RosSensor
from .driver import Driver
from .chassis import fill_ros2


def measurement_covariance(values, size, floor):
    cov = np.asarray(values, dtype=float).reshape(size, size)
    if (not np.isfinite(cov).all() or not np.allclose(cov, cov.T, atol=1e-8)
            or np.linalg.eigvalsh(cov).min() < -1e-8):
        raise ValueError('invalid odometry covariance')
    cov = cov.copy()
    cov[np.diag_indices(size)] = np.maximum(np.diag(cov), np.square(floor))
    return cov


class InertialOdometry(Component):
    """Native EKF independent of mapping and actuator ownership.

    Inputs are local, ROS-independent Signals. Wheel twist is fused instead of
    duplicating its integrated pose. IMU must already be mounted in body axes,
    with gravity removed. Lagged scan poses update retained native filter state.
    Public poses stop when wheel/IMU timestamps stop; native extrapolation never
    renews controller freshness. No TF or DDS sharing provider is created here.
    """
    def __init__(self, wheel, imu, *, ros=None, scan=None, prefix='/rsim/odometry',
                 world_frame='odom_fused', body_frame='base_footprint', directory,
                 parameters=None, input_timeout=.3, history=512,
                 use_orientation=True, use_acceleration=True, lateral_acceleration=False,
                 pose_history=None, covariance_history=None):
        import yaml
        if not prefix.startswith('/') or prefix == '/' or not world_frame or not body_frame or world_frame == body_frame:
            raise ValueError('odometry requires a namespace and distinct world/body frames')
        if not np.isfinite(input_timeout) or input_timeout <= 0:
            raise ValueError('input_timeout must be positive')
        self.wheel, self.imu = as_signal(wheel), as_signal(imu)
        self.scan = scan
        self.sources = dict(wheel=self.wheel, imu=self.imu)
        if scan is not None:
            self.sources['scan'] = as_signal(scan.odometry)
        for source in self.sources.values():
            if source.clock not in (None, 'ros:system'):
                raise ValueError('native EKF requires ROS system-time inputs')
        self.ros = ros or RosContext()
        self.local, self.world_frame, self.body_frame = prefix, world_frame, body_frame
        self.input_timeout = input_timeout
        self.use_orientation, self.use_acceleration = use_orientation, use_acceleration
        self.history, self.covariance_history = pose_history, covariance_history
        self.latest = None
        self.last_output_stamp = -1
        config = dict(frequency=50., sensor_timeout=.2, smooth_lagged_data=True,
                      history_length=3., print_diagnostics=True)
        config.update(parameters or {})
        contract = dict(two_d_mode=True, publish_tf=False, use_control=False,
            world_frame=world_frame, odom_frame=world_frame, map_frame=world_frame+'_map',
            base_link_frame=body_frame, predict_to_current_time=False, permit_corrected_publication=False,
            odom0=prefix+'/inputs/wheel', odom0_config=[False]*6+[True, True, False, False, False, True]+[False]*3,
            odom0_queue_size=256, odom0_differential=False, odom0_relative=False,
            imu0=prefix+'/inputs/imu', imu0_config=[False]*5+[bool(use_orientation)]+[False]*5+
                [True, bool(use_acceleration), bool(use_acceleration and lateral_acceleration), False],
            imu0_queue_size=512, imu0_differential=False, imu0_relative=True,
            imu0_remove_gravitational_acceleration=False)
        if scan is not None:
            contract.update(odom1=prefix+'/inputs/scan', odom1_config=[True, True, False, False, False, True]+[False]*9,
                            odom1_queue_size=128, odom1_relative=False, odom1_differential=False)
        conflicts = {key for key in contract if key in (parameters or {}) and parameters[key] != contract[key]}
        if conflicts:
            raise ValueError(f'odometry owns frame/input contract: {sorted(conflicts)}')
        config.update(contract)
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory/'ekf-parameters.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
        self.driver = Driver('robot_localization', 'ekf_node', key='odometry:'+prefix,
                             ros_args=['--params-file', str(path)], log_path=directory/'ekf.log',
                             remappings={'__ns': prefix, 'odometry/filtered': prefix+'/filtered'})
        self.filtered = RosSensor(prefix+'/filtered', 'odom', ros=self.ros, clock='ros:system', history=history, hz=500)
        super().__init__(self.ros, self.driver, *((scan,) if scan else ()),
                         inputs=(*self.sources.values(), self.filtered.odom))
        self.pose = self.signal('pose', history=history, clock='ros:system')
        self.odometry = self.signal('odometry', history=history, clock='ros:system')
        self.status = self.signal('status', history=32, clock='ros:system')
        self.estimate = self.odometry
        self.counts = dict.fromkeys(self.sources, 0)
        self.stamps = dict.fromkeys(self.sources, -1)
        self.invalid = 0

    async def open(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        self.node = self.dependencies[0].node
        self.publishers = {name: self.node.create_publisher(Imu if name == 'imu' else Odometry,
                           self.local+'/inputs/'+name, 128) for name in self.sources}
        self.previous = dict.fromkeys(self.sources, 0)
        self.stamps = dict.fromkeys(self.sources, -1)
        self.received = dict.fromkeys(self.sources, 0)
        self.counts = dict.fromkeys(self.sources, 0)
        self.invalid = self.last_output_sequence = 0
        self.latest, self.last_output_stamp = None, -1
        self.task('ekf-inputs', self.feed, hz=500)
        self.task('ekf-output', self.receive, hz=200)
        self.task('ekf-status', self.report, hz=10)

    def prepare(self, name, data):
        value = copy.deepcopy(data)
        if name == 'imu':
            if value['header']['frame_id'] != self.body_frame:
                raise ValueError('mount IMU into body axes before fusion')
            if self.use_orientation and value['orientation_covariance'][0] < 0:
                raise ValueError('configured IMU orientation is unavailable')
            if self.use_acceleration and value['linear_acceleration_covariance'][0] < 0:
                raise ValueError('configured IMU acceleration is unavailable')
            if value['angular_velocity_covariance'][0] < 0:
                raise ValueError('configured IMU angular velocity is unavailable')
        else:
            if value['child_frame_id'] != self.body_frame:
                raise ValueError('odometry child frame differs from body')
            if name == 'wheel':
                value['twist']['covariance'] = measurement_covariance(value['twist']['covariance'], 6,
                    [.03, .03, .1, .1, .1, .08]).ravel().tolist()
            else:
                if value['header']['frame_id'] != self.world_frame:
                    raise ValueError('scan and EKF must use the same odometry origin')
                cov = measurement_covariance(value['pose']['covariance'], 6,
                                              [.03, .03, .1, .1, .1, np.deg2rad(1.)])
                if max(cov[0, 0], cov[1, 1], cov[5, 5]) >= 1e3:
                    raise ValueError('scan registration is unavailable')
                value['pose']['covariance'] = cov.ravel().tolist()
        return value

    async def feed(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        for name, source in self.sources.items():
            if source.frames:
                await source.get(timeout=.01)
            for frame in source.frames:
                if frame.sequence <= self.previous[name]:
                    continue
                self.previous[name] = frame.sequence
                if frame.clock != 'ros:system':
                    raise ValueError('native EKF input clock changed')
                if frame.stamp_ns <= self.stamps[name]:
                    self.invalid += 1
                    continue
                try:
                    value = self.prepare(name, frame.data)
                except ValueError:
                    if name != 'scan':
                        raise
                    self.invalid += 1
                    continue
                msg = fill_ros2(Imu() if name == 'imu' else Odometry(), value)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(frame.stamp_ns, 10**9)
                self.publishers[name].publish(msg)
                self.stamps[name], self.received[name] = frame.stamp_ns, frame.received_ns
                self.counts[name] += 1

    async def receive(self):
        from nav_msgs.msg import Odometry
        if not self.filtered.output.frames:
            return
        frame = await self.filtered.get(timeout=.01)
        if frame.sequence <= self.last_output_sequence:
            return
        self.last_output_sequence = frame.sequence
        now = time.time_ns()
        if any(abs(now-self.stamps[key]) > self.input_timeout*1e9 for key in ('wheel', 'imu')):
            return
        if self.scan is not None and self.counts['scan'] == 0:
            return
        if frame.stamp_ns <= self.last_output_stamp:
            return
        data = frame.data
        if (data['header']['frame_id'], data['child_frame_id']) != (self.world_frame, self.body_frame):
            raise ValueError('EKF output frame changed')
        p, q = data['pose']['pose']['position'], data['pose']['pose']['orientation']
        position, rotation = [p[k] for k in 'xyz'], [q[k] for k in 'xyzw']
        if not np.isfinite(position + rotation).all() or abs(np.linalg.norm(rotation)-1) > .01:
            raise ValueError('EKF returned an invalid pose')
        pose = Pose(position=position, rotation=rotation,
                    wrd_frame=self.world_frame, ego_frame=self.body_frame)
        cov = measurement_covariance(data['pose']['covariance'], 6, [0.]*6)
        self.latest = fill_ros2(Odometry(), data)
        self.last_output_stamp = frame.stamp_ns
        if self.history is not None:
            self.history.add(frame.stamp_ns, pose)
        if self.covariance_history is not None:
            self.covariance_history.add(frame.stamp_ns, cov)
        meta = dict(stamp_ns=frame.stamp_ns, clock=frame.clock,
                    received_ns=min(frame.received_ns, frame.stamp_ns, self.received['wheel'], self.received['imu']))
        await self.pose.publish(pose, **meta)
        await self.odometry.publish(data, **meta)

    def diagnostics(self):
        now = time.time_ns()
        return dict(source='wheel_twist+imu_attitude_gyro_acceleration'+('+scan2d_pose' if self.scan else ''),
                    counts=dict(self.counts), invalid=self.invalid,
                    input_age_s={key: (now-value)*1e-9 for key, value in self.stamps.items()},
                    odometry_age_s=None if self.last_output_stamp < 0 else (now-self.last_output_stamp)*1e-9)

    async def report(self):
        await self.status.publish(self.diagnostics(), stamp_ns=time.time_ns(), clock='ros:system')

    async def close(self):
        for publisher in getattr(self, 'publishers', {}).values():
            self.node.destroy_publisher(publisher)
