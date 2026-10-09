"""IMU mounting, covariance and gravity handling without ROS dependencies."""
import copy
import numpy as np
from graphmap.pose import Pose

from rsim.core.component import PrimaryComponent
from rsim.core.signal import as_signal
from .odometry import _covariance, _vector


class BodyIMU(PrimaryComponent):
    """Express an ENU IMU in the body frame using a graphmap mount.

    Full attitude is used to remove gravity, even for planar localization.
    Biases are in sensor axes and SI units; zero means no additional calibration.
    Missing orientation/acceleration stays unavailable, never a zero observation.
    """
    def __init__(self, source, mount, *, gravity=9.8, remove_gravity=True,
                 gyro_bias=(0., 0., 0.), acceleration_bias=(0., 0., 0.),
                 orientation_std=.07, gyro_std=.02, acceleration_std=.3,
                 history=512, hz=500):
        self.source = as_signal(source)
        self.mount = Pose(**mount) if isinstance(mount, dict) else mount
        if (not isinstance(self.mount, Pose) or not self.mount.is_rigid
                or not self.mount.wrd_frame or not self.mount.ego_frame):
            raise ValueError('IMU mount requires a rigid graphmap Pose with frame labels')
        if not all(np.isfinite(v) and v > 0 for v in (gravity, orientation_std, gyro_std, acceleration_std, hz)):
            raise ValueError('IMU noise, gravity and rate must be positive')
        super().__init__(inputs=(self.source,), output_name='imu', history=history, clock=self.source.clock)
        self.imu = self.output
        self.gravity, self.remove_gravity, self.hz = gravity, remove_gravity, hz
        self.biases = {key: np.asarray(value, dtype=float) for key, value in (
            ('angular_velocity', gyro_bias), ('linear_acceleration', acceleration_bias))}
        if any(value.shape != (3,) or not np.isfinite(value).all() for value in self.biases.values()):
            raise ValueError('IMU biases must contain three finite values')
        self.noise = dict(orientation=orientation_std, angular_velocity=gyro_std,
                          linear_acceleration=acceleration_std)
        self.previous = 0

    def convert(self, data):
        if data['header']['frame_id'] != self.mount.ego_frame:
            raise ValueError('IMU frame differs from configured mount')
        result, R = copy.deepcopy(data), self.mount.rot_mat
        result['header']['frame_id'] = self.mount.wrd_frame
        attitude = None
        for name, std in self.noise.items():
            if data[name + '_covariance'][0] < 0:
                continue
            cov = _covariance(data[name + '_covariance'], 3, [0, 1, 2], [std]*3)
            cov = R @ cov @ R.T
            cov[np.diag_indices(3)] = np.maximum(np.diag(cov), std**2)
            result[name + '_covariance'] = cov.ravel().tolist()
            if name == 'orientation':
                quat = _vector(data[name], 'xyzw')
                if abs(np.linalg.norm(quat)-1) > .01:
                    raise ValueError('invalid IMU orientation quaternion')
                world_imu = Pose(rotation=quat, wrd_frame='imu_navigation', ego_frame=self.mount.ego_frame)
                attitude = world_imu * ~self.mount
                result[name] = dict(zip('xyzw', map(float, attitude.quat)))
            else:
                vector = R @ (_vector(data[name], 'xyz') - self.biases[name])
                if name == 'linear_acceleration' and self.remove_gravity:
                    if attitude is None:
                        result[name + '_covariance'] = [-1.] + [0.]*8
                        continue
                    vector -= attitude.rot_mat.T @ np.array([0., 0., self.gravity])
                result[name] = dict(zip('xyz', map(float, vector)))
        return result

    async def open(self):
        self.previous = 0
        self.task('body-imu', self.update, hz=self.hz)

    async def update(self):
        await self.source.get()
        for frame in self.source.frames:
            if frame.sequence <= self.previous:
                continue
            self.previous = frame.sequence
            await self.publish(self.convert(frame.data), stamp_ns=frame.stamp_ns, clock=frame.clock,
                               received_ns=frame.received_ns)
