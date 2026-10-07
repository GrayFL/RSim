"""Stationary, reference-checked calibration for planar angular velocity.

Gravity provides two tilt degrees of freedom. Yaw, translation, accelerometer
bias, scale and dynamic timing are deliberately not estimated from rest data.
"""
from dataclasses import asdict, dataclass
import math

import numpy as np
from graphmap.pose import Pose

from rsim.core.component import PrimaryComponent
from rsim.core.signal import as_signal


def _vector(message, field):
    result = np.array([message[field][axis] for axis in 'xyz'], dtype=float)
    covariance = np.asarray(message[field + '_covariance'], dtype=float)
    if (not np.isfinite(result).all() or covariance.size != 9 or
            not np.isfinite(covariance).all() or covariance.flat[0] < 0):
        raise ValueError('missing or invalid IMU measurement')
    return result


@dataclass(frozen=True)
class PlanarIMUCalibration:
    sensor_frame: str
    body_frame: str
    vertical_axis: tuple[float, float, float]
    gyro_bias: tuple[float, float, float]
    gyro_variance: float
    samples: int
    reference_samples: int
    reference_tilt_deg: float
    gravity_norm: float
    reference_gravity_norm: float
    status: str = 'stationary_level_planar_only'

    def __post_init__(self):
        axis, bias = np.asarray(self.vertical_axis), np.asarray(self.gyro_bias)
        if (axis.shape != (3,) or bias.shape != (3,) or not np.isfinite(axis).all() or
                not np.isfinite(bias).all() or not np.isclose(np.linalg.norm(axis), 1.) or
                not self.sensor_frame or not self.body_frame or
                not math.isfinite(self.gyro_variance) or self.gyro_variance <= 0 or
                self.samples < 100 or self.reference_samples < 100 or
                not 0 <= self.reference_tilt_deg <= 3 or
                not 8 <= self.gravity_norm <= 12 or not 8 <= self.reference_gravity_norm <= 12 or
                self.status != 'stationary_level_planar_only'):
            raise ValueError('invalid planar IMU calibration')

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, value):
        value = dict(value)
        for key in ('vertical_axis', 'gyro_bias'):
            value[key] = tuple(value[key])
        return cls(**value)

    def tilt_pose(self):
        """Minimal gravity alignment for inspection; NOT a full mount extrinsic.

        Translation is an arbitrary zero and yaw is unobservable. This Pose is
        not suitable for LiDAR deskew or transforming arbitrary sensor points.
        """
        axis = np.asarray(self.vertical_axis)
        target = np.array([0., 0., 1.])
        quaternion = np.r_[np.cross(axis, target), 1 + axis[2]]
        if np.linalg.norm(quaternion) < 1e-8:
            quaternion = np.array([1., 0., 0., 0.])
        return Pose(rotation=quaternion / np.linalg.norm(quaternion),
                    wrd_frame=self.body_frame, ego_frame=self.sensor_frame)


def calibrate_planar_imu(local, reference, *, reference_to_body):
    """Estimate gyro zero and vertical axis from simultaneous stationary windows.

    Inputs are sequences of SI-unit Imu-shaped dictionaries. The caller collects
    several seconds while wheel velocity is zero; both IMUs must be at rest and
    the known reference installation must confirm a nearly level body (<=3 deg).
    Static windows need no speculative alignment of independent source clocks.
    """
    if (not isinstance(reference_to_body, Pose) or not reference_to_body.is_rigid or
            not reference_to_body.wrd_frame or not reference_to_body.ego_frame):
        raise ValueError('reference_to_body requires a labelled rigid graphmap Pose')
    if len(local) < 100 or len(reference) < 100:
        raise ValueError('collect at least 100 stationary samples from each IMU')
    frame = local[0]['header']['frame_id']
    arrays = []
    for messages, expected in ((local, frame), (reference, reference_to_body.ego_frame)):
        if any(message['header']['frame_id'] != expected for message in messages):
            raise ValueError('IMU frame changed or reference mount does not match')
        gyro = np.array([_vector(message, 'angular_velocity') for message in messages])
        accel = np.array([_vector(message, 'linear_acceleration') for message in messages])
        if (np.max(np.linalg.norm(gyro, axis=1)) > .03 or
                np.max(np.std(accel, axis=0)) > .15 or
                np.max(np.linalg.norm(accel - accel.mean(axis=0), axis=1)) > .8):
            raise ValueError('IMUs must remain stationary during calibration')
        arrays.append((gyro, accel))
    (gyro, accel), (_, ref_accel) = arrays
    gravity, ref_gravity = accel.mean(axis=0), reference_to_body.rot_mat @ ref_accel.mean(axis=0)
    norm, ref_norm = np.linalg.norm(gravity), np.linalg.norm(ref_gravity)
    if not (8 <= norm <= 12 and 8 <= ref_norm <= 12):
        raise ValueError('acceleration must include gravity in m/s²')
    tilt = math.degrees(math.acos(float(np.clip(ref_gravity[2] / ref_norm, -1., 1.))))
    if tilt > 3:
        raise ValueError('reference IMU does not confirm a level body (<=3 degrees)')
    axis, bias = gravity / norm, gyro.mean(axis=0)
    return PlanarIMUCalibration(frame, reference_to_body.wrd_frame, tuple(axis), tuple(bias),
        max(float(np.var((gyro - bias) @ axis, ddof=1)), 1e-6), len(local), len(reference),
        tilt, float(norm), float(ref_norm))


class PlanarIMU(PrimaryComponent):
    """Project bias-corrected gyro onto the calibrated vertical, preserving time.

    This is a virtual planar measurement in the body frame: only angular Z is
    measured. X/Y are zero placeholders with large variance; orientation and
    acceleration are unavailable. Do not feed it to a 3D inertial estimator.
    """
    def __init__(self, source, calibration, *, hz=500, history=256):
        self.source = as_signal(source)
        if not isinstance(calibration, PlanarIMUCalibration):
            raise TypeError('expected PlanarIMUCalibration')
        super().__init__(inputs=(self.source,), output_name='imu', clock=self.source.clock, history=history)
        self.imu = self.output
        self.calibration, self.hz = calibration, hz

    async def open(self):
        self.previous = 0
        self.task('project', self.project, hz=self.hz)

    async def project(self):
        if not self.source.frames:
            return
        await self.source.get(timeout=.01)  # propagate producer failures
        for frame in self.source.frames:
            if frame.sequence <= self.previous:
                continue
            self.previous = frame.sequence
            cal = self.calibration
            if frame.data['header']['frame_id'] != cal.sensor_frame:
                raise ValueError('calibration sensor frame does not match input')
            rate = float((_vector(frame.data, 'angular_velocity') - cal.gyro_bias) @ cal.vertical_axis)
            data = dict(header={'frame_id': cal.body_frame},
                angular_velocity=dict(x=0., y=0., z=rate),
                angular_velocity_covariance=[1e6,0.,0.,0.,1e6,0.,0.,0.,cal.gyro_variance],
                orientation=dict(x=0.,y=0.,z=0.,w=1.), orientation_covariance=[-1.] + [0.] * 8,
                linear_acceleration=dict(x=0.,y=0.,z=0.), linear_acceleration_covariance=[-1.] + [0.] * 8,
                metadata={'measured_axes':'angular_velocity.z', 'calibration_status':cal.status,
                          'yaw_extrinsic_known':False, 'translation_known':False})
            await self.imu.publish(data, stamp_ns=frame.stamp_ns, clock=frame.clock,
                                   received_ns=frame.received_ns)
