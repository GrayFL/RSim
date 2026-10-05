"""Planar wheel-odometry / gyroscope EKF with graphmap pose outputs.

Input Signals carry the ROS-free Imu/Odometry dictionaries exposed by Chassis.
No ROS package is imported. State order: x, y, yaw, forward speed, yaw rate,
gyroscope bias; all angles inside the filter are radians.
"""
from __future__ import annotations

from collections import deque
import math

import numpy as np
from graphmap.pose import Pose

from .core import Component
from .signal import as_signal


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _vector(value, names):
    result = np.array([value[name] for name in names], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError("nonfinite sensor measurement")
    return result


def _covariance(values, size, indices, fallback):
    """ROS zero covariance means unknown, -1 means unavailable (IMU)."""
    array = np.asarray(values, dtype=float)
    if array.size != size * size or not np.isfinite(array).all():
        raise ValueError("invalid sensor covariance")
    if array.flat[0] < 0:
        raise ValueError("sensor measurement is marked unavailable")
    if not array.any():
        return np.diag(np.square(fallback))
    result = array.reshape(size, size)[np.ix_(indices, indices)]
    if not np.allclose(result, result.T) or np.linalg.eigvalsh(result).min() < -1e-10:
        raise ValueError("sensor covariance must be positive semidefinite")
    # Unspecified individual variances must not become perfect measurements.
    result = result.copy()
    for index, std in enumerate(fallback):
        if result[index, index] <= 0:
            result[index, index] = std ** 2
    return result + np.eye(len(indices)) * 1e-12


class PlanarEKF:
    """Small numerical EKF, independent of scheduling and sensor adapters.

    Constant forward speed / yaw-rate prediction with midpoint integration;
    random walks on speed, rate and gyro bias. Joseph-form covariance updates.
    Odometry contributes pose only, avoiding double counting its derived twist.
    """
    def __init__(self, *, acceleration_std=.5, angular_acceleration_std=1.,
                 bias_walk_std=.003):
        self.noise = np.array([acceleration_std, angular_acceleration_std, bias_walk_std])
        if not np.isfinite(self.noise).all() or (self.noise <= 0).any():
            raise ValueError("process noise standard deviations must be positive")
        self.x = np.zeros(6)
        self.P = np.diag([1., 1., 1., .25, .25, .01])
        self.stamp_ns = None

    def initialize(self, pose, covariance, stamp_ns):
        self.x[:] = 0
        self.x[:3] = pose
        self.P = np.diag([1., 1., 1., .25, .25, .01])
        self.P[:3, :3] = covariance
        self.stamp_ns = stamp_ns

    def predict(self, stamp_ns):
        if self.stamp_ns is None or stamp_ns < self.stamp_ns:
            raise ValueError("EKF requires initialized, monotonic timestamps")
        dt = (stamp_ns - self.stamp_ns) * 1e-9
        # Bounded integration steps keep Jacobians meaningful across packet loss.
        while dt > 1e-12:
            step = min(dt, .05)
            _, _, yaw, velocity, rate, _ = self.x
            angle = yaw + rate * step / 2
            c, s = math.cos(angle), math.sin(angle)
            F = np.eye(6)
            F[0, 2], F[0, 3], F[0, 4] = -velocity * s * step, c * step, -velocity * s * step**2 / 2
            F[1, 2], F[1, 3], F[1, 4] = velocity * c * step, s * step, velocity * c * step**2 / 2
            F[2, 4] = step
            self.x[0] += velocity * c * step
            self.x[1] += velocity * s * step
            self.x[2] = wrap_angle(yaw + rate * step)
            # Continuous white acceleration/bias spectral density approximation.
            G = np.zeros((6, 3))
            G[0, 0], G[1, 0], G[3, 0] = c * step / 2, s * step / 2, 1
            G[2, 1], G[4, 1], G[5, 2] = step / 2, 1, 1
            Q = (G * np.square(self.noise)) @ G.T * step
            self.P = F @ self.P @ F.T + Q
            dt -= step
        self.stamp_ns = stamp_ns

    def _update(self, residual, H, R):
        gain = np.linalg.solve(H @ self.P @ H.T + R, H @ self.P).T
        self.x += gain @ residual
        self.x[2] = wrap_angle(self.x[2])
        residual_map = np.eye(6) - gain @ H
        self.P = residual_map @ self.P @ residual_map.T + gain @ R @ gain.T
        self.P = (self.P + self.P.T) / 2

    def odometry(self, pose, covariance):
        H = np.eye(6)[:3]
        residual = np.asarray(pose) - self.x[:3]
        residual[2] = wrap_angle(residual[2])
        self._update(residual, H, covariance)

    def gyroscope(self, rate, variance):
        H = np.array([[0., 0., 0., 0., 1., 1.]])
        self._update(np.array([rate - self.x[4] - self.x[5]]), H, np.array([[variance]]))


class PlanarOdometry(Component):
    """Fuse wheel pose and IMU angular velocity; expose Signal[graphmap.Pose].

    T_body_imu explicitly maps the IMU axes into the odometry child frame.
    If omitted, IMU and body frame names must match. World/body names are taken
    from the first odometry packet and checked thereafter. Inputs must share a
    source clock. A bounded watermark waits for both streams, processes retained
    samples in timestamp order and discards samples arriving too late.
    """
    def __init__(self, odom, imu, *, T_body_imu=None, hz=100, history=128,
                 max_skew=.15, max_gap=1., odom_std=(.03, .03, .08),
                 gyro_std=.02, acceleration_std=.5, angular_acceleration_std=1.,
                 bias_walk_std=.003):
        self.odom, self.imu = as_signal(odom), as_signal(imu)
        super().__init__(inputs=(self.odom, self.imu))
        if self.odom.clock and self.imu.clock and self.odom.clock != self.imu.clock:
            raise ValueError("odometry and IMU require the same clock domain")
        values = [hz, max_skew, max_gap, gyro_std, *odom_std]
        if len(odom_std) != 3 or not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("rates, limits and noise standard deviations must be positive")
        if T_body_imu is not None and (not isinstance(T_body_imu, Pose)
                                       or not T_body_imu.is_rigid
                                       or not T_body_imu.wrd_frame or not T_body_imu.ego_frame):
            raise ValueError("T_body_imu must be a rigid graphmap Pose with both frame labels")
        self.T_body_imu = T_body_imu
        self.hz, self.max_skew_ns, self.max_gap_ns = hz, int(max_skew * 1e9), int(max_gap * 1e9)
        self.odom_std, self.gyro_std = tuple(odom_std), gyro_std
        self._filter_options = dict(acceleration_std=acceleration_std,
                                   angular_acceleration_std=angular_acceleration_std,
                                   bias_walk_std=bias_walk_std)
        self.pose = self.signal("pose", history=history)
        self.estimate = self.signal("estimate", history=history)

    async def open(self):
        self.filter = PlanarEKF(**self._filter_options)
        self._seen = {"imu": 0, "odom": 0}
        self._latest = {"imu": None, "odom": None}
        self._pending = {name: deque(maxlen=max(128, sig.history_size))
                         for name, sig in (("imu", self.imu), ("odom", self.odom))}
        self._clock = self._world = self._body = self._gyro_frame = None
        self.dropped = 0
        self.task("fuse", self._fuse, hz=self.hz)

    def _read_odom(self, data):
        world, body = data["header"]["frame_id"], data["child_frame_id"]
        if not world or not body:
            raise ValueError("odometry requires world and body frame labels")
        if self._world is None:
            self._world, self._body = world, body
        if (world, body) != (self._world, self._body):
            raise ValueError("odometry frame labels changed")
        value = data["pose"]["pose"]
        pose = Pose(position=_vector(value["position"], "xyz"),
                    rotation=_vector(value["orientation"], "xyzw"),
                    wrd_frame=world, ego_frame=body)
        covariance = _covariance(data["pose"]["covariance"], 6, [0, 1, 5], self.odom_std)
        return np.array([*pose.position[:2], pose.euler_rad[2]]), covariance

    def _read_imu(self, data):
        frame = data["header"]["frame_id"]
        transform = self.T_body_imu
        if transform is None:
            if frame != self._body:
                raise ValueError("IMU frame differs from body; supply T_body_imu")
            rotation = np.eye(3)
        else:
            if (transform.wrd_frame, transform.ego_frame) != (self._body, frame):
                raise ValueError("T_body_imu frame labels do not match sensor frames")
            rotation = transform.rot_mat
        vector = rotation @ _vector(data["angular_velocity"], "xyz")
        covariance = _covariance(data["angular_velocity_covariance"], 3, [0, 1, 2], [self.gyro_std] * 3)
        return float(vector[2]), float((rotation @ covariance @ rotation.T)[2, 2])

    async def _fuse(self):
        for name, signal in (("imu", self.imu), ("odom", self.odom)):
            frames = signal.frames
            if frames:
                # Propagate producer failure instead of trusting cached frames.
                await signal.get(timeout=0.01)
            for frame in frames:
                if frame.sequence <= self._seen[name]:
                    continue
                self._seen[name] = frame.sequence
                if self._clock is None:
                    self._clock = frame.clock
                if frame.clock != self._clock:
                    raise ValueError("odometry and IMU require one unchanged clock domain")
                if self._latest[name] is not None and frame.stamp_ns <= self._latest[name]:
                    self.dropped += 1
                    continue
                self._latest[name] = frame.stamp_ns
                if len(self._pending[name]) == self._pending[name].maxlen:
                    self.dropped += 1
                self._pending[name].append(frame)
        if any(value is None for value in self._latest.values()):
            return
        watermark = min(self._latest.values())
        # Odometry anchors each published estimate. Never publish extrapolated
        # poses indefinitely when either sensor has stopped.
        while self._pending["odom"] and self._pending["odom"][0].stamp_ns <= watermark:
            odom = self._pending["odom"].popleft()
            measurement, covariance = self._read_odom(odom.data)
            gyros = []
            while self._pending["imu"] and self._pending["imu"][0].stamp_ns <= odom.stamp_ns:
                gyros.append(self._pending["imu"].popleft())
            if gyros:
                self._gyro_frame = gyros[-1]
            if (self._gyro_frame is None or
                    odom.stamp_ns - self._gyro_frame.stamp_ns > self.max_skew_ns):
                self.dropped += 1
                continue
            ekf = self.filter
            if ekf.stamp_ns is None:
                ekf.initialize(measurement, covariance, odom.stamp_ns)
                ekf.gyroscope(*self._read_imu(self._gyro_frame.data))
            else:
                if odom.stamp_ns <= ekf.stamp_ns:
                    self.dropped += 1
                    continue
                if odom.stamp_ns - ekf.stamp_ns > self.max_gap_ns:
                    raise ValueError("sensor time gap exceeds max_gap; restart localization")
                for gyro in gyros:
                    if gyro.stamp_ns <= ekf.stamp_ns:
                        self.dropped += 1
                        continue
                    ekf.predict(gyro.stamp_ns)
                    ekf.gyroscope(*self._read_imu(gyro.data))
                ekf.predict(odom.stamp_ns)
                ekf.odometry(measurement, covariance)
            pose = Pose(x=ekf.x[0], y=ekf.x[1], yaw=ekf.x[2], degrees=False,
                        wrd_frame=self._world, ego_frame=self._body)
            # Preserve the older reception time for controller freshness checks.
            meta = dict(stamp_ns=odom.stamp_ns, clock=self._clock,
                        received_ns=min(odom.received_ns, self._gyro_frame.received_ns))
            await self.pose.publish(pose, **meta)
            await self.estimate.publish({"pose": pose, "covariance": ekf.P.copy(),
                                         "velocity": float(ekf.x[3]), "yaw_rate": float(ekf.x[4]),
                                         "gyro_bias": float(ekf.x[5]), "dropped": self.dropped}, **meta)
