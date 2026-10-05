"""In-memory chassis for the motion notebook; never connects to hardware."""
import math
import time

import numpy as np
from graphmap.pose import Pose

from rsim import Component, CommandSink, VelocityCommand


def odom_message(pose, *, position_std=.005, yaw_std=.01):
    covariance = np.diag([position_std**2, position_std**2, 1e6, 1e6, 1e6, yaw_std**2])
    return {"header": {"frame_id": pose.wrd_frame}, "child_frame_id": pose.ego_frame,
            "pose": {"pose": {"position": dict(zip("xyz", map(float, pose.position))),
                              "orientation": dict(zip("xyzw", map(float, pose.quat)))},
                     "covariance": covariance.ravel().tolist()}}


def imu_message(rate, *, frame="body", std=.01):
    return {"header": {"frame_id": frame},
            "angular_velocity": {"x": 0., "y": 0., "z": float(rate)},
            "angular_velocity_covariance": (np.eye(3) * std**2).ravel().tolist()}


class SimulatedChassis(Component):
    """Planar kinematics plus deterministic noisy odometry and biased gyro.

    speedup accelerates simulation time, not the controller's wall-clock TTL.
    Commands are recorded and applied solely to this Python object's state.
    """
    def __init__(self, *, hz=100, speedup=1., noise=True, gyro_bias=.025):
        super().__init__()
        self.imu = self.signal("imu", history=256, clock="simulation")
        self.odom = self.signal("odom", history=256, clock="simulation")
        self.truth = self.signal("truth", history=1024, clock="simulation")
        self.velocity = CommandSink(self, "velocity", self._apply, fallback=VelocityCommand(), hz=200)
        self.hz, self.speedup, self.noise, self.gyro_bias = hz, speedup, noise, gyro_bias
        self.commands = []

    async def _apply(self, envelope):
        self.command = envelope.value
        self.commands.append((time.monotonic(), envelope.value))

    async def open(self):
        self.command = VelocityCommand()
        self.state = np.zeros(3)
        self.clock_ns = 0
        self.rng = np.random.default_rng(12)
        self._last = time.monotonic()
        self.task("simulate", self._step, hz=self.hz)

    async def _step(self):
        now = time.monotonic()
        dt = min(now - self._last, .1) * self.speedup
        self._last = now
        self.clock_ns += max(1, int(dt * 1e9))
        distance, angle = self.command.linear_x * dt, self.command.angular_z * dt
        yaw = self.state[2] + angle / 2
        self.state += [distance * math.cos(yaw), distance * math.sin(yaw), angle]
        truth = Pose(x=self.state[0], y=self.state[1], yaw=self.state[2], degrees=False,
                     wrd_frame="odom", ego_frame="body")
        noise = self.rng.normal(size=3) * [.002, .002, .003] if self.noise else np.zeros(3)
        measured = Pose(x=self.state[0] + noise[0], y=self.state[1] + noise[1],
                        yaw=self.state[2] + noise[2], degrees=False,
                        wrd_frame="odom", ego_frame="body")
        gyro = self.command.angular_z + self.gyro_bias
        if self.noise:
            gyro += self.rng.normal(0, .003)
        meta = dict(stamp_ns=self.clock_ns, clock="simulation")
        await self.imu.publish(imu_message(gyro), **meta)
        await self.odom.publish(odom_message(measured), **meta)
        await self.truth.publish(truth, **meta)
