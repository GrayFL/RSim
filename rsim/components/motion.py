"""Async relative chassis movements driven by graphmap pose feedback."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
import math
import time

from graphmap.pose import Pose

from rsim.core.commands import CommandSink, VelocityCommand
from rsim.core.component import Component, ComponentError
from .odometry import PlanarOdometry
from rsim.core.signal import as_signal


class MotionError(ComponentError):
    """A requested movement could not safely complete."""


@dataclass
class _Movement:
    kind: str
    target: float
    result: asyncio.Future
    start: Pose | None = None
    previous: Pose | None = None
    sequence: int = 0
    angle: float = 0.
    settled: int = 0


class ChassisController(Component):
    """Relative move / rotate on one event loop; one motion at a time.

    Pass a Chassis-like component, or explicit pose and velocity ports to reuse
    another estimator/provider. Motion is disabled by default: zero-distance
    requests and stop work, nonzero targets require motion_enabled=True.
    Methods return the final graphmap Pose. Cancellation, stale feedback, timeout
    and Runtime closure stop the actuator. Provider TTL remains the independent
    safeguard if this process or event loop stalls.
    """
    def __init__(self, chassis=None, *, pose=None, velocity=None, T_body_imu=None,
                 motion_enabled=False, hz=30, max_linear=.15, max_angular=.5,
                 min_angular=0.,
                 linear_acceleration=.3, angular_acceleration=1.,
                 distance_tolerance=.01, angle_tolerance=.02, pose_timeout=.5,
                 command_ttl=.25, settle_samples=3, ekf_options=None):
        if chassis is not None:
            if pose is not None or velocity is not None:
                raise ValueError("pass chassis or explicit pose/velocity ports")
            self.odometry = PlanarOdometry(chassis.odom, chassis.imu,
                                          T_body_imu=T_body_imu, **(ekf_options or {}))
            pose, velocity = self.odometry.pose, chassis.velocity
        else:
            self.odometry = None
        if not isinstance(velocity, CommandSink):
            raise TypeError("velocity must be a CommandSink")
        source = as_signal(pose)
        super().__init__(velocity.producer, inputs=(source,))
        values = [hz, max_linear, max_angular, linear_acceleration, angular_acceleration,
                  distance_tolerance, angle_tolerance, pose_timeout, command_ttl]
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("controller rates, limits and tolerances must be positive")
        if not math.isfinite(min_angular) or not 0 <= min_angular <= max_angular:
            raise ValueError("min_angular must be finite and between zero and max_angular")
        if not isinstance(settle_samples, int) or settle_samples < 1:
            raise ValueError("settle_samples must be a positive integer")
        if command_ttl > velocity.guard.max_ttl_ns * 1e-9 or command_ttl <= 2 / hz:
            raise ValueError("command_ttl must allow two control ticks and fit provider TTL")
        self.pose = self.signal("pose", history=source.history_size, clock=source.clock)
        self.pose._target = source
        self.velocity, self.command_targets = velocity, (velocity,)
        self.motion_enabled = bool(motion_enabled)
        self.hz, self.max_linear, self.max_angular = hz, max_linear, max_angular
        self.min_angular = min_angular
        self.linear_acceleration, self.angular_acceleration = linear_acceleration, angular_acceleration
        self.distance_tolerance, self.angle_tolerance = distance_tolerance, angle_tolerance
        self.pose_timeout, self.command_ttl, self.settle_samples = pose_timeout, command_ttl, settle_samples
        self._movement = None
        self._command_lock = None
        self._sent = False

    async def open(self):
        self._movement = None
        self._command_lock = asyncio.Lock()
        self._sent = False
        self.task("motion", self._tick, hz=self.hz)

    def _available(self):
        if self._binding is not None:
            raise MotionError("move/rotate are local methods; keep the controller in the calling process")
        if self._closed or self._closing or self._failure is not None:
            raise MotionError("controller is not running") from self._failure

    async def move(self, distance_m, *, timeout=None):
        """Travel along the starting body's X axis in metres; negative reverses."""
        return await self._run("move", float(distance_m), timeout)

    async def rotate(self, yaw_deg=None, *, yaw_rad=None, timeout=None):
        """Relative turn: exactly one angle unit, positive left, negative right.

        Angles may exceed 180 degrees or a full turn; progress is unwrapped.
        """
        if (yaw_deg is None) == (yaw_rad is None):
            raise ValueError("specify exactly one of yaw_deg or yaw_rad")
        angle = math.radians(float(yaw_deg)) if yaw_deg is not None else float(yaw_rad)
        return await self._run("rotate", angle, timeout)

    async def _run(self, kind, target, timeout):
        self._available()
        if not math.isfinite(target):
            raise ValueError("motion target must be finite")
        if target != 0 and not self.motion_enabled:
            raise MotionError("nonzero movement requires motion_enabled=True")
        if self._movement is not None:
            raise MotionError("another movement is active; await it or call stop()")
        speed = self.max_linear if kind == "move" else self.max_angular
        timeout = 5 + 3 * abs(target) / speed if timeout is None else timeout
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        movement = _Movement(kind, target, asyncio.get_running_loop().create_future())
        self._movement = movement
        failed = asyncio.create_task(self.wait())
        try:
            async with asyncio.timeout(timeout):
                done, _ = await asyncio.wait((movement.result, failed), return_when=asyncio.FIRST_COMPLETED)
                if movement.result in done:
                    return movement.result.result()
                try:
                    await failed
                except ComponentError as error:
                    raise MotionError("controller closed or failed") from error
        finally:
            failed.cancel()
            await asyncio.gather(failed, return_exceptions=True)
            if not movement.result.done():
                movement.result.cancel()
            elif not movement.result.cancelled():
                movement.result.exception()
            if self._movement is movement:
                self._movement = None
                # Serialize behind any in-flight command, so cancellation cannot
                # be followed by a late nonzero command from the same tick.
                await self._zero()

    async def _send(self, command, *, ttl=None):
        self._sent = True
        async with asyncio.timeout(max(.5, self.command_ttl * 2)):
            await self.velocity.set(command, ttl=self.command_ttl if ttl is None else ttl, _writer=self)

    async def _zero(self):
        async with self._command_lock:
            await self._send(VelocityCommand())

    async def stop(self):
        """Interrupt the current movement and acknowledge a zero command."""
        self._available()
        movement, self._movement = self._movement, None
        try:
            await self._zero()
        finally:
            if movement is not None and not movement.result.done():
                movement.result.set_exception(MotionError("movement interrupted by stop()"))

    async def drive(self, command, *, ttl=None):
        """One manual velocity update; mutually exclusive with move/rotate."""
        self._available()
        if not isinstance(command, VelocityCommand):
            raise TypeError('drive requires VelocityCommand')
        if command != VelocityCommand() and not self.motion_enabled:
            raise MotionError('nonzero movement requires motion_enabled=True')
        if abs(command.linear_x)>self.max_linear or abs(command.angular_z)>self.max_angular:
            raise MotionError('manual velocity exceeds configured controller limits')
        async with self._command_lock:
            if self._movement is not None:
                raise MotionError('another movement is active')
            if command != VelocityCommand():
                frame = await self.pose.get(timeout=self.pose_timeout)
                if time.time_ns()-frame.received_ns > self.pose_timeout*1e9:
                    raise MotionError('pose feedback is stale')
            await self._send(command, ttl=ttl)

    async def close(self):
        movement, self._movement = self._movement, None
        try:
            if self._sent:
                await self._zero()
        finally:
            if movement is not None and not movement.result.done():
                movement.result.set_exception(MotionError("controller closed"))

    @staticmethod
    def _speed(error, maximum, acceleration, gain, minimum=0.):
        return math.copysign(min(maximum, max(minimum, gain * abs(error)),
                                 math.sqrt(2 * acceleration * abs(error))), error)

    async def _tick(self):
        movement = self._movement
        if movement is None:
            return
        try:
            frame = await self.pose.get(timeout=self.pose_timeout)
            if time.time_ns() - frame.received_ns > int(self.pose_timeout * 1e9):
                raise MotionError("pose feedback is stale")
            pose = frame.data
            if not isinstance(pose, Pose) or not pose.is_rigid or not pose.wrd_frame or not pose.ego_frame:
                raise MotionError("controller requires a labelled rigid graphmap Pose")
            if movement.start is None:
                movement.start = pose.copy()
                movement.previous = pose.copy()
            if (pose.wrd_frame, pose.ego_frame) != (movement.start.wrd_frame, movement.start.ego_frame):
                raise MotionError("pose frame labels changed during movement")
            fresh = frame.sequence != movement.sequence
            if fresh:
                relative = (~movement.previous) * pose
                movement.angle += float(relative.euler_rad[2])
                movement.previous = pose.copy()
                movement.sequence = frame.sequence
            if movement.kind == "move":
                relative = (~movement.start) * pose
                error = movement.target - float(relative.position[0])
                heading = float(relative.euler_rad[2])
                cross_track = float(relative.position[1])
                reached = abs(error) <= self.distance_tolerance and abs(heading) <= self.angle_tolerance
                linear = self._speed(error, self.max_linear, self.linear_acceleration, 1.5)
                angular = max(-self.max_angular, min(self.max_angular,
                              -2 * heading - math.copysign(1, movement.target) * cross_track))
                command = VelocityCommand(linear, angular)
            else:
                error = movement.target - movement.angle
                reached = abs(error) <= self.angle_tolerance
                command = VelocityCommand(0., self._speed(error, self.max_angular,
                                                          self.angular_acceleration, 2., self.min_angular))
            # A zero request is a stop probe, never a drift-correction movement.
            if movement.target == 0:
                reached = True
            if fresh:
                movement.settled = movement.settled + 1 if reached else 0
            if reached:
                command = VelocityCommand()
            async with self._command_lock:
                if self._movement is not movement:
                    return
                await self._send(command)
            if movement.settled >= self.settle_samples:
                if not movement.result.done():
                    movement.result.set_result(pose.copy())
        except asyncio.CancelledError:
            raise
        except Exception as error:
            if self._movement is movement:
                self._movement = None
                try:
                    await self._zero()
                finally:
                    if not movement.result.done():
                        movement.result.set_exception(error)
