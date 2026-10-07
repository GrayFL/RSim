"""Keyboard-driven differential chassis with virtual vehicle inertia.

Both signed speeds integrate applied acceleration minus Coulomb friction and
quadratic drag. Turning is bounded by a speed-dependent virtual steering angle,
with a smooth low-speed pivot contribution for a differential-drive chassis.
"""

from dataclasses import dataclass, asdict
import math

from rsim.core.commands import VelocityCommand


@dataclass(frozen=True)
class VehicleParameters:
    acceleration: float = 0.22
    friction: float = 0.06
    drag: float = 16.0
    angular_acceleration: float = 0.5
    angular_friction: float = 0.12
    angular_drag: float = 16.8888888889
    wheelbase: float = 0.45
    steering_max_deg: float = 35.0
    steering_speed_scale: float = 0.12
    pivot_rate: float = 0.15
    pivot_transition_speed: float = 0.035
    hz: float = 30.0
    command_ttl: float = 0.25
    max_loop_gap: float = 0.2

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError("vehicle parameters must be finite and positive")
        if (
            self.acceleration <= self.friction
            or self.angular_acceleration <= self.angular_friction
        ):
            raise ValueError("drive acceleration must exceed static friction")
        if (
            not 0 < self.steering_max_deg < 85
            or not 2 / self.hz < self.command_ttl <= 0.5
        ):
            raise ValueError("invalid steering angle or command timing")

    @property
    def terminal_speed(self):
        return math.sqrt((self.acceleration - self.friction) / self.drag)

    @property
    def terminal_yaw_rate(self):
        return math.sqrt(
            (self.angular_acceleration - self.angular_friction) / self.angular_drag
        )


class VehicleDynamics:
    def __init__(self, parameters=None):
        self.parameters = parameters or VehicleParameters()
        self.reset()

    def reset(self):
        self.linear = self.angular = 0.0

    def steering_limit(self, speed=None):
        p = self.parameters
        speed = self.linear if speed is None else speed
        return math.radians(p.steering_max_deg) / (
            1 + (speed / p.steering_speed_scale) ** 2
        )

    def yaw_limit(self, speed=None):
        p = self.parameters
        speed = self.linear if speed is None else speed
        car = abs(speed) / p.wheelbase * math.tan(self.steering_limit(speed))
        pivot = p.pivot_rate * math.exp(-((speed / p.pivot_transition_speed) ** 2))
        return min(p.terminal_yaw_rate, car + pivot)

    @staticmethod
    def _integrate(value, effort, accel, friction, drag, dt):
        sign = (
            math.copysign(1.0, value if value else effort) if value or effort else 0.0
        )
        result = (
            value + (effort * accel - sign * (friction + drag * value * value)) * dt
        )
        if not effort and value * result <= 0:
            return 0.0  # dry friction stops; it cannot reverse a resting vehicle
        terminal = math.sqrt((accel - friction) / drag)
        return max(-terminal, min(terminal, result))

    def step(self, keys, dt):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        if not set(keys) <= {"w", "s", "a", "d"}:
            raise ValueError("unknown driving key")
        p = self.parameters
        if dt > p.max_loop_gap:
            self.reset()
            raise ValueError("control loop stalled; vehicle state reset")
        throttle, steering = (
            int("w" in keys) - int("s" in keys),
            int("a" in keys) - int("d" in keys),
        )
        # Bounded substeps avoid frame-rate dependent friction sign oscillation.
        steps = max(1, math.ceil(dt / 0.005))
        for _ in range(steps):
            self.linear = self._integrate(
                self.linear, throttle, p.acceleration, p.friction, p.drag, dt / steps
            )
            self.angular = self._integrate(
                self.angular,
                steering,
                p.angular_acceleration,
                p.angular_friction,
                p.angular_drag,
                dt / steps,
            )
            limit = self.yaw_limit()
            self.angular = max(-limit, min(limit, self.angular))
        return VelocityCommand(self.linear, self.angular)

    def state(self):
        limit = self.yaw_limit()
        return dict(
            linear_x=self.linear,
            angular_z=self.angular,
            steering_limit_deg=math.degrees(self.steering_limit()),
            steering_deg=math.degrees(self.steering_limit())
            * (self.angular / limit if limit else 0.0),
            yaw_limit=limit,
            terminal_speed=self.parameters.terminal_speed,
        )
