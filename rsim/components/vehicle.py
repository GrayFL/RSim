"""Keyboard-driven differential chassis with virtual vehicle inertia.

Both signed speeds integrate applied acceleration minus Coulomb friction and
quadratic drag. Turning is bounded by a speed-dependent virtual steering angle,
with a smooth low-speed pivot contribution for a differential-drive chassis.
"""

from dataclasses import dataclass, asdict
import math

from rsim.core.commands import VelocityCommand


@dataclass(frozen=True)
class TowingParameters:
    """Chassis dynamics in SI units, shared by both driving modes.

    Acceleration is a bound on drive effort per unit mass and on net braking.
    Drag/friction act before the 1.6 m/s chassis cap. The unconstrained drive
    equilibrium is deliberately higher than the cap, so full input reaches it.
    rear_extent describes the towing attachment/person, not the pedal state.
    """
    max_linear: float = 1.6
    max_angular: float = .8
    linear_acceleration: float = .6
    angular_acceleration: float = .8
    linear_deceleration: float = 1.2
    angular_deceleration: float = 1.6
    linear_friction: float = .05
    linear_drag: float = .15
    angular_friction: float = .08
    angular_drag: float = .7
    min_turn_radius: float = .3
    rear_extent: float = .81
    rear_lateral_speed: float = .12
    max_loop_gap: float = .3
    wheelbase: float = .45
    steering_max_deg: float = 35.
    steering_rate: float = .8
    steering_acceleration: float = 1.6
    steering_deceleration: float = 2.
    heading_max_deg: float = 90.

    def __post_init__(self):
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('drive parameters must be finite and positive')
        if self.max_linear > 1.6:
            raise ValueError('chassis speed cannot exceed 1.6 m/s')
        if self.max_angular > .8 or self.steering_max_deg >= 85 or self.heading_max_deg > 180 or not .1 <= self.max_loop_gap <= .5:
            raise ValueError('invalid angular or steering limit')
        if self.linear_acceleration <= self.linear_friction or self.angular_acceleration <= self.angular_friction:
            raise ValueError('drive effort must exceed static friction')

    @property
    def terminal_speed(self):
        return math.sqrt((self.linear_acceleration - self.linear_friction) / self.linear_drag)

    @property
    def terminal_yaw_rate(self):
        return math.sqrt((self.angular_acceleration - self.angular_friction) / self.angular_drag)


class TowingDynamics:
    """Force/drag dynamics with signed travel and non-pivot towing constraints.

    Targets are desired velocities. A bounded drive force tracks each target;
    Coulomb and quadratic resistance reduce acceleration at higher speed. This
    same transition is used for commands, prediction, simulation and braking.
    """
    mode = 'towing'
    def __init__(self, parameters=None):
        self.parameters = parameters or TowingParameters()

    def yaw_limit(self, speed):
        import numpy as np
        p = self.parameters
        return np.minimum(np.abs(speed) / p.min_turn_radius,
                          min(p.max_angular, p.rear_lateral_speed / p.rear_extent))

    def targets(self, throttle, steering):
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (throttle, steering)):
            raise ValueError('manual axes must be finite and in [-1, 1]')
        speed = throttle * self.parameters.max_linear
        return VelocityCommand(speed, steering * math.copysign(float(self.yaw_limit(speed)), speed))

    def advance_velocity(self, current, target, dt):
        """Vectorized [v,w] transition, also used for the braking tail."""
        import numpy as np
        p = self.parameters
        if not math.isfinite(dt) or not 0 < dt <= p.max_loop_gap:
            raise ValueError('invalid or stalled control interval')
        current, target = np.asarray(current, float), np.asarray(target, float)
        if not np.isfinite(current).all() or not np.isfinite(target).all():
            raise ValueError('velocity must be finite')
        goal_v = np.clip(target[..., 0], -p.max_linear, p.max_linear)
        goal_w = np.clip(target[..., 1], -self.yaw_limit(goal_v), self.yaw_limit(goal_v))
        v = self._axis(current[..., 0], goal_v, dt, p.linear_acceleration, p.linear_friction, p.linear_drag, p.linear_deceleration)
        w = self._axis(current[..., 1], goal_w, dt, p.angular_acceleration, p.angular_friction, p.angular_drag, p.angular_deceleration)
        # Retain forward travel until yaw has decelerated: never turn a braking
        # trajectory into an unmodelled pivot by independently clipping v to zero.
        if self.mode == 'towing':
            direction = np.where(abs(v) > 1e-10, np.sign(v), np.sign(current[..., 0]))
            v = direction * np.maximum(abs(v), np.abs(w) * p.min_turn_radius)
        w = np.clip(w, -self.yaw_limit(v), self.yaw_limit(v))
        return np.stack((v, w), axis=-1)

    @staticmethod
    def _axis(current, target, dt, acceleration, friction, drag, deceleration=None):
        import numpy as np
        value, target = np.broadcast_arrays(np.asarray(current, float), np.asarray(target, float))
        value = value.copy()
        deceleration = acceleration if deceleration is None else deceleration
        # Mid-step sign changes must not turn friction into an oscillating force.
        count = max(1, math.ceil(dt / .02))
        h = dt / count
        for _ in range(count):
            error = target - value
            braking = value * error < 0
            sign = np.where(abs(value) > 1e-10, np.sign(value), np.sign(error))
            resistance = sign * (friction + drag * value ** 2)
            limit = np.where(braking, deceleration, acceleration)
            drive = np.clip(error / h + resistance, -limit, limit)
            net = np.clip(drive - resistance, -limit, limit)
            net = np.where((abs(value) < 1e-10) & (net * error < 0), 0., net)
            updated = value + net * h
            # A reverse target brakes to zero before accelerating in reverse.
            updated = np.where(value * updated < 0, 0., updated)
            # Do not overshoot the requested velocity or reverse through zero
            # when braking. Maintaining a target supplies only its resistance.
            value = np.where(error * (target - updated) <= 0, target, updated)
        return value

    def step(self, measured, target, dt):
        result = self.advance_velocity([measured.linear_x, measured.angular_z],
                                       [target.linear_x, target.angular_z], dt)
        return VelocityCommand(*map(float, result))


class DifferentialDynamics(TowingDynamics):
    """The same force model with signed travel and independent pivot turns."""
    mode = 'differential'

    def yaw_limit(self, speed):
        import numpy as np
        return np.full_like(np.asarray(speed, float), self.parameters.max_angular)

    def targets(self, throttle, steering):
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (throttle, steering)):
            raise ValueError('manual axes must be finite and in [-1, 1]')
        return VelocityCommand(throttle * self.parameters.max_linear, steering * self.parameters.max_angular)


def drive_model(parameters=None, mode='towing'):
    if mode == 'towing': return TowingDynamics(parameters)
    if mode == 'differential': return DifferentialDynamics(parameters)
    raise ValueError('drive mode must be towing or differential')


@dataclass(frozen=True)
class VehicleParameters:
    acceleration: float = 0.4
    friction: float = 0.06
    drag: float = 4.0
    angular_acceleration: float = 1.2
    angular_friction: float = 0.12
    angular_drag: float = 5.5
    wheelbase: float = 0.45
    steering_max_deg: float = 35.0
    steering_speed_scale: float = 0.24
    pivot_rate: float = 0.4
    pivot_transition_speed: float = 0.07
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
