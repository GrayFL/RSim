"""Keyboard and joystick axes share one measured-state manual controller."""
import math
from .vehicle import TowingDynamics
from ..core.commands import VelocityCommand


class ManualController:
    def __init__(self):
        self.reset()

    def reset(self):
        self.y = self.x = 0.
        self.brake_locked = False
        self.steering_angle = self.steering_speed = 0.
        self.heading_origin = None

    def input(self, y, x, measured, yaw=None):
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (y, x)): raise ValueError('manual axes must be in [-1, 1]')
        old_sign, new_sign = (0 if not self.y else math.copysign(1, self.y)), (0 if not y else math.copysign(1, y))
        if new_sign != old_sign:
            self.brake_locked = bool(y * measured[0] < -.001)
        if y == 0: self.brake_locked = False
        if self.heading_origin is None and yaw is not None: self.heading_origin = yaw
        self.y, self.x = y, x

    def target(self, measured, yaw, model, axes, limits, dt=.1):
        p = model.parameters
        v, w = measured
        linear_limit, angular_limit = limits
        if self.brake_locked:
            goal_v = 0.
        elif axes['y_mode'] == 'velocity_target':
            goal_v = self.y * linear_limit
        elif not self.y:
            # Center is a brake/deadman, never an uncommanded coast.
            goal_v = 0.
        else:
            effort = abs(self.y) * p.linear_acceleration
            goal_v = float(TowingDynamics._axis(v, math.copysign(linear_limit, self.y), dt,
                                                effort, p.linear_friction, p.linear_drag, p.linear_deceleration))
        next_v = float(TowingDynamics._axis(v, goal_v, dt, p.linear_acceleration,
                                            p.linear_friction, p.linear_drag, p.linear_deceleration))
        x_mode = axes['x_mode']
        if x_mode == 'velocity_target':
            goal_w = self.x * angular_limit
        elif x_mode == 'acceleration':
            goal_w = float(TowingDynamics._axis(w, math.copysign(angular_limit, self.x) if self.x else 0., dt,
                                                max(1e-9, abs(self.x) * p.angular_acceleration),
                                                p.angular_friction, p.angular_drag, p.angular_deceleration))
        elif axes.get('angle_reference', 'steering') == 'heading':
            if self.heading_origin is None: self.heading_origin = yaw
            error = (self.heading_origin + self.x * math.radians(p.heading_max_deg) - yaw + math.pi) % (2 * math.pi) - math.pi
            goal_w = math.copysign(min(angular_limit, math.sqrt(2 * p.angular_deceleration * abs(error)), abs(error) / dt), error)
        else:
            target = self.x * math.radians(p.steering_max_deg)
            error = target - self.steering_angle
            desired = math.copysign(min(p.steering_rate, math.sqrt(2 * p.steering_deceleration * abs(error)), abs(error) / dt), error)
            self.steering_speed = float(TowingDynamics._axis(self.steering_speed, desired, dt,
                                         p.steering_acceleration, 0., 0., p.steering_deceleration))
            change = self.steering_speed * dt
            if error * change >= 0 and abs(change) >= abs(error):
                self.steering_angle, self.steering_speed = target, 0.
            else: self.steering_angle += change
            # Signed speed reverses yaw, like a front-steered vehicle.
            goal_w = next_v * math.tan(self.steering_angle) / p.wheelbase
        goal_w = max(-angular_limit, min(angular_limit, goal_w))
        return VelocityCommand(goal_v, goal_w)
