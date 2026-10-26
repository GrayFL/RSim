"""Keyboard and joystick axes share one measured-state manual controller."""
import math
from .vehicle import TowingDynamics
from ..core.commands import VelocityCommand


class ManualController:
    def __init__(self):
        self.reset()

    def reset(self):
        self.y = self.x = 0.
        self.source = 'keys'
        self.active = False
        self.brake_locked = False
        self.steering_angle = self.steering_speed = 0.
        self.heading_origin = None

    def input(self, y, x, measured, yaw=None, *, source='keys', active=True):
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (y, x)): raise ValueError('manual axes must be in [-1, 1]')
        if source not in ('keys', 'joystick'): raise ValueError('invalid manual input source')
        if type(active) is not bool or (not active and (y or x)):
            raise ValueError('released manual input must have zero axes')
        self.source, self.active = source, active
        old_sign, new_sign = (0 if not self.y else math.copysign(1, self.y)), (0 if not y else math.copysign(1, y))
        if new_sign != old_sign:
            # Wheel/filtered odometry jitters around zero. A fresh press after
            # stopping must not latch the brake because of a few mm/s of noise.
            self.brake_locked = bool(new_sign * measured[0] < -.03)
        if y == 0: self.brake_locked = False
        if self.heading_origin is None and yaw is not None: self.heading_origin = yaw
        self.y, self.x = y, x

    @staticmethod
    def coast_reference(measured, previous):
        # Wheel noise must not add drive effort after the operator releases an
        # input. Follow faster measured slowing, but never grow the
        # reference or reverse its sign while coasting.
        if previous is None: return measured
        return math.copysign(min(abs(measured), abs(previous)), measured) if measured * previous > 0 else 0.

    def target(self, measured, yaw, model, axes, limits, dt=.1, *, previous=None):
        p = model.parameters
        v, w = measured
        linear_limit, angular_limit = limits
        coast = axes.get('release_mode', 'coast') == 'coast'
        def neutral(index, friction, drag, deceleration):
            if not coast: return 0.
            reference = self.coast_reference(measured[index], previous[index] if previous is not None else None)
            return TowingDynamics.coast_axis(reference, dt, friction, drag, deceleration)
        if not self.active:
            # Pointer release and holding a centered joystick are different
            # intents, even though both report zero axes. Axis binding does not
            # determine whether release selects neutral or a zero-speed target.
            return VelocityCommand(neutral(0, p.linear_friction, p.linear_drag, p.linear_deceleration),
                                   neutral(1, p.angular_friction, p.angular_drag, p.angular_deceleration))
        if getattr(self, 'source', 'keys') == 'joystick' and axes['y_mode'] == 'velocity_target':
            self.brake_locked = False
        if self.brake_locked:
            goal_v = 0.
        elif not self.y:
            goal_v = (neutral(0, p.linear_friction, p.linear_drag, p.linear_deceleration)
                      if self.source == 'keys' else 0.)
        elif axes['y_mode'] == 'velocity_target':
            goal_v = self.y * linear_limit
        else:
            effort = abs(self.y) * p.linear_acceleration
            goal_v = float(TowingDynamics._axis(v, math.copysign(linear_limit, self.y), dt,
                                                effort, p.linear_friction, p.linear_drag, p.linear_deceleration))
        next_v = float(TowingDynamics._axis(v, goal_v, dt, p.linear_acceleration,
                                            p.linear_friction, p.linear_drag, p.linear_deceleration))
        x_mode = axes['x_mode']
        if not self.x and x_mode != 'angle_target':
            goal_w = (neutral(1, p.angular_friction, p.angular_drag, p.angular_deceleration)
                      if self.source == 'keys' else 0.)
        elif x_mode == 'velocity_target':
            goal_w = self.x * angular_limit
        elif x_mode == 'acceleration':
            goal_w = float(TowingDynamics._axis(w, math.copysign(angular_limit, self.x), dt,
                                               abs(self.x) * p.angular_acceleration,
                                               p.angular_friction, p.angular_drag, p.angular_deceleration))
        elif axes.get('angle_reference', 'steering') == 'heading':
            if self.heading_origin is None: self.heading_origin = yaw
            error = (self.heading_origin + self.x * math.radians(p.heading_max_deg) - yaw + math.pi) % (2 * math.pi) - math.pi
            # Releasing all controls must not turn back to the initial heading.
            goal_w = (math.copysign(min(angular_limit, math.sqrt(2 * p.angular_deceleration * abs(error)), abs(error) / dt), error)
                      if self.y or self.x else 0.)
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
