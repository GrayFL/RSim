"""Motion-adaptive keyframe rates from local graphmap pose history."""
import math

import numpy as np


class MotionKeyframeRate:
    """Use windowed SE(3) motion, never map-frame loop-closure corrections.

    Either translation or rotation can raise the rate to ``max_hz``. Missing
    history selects the cap rather than misclassifying unknown motion as rest.
    This policy only selects mapping inputs; it does not throttle odometry.
    """
    def __init__(self, history, *, max_hz, min_hz=.1, linear_speed_m_s=.3,
                 angular_speed_rad_s=.5, window_s=.5,
                 linear_deadband_m_s=.01, angular_deadband_rad_s=.02):
        positive = (min_hz, max_hz, linear_speed_m_s, angular_speed_rad_s, window_s)
        deadbands = (linear_deadband_m_s, angular_deadband_rad_s)
        if (any(not math.isfinite(v) or v <= 0 for v in positive)
                or any(not math.isfinite(v) or v < 0 for v in deadbands)
                or min_hz > max_hz or window_s < 1e-9
                or linear_deadband_m_s >= linear_speed_m_s
                or angular_deadband_rad_s >= angular_speed_rad_s):
            raise ValueError('invalid adaptive keyframe rate bounds or speed thresholds')
        self.history = history
        self.min_hz, self.max_hz = float(min_hz), float(max_hz)
        self.linear_reference, self.angular_reference = linear_speed_m_s, angular_speed_rad_s
        self.linear_deadband, self.angular_deadband = deadbands
        self.window_ns = round(window_s * 1e9)
        self.linear_speed = self.angular_speed = None

    def frequency(self, stamp_ns):
        previous = self.history.at(stamp_ns - self.window_ns)
        current = self.history.at(stamp_ns)
        self.linear_speed = self.angular_speed = None
        if previous is None or current is None:
            return self.max_hz
        if ((previous.wrd_frame, previous.ego_frame) != (current.wrd_frame, current.ego_frame)
                or previous.scale != 1 or current.scale != 1):
            raise ValueError('keyframe motion needs SE(3) poses in the same local frames')
        delta = ~previous * current
        seconds = self.window_ns * 1e-9
        self.linear_speed = float(np.linalg.norm(delta.position) / seconds)
        self.angular_speed = float(delta.rotation.magnitude() / seconds)
        if not math.isfinite(self.linear_speed + self.angular_speed):
            raise ValueError('keyframe motion is not finite')
        fraction = max(
            (self.linear_speed - self.linear_deadband) / (self.linear_reference - self.linear_deadband),
            (self.angular_speed - self.angular_deadband) / (self.angular_reference - self.angular_deadband),
        )
        return self.min_hz + (self.max_hz - self.min_hz) * min(1., max(0., fraction))

    def diagnostics(self):
        return dict(linear_speed_m_s=self.linear_speed, angular_speed_rad_s=self.angular_speed,
                    window_s=self.window_ns * 1e-9, min_hz=self.min_hz,
                    history_ready=self.linear_speed is not None)
