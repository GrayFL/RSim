"""Bounded acquisition-time interpolation, independent of transport latency."""
from bisect import bisect_left
from collections import deque

import numpy as np


class SampleHistory:
    """Interpolate numeric samples only between nearby, ordered observations.

    Matrices (including covariance) use a convex combination, without assuming
    the two observations are statistically independent. Never extrapolates.
    """
    def __init__(self, *, capacity=4096, max_gap_s=.15):
        if capacity < 2 or not np.isfinite(max_gap_s) or max_gap_s <= 0:
            raise ValueError('invalid interpolation history bounds')
        self.samples = deque(maxlen=capacity)
        self.max_gap_ns = round(max_gap_s * 1e9)

    def add(self, stamp_ns, value):
        value = np.asarray(value, dtype=float)
        if not np.isfinite(value).all():
            raise ValueError('interpolation samples must be finite')
        if self.samples:
            if stamp_ns <= self.samples[-1][0]:
                return False
            if value.shape != self.samples[-1][1].shape:
                raise ValueError('interpolation sample shape changed')
        self.samples.append((int(stamp_ns), value.copy()))
        return True

    def at(self, stamp_ns):
        stamps = [stamp for stamp, _ in self.samples]
        right = bisect_left(stamps, stamp_ns)
        if right < len(stamps) and stamps[right] == stamp_ns:
            return self.samples[right][1].copy()
        if right == 0 or right == len(stamps):
            return None
        t0, v0 = self.samples[right-1]
        t1, v1 = self.samples[right]
        if t1-t0 > self.max_gap_ns:
            return None
        alpha = (stamp_ns-t0)/(t1-t0)
        return v0*(1-alpha) + v1*alpha
