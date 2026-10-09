"""In-place angular visibility masks over ROS-independent LaserScan values."""
import numpy as np

from rsim.core import PrimaryComponent
from rsim.core.signal import as_signal


class AngularScanMask(PrimaryComponent):
    """Mark fixed occluded sectors unknown, retaining beam indices and timing.

    Angle bounds are radians in the scan frame, measured CCW about +Z from +X.
    A wrapping interval must be split at +/-pi. No hardware-specific mask is
    implicit. NaN denotes unobserved space, not a no-return free-space ray.
    """
    def __init__(self, source, *, angle_masks, history=32):
        self.source = as_signal(source)
        masks = np.asarray(angle_masks, dtype=float)
        if (masks.ndim != 2 or masks.shape[1] != 2 or not len(masks)
                or not np.isfinite(masks).all() or np.any(masks[:, 0] >= masks[:, 1])
                or np.any(np.abs(masks) > np.pi)):
            raise ValueError('angle_masks must be ordered radian pairs within [-pi, pi]')
        self.angle_masks = tuple(map(tuple, masks.tolist()))
        super().__init__(inputs=(self.source,), output_name='scan', history=history,
                         clock=self.source.clock)
        self.scan = self.output
        self.masked_beams = self.removed_returns = 0

    def configuration(self):
        return (super().configuration(), self.angle_masks, self.source.name,
                self.source.producer.key)

    def convert(self, data):
        start, step = data['angle_min'], data['angle_increment']
        ranges = np.asarray(data['ranges'], dtype=np.float32)
        if ranges.ndim != 1 or not np.isfinite([start, step]).all() or step == 0:
            raise ValueError('scan needs one-dimensional ranges and finite beam angles')
        angles = (start + np.arange(len(ranges))*step + np.pi) % (2*np.pi) - np.pi
        mask = np.zeros(len(ranges), dtype=bool)
        for lower, upper in self.angle_masks:
            mask |= (angles >= lower) & (angles <= upper)
        result = dict(data)
        result['ranges'] = ranges.copy()
        result['ranges'][mask] = np.nan
        intensity = np.asarray(data['intensities'], dtype=np.float32)
        if len(intensity) not in (0, len(ranges)):
            raise ValueError('scan intensities must be empty or match beam count')
        result['intensities'] = intensity.copy()
        if len(intensity):
            result['intensities'][mask] = 0
        self.masked_beams = int(mask.sum())
        self.removed_returns = int(np.count_nonzero(mask & np.isfinite(ranges)))
        return result

    async def open(self):
        self.previous = 0
        self.task('scan-mask', self.update, hz=100)

    async def update(self):
        await self.source.get()
        for frame in self.source.frames:
            if frame.sequence <= self.previous:
                continue
            self.previous = frame.sequence
            value = self.convert(frame.data)
            await self.publish(value, stamp_ns=frame.stamp_ns, clock=frame.clock,
                received_ns=frame.received_ns, metadata={'masked_beams': self.masked_beams,
                    'removed_returns': self.removed_returns})
