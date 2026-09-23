"""Public data models. No ROS or DDS imports are needed to use these types."""
from dataclasses import dataclass, field
import time
from typing import Any

import numpy as np


@dataclass(frozen=True)
class Frame:
    data: Any
    stamp_ns: int
    clock: str
    received_ns: int = field(default_factory=time.time_ns)
    sequence: int = 0


@dataclass(frozen=True)
class Image:
    pixels: np.ndarray
    encoding: str
    frame_id: str


@dataclass(frozen=True)
class PointCloud:
    points: np.ndarray
    frame_id: str

    @property
    def xyz(self):
        return self.points[["x", "y", "z"]]
