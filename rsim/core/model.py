"""Public data models. No ROS or DDS imports are needed to use these types."""
from dataclasses import dataclass, field
import time
from typing import Generic, TypeVar

import numpy as np


T = TypeVar("T")


@dataclass(frozen=True)
class SampleId:
    """Publication identity, independent of a reader's local history cursor."""
    producer_instance_id: str
    canonical_port_id: str
    publication_sequence: int


@dataclass(frozen=True)
class Frame(Generic[T]):
    data: T
    stamp_ns: int
    clock: str
    received_ns: int = field(default_factory=time.time_ns)
    sequence: int = 0
    sample_id: SampleId | None = None
    metadata: dict = field(default_factory=dict)


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
