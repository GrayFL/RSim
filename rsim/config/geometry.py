"""Fixed labelled mounting geometry; no device factories."""
from dataclasses import dataclass
import numpy as np
from graphmap.pose import Pose

class UncalibratedMount(ValueError):
    pass


@dataclass(frozen=True)
class Mount:
    """Fixed T_parent_sensor; partial geometry is preserved, never filled in."""
    parent: str
    frame: str
    position: tuple | None
    rotation: tuple | None
    status: str = "measured"
    note: str = ""

    def __post_init__(self):
        if not self.parent or not self.frame or self.parent == self.frame:
            raise ValueError("mount needs distinct nonempty parent and sensor frames")
        if self.status not in ("measured", "reference", "estimated", "unknown"):
            raise ValueError("mount status must be measured/reference/estimated/unknown")
        for name, value in (("position", self.position), ("rotation", self.rotation)):
            if value is not None and (len(value) != 3 or any(
                    v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                                      or not np.isfinite(v)) for v in value)):
                raise ValueError(f"{name} must contain three finite numbers or nulls")
        if self.status in ("measured", "reference") and not self.complete:
            raise ValueError("known mounts require complete position and rotation")

    @property
    def complete(self):
        return all(value is not None and all(v is not None for v in value)
                   for value in (self.position, self.rotation))

    def pose(self, *, allow_estimated=False):
        if (not self.complete or self.status == "unknown"
                or (self.status == "estimated" and not allow_estimated)):
            raise UncalibratedMount(f"mount {self.parent} <- {self.frame} is {self.status}; supply calibrated geometry")
        return Pose(position=self.position, rotation=self.rotation, degrees=True,
                    wrd_frame=self.parent, ego_frame=self.frame)
