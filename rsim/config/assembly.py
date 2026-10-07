"""Reusable sensor composition and geometry queries."""
from copy import deepcopy
from graphmap.pose import Pose
from rsim.core import Component

class MountedSensor(Component):
    """One source with installation metadata; payloads keep native frame IDs."""
    def __init__(self, source, mount, *, name, parameters):
        super().__init__(source)
        self.source, self.mount, self.name = source, mount, name
        self.parameters = deepcopy(parameters)
        for key, output in source.outputs.items():
            if hasattr(self, key):
                raise ValueError(f"output name conflicts with mounted-sensor attribute: {key}")
            self.expose(key, output)

    @property
    def T_base_sensor(self):
        return self.mount.pose()


class SensorAssembly(Component):
    def __init__(self, members, *, name, base_frame):
        super().__init__(*dict.fromkeys(members.values()))
        self.members, self.name, self.base_frame = dict(members), name, base_frame

    def __getitem__(self, name):
        return self.members[name]


class SensorRig(SensorAssembly):
    """Named sensors and reusable nested assemblies, rooted at one body frame."""
    def __init__(self, members, sensors, assemblies, *, base_frame, config):
        super().__init__(members, name="rig", base_frame=base_frame)
        self.sensors, self.assemblies, self.config = sensors, assemblies, config

    def __getitem__(self, name):
        if name in self.sensors:
            return self.sensors[name]
        return self.assemblies[name]

    def transform(self, target, source, *, allow_estimated=False):
        """T_target_source maps points in source coordinates into target."""
        def pose(name):
            if name == self.base_frame:
                return Pose(wrd_frame=self.base_frame, ego_frame=self.base_frame)
            return self.sensors[name].mount.pose(allow_estimated=allow_estimated)
        return (~pose(target)) * pose(source)
