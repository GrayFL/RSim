"""YAML sensor recipes and fixed, labelled graphmap mounting transforms.

Loading constructs a graph only. Hardware opens when that graph enters Runtime.
Recipes use an explicit factory registry, never arbitrary module imports in YAML.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from graphmap.pose import Pose

from .core import Component


class UncalibratedMount(ValueError):
    pass


def read_config(path):
    """Load a YAML mapping safely, requiring unique string keys."""
    import yaml

    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node, deep=False):
        loader.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if not isinstance(key, str) or key in result:
                raise ValueError(f"duplicate or non-string YAML key: {key!r}")
            result[key] = loader.construct_object(value_node, deep=deep)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    with Path(path).expanduser().open() as file:
        config = yaml.load(file, Loader=UniqueLoader)
    if not isinstance(config, dict):
        raise ValueError("configuration must be a mapping")
    return config


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


class MountedSensor(Component):
    """One source with installation metadata; payloads keep native frame IDs."""
    def __init__(self, source, mount, *, name, parameters):
        super().__init__(source)
        self.source, self.mount, self.name = source, mount, name
        self.parameters = deepcopy(parameters)
        for key, output in source.outputs.items():
            alias = self.signal(key, history=output.history_size, clock=output.clock)
            alias._target = output
            if hasattr(self, key):
                raise ValueError(f"output name conflicts with mounted-sensor attribute: {key}")
            setattr(self, key, alias)

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


def _factories(providers):
    from . import drivers, devices
    module = drivers if providers else devices
    factories = {"d435": module.D435, "robinw": module.RobinW,
                 "camera": module.Camera, "hipnuc": module.Hipnuc}
    if providers:
        return factories
    # Connection-only views need source identity/history, not driver settings.
    # Explicit port is required here: client discovery must not load pyserial.
    import inspect
    def client(factory):
        accepted = set(inspect.signature(factory).parameters)
        def connect(**parameters):
            if factory is devices.Hipnuc and not parameters.get("port"):
                raise ValueError("connection-only IMU recipes require an explicit port")
            return factory(**{name: value for name, value in parameters.items() if name in accepted})
        return connect
    return {name: client(factory) for name, factory in factories.items()}


def load_rig(path, *, select=None, providers=True, overrides=None, factories=None):
    """Create a rig from a YAML `rsim` section; select avoids offline devices.

    overrides is {sensor_name: {constructor_parameter: value}}. Every enabled
    reference to the same sensor uses one MountedSensor object. Native driver
    parameters/ROS argv remain ordinary nested constructor options.
    """
    path = Path(path).expanduser().resolve()
    config = read_config(path)
    settings = config.get("rsim")
    if not isinstance(settings, dict):
        raise ValueError("expected an rsim section with sensor recipes")
    if set(settings) - {"base_frame", "geometry_file", "sensors", "assemblies"}:
        raise ValueError("unknown rsim settings")
    base = settings.get("base_frame", "base_footprint")
    if not isinstance(base, str) or not base:
        raise ValueError("base_frame must be a nonempty string")
    geometry = config
    if settings.get("geometry_file"):
        geometry = read_config(path.parent / settings["geometry_file"])
    geometry = geometry.get("sensor_params", {})
    recipes, groups = settings.get("sensors", {}), settings.get("assemblies", {})
    if not isinstance(recipes, dict) or not recipes or not isinstance(groups, dict):
        raise ValueError("sensor recipes and assemblies must be mappings")
    if set(recipes) & set(groups) or base in recipes or base in groups:
        raise ValueError("sensor, assembly and base names must be distinct")
    overrides = deepcopy(overrides or {})
    if set(overrides) - set(recipes):
        raise ValueError("override references an unknown sensor")
    registry = _factories(providers) if factories is None else dict(factories)
    sensors, assemblies, visiting = {}, {}, set()

    def build(name):
        if name in sensors:
            return sensors[name]
        if name in assemblies:
            return assemblies[name]
        if name in visiting:
            raise ValueError("cyclic sensor assembly")
        visiting.add(name)
        if name in recipes:
            recipe = recipes[name]
            if not isinstance(recipe, dict) or set(recipe) - {"driver", "parameters", "mount"}:
                raise ValueError(f"invalid sensor recipe: {name}")
            if recipe.get("driver") not in registry:
                raise ValueError(f"unknown driver: {recipe.get('driver')}")
            parameters = deepcopy(recipe.get("parameters", {}))
            parameters.update(overrides.get(name, {}))
            mounting = deepcopy(recipe.get("mount", {}))
            if set(mounting) - {"from", "position", "rotation", "status", "frame", "note"}:
                raise ValueError(f"unknown mount settings: {name}")
            original = {}
            if "from" in mounting:
                original = geometry[mounting.pop("from")]
            position = mounting.get("position", original.get("position"))
            rotation = mounting.get("rotation", original.get("rotation"))
            mount = Mount(base, mounting.get("frame", name),
                          None if position is None else tuple(position),
                          None if rotation is None else tuple(rotation),
                          mounting.get("status", "unknown"), mounting.get("note", ""))
            source = registry[recipe["driver"]](**parameters)
            if not isinstance(source, Component):
                raise TypeError("sensor factory must return a Component")
            result = sensors[name] = MountedSensor(source, mount, name=name, parameters=parameters)
        elif name in groups:
            members = groups[name]
            if not isinstance(members, list) or not members or any(not isinstance(x, str) for x in members):
                raise ValueError("assembly must be a nonempty list of sensor/assembly names")
            if len(set(members)) != len(members):
                raise ValueError("duplicate assembly members")
            result = assemblies[name] = SensorAssembly({item: build(item) for item in members},
                                                       name=name, base_frame=base)
        else:
            raise ValueError(f"unknown sensor or assembly: {name}")
        visiting.remove(name)
        return result

    names = list(recipes) + list(groups) if select is None else ([select] if isinstance(select, str) else list(select))
    if not names:
        raise ValueError("select must include at least one sensor or assembly")
    members = {name: build(name) for name in names}
    return SensorRig(members, sensors, assemblies, base_frame=base, config=config)
