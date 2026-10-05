"""Safe YAML recipes and explicit factory selection."""
from copy import deepcopy
from pathlib import Path
from rsim.core import Component
from .geometry import Mount
from .assembly import MountedSensor, SensorAssembly, SensorRig

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


def _factories(providers):
    if providers:
        from rsim import drivers as module
    else:
        from rsim import devices as module
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
            if factory is module.Hipnuc and not parameters.get("port"):
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
