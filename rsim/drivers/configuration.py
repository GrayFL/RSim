"""Immutable recipe data and explicit native-argument preparation helpers.

No device declarations or registry live here. Each factory owns its complete
configuration, port declaration and construction logic.
"""
from collections.abc import Mapping
from dataclasses import dataclass, fields
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class FrozenSettings(Mapping):
    entries: tuple

    def __getitem__(self, key):
        for name, value in self.entries:
            if name == key:
                return value
        raise KeyError(key)

    def __iter__(self):
        return (name for name, _ in self.entries)

    def __len__(self):
        return len(self.entries)


def freeze(value):
    """Detach nested data and reject runtime resources instead of pickling them."""
    if isinstance(value, FrozenSettings):
        return value
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise TypeError('recipe configuration keys must be strings')
        return FrozenSettings(tuple((key, freeze(item)) for key, item in sorted(value.items())))
    if isinstance(value, (tuple, list)):
        return tuple(freeze(item) for item in value)
    if isinstance(value, Path):
        return str(value)
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float and math.isfinite(value):
        return value
    raise TypeError('recipe configuration must contain finite plain data; inject resources into build()')


def plain(value):
    if isinstance(value, Mapping):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def fingerprint(recipe):
    config = {field.name: plain(getattr(recipe, field.name)) for field in fields(recipe)
              if field.name != 'log_path' and field.metadata.get('fingerprint', True)}
    identity = dict(factory=type(recipe).__module__ + '.' + type(recipe).__qualname__,
                    version=recipe.FACTORY_VERSION, config=config)
    if hasattr(recipe, 'INTERFACE_VERSION'):
        identity['interface'] = recipe.INTERFACE_VERSION
    return hashlib.sha256(json.dumps(identity, sort_keys=True, allow_nan=False,
                                     separators=(',', ':')).encode()).hexdigest()


def freeze_fields(recipe, *names):
    if not names:
        names = (field.name for field in fields(recipe))
    for name in names:
        object.__setattr__(recipe, name, freeze(getattr(recipe, name)))


def native_setup_required(ros_args, prepared):
    if ros_args and prepared is None:
        raise ValueError('native ros_args require explicit factory.prepare() before build/shared/device')


def snapshot_setup(setup):
    values = dict(setup)
    if values['options'] is not None:
        values['options'] = json.loads(values['options'].signature())
    return json.dumps(values, sort_keys=True, allow_nan=False)


def restore_setup(snapshot):
    from rsim.adapters.ros2.arguments import RosArguments
    values = json.loads(snapshot)
    saved = values['options']
    if saved is None:
        return values
    options = RosArguments(saved['parameters'], saved['ros_args'], saved['remappings'])
    if options.files != saved['files']:
        raise ValueError('ROS params file changed; recreate and prepare the factory')
    values['options'] = options
    return values


def prepare_log_directory(log_path):
    if log_path is not None:
        Path(log_path).expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)


def require_keys(value, allowed, name):
    if not isinstance(value, Mapping):
        raise TypeError(f'{name} must be a mapping')
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(f'unknown {name} settings: {sorted(unknown)}')


def positive(value, name, *, integer=False):
    if (type(value) not in ((int,) if integer else (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError(f'{name} must be a positive {"integer" if integer else "number"}')


def absolute_name(value, name):
    if not isinstance(value, str) or not value.startswith('/') or value == '/' or '//' in value:
        raise ValueError(f'{name} must be an absolute ROS name')


def mount_data(value, *, name='mount', base=None):
    """Canonical SE(3) snapshot; an optional local Pose is converted, never held."""
    from graphmap.pose import Pose
    if isinstance(value, Mapping):
        require_keys(value, ('position', 'rotation', 'wrd_frame', 'ego_frame', 'degrees',
                             'euler_order', 'rotation_type'), name)
        value = Pose(**plain(value))
    if (not isinstance(value, Pose) or not value.is_rigid or not value.wrd_frame
            or not value.ego_frame or value.wrd_frame == value.ego_frame
            or (base is not None and value.wrd_frame != base)):
        raise ValueError(f'{name} requires a rigid mount with distinct frame labels'
                         + (f' and base {base!r}' if base is not None else ''))
    return freeze(dict(position=value.position.tolist(), rotation=value.quat.tolist(),
                       wrd_frame=value.wrd_frame, ego_frame=value.ego_frame))


def from_config(cls, path, *, section, overrides=None, path_fields=()):
    """Strict, portable recipe loading; this helper never constructs a graph."""
    path = Path(path).expanduser().resolve(strict=True)
    document = read_config(path)
    if section in document:
        require_keys(document, ('schema_version', section), 'configuration root')
        settings = dict(document[section])
        settings.setdefault('schema_version', document.get('schema_version', 1))
    else:
        settings = dict(document)
    def merge(target, update):
        for key, value in update.items():
            if isinstance(target.get(key), Mapping) and isinstance(value, Mapping):
                target[key] = merge(dict(target[key]), value)
            else:
                target[key] = value
        return target
    require_keys(overrides or {}, (field.name for field in fields(cls) if field.init and not field.name.startswith('_')),
                 'overrides')
    settings = merge(settings, overrides or {})
    require_keys(settings, (field.name for field in fields(cls) if field.init and not field.name.startswith('_')),
                 cls.__name__)
    for dotted in path_fields:
        current, names = settings, dotted.split('.')
        for name in names[:-1]:
            current = current.get(name, {})
            if not isinstance(current, Mapping):
                break
        else:
            if current.get(names[-1]) is not None:
                current[names[-1]] = str((path.parent / Path(current[names[-1]]).expanduser()).resolve())
    return cls(**settings)


def read_config(path):
    """Safely parse pure recipe data; no adapter or application imports."""
    path = Path(path).expanduser()
    if path.suffix.lower() == '.toml':
        import tomllib
        with path.open('rb') as file:
            return tomllib.load(file)
    import yaml

    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node, deep=False):
        loader.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if not isinstance(key, str) or key in result:
                raise ValueError(f'duplicate or non-string YAML key: {key!r}')
            result[key] = loader.construct_object(value_node, deep=deep)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    with path.open() as file:
        config = yaml.load(file, Loader=UniqueLoader)
    if not isinstance(config, dict):
        raise ValueError('configuration must be a mapping')
    return config


def source_reference(value, *, kind, default_topic=None, history=512):
    """Validate a pure connection description without starting a provider."""
    if value is None:
        if default_topic is None:
            return None
        value = dict(kind='ros_topic', topic=default_topic)
    if not isinstance(value, Mapping):
        raise TypeError('source reference must be pure data; inject live sources into build()')
    common = {'kind', 'clock', 'history', 'revision'}
    source_kind = value.get('kind')
    allowed = common | ({'topic'} if source_kind == 'ros_topic' else
                        {'key', 'port', 'interface_version', 'schema', 'transport'}
                        if source_kind == 'shared_device' else set())
    require_keys(value, allowed, 'source reference')
    if source_kind not in ('ros_topic', 'shared_device'):
        raise ValueError('source kind must be ros_topic or shared_device')
    result = dict(value)
    result.setdefault('clock', 'ros:system')
    result.setdefault('history', history)
    positive(result['history'], 'source history', integer=True)
    if result['clock'] != 'ros:system':
        raise ValueError('native sources require ros:system clock')
    if source_kind == 'ros_topic':
        absolute_name(result.get('topic'), 'source topic')
    else:
        for name in ('key', 'interface_version'):
            if not isinstance(result.get(name), str) or not result[name]:
                raise ValueError(f'shared_device source requires {name}')
        result.setdefault('port', kind)
        result.setdefault('schema', 'rsim.value.v1')
        if not isinstance(result['port'], str) or not result['port']:
            raise ValueError('source port must be a nonempty string')
        if result.get('transport') is not None:
            from rsim.transport.descriptor import TransportConfig
            require_keys(result['transport'], ('backend', 'domain_id'), 'source transport')
            TransportConfig(**result['transport'])
    return freeze(result)


def connect_source(reference, *, kind, ros=None):
    settings = plain(reference)
    if settings['kind'] == 'ros_topic':
        from rsim.adapters.ros2.sensor import RosSensor
        return getattr(RosSensor(settings['topic'], kind, ros=ros, clock=settings['clock'],
                                 history=settings['history'], hz=500), kind)
    from rsim.runtime.sharing import SharedComponent, PortSpec
    from rsim.transport.descriptor import TransportConfig
    name = settings['port']
    transport = settings.get('transport')
    device = SharedComponent(key=settings['key'], interface_version=settings['interface_version'],
        ports={name: PortSpec(schema=settings['schema'], clock=settings['clock'], history_capacity=settings['history'])},
        transport=TransportConfig(**transport) if transport is not None else None)
    return getattr(device, name)
