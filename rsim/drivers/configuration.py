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
              if field.name != 'log_path'}
    identity = dict(factory=type(recipe).__module__ + '.' + type(recipe).__qualname__,
                    version=recipe.FACTORY_VERSION, config=config)
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
