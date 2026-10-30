"""Mapping recipe assembly; host-specific addresses remain in user YAML."""
from pathlib import Path

from .loader import read_config


def load_mapper(path, *, providers=True, overrides=None):
    """Read the mapping section and return named pose, odometry and RGB map ports."""
    path = Path(path).expanduser().resolve()
    config = read_config(path)
    settings = dict(config['mapping'])
    settings.update(overrides or {})
    if not providers:
        from rsim.devices.mapping import Mapper
        from rsim.drivers.mapping import MappingFactory
        return Mapper(**{key: settings[key] for key in ('name', 'transport') if key in settings},
            ports=MappingFactory.port_specs(history=settings.get('history', 3)),
            interface_version=MappingFactory.INTERFACE_VERSION)
    from rsim.drivers.mapping import MappingFactory
    return MappingFactory.from_config(path, overrides=overrides).shared()
