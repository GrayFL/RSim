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
        return Mapper(**{key: settings[key] for key in ('name', 'history', 'transport') if key in settings})
    from rsim.drivers.mapping import Mapper
    if 'geometry_file' in settings:
        geometry = read_config(path.parent / settings.pop('geometry_file'))['sensor_params']
        mounts = dict(settings.get('mounts', {}))
        for name, source, frame in (('body_lidar', 'lidar3d', 'mapping_lidar'),
                                     ('body_camera', 'rgb', 'camera_link')):
            mounts.setdefault(name, dict(position=geometry[source]['position'],
                rotation=geometry[source]['rotation'], degrees=True,
                wrd_frame='base_footprint', ego_frame=frame))
        settings['mounts'] = mounts
    if 'database' not in settings:
        raise ValueError('mapping recipe needs an explicit database path')
    settings['database'] = str((path.parent / settings['database']).resolve())
    return Mapper(**settings)
