"""Portable native-ROS chassis recipes and legacy local-calibration recipes."""
from pathlib import Path
from .loader import read_config


def load_chassis(path, *, motion_enabled=False, hardware=True):
    path = Path(path).expanduser().resolve()
    config = read_config(path)
    if 'calibration' in config or 'chassis' not in config:
        if not hardware:
            raise ValueError('chassis service needs a topic-only chassis recipe; legacy recipes own hardware')
        from .local_chassis import load_local_chassis
        return load_local_chassis(path, motion_enabled=motion_enabled)
    from rsim.drivers.native_chassis import NativeChassis, TopicChassis
    settings = dict(config['chassis'])
    settings['directory'] = (path.parent/settings['directory']).resolve()
    factory = NativeChassis if hardware else TopicChassis
    return factory(**settings, motion_enabled=motion_enabled)
