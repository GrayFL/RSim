"""Portable native-ROS chassis recipes and legacy local-calibration recipes."""
from pathlib import Path
from .loader import read_config


def load_chassis(path, *, motion_enabled=False):
    path = Path(path).expanduser().resolve()
    config = read_config(path)
    if 'calibration' in config or 'chassis' not in config:
        from .local_chassis import load_local_chassis
        return load_local_chassis(path, motion_enabled=motion_enabled)
    from rsim.drivers.native_chassis import NativeChassis
    settings = dict(config['chassis'])
    settings['directory'] = (path.parent/settings['directory']).resolve()
    return NativeChassis(**settings, motion_enabled=motion_enabled)
