"""Machine-local chassis assembly configuration."""

import json
from pathlib import Path
from .loader import read_config


def local_settings(path):
    path = Path(path).expanduser().resolve()
    config = read_config(path)
    return config, (path.parent / config["calibration"]).resolve()


def load_local_chassis(path, *, motion_enabled=False):
    from rsim.drivers.local_chassis import build

    config, calibration = local_settings(path)
    return build(
        config, json.loads(calibration.read_text()), motion_enabled=motion_enabled
    )


async def calibrate_local_chassis(path, *, seconds=10.0):
    from rsim.drivers.local_chassis import calibrate

    config, calibration = local_settings(path)
    return await calibrate(config, calibration, seconds=seconds)
