"""Optional MuJoCo adapter. Importing this namespace opens no simulation."""
from importlib import import_module
from .config import RobotConfig, SensorConfig

__all__ = ["MuJoCoSource", "Simulation", "RobotConfig", "SensorConfig", "prepare_scene"]


def __getattr__(name):
    modules = {"MuJoCoSource": "source", "Simulation": "simulation", "prepare_scene": "scene"}
    if name not in modules:
        raise AttributeError(name)
    value = getattr(import_module(f"{__name__}.{modules[name]}"), name)
    globals()[name] = value
    return value
