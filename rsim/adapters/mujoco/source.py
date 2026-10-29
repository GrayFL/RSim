"""Deprecated import bridge. Runtime implementation belongs to MujocoSim."""


def __getattr__(name):
    if name != 'MuJoCoSource':
        raise AttributeError(name)
    from . import load_compatibility
    return load_compatibility(name)
