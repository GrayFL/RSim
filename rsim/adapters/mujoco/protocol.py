"""Deprecated type aliases; no second protocol implementation is maintained."""

__all__ = ['CommandSpec', 'Observation', 'SimulationBackend']


def __getattr__(name):
    if name not in __all__:
        raise AttributeError(name)
    from . import load_compatibility
    return load_compatibility(name)
