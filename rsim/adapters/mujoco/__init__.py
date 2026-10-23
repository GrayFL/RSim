"""Protocol adapter; simulation models and applications live outside RSim."""
from .protocol import CommandSpec, Observation, SimulationBackend

__all__ = ['MuJoCoSource', 'CommandSpec', 'Observation', 'SimulationBackend']


def __getattr__(name):
    if name != 'MuJoCoSource':
        raise AttributeError(name)
    from .source import MuJoCoSource
    return MuJoCoSource
