"""Deprecated lazy bridges to MujocoSim, scheduled for removal in RSim 0.5."""
from importlib import import_module
import warnings

__all__ = ['MuJoCoSource', 'CommandSpec', 'Observation', 'SimulationBackend']


def load_compatibility(name):
    module = 'source' if name == 'MuJoCoSource' else 'protocol'
    try:
        value = getattr(import_module('mujocosim.rsim.components.' + module), name)
    except ModuleNotFoundError as error:
        if error.name != 'mujocosim':
            raise
        raise ImportError('MuJoCo support moved to mujocosim.rsim; install the local '
                          'MujocoSim project with its rsim extra (engine extra on providers).') from error
    warnings.warn('rsim.adapters.mujoco moved to mujocosim.rsim; this alias is removed in RSim 0.5. '
                  'New sources use native reset time with episode; select legacy_continuous explicitly if needed.',
                  DeprecationWarning, stacklevel=3)
    return value


def __getattr__(name):
    if name not in __all__:
        raise AttributeError(name)
    return load_compatibility(name)
