"""Deprecated construction bridge to the MujocoSim-owned source."""


def MuJoCo(backend_factory, **options):
    from rsim.adapters.mujoco.source import MuJoCoSource
    options.setdefault('reset_policy', 'legacy_continuous')
    return MuJoCoSource(backend_factory, **options)
