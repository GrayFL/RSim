"""Explicit MuJoCo source factory; application algorithms use its ports."""


def MuJoCo(backend_factory, **options):
    from rsim.adapters.mujoco.source import MuJoCoSource
    return MuJoCoSource(backend_factory, **options)
