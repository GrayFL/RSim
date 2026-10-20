"""Explicit MuJoCo source factory; application algorithms use its ports."""


def MuJoCo(scene=None, **options):
    from rsim.adapters.mujoco.source import MuJoCoSource
    return MuJoCoSource(scene, **options)
