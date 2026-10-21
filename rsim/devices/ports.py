"""Typed port bundles for assembly; no tasks or independent lifecycle."""
from dataclasses import dataclass
from rsim.core import Signal, CommandSink, VelocityCommand


@dataclass(frozen=True)
class ChassisPorts:
    pose: Signal
    velocity: CommandSink[VelocityCommand]
    state: Signal

    def __post_init__(self):
        if not isinstance(self.pose, Signal) or not isinstance(self.state, Signal):
            raise TypeError('pose and state must be Signals')
        if not isinstance(self.velocity, CommandSink):
            raise TypeError('velocity must be a CommandSink')
