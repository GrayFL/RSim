"""Immutable demand view; contains no runtime, transport or OS resources."""
from dataclasses import dataclass


@dataclass(frozen=True)
class DemandSnapshot:
    generation: int = 0
    ports: frozenset[str] = frozenset()

    def has(self, *names):
        return any(name in self.ports for name in names)
