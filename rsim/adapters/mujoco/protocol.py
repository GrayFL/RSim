"""Project-owned simulation boundary. No robot, scene or sensor implementation."""
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol


@dataclass(frozen=True)
class CommandSpec:
    fallback: Any
    is_safe: Callable[[Any], bool]
    validate: Callable[[Any], None] | None = None


@dataclass(frozen=True)
class Observation:
    time: float
    values: Mapping[str, Any]
    metadata: Mapping[str, Any] = field(default_factory=dict)
    port_metadata: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)


class SimulationBackend(Protocol):
    """All methods run on one owner thread; return owned immutable snapshots.

    A factory constructs this object on that thread. DemandSnapshot is read only;
    the backend decides which sensors to sample and their rates. Command names,
    payloads, sensor schemas, model construction and reset arguments belong to
    the application. open/close must also handle partial initialization.
    """

    def open(self) -> None: ...
    def step(self, duration: float, commands: Mapping[str, Any]) -> None: ...
    def sample(self, demand, *, force: bool = False) -> Observation: ...
    def reset(self, *args, **kwargs) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...
