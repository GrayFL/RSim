"""Optional deployment settings, resolved once before opening a Runtime."""
from dataclasses import dataclass, field, asdict, replace
import math
import os
from pathlib import Path


def positive(name, value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True)
class RuntimePaths:
    registry_root: Path | None = None
    storage_root: Path | None = None
    log_root: Path | None = None

    def resolved(self):
        return RuntimePaths(**{name: Path(value).expanduser().absolute() if value else None
                               for name in self.__dataclass_fields__
                               for value in [getattr(self, name) or os.environ.get('RSIM_' + name.upper())]})


@dataclass(frozen=True)
class ShutdownPolicy:
    grace_s: float = 5.0
    kill_wait_s: float = 5.0

    def __post_init__(self):
        positive('grace_s', self.grace_s)
        positive('kill_wait_s', self.kill_wait_s)


@dataclass(frozen=True)
class RuntimeOptions:
    paths: RuntimePaths = field(default_factory=RuntimePaths)
    shutdown: ShutdownPolicy = field(default_factory=ShutdownPolicy)
    startup_timeout_s: float = 30.0

    def __post_init__(self):
        if not isinstance(self.paths, RuntimePaths) or not isinstance(self.shutdown, ShutdownPolicy):
            raise TypeError('paths and shutdown must be RuntimePaths and ShutdownPolicy')
        positive('startup_timeout_s', self.startup_timeout_s)

    def resolved(self):
        from rsim.platforms.paths import roots
        return replace(self, paths=RuntimePaths(**roots(self.paths)))

    def to_dict(self):
        result = asdict(self)
        result['paths'] = {name: str(value) if value is not None else None
                           for name, value in result['paths'].items()}
        return result

    @classmethod
    def from_dict(cls, value):
        if value is None:
            return cls().resolved()
        return cls(paths=RuntimePaths(**value['paths']), shutdown=ShutdownPolicy(**value['shutdown']),
                   startup_timeout_s=value['startup_timeout_s']).resolved()
