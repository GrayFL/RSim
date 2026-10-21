"""Small internal boundaries shared by runtime and transport."""
from dataclasses import dataclass
from types import ModuleType


@dataclass(frozen=True)
class HostIdentity:
    boot_id: str
    user_id: str
    platform: str


@dataclass(frozen=True)
class PlatformCapabilities:
    local_ipc: bool
    supervised_process: bool
    mapped_file_unlink: bool
    hardlink_reuse: bool
    managed_tree_cleanup: bool
    os_enforced_tree_containment: bool


@dataclass(frozen=True)
class Platform:
    name: str
    identity: ModuleType
    process: ModuleType
    ipc: ModuleType
    locks: ModuleType
    files: ModuleType
    paths: ModuleType
    # OS guarantees only. Filesystem and loop capabilities are probed on use.
    managed_tree_cleanup: bool = True
    os_enforced_tree_containment: bool = False
