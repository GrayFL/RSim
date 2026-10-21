"""Private directories and owner checks, without repairing untrusted paths."""
import os
from pathlib import Path
import stat
import tempfile


def defaults():
    private = Path(tempfile.gettempdir()) / f'rsim-v3-{os.getuid()}'
    return dict(registry_root=private, storage_root=Path('/dev/shm'),
                log_root=Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')) / 'rsim/logs')


def validate_private(path):
    path = Path(path)
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise PermissionError(f'RSim path must be private and owned by this user: {path}')


def private_directory(path):
    path = Path(path)
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.mkdir(mode=0o700, exist_ok=True)
    validate_private(path)
    return path
