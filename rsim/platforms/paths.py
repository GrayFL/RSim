"""Common path resolution; native permissions are delegated to the platform."""
import os
from pathlib import Path
import tempfile

from .detect import get_platform


def roots(paths=None):
    values = get_platform().paths.defaults()
    for name in values:
        explicit = getattr(paths, name, None) if paths is not None else None
        values[name] = Path(explicit or os.environ.get('RSIM_' + name.upper()) or values[name]).expanduser().absolute()
    return values


def checked_path(path):
    # Refuse traversing symlinks/junctions, including existing parent components.
    path = Path(path).absolute()
    for candidate in (path, *path.parents):
        if candidate.is_symlink() or candidate.is_junction():
            raise PermissionError(f'RSim paths must not traverse a link: {candidate}')
    return path


def private_directory(path):
    return get_platform().paths.private_directory(checked_path(path))


def new_directory(*, prefix, root=None):
    root = checked_path(root if root is not None else roots()['storage_root'])
    root.mkdir(parents=True, exist_ok=True)
    path = Path(tempfile.mkdtemp(prefix=prefix, dir=root))
    # A new owned directory can have its ACL restricted; existing directories
    # must pass validation rather than being silently repaired.
    secure = getattr(get_platform().paths, 'secure_new', None)
    try:
        if secure is not None:
            secure(path)
        return private_directory(path)
    except BaseException as error:
        try:
            path.rmdir()  # Only the new, still-empty directory belongs to us.
        except OSError as cleanup:
            error.add_note(f'new directory rollback failed: {path}: {cleanup}')
        raise
