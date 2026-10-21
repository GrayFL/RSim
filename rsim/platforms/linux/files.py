import os
from pathlib import Path
import shutil
import re


def filesystem_info(path):
    path = Path(path).resolve()
    selected = None
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        left, right = line.split(' - ', 1)
        fields, filesystem = left.split(), right.split()[0]
        mount = Path(re.sub(r'\\([0-7]{3})', lambda m: chr(int(m[1], 8)), fields[4]))
        if path.is_relative_to(mount) and (selected is None or len(str(mount)) > len(selected['mount'])):
            selected = dict(filesystem=filesystem, mount=str(mount), device=fields[2],
                            remote=filesystem in {'nfs', 'nfs4', 'cifs', 'smbfs', '9p', 'ceph', 'fuse.sshfs'})
    if selected is None:
        raise OSError(f'cannot identify storage filesystem: {path}')
    return selected


def open_shared(path, *, write=False, create=False, delete=False):
    flags = os.O_RDWR if write or create else os.O_RDONLY
    if create:
        flags |= os.O_CREAT | os.O_TRUNC
    fd = os.open(path, flags | os.O_NOFOLLOW, 0o600)
    return os.fdopen(fd, 'r+b' if write or create else 'rb')


def identity(file):
    stat = os.fstat(file.fileno())
    return str(stat.st_dev), str(stat.st_ino)


def hardlink(source, target):
    os.link(source, target, follow_symlinks=False)


def unlink(path, *, missing_ok=True):
    Path(path).unlink(missing_ok=missing_ok)


def remove_tree(directory):
    path = Path(directory)
    if path.is_symlink():
        path.unlink()
    elif path.exists():
        shutil.rmtree(path)
