"""Cooperative physical device exclusion, independent of the ROS domain."""
import getpass
import hashlib
import os
from pathlib import Path
import tempfile

from rsim.core.errors import ComponentError


def open_lock(path):
    file = Path(path).open('a+b')
    if file.seek(0, 2) == 0:
        file.write(b'0')
        file.flush()
    file.seek(0)
    return file


def try_lock(file):
    if os.name == 'nt':
        import msvcrt
        file.seek(0)
        try:
            msvcrt.locking(file.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError as error:
            if error.errno in (13, 36):
                raise BlockingIOError('lock is owned') from error
            raise
    else:
        import fcntl
        fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)


def acquire_device(key):
    if os.name == "nt":
        import msvcrt
        user = hashlib.sha256(getpass.getuser().encode()).hexdigest()[:16]
        directory = Path(tempfile.gettempdir()) / f"rsim-devices-{user}"
        directory.mkdir(exist_ok=True)
        if directory.is_symlink():
            raise ComponentError("device lock directory must not be a symlink")
        lease = (directory / (hashlib.sha256(key.encode()).hexdigest() + ".lock")).open("a+b")
        try:
            if lease.seek(0, 2) == 0:
                lease.write(b"0")
                lease.flush()
            lease.seek(0)
            msvcrt.locking(lease.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError as error:
            lease.close()
            raise ComponentError(f"physical device already owned: {key}; use the shared source") from error
        return lease  # Closing the descriptor releases the byte-range lock.
    import fcntl
    directory = Path(tempfile.gettempdir()) / f"rsim-devices-{os.getuid()}"
    directory.mkdir(mode=0o700, exist_ok=True)
    if directory.is_symlink() or directory.stat().st_uid != os.getuid():
        raise ComponentError("device lock directory has the wrong owner")
    name = hashlib.sha256(key.encode()).hexdigest() + ".lock"
    lease = (directory / name).open("a+")
    try:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lease.close()
        raise ComponentError(f"physical device already owned: {key}; use the shared source") from error
    return lease
