"""Cooperative physical device exclusion, independent of the ROS domain."""
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile

from .core import SensorError


def acquire_device(key):
    directory = Path(tempfile.gettempdir()) / f"rsim-devices-{os.getuid()}"
    directory.mkdir(mode=0o700, exist_ok=True)
    if directory.is_symlink() or directory.stat().st_uid != os.getuid():
        raise SensorError("device lock directory has the wrong owner")
    name = hashlib.sha256(key.encode()).hexdigest() + ".lock"
    lease = (directory / name).open("a+")
    try:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lease.close()
        raise SensorError(f"physical device already owned: {key}; use the shared source") from error
    return lease
