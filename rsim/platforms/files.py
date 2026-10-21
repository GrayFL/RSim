"""Dispatch file primitives and keep bounded cleanup failure evidence."""
import time
from .detect import get_platform
from .errors import record_cleanup


def open_shared(path, *, write=False, create=False, delete=False):
    return get_platform().files.open_shared(path, write=write, create=create, delete=delete)


def hardlink(source, target):
    return get_platform().files.hardlink(source, target)


def unlink(path, *, missing_ok=True):
    return get_platform().files.unlink(path, missing_ok=missing_ok)


def remove_tree(directory, *, ignore_errors=False, instance_id=None):
    for attempt in range(3):
        try:
            get_platform().files.remove_tree(directory)
            return []
        except OSError as error:
            if attempt < 2:
                time.sleep(.02)
                continue
            record = record_cleanup('remove_tree', directory, error, instance_id)
            if not ignore_errors:
                raise
            return [record]
