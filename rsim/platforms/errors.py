"""Platform failures without dependencies on runtime or data codecs."""
from collections import deque
import warnings


class CapabilityUnavailable(RuntimeError):
    def __init__(self, operation, message):
        self.operation = operation
        super().__init__(message)


cleanup_failures = deque(maxlen=256)


def record_cleanup(operation, path, error, instance_id=None):
    record = dict(operation=operation, path=str(path),
                  error_code=getattr(error, 'winerror', None) or getattr(error, 'errno', None),
                  error=str(error), instance_id=instance_id)
    cleanup_failures.append(record)
    warnings.warn(f'{operation} failed for {path}: {error}', RuntimeWarning, stacklevel=2)
    return record
