import uuid

import pytest

from rsim import SensorError
from rsim.lifecycle import acquire_device


def test_device_ownership_is_exclusive_until_release():
    key = "test:" + uuid.uuid4().hex
    first = acquire_device(key)
    try:
        with pytest.raises(SensorError, match="already owned"):
            acquire_device(key)
    finally:
        first.close()
    acquire_device(key).close()
