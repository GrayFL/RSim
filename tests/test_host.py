import asyncio
import os
import uuid

import numpy as np
import pytest

from rsim import Runtime
from rsim.host import SharedSensor
from rsim.synthetic import CounterArray


def test_shared_source_survives_first_owner_and_maps_one_inode():
    async def run():
        key = "test:" + uuid.uuid4().hex
        first = SharedSensor(lambda: CounterArray(hz=10), key=key)
        second = SharedSensor(lambda: CounterArray(hz=10), key=key)
        one, two = Runtime(first), Runtime(second)
        try:
            await asyncio.gather(one.__aenter__(), two.__aenter__())
            a, b = await asyncio.gather(first.get(timeout=15), second.get(timeout=15))
            assert first.directory == second.directory
            assert first.worker_pid == second.worker_pid
            # Match by source timestamp across independently received histories.
            async with asyncio.timeout(5):
                while a.stamp_ns != b.stamp_ns:
                    if a.stamp_ns < b.stamp_ns:
                        a = await first.get(after=a.sequence)
                    else:
                        b = await second.get(after=b.sequence)
            assert os.stat(a.data.filename).st_ino == os.stat(b.data.filename).st_ino
            await one.aclose()
            fresh = await second.get(after=b.sequence, timeout=5)
            assert fresh.stamp_ns > b.stamp_ns
            assert first.directory.exists()
            conflict = SharedSensor(lambda: CounterArray(), key=key, version="incompatible")
            with pytest.raises(ValueError, match="conflicting"):
                async with Runtime(conflict):
                    pass
        finally:
            await one.aclose()
            await two.aclose()
        async with asyncio.timeout(7):
            while second.directory.exists():
                await asyncio.sleep(0.05)
        np.testing.assert_array_equal(a.data, b.data)
    asyncio.run(run())
