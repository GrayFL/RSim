import asyncio
import os
import uuid

import numpy as np
import pytest

from rsim import Runtime, SensorError, TransportConfig
from rsim.runtime.host import SharedSensor
from rsim.components.synthetic import CounterArray


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


def test_connect_only_never_launches_a_missing_source():
    async def run():
        client = SharedSensor(key="absent:" + uuid.uuid4().hex)
        with pytest.raises(SensorError, match="start its provider first"):
            async with Runtime(client):
                pass
        assert client.directory is None and client.process is None
        assert not client._tasks
    asyncio.run(run())


def test_connect_only_checks_configuration_and_domain():
    async def run():
        key = "domain:" + uuid.uuid4().hex
        config = TransportConfig("cyclonedds", 74)
        owner = SharedSensor(lambda: CounterArray(), key=key, transport=config)
        async with Runtime(owner):
            await owner.get(timeout=15)
            wrong_config = SharedSensor(key=key, version="other", transport=config)
            with pytest.raises(ValueError, match="conflicting"):
                async with Runtime(wrong_config):
                    pass
            wrong_domain = SharedSensor(key=key, transport=TransportConfig("cyclonedds", 75))
            with pytest.raises(SensorError, match="not running"):
                async with Runtime(wrong_domain):
                    pass
            attached = SharedSensor(key=key, transport=config)
            async with Runtime(attached):
                await attached.get(timeout=10)
                assert attached.worker_pid == owner.worker_pid
    asyncio.run(run())


def test_provider_only_configuration_conflicts_leave_clients_working():
    async def run():
        key = "provider:" + uuid.uuid4().hex
        factory = lambda: CounterArray(hz=10)
        owner = SharedSensor(factory, key=key, provider_version="settings-a")
        async with Runtime(owner):
            await owner.get(timeout=15)
            client = SharedSensor(key=key)
            equivalent = SharedSensor(factory, key=key, provider_version="settings-a")
            async with Runtime(client, equivalent):
                frame = await client.get(timeout=5)
                await equivalent.get(timeout=5)
                assert owner.worker_pid == client.worker_pid == equivalent.worker_pid
                conflict = SharedSensor(factory, key=key, provider_version="settings-b")
                with pytest.raises(ValueError, match="provider configuration"):
                    async with Runtime(conflict):
                        pass
                assert (await client.get(after=frame.sequence, timeout=5)).sequence > frame.sequence
    asyncio.run(run())
