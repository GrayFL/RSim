import asyncio
import pytest
from rsim.transport.backends.inprocess import InProcessTransport
from rsim.runtime.locks import acquire_device
from rsim.core.errors import ComponentError


def test_bounded_retention_domain_isolation_and_cleanup():
    async def run():
        a, b, isolated = [InProcessTransport(i) for i in (1, 1, 2)]
        for bus in (a, b, isolated):
            bus.open()
        received, volatile, elsewhere = [], [], []
        writer = a.publisher('sample', depth=2)
        for value in ('1', '2', '3'):
            writer.publish(value)
        b.subscribe('sample', received.append, depth=2)
        b.subscribe('sample', volatile.append, durable=False)
        isolated.subscribe('sample', elsewhere.append)
        b.poll()
        assert received == ['2', '3'] and not volatile
        writer.publish('4')
        b.poll()
        isolated.poll()
        assert received == ['2', '3', '4'] and volatile == ['4'] and not elsewhere
        for bus in (a, b, isolated):
            bus.close()
            bus.close()
            assert not bus.endpoints
        assert not a.bus
        with pytest.raises(RuntimeError):
            writer.publish('closed')
    asyncio.run(run())


def test_exclusive_device_lock_releases_on_close():
    handle = acquire_device('test_mujoco_lock')
    try:
        with pytest.raises(ComponentError):
            acquire_device('test_mujoco_lock')
    finally:
        handle.close()
    acquire_device('test_mujoco_lock').close()
