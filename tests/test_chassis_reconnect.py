import asyncio
from types import SimpleNamespace
import uuid

import pytest


def test_service_rebuilds_after_runtime_timeout_without_replaying_commands(monkeypatch):
    pytest.importorskip('cyclonedds')
    pytest.importorskip('graphmap')
    from rsim.apps.chassis_service import run
    from rsim import drivers
    from rsim.devices import Chassis
    from rsim.core import VelocityCommand
    from rsim.runtime import Runtime
    from rsim.transport.descriptor import TransportConfig

    services = []
    factory = drivers.Chassis
    def create(*args, **kwargs):
        service = factory(*args, **kwargs)
        services.append(service)
        return service
    monkeypatch.setattr(drivers, 'Chassis', create)

    async def wait_for(predicate):
        async with asyncio.timeout(8):
            while not predicate():
                await asyncio.sleep(.01)

    async def exercise():
        args = SimpleNamespace(simulate=True, name='recover_'+uuid.uuid4().hex, domain=86,
            enable_motion=False, blas_threads=1, reconnect_delay=.1, no_reconnect=False)
        task = asyncio.create_task(run(args))
        try:
            await wait_for(lambda: services and services[0].pose.frames)
            first = services[0]
            generation = first.generation
            async def lost_hardware():
                raise TimeoutError('injected hardware RPC timeout')
            first.controller.task('injected-timeout', lost_hardware, hz=100)
            await wait_for(lambda: len(services) >= 2 and services[-1].pose.frames)
            assert first._closed
            second = services[-1]
            assert second.generation != generation
            assert second.owner is None
            assert second.controller.velocity.producer.command == VelocityCommand()
            client = Chassis(args.name, transport=TransportConfig(domain_id=86))
            async with Runtime(client):
                await client.pose.get(timeout=3)
                await client.drive(VelocityCommand(), ttl=.25)
            assert not task.done()
        finally:
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
        assert all(s._closed for s in services)

    asyncio.run(exercise())
