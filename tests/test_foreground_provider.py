import asyncio
import os
from uuid import uuid4
import pytest

from rsim import Component, CommandSink, Runtime, SharedComponent, serve_component, TransportConfig, ComponentError
from rsim.runtime.sharing import component_ports
from rsim.runtime.options import RuntimeOptions, RuntimePaths


def test_foreground_owner_uses_current_pid_and_releases_registry(tmp_path):
    pytest.importorskip('cyclonedds')
    class Source(Component):
        def __init__(self):
            super().__init__()
            self.count = self.signal('count', clock='logical')
            self.add = CommandSink(self, 'add', self.apply, fallback=None, safe=lambda value: None, max_ttl=1.)
            self.value = 0
        async def open(self):
            await self.count.publish(0, stamp_ns=0, clock='logical')
        async def apply(self, envelope):
            self.value += envelope.value
            await self.count.publish(self.value, stamp_ns=self.value, clock='logical')
            return dict(value=self.value, pid=os.getpid())
    async def run():
        source = Source()
        key = 'foreground-'+uuid4().hex
        transport = TransportConfig(domain_id=181)
        options = RuntimeOptions(paths=RuntimePaths(registry_root=tmp_path/'registry', storage_root=tmp_path/'storage', log_root=tmp_path/'logs'))
        ready, stopped = asyncio.Event(), asyncio.Event()
        manifest = {}
        def on_ready(value):
            manifest.update(value)
            ready.set()
        owner = asyncio.create_task(serve_component(source, key=key, transport=transport, options=options,
                                                   stopped=stopped, on_ready=on_ready))
        try:
            await asyncio.wait_for(ready.wait(), 15)
            assert manifest['pid'] == os.getpid()
            with pytest.raises(ComponentError, match='already running'):
                await serve_component(Source(), key=key, transport=transport, options=options)
            view = SharedComponent(key=key, ports=component_ports(source), transport=transport, options=options)
            async with Runtime(view.count, view.add, options=options):
                assert (await view.count.get(timeout=10)).data == 0
                result = await view.add.set(2)
                assert result == dict(value=2, pid=os.getpid())
                assert (await view.count.get(timestamp_ns=2, clock='logical', timeout=10)).data == 2
        finally:
            stopped.set()
            await asyncio.wait_for(owner, 15)
        assert source._closed
        assert not list((tmp_path/'registry').glob('*.sock'))
        assert not list((tmp_path/'storage').glob('rsim-foreground-*'))
    asyncio.run(run())


@pytest.mark.parametrize('failure', ['open', 'ready', 'cancel'])
def test_foreground_failures_release_storage_and_ownership(tmp_path, failure):
    pytest.importorskip('cyclonedds')
    class Source(Component):
        def __init__(self):
            super().__init__()
            self.value = self.signal('value', clock='logical')
        async def open(self):
            if failure == 'open': raise RuntimeError('deliberate open failure')
            await self.value.publish(1, stamp_ns=0, clock='logical')
    async def run():
        options = RuntimeOptions(paths=RuntimePaths(registry_root=tmp_path/'registry', storage_root=tmp_path/'storage', log_root=tmp_path/'logs'))
        ready = asyncio.Event()
        def on_ready(manifest):
            ready.set()
            if failure == 'ready': raise RuntimeError('deliberate ready failure')
        task = asyncio.create_task(serve_component(Source(), key='failure-'+uuid4().hex,
            transport=TransportConfig(domain_id=182), options=options, on_ready=on_ready))
        if failure == 'cancel':
            await asyncio.wait_for(ready.wait(), 10)
            task.cancel()
            with pytest.raises(asyncio.CancelledError): await task
        else:
            with pytest.raises(Exception, match='deliberate'):
                await asyncio.wait_for(task, 10)
        assert not list((tmp_path/'registry').glob('*.sock'))
        assert not list((tmp_path/'storage').glob('rsim-foreground-*'))
    asyncio.run(run())


def test_native_action_longer_than_discovery_timeout_keeps_its_deadline(tmp_path):
    pytest.importorskip('cyclonedds')
    class Source(Component):
        def __init__(self):
            super().__init__()
            self.value = self.signal('value', clock='logical')
            self.calls = 0
            self.slow = CommandSink(self, 'slow', self.apply, fallback=None, safe=lambda _:None, max_ttl=3.)
        async def apply(self, envelope):
            self.calls += 1
            await asyncio.sleep(.8)
            return {'calls':self.calls}
    async def run():
        source = Source()
        key = 'slow-'+uuid4().hex
        ready, stopped = asyncio.Event(), asyncio.Event()
        transport = TransportConfig(domain_id=183)
        options = RuntimeOptions(paths=RuntimePaths(registry_root=tmp_path/'registry', storage_root=tmp_path/'storage', log_root=tmp_path/'logs'))
        owner = asyncio.create_task(serve_component(source, key=key, transport=transport, options=options,
            stopped=stopped, on_ready=lambda _:ready.set()))
        try:
            await asyncio.wait_for(ready.wait(), 10)
            client = SharedComponent(key=key, ports=component_ports(source), transport=transport, options=options)
            async with Runtime(client.slow, options=options):
                assert await client.slow.set('one native action') == {'calls':1}
                assert source.calls == 1  # DDS request retries do not repeat it.
        finally:
            stopped.set()
            await asyncio.wait_for(owner, 10)
    asyncio.run(run())
