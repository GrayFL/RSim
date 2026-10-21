"""Native-process startup transactions and last-source-lease races (P12-P14)."""
import asyncio
import json
import os
from pathlib import Path
import uuid

import pytest
from rsim import Component, Runtime, SharedProvider, SharedComponent, ProcessPlacement, PortSpec
from rsim.runtime.options import RuntimeOptions, RuntimePaths, ShutdownPolicy
from rsim.runtime.registry import registry_paths
from process_support import alive


def test_cancel_during_native_spawn_still_closes_the_inherited_lease(tmp_path, monkeypatch):
    from rsim.platforms.process import spawn_leased
    async def run():
        directory = tmp_path / 'guard'
        directory.mkdir()
        (directory / 'guard.json').write_text(json.dumps(RuntimeOptions().to_dict()))
        created, release = asyncio.Event(), asyncio.Event()
        native_create = asyncio.create_subprocess_exec
        processes = []
        async def delayed(*args, **kwargs):
            child = await native_create(*args, **kwargs)
            processes.append(child)
            created.set()
            await release.wait()
            return child
        monkeypatch.setattr(asyncio, 'create_subprocess_exec', delayed)
        opening = asyncio.create_task(spawn_leased('rsim.runtime.store_guard', directory))
        await asyncio.wait_for(created.wait(), 5)
        opening.cancel()
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await opening
        assert processes[0].returncode == 0
        assert not directory.exists()
    asyncio.run(run())


class SlowStart(Component):
    def __init__(self, marker):
        super().__init__()
        self.marker = str(marker)
        self.sample = self.signal('sample')
    async def open(self):
        Path(self.marker).write_text(str(os.getpid()))
        await asyncio.Future()


@pytest.mark.parametrize('deployment', ['placement', 'shared'])
def test_cancel_initializing_real_worker_cleans_only_its_instance(tmp_path, deployment):
    async def run():
        options = RuntimeOptions(paths=RuntimePaths(storage_root=tmp_path / 'payload',
            registry_root=tmp_path / 'registry', log_root=tmp_path / 'logs'),
            shutdown=ShutdownPolicy(.2, 1), startup_timeout_s=8)
        marker = tmp_path / 'worker.pid'
        source = SlowStart(marker)
        if deployment == 'shared':
            source = SharedProvider(source, key='cancel:' + uuid.uuid4().hex, options=options)
            runtime = Runtime(source, options=options)
        else:
            runtime = Runtime(source, placement={source: ProcessPlacement('slow')}, options=options)
        opening = asyncio.create_task(runtime.__aenter__())
        try:
            async with asyncio.timeout(7):
                while not marker.exists():
                    if opening.done():
                        await opening
                    await asyncio.sleep(.02)
            pid = int(marker.read_text())
            opening.cancel()
            with pytest.raises(asyncio.CancelledError):
                await opening
            async with asyncio.timeout(5):
                while alive(pid) or list((tmp_path / 'payload').iterdir()):
                    await asyncio.sleep(.02)
            assert list((tmp_path / 'logs').glob('*.log'))
        finally:
            if not opening.done():
                opening.cancel()
            await asyncio.gather(opening, return_exceptions=True)
            await runtime.aclose()
    asyncio.run(run())


class ExclusiveSource(Component):
    def __init__(self, key):
        super().__init__()
        self.device_key = key
        self.sample = self.signal('sample', clock='test')
    async def open(self):
        from rsim.runtime.locks import acquire_device
        self.lease = acquire_device(self.device_key)
        self.task('sample', self.tick, hz=30)
    async def tick(self):
        await self.sample.publish(os.getpid(), stamp_ns=1, clock='test')
    async def close(self):
        await asyncio.sleep(.05)
        if hasattr(self, 'lease'):
            self.lease.close()


def test_parallel_launch_and_last_release_share_one_ownership_rule(tmp_path):
    async def run():
        key = 'race:' + uuid.uuid4().hex
        options = RuntimeOptions(paths=RuntimePaths(registry_root=tmp_path / 'registry',
            storage_root=tmp_path / 'payload', log_root=tmp_path / 'logs'))
        def make():
            provider = SharedProvider(ExclusiveSource(key), key=key, options=options)
            return provider, Runtime(provider, options=options)
        first, a = make()
        second, b = make()
        await asyncio.gather(a.__aenter__(), b.__aenter__())
        try:
            assert first.manifest['pid'] == second.manifest['pid']
            await a.aclose()
            assert (await second.sample.get(timeout=5)).data == second.manifest['pid']
            for _ in range(4):
                third, c = make()
                await asyncio.gather(b.aclose(), c.__aenter__())
                assert (await third.sample.get(timeout=5)).data == third.manifest['pid']
                second, b = third, c
        finally:
            await a.aclose()
            await b.aclose()
        async with asyncio.timeout(10):
            while list((tmp_path / 'payload').iterdir()):
                await asyncio.sleep(.05)
    asyncio.run(run())


def test_restart_changes_instance_and_old_sample_identity(tmp_path):
    async def run():
        key = 'restart:' + uuid.uuid4().hex
        seen = []
        for _ in range(2):
            provider = SharedProvider(ExclusiveSource(key), key=key)
            async with Runtime(provider):
                frame = await provider.sample.get(timeout=5)
                seen.append((provider.manifest['instance_id'], frame.sample_id))
            async with asyncio.timeout(10):
                while provider.directory.exists():
                    await asyncio.sleep(.02)
        assert seen[0][0] != seen[1][0] and seen[0][1] != seen[1][1]
    asyncio.run(run())
