"""Platform-independent lifecycle, demand and options contracts (P01/P03/P12/P21/P24)."""
import asyncio
from dataclasses import FrozenInstanceError
import json
from pathlib import Path

import pytest
from rsim import Component, Runtime
from rsim.runtime.options import RuntimeOptions, RuntimePaths, ShutdownPolicy
from rsim.runtime.supervision import State
from rsim.core.errors import CleanupFailed, StartupFailed


@pytest.mark.parametrize('value', [0, -1, float('inf'), float('nan'), True])
def test_durations_must_be_finite_positive(value):
    with pytest.raises(ValueError):
        ShutdownPolicy(grace_s=value)
    with pytest.raises(ValueError):
        ShutdownPolicy(kill_wait_s=value)
    with pytest.raises(ValueError):
        RuntimeOptions(startup_timeout_s=value)


def test_options_precedence_resolution_and_roundtrip(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('RSIM_LOG_ROOT', str(tmp_path / 'environment'))
    options = RuntimeOptions(paths=RuntimePaths(log_root=Path('explicit'))).resolved()
    assert options.paths.log_root == tmp_path / 'explicit'
    assert RuntimeOptions.from_dict(options.to_dict()) == options
    assert not (tmp_path / 'explicit').exists()
    frozen_storage = options.paths.storage_root
    monkeypatch.setenv('RSIM_STORAGE_ROOT', str(tmp_path / 'changed-after-open'))
    assert options.resolved().paths.storage_root == frozen_storage


def test_views_in_different_discovery_domains_do_not_coalesce(tmp_path):
    from rsim import SharedComponent, PortSpec
    views = [SharedComponent(key='camera', ports={'image': PortSpec()},
             options=RuntimeOptions(paths=RuntimePaths(storage_root=tmp_path / name))) for name in ('a', 'b')]
    assert views[0].key != views[1].key


def test_static_dynamic_demand_and_snapshot_are_independent_of_hooks():
    async def run():
        source = Component()
        source.pose, source.image = source.signal('pose'), source.signal('image')
        wrapper = Component()
        wrapper.expose('alias', source.image)
        runtime = Runtime(source.pose, Component(wrapper))
        async with runtime:
            assert source.has_demand('pose') and not source.has_demand('image')
            original = source.demand_snapshot()
            source.image.add_prepare_hook('unrelated', lambda value: value)
            assert not source.has_demand('image')
            runtime.demand.acquire(wrapper.alias, 'first')
            runtime.demand.acquire(source.image, 'first')
            runtime.demand.acquire(source.image, 'second')
            assert runtime.demand.count(source.image) == 2
            current = source.demand_snapshot()
            assert current.ports == frozenset({'pose', 'image'})
            assert original.ports == frozenset({'pose'})
            assert current.generation > original.generation
            with pytest.raises(FrozenInstanceError):
                current.generation = 0
            runtime.demand.release(source.image, 'first')
            assert source.has_demand('image')
            runtime.demand.release(source.image, 'second')
            assert not source.has_demand('image')
            json.dumps(runtime.diagnostics())
        assert runtime.lifecycle.state == State.CLOSED
        assert not source.has_demand('pose')
        async with runtime:
            assert source.has_demand('pose')
    asyncio.run(run())


def test_startup_timeout_and_cancellation_keep_original_failure():
    class Slow(Component):
        async def open(self):
            self.entered.set()
            await asyncio.Future()
        async def close(self):
            self.closed = True
    async def run():
        for cancelled in (False, True):
            source = Slow()
            source.entered, source.closed = asyncio.Event(), False
            runtime = Runtime(source, options=RuntimeOptions(startup_timeout_s=.05))
            task = asyncio.create_task(runtime.__aenter__())
            await source.entered.wait()
            if cancelled:
                task.cancel()
            with pytest.raises(asyncio.CancelledError if cancelled else StartupFailed):
                await task
            assert source.closed and source._runtime is None
            assert runtime.lifecycle.state == State.FAILED
    asyncio.run(run())


def test_inprocess_unfinished_close_is_reported_and_retryable():
    class SlowClose(Component):
        async def close(self):
            await self.release.wait()
    async def run():
        source = SlowClose()
        source.release = asyncio.Event()
        runtime = Runtime(source, options=RuntimeOptions(shutdown=ShutdownPolicy(.01, .01)))
        await runtime.__aenter__()
        with pytest.raises(ExceptionGroup) as error:
            await runtime.aclose()
        assert any(isinstance(item, CleanupFailed) for item in error.value.exceptions)
        assert not source._closed and source._runtime is runtime
        assert runtime.lifecycle.state == State.FAILED
        source.release.set()
        await runtime.aclose()
        assert source._closed and source._runtime is None
    asyncio.run(run())


def test_history_eviction_is_visible_without_claiming_lossless_reads():
    async def run():
        component = Component()
        output = component.signal('sample', history=2)
        async with Runtime(output):
            for index in range(9):
                await output.publish(index, stamp_ns=index, clock='test')
            assert output.history_evictions == 7
            assert (await output.get()).data == 8
    asyncio.run(run())
