"""Same file/IPC/namespace definitions on both native implementations."""
import asyncio
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest

from rsim.core import Frame
from rsim.platforms import get_platform
from rsim.platforms.errors import CapabilityUnavailable
from rsim.platforms.paths import new_directory
from rsim.platforms.probe import probe_storage
from rsim.transport.files import backing_identity
from rsim.transport.shared import SharedStore, decode


@pytest.mark.parametrize('value', [np.empty((0, 3)), np.array(4), np.arange(20).reshape(4, 5).T,
    np.arange(9, dtype='>i4'), np.ones((2, 3), dtype=[('点', 'f4'), ('name', 'U4')])])
def test_npy_identity_layout_and_retained_unlink(tmp_path, value):
    store = SharedStore(tmp_path / '中文 path')
    prepared = store.prepare(value)
    descriptor = store.put(Frame(prepared, 10, 'sample'))
    first = decode(descriptor['data'], store.directory)
    second = decode(descriptor['data'], store.directory)
    assert backing_identity(first) == backing_identity(second) == backing_identity(prepared)
    store.close()
    assert not store.directory.exists()
    assert first.dtype == value.dtype and first.shape == value.shape
    np.testing.assert_array_equal(first, value)
    assert not first.flags.writeable


def test_copy_paths_and_removed_backing_have_explicit_metrics(tmp_path):
    first, second = SharedStore(tmp_path / 'first'), SharedStore(tmp_path / 'second')
    array = first.prepare(np.arange(12))
    result = decode(second.put(Frame(array[2:7], 1, 'test'))['data'], second.directory)
    assert second.diagnostics()['copy_reasons']['view_or_layout'] == 1
    first.close()
    result2 = decode(second.put(Frame(array, 2, 'test'))['data'], second.directory)
    assert second.diagnostics()['copy_reasons']['source_path_removed'] == 1
    np.testing.assert_array_equal(result, np.arange(2, 7))
    np.testing.assert_array_equal(result2, np.arange(12))
    second.close()


@pytest.mark.parametrize('race,reason', [('removed', 'source_path_removed'),
    ('replaced', 'backing_changed_during_link'), ('cross_volume', 'cross_volume')])
def test_forwarding_never_links_a_replaced_or_removed_backing(tmp_path, monkeypatch, race, reason):
    from rsim.transport import shared
    from rsim.transport.files import unlink
    first, second = SharedStore(tmp_path / 'first'), SharedStore(tmp_path / 'second')
    data = first.prepare(np.arange(8))
    native_link = shared.hardlink
    def interrupted(source, destination):
        if race == 'cross_volume':
            raise OSError(18, 'injected cross-volume link')
        unlink(source)
        if race == 'replaced':
            np.save(source, np.full(8, -1, dtype=data.dtype))
        return native_link(source, destination)
    monkeypatch.setattr(shared, 'hardlink', interrupted)
    result = decode(second.put(Frame(data, 1, 'test'))['data'], second.directory)
    np.testing.assert_array_equal(result, np.arange(8))
    assert second.diagnostics()['copy_reasons'][reason] == 1
    first.close()
    second.close()


def test_real_child_mapping_capability(tmp_path):
    report = probe_storage(str(tmp_path), True)
    assert report['child_mapping']
    assert report['filesystem']['filesystem'] and not report['filesystem']['remote']


def test_cleanup_failure_is_bounded_and_reported(tmp_path, monkeypatch):
    from rsim.platforms.files import remove_tree
    from rsim.platforms.errors import cleanup_failures
    attempts = []
    def denied(path):
        attempts.append(path)
        raise PermissionError(13, 'injected cleanup denial')
    monkeypatch.setattr(get_platform().files, 'remove_tree', denied)
    with pytest.warns(RuntimeWarning, match='remove_tree failed'):
        records = remove_tree(tmp_path / 'owned', ignore_errors=True, instance_id='test')
    assert len(attempts) == 3 and records[0]['instance_id'] == 'test'
    assert records[0]['error_code'] == 13
    cleanup_failures.remove(records[0])


def test_unknown_platform_is_not_linux(monkeypatch):
    monkeypatch.setattr(sys, 'platform', 'darwin')
    with pytest.raises(CapabilityUnavailable, match='unsupported'):
        get_platform()


def test_storage_symlink_or_junction_is_refused_before_creating_files(tmp_path):
    import subprocess
    target, link = tmp_path / 'untouched', tmp_path / 'redirect'
    target.mkdir()
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        if get_platform().name != 'win32':
            raise
        subprocess.run(['cmd', '/d', '/c', 'mklink', '/J', str(link), str(target)],
                       capture_output=True, check=True)
    with pytest.raises(PermissionError, match='link'):
        new_directory(prefix='must-not-create-', root=link)
    assert list(target.iterdir()) == []


def test_storage_scope_changes_registry_and_manifest_identity(tmp_path):
    from rsim.runtime.options import RuntimeOptions, RuntimePaths
    from rsim.runtime.registry import host_id, registry_paths
    a = RuntimeOptions(paths=RuntimePaths(registry_root=tmp_path / 'registry', storage_root=tmp_path / 'a'))
    b = RuntimeOptions(paths=RuntimePaths(registry_root=tmp_path / 'registry', storage_root=tmp_path / 'b'))
    assert host_id(a) != host_id(b)
    assert registry_paths('same', 11, a) != registry_paths('same', 11, b)


def test_local_ipc_is_usable_with_private_endpoint(tmp_path):
    async def run():
        from rsim.platforms import get_platform
        directory = new_directory(prefix='ipc-', root=tmp_path / ('long-registry-' * 5))
        async def handle(reader, writer):
            assert await reader.readline() == b'hello\n'
            writer.write(b'ok\n')
            await writer.drain()
            writer.close()
        server = await get_platform().ipc.start_server(handle, path=directory / 'endpoint')
        try:
            reader, writer = await get_platform().ipc.open_connection(directory / 'endpoint')
            writer.write(b'hello\n')
            await writer.drain()
            assert await asyncio.wait_for(reader.readline(), 2) == b'ok\n'
            writer.close()
            await writer.wait_closed()
        finally:
            server.close()
            await server.wait_closed()
    asyncio.run(run())
