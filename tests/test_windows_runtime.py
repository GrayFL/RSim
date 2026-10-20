"""Native Windows failure and file semantics, beyond ordinary graph tests."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest

from process_support import alive

pytestmark = pytest.mark.skipif(os.name != 'nt', reason='Windows native primitives')


def test_job_owner_death_kills_worker_and_unmanaged_grandchild(tmp_path):
    ready = tmp_path / 'ready.json'
    worker = """
import json,subprocess,sys,time
from pathlib import Path
child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])
Path(sys.argv[1]).write_text(json.dumps({'worker':__import__('os').getpid(),'child':child.pid}))
time.sleep(60)
"""
    owner = """
import sys,time
from rsim.runtime.windows_process import JobProcess
worker=JobProcess([sys.executable,'-c',sys.argv[1],sys.argv[2]])
time.sleep(60)
"""
    process = subprocess.Popen([sys.executable, '-c', owner, worker, str(ready)], creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        deadline = time.monotonic()+10
        while not ready.exists():
            assert process.poll() is None
            assert time.monotonic() < deadline
            time.sleep(.02)
        pids = json.loads(ready.read_text())
        process.kill()
        process.wait(5)
        deadline = time.monotonic()+5
        while any(alive(pid) for pid in pids.values()):
            assert time.monotonic() < deadline
            time.sleep(.02)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(5)


def test_local_endpoint_rejects_wrong_token_before_dispatch(tmp_path):
    from rsim.runtime import local_socket
    async def run():
        called = []
        async def handler(reader, writer):
            called.append(True)
            writer.close()
        path = tmp_path / 'endpoint.sock'
        server = await local_socket.start_server(handler, path=path)
        try:
            endpoint = json.loads(path.read_text())
            reader, writer = await asyncio.open_connection(endpoint['host'], endpoint['port'])
            writer.write(b'0'*64+b'\n')
            await writer.drain()
            assert await asyncio.wait_for(reader.read(1), 2) == b''
            assert not called
            writer.close()
            await writer.wait_closed()
            reader, writer = await local_socket.open_connection(path)
            await reader.read(1)
            assert called == [True]
            writer.close()
            await writer.wait_closed()
        finally:
            server.close()
            await server.wait_closed()
    asyncio.run(run())


@pytest.mark.parametrize('value', [np.empty((0, 3)), np.arange(20).reshape(4, 5).T,
                                  np.ones(3, dtype=[('点', 'f4')])])
def test_mapped_arrays_preserve_npy_layout_after_unlink(tmp_path, value):
    from rsim.transport.shared import SharedStore, decode
    from rsim.core.model import Frame
    store = SharedStore(tmp_path / 'mapping')
    array = store.prepare(value)
    result = decode(store.put(Frame(array, 1, 'test'))['data'], store.directory)
    store.close()
    assert not store.directory.exists()
    np.testing.assert_array_equal(result, value)
    assert not result.flags.writeable
