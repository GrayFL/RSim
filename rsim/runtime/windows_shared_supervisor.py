"""Independent Windows source leases, registry proxy and worker Job ownership."""
import asyncio
import json
from pathlib import Path
import sys

from rsim.transport.files import remove_tree
from . import local_socket
from .locks import open_lock, try_lock
from .platform import spawn_worker, stop_worker


async def run(directory, path, lock_path, mode):
    clients = set()
    tasks = set()
    connected = asyncio.Event()
    worker = None
    cleanup_lock = None

    async def copy(reader, writer):
        writer.transport.set_write_buffer_limits(high=65536)
        while packet := await reader.read(65536):
            writer.write(packet)
            await writer.drain()

    async def accept(reader, writer):
        tasks.add(asyncio.current_task())
        clients.add(writer)
        connected.set()
        upstream = None
        forwarding = []
        try:
            if mode == 'sensor':
                while await reader.read(1024):
                    pass
            else:
                async with asyncio.timeout(30):
                    while True:
                        try:
                            back_reader, upstream = await local_socket.open_connection(directory / 'registry.sock')
                            break
                        except (OSError, asyncio.IncompleteReadError):
                            if worker is not None and worker.poll() is not None:
                                return
                            await asyncio.sleep(.02)
                forwarding = [asyncio.create_task(copy(reader, upstream)), asyncio.create_task(copy(back_reader, writer))]
                await asyncio.wait(forwarding, return_when=asyncio.FIRST_COMPLETED)
        except (OSError, TimeoutError):
            pass
        finally:
            clients.discard(writer)
            tasks.discard(asyncio.current_task())
            for task in forwarding:
                task.cancel()
            await asyncio.gather(*forwarding, return_exceptions=True)
            if upstream is not None:
                upstream.close()
                try:
                    await upstream.wait_closed()
                except OSError:
                    pass
            writer.close()

    server = await local_socket.start_server(accept, path=path)
    try:
        await asyncio.wait_for(connected.wait(), 30)
        module = 'rsim.runtime.worker' if mode == 'sensor' else 'rsim.runtime.shared_provider_worker'
        worker = spawn_worker(module, directory, *(() if mode == 'sensor' else (directory / 'registry.sock',)))
        failed = False
        while True:
            if worker.poll() is not None and not failed:
                if mode == 'ports':
                    for client in tuple(clients):
                        client.close()
                else:
                    status = directory / 'status.json'
                    state = json.loads(status.read_text(encoding='utf-8')) if status.exists() else {}
                    if 'error' not in state:
                        temporary = directory / 'supervisor-status.tmp'
                        temporary.write_text(json.dumps(dict(pid=worker.pid, error=f'worker exited: {worker.returncode}')), encoding='utf-8')
                        temporary.replace(status)
                failed = True
            if not clients:
                candidate = open_lock(lock_path)
                try:
                    try_lock(candidate)
                except BlockingIOError:
                    candidate.close()
                else:
                    cleanup_lock = candidate
                    # No await between observing empty, owning the startup lock,
                    # and refusing new clients; launch/last-close are serialized.
                    server.close()
                    break
            await asyncio.sleep(.05)
    finally:
        server.close()
        await server.wait_closed()
        for client in tuple(clients):
            client.close()
        for task in tuple(tasks):
            task.cancel()
        await asyncio.gather(*tuple(tasks), return_exceptions=True)
        await asyncio.to_thread(stop_worker, worker, directory)
        remove_tree(directory, ignore_errors=True)
        path.unlink(missing_ok=True)
        if cleanup_lock is not None:
            cleanup_lock.close()


if __name__ == '__main__':
    asyncio.run(run(*(Path(value) for value in sys.argv[1:4]), sys.argv[4]))
