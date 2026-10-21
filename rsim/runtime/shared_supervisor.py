"""Shared source leases and registry proxy; independent of the worker event loop."""
import asyncio
import json
from pathlib import Path
import sys

from rsim.transport.files import remove_tree
from .options import RuntimeOptions
from .supervision import Lifecycle, State, read_status, stop_worker
from dataclasses import asdict
from rsim.platforms import get_platform


async def run(directory, path, lock_path, mode):
    platform = get_platform()
    config = json.loads((directory / 'config.json').read_text(encoding='utf-8'))
    options = RuntimeOptions.from_dict(config.get('options'))
    lifecycle = Lifecycle('shared')
    lifecycle.transition(State.STARTING)
    report = Path(config['log_path']).with_suffix('.report.json')
    def save():
        temporary = report.with_suffix('.tmp')
        temporary.write_text(json.dumps(asdict(lifecycle)), encoding='utf-8')
        temporary.replace(report)
    save()
    stopped = asyncio.Event()
    platform.process.install_shutdown(stopped, directory)
    ready = asyncio.Event()
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
                await ready.wait()
                writer.write(b'1')
                await writer.drain()
                while await reader.read(1024):
                    pass
            else:
                async with asyncio.timeout(options.startup_timeout_s):
                    while True:
                        try:
                            back_reader, upstream = await platform.ipc.open_connection(directory / 'registry.sock')
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

    owner_lock = platform.locks.open_lock(path.with_suffix('.owner.lock'))
    platform.locks.try_lock(owner_lock)
    server = await platform.ipc.start_server(accept, path=path)
    endpoint_identity = platform.ipc.endpoint_identity(path)
    try:
        await asyncio.wait_for(connected.wait(), options.startup_timeout_s)
        module = 'rsim.runtime.worker' if mode == 'sensor' else 'rsim.runtime.shared_provider_worker'
        worker = platform.process.spawn_worker(module, directory, *(() if mode == 'sensor' else (directory / 'registry.sock',)))
        failed = False
        loop = asyncio.get_running_loop()
        startup_deadline = loop.time() + options.startup_timeout_s
        while not stopped.is_set() and not (directory / 'supervisor.stop').exists():
            if not ready.is_set():
                if read_status(directory).get('ready'):
                    ready.set()
                    lifecycle.transition(State.RUNNING)
                    save()
                elif loop.time() >= startup_deadline:
                    raise TimeoutError('provider initialization timed out')
            if worker.poll() is not None and not failed:
                if mode == 'sensor':
                    status = directory / 'status.json'
                    state = read_status(directory)
                    if 'error' not in state:
                        temporary = directory / 'supervisor-status.tmp'
                        temporary.write_text(json.dumps(dict(pid=worker.pid, error=f'worker exited: {worker.returncode}')), encoding='utf-8')
                        temporary.replace(status)
                failed = True
                lifecycle.transition(State.FAILED, RuntimeError(f'worker exited: {worker.returncode}'))
                save()
                break
            if not clients:
                candidate = platform.locks.open_lock(lock_path)
                try:
                    platform.locks.try_lock(candidate)
                except BlockingIOError:
                    candidate.close()
                else:
                    cleanup_lock = candidate
                    # No await between observing empty, owning the startup lock,
                    # and refusing new clients; launch/last-close are serialized.
                    server.close()
                    break
            await asyncio.sleep(.05)
    except BaseException as error:
        lifecycle.transition(State.FAILED, error)
        save()
        raise
    finally:
        lifecycle.transition(State.STOPPING)
        save()
        server.close()
        await server.wait_closed()
        # Release containment before making failed connections look reconnectable.
        try:
            await asyncio.to_thread(stop_worker, worker, directory, options.shutdown)
        except Exception as error:
            lifecycle.cleanup.append(dict(operation='stop-worker', error=f'{type(error).__name__}: {error}'))
        for client in tuple(clients):
            client.close()
        for task in tuple(tasks):
            task.cancel()
        await asyncio.gather(*tuple(tasks), return_exceptions=True)
        try:
            lifecycle.cleanup.extend(remove_tree(directory, ignore_errors=True,
                                                 instance_id=config.get('manifest', {}).get('instance_id')))
            try:
                if platform.ipc.endpoint_identity(path) == endpoint_identity:
                    path.unlink()
            except FileNotFoundError:
                pass
            lifecycle.transition(State.FAILED if lifecycle.failure or lifecycle.cleanup else State.CLOSED)
        except BaseException as error:
            lifecycle.transition(State.FAILED, error)
            raise
        finally:
            save()
            if cleanup_lock is not None:
                cleanup_lock.close()
            owner_lock.close()


if __name__ == '__main__':
    asyncio.run(run(*(Path(value) for value in sys.argv[1:4]), sys.argv[4]))
