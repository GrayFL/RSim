"""Windows shared-source launch helpers. Only local launchers serialize recipes."""
import asyncio
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import uuid

import cloudpickle

from rsim.core.errors import ProviderDisconnected
from rsim.transport.files import storage_root, remove_tree, open_shared
from . import local_socket
from .locks import open_lock, try_lock
from .platform import user_id, supervisor_options


async def locked(path):
    lock = open_lock(path)
    try:
        async with asyncio.timeout(35):
            while True:
                try:
                    try_lock(lock)
                    return lock
                except BlockingIOError:
                    await asyncio.sleep(.02)
    except BaseException:
        lock.close()
        raise


async def launch(directory, path, lock_path, mode, log_path=None):
    log_path = Path(log_path) if log_path else directory / 'worker.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    path.unlink(missing_ok=True)
    with open_shared(log_path, write=True, create=not log_path.exists()) as log:
        log.seek(0, 2)
        process = subprocess.Popen([sys.executable, '-m', 'rsim.runtime.windows_shared_supervisor',
                                    str(directory), str(path), str(lock_path), mode],
                                   stdout=log, stderr=log, **supervisor_options())
    threading.Thread(target=process.wait, daemon=True, name='rsim-provider-reaper').start()
    try:
        async with asyncio.timeout(30):
            while True:
                if process.poll() is not None:
                    raise RuntimeError('shared supervisor exited:\n' + log_path.read_text(encoding='utf-8')[-8000:])
                try:
                    return await local_socket.open_connection(path)
                except (OSError, asyncio.IncompleteReadError):
                    await asyncio.sleep(.02)
    except BaseException:
        process.terminate()
        await asyncio.to_thread(process.wait)
        remove_tree(directory, ignore_errors=True)
        raise


async def open_provider(provider):
    from .registry import registry_paths, Connection, PROTOCOL, host_id
    path, lock_path = registry_paths(provider.component_key, provider.transport.domain_id)
    lock = await locked(lock_path)
    try:
        try:
            provider.connection = await Connection.connect(path)
            return
        except ProviderDisconnected:
            pass
        directory = Path(tempfile.mkdtemp(prefix=f'rsim-provider-{user_id()}-', dir=storage_root()))
        provider.directory = directory
        recipe = dict(component=provider.component, factory=provider.factory, placement=provider.provider_placement)
        (directory / 'recipe.pkl').write_bytes(cloudpickle.dumps(recipe))
        manifest = dict(protocol_version=PROTOCOL, component_key=provider.component_key,
                        instance_id=uuid.uuid4().hex, interface_version=provider.interface_version,
                        provider_version=provider.provider_version, host_id=host_id(),
                        domain_id=provider.transport.domain_id,
                        ports={name: spec.manifest() for name, spec in provider.port_specs.items()})
        (directory / 'config.json').write_text(json.dumps(dict(manifest=manifest,
            transport=asdict(provider.transport), sys_path=[str(Path(p).resolve()) for p in sys.path])), encoding='utf-8')
        provider.connection = Connection(*await launch(directory, path, lock_path, 'ports', provider.log_path))
    finally:
        lock.close()


async def open_sensor(sensor):
    import hashlib
    from .host import registry_directory
    from rsim.core.errors import ComponentError
    digest = hashlib.sha256(f'{sensor.transport.domain_id}:{sensor.source_key}'.encode()).hexdigest()[:32]
    base = registry_directory() / digest
    path, lock_path, state_path = [base.with_suffix(suffix) for suffix in ('.sock', '.lock', '.json')]
    lock = await locked(lock_path)
    signature = dict(version=sensor.version, history=sensor._history.maxlen)
    try:
        state = None
        if state_path.exists():
            try:
                sensor._reader, sensor._writer = await local_socket.open_connection(path)
            except (OSError, asyncio.IncompleteReadError):
                pass
            else:
                state = json.loads(state_path.read_text(encoding='utf-8'))
                if state['signature'] != signature:
                    raise ValueError(f'conflicting shared source configuration: {sensor.source_key}')
                if sensor.factory is not None and state['provider_version'] != sensor.provider_version:
                    raise ValueError(f'conflicting shared provider configuration: {sensor.source_key}')
        if state is None:
            if sensor.factory is None:
                raise ComponentError(f'source is not running in DDS domain {sensor.transport.domain_id}: '
                                     f'{sensor.source_key}; start its provider first')
            directory = Path(tempfile.mkdtemp(prefix=f'rsim-{user_id()}-', dir=storage_root()))
            topic = '/rsim/frames/p' + uuid.uuid4().hex
            (directory / 'factory.pkl').write_bytes(cloudpickle.dumps(sensor.factory))
            (directory / 'config.json').write_text(json.dumps(dict(history=sensor._history.maxlen,
                topic=topic, transport=asdict(sensor.transport),
                sys_path=[str(Path(p).resolve()) for p in sys.path])), encoding='utf-8')
            state = dict(directory=str(directory), topic=topic, signature=signature,
                         provider_version=sensor.provider_version)
            state_path.write_text(json.dumps(state), encoding='utf-8')
            sensor._reader, sensor._writer = await launch(directory, path, lock_path, 'sensor')
        sensor.directory = Path(state['directory'])
        sensor._remote_sequence = 0
        sensor.pending.clear()
        sensor.subscription = sensor.children[0].subscribe(state['topic'], sensor.pending.append)
        sensor.task('receive', sensor.receive, hz=sensor.hz)
    finally:
        lock.close()
