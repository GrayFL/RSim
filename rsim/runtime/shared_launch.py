"""Shared startup transaction; recipes stay in the launcher's environment."""
import asyncio
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import threading
import uuid

import cloudpickle

from rsim.core.errors import ProviderDisconnected, ComponentError
from rsim.transport.files import remove_tree, open_shared
from rsim.platforms import get_platform
from .locks import locked
from .paths import instance_directory, worker_log, host_id, registry_directory
from .registry import registry_paths, Connection, PROTOCOL


def configuration(options, transport, **fields):
    return dict(options=options.to_dict(), transport=asdict(transport),
                sys_path=[str(Path(p).resolve()) for p in sys.path], **fields)


async def launch(directory, path, lock_path, mode, options, log_path=None):
    log_path = worker_log(directory, options, log_path)
    config_path = directory / 'config.json'
    config = json.loads(config_path.read_text(encoding='utf-8'))
    config['log_path'] = str(log_path)
    config_path.write_text(json.dumps(config), encoding='utf-8')
    # A refused endpoint can still belong to a supervisor finishing teardown.
    # Its separate ownership lock is released only after the worker tree/files.
    owner = await locked(path.with_suffix('.owner.lock'), options.startup_timeout_s)
    owner.close()
    # The caller still holds the startup lock, excluding a competing launcher.
    path.unlink(missing_ok=True)
    process = None
    writer = None
    try:
        with open_shared(log_path, write=True, create=not log_path.exists()) as log:
            log.seek(0, 2)
            process = subprocess.Popen([sys.executable, '-m', 'rsim.runtime.shared_supervisor',
                                        str(directory), str(path), str(lock_path), mode],
                                       stdout=log, stderr=log, **get_platform().process.supervisor_options())
        threading.Thread(target=process.wait, daemon=True, name='rsim-provider-reaper').start()
        async with asyncio.timeout(options.startup_timeout_s):
            while True:
                if process.poll() is not None:
                    detail = log_path.read_text(encoding='utf-8', errors='replace')[-8000:]
                    raise ComponentError(f'shared supervisor exited; log: {log_path}\n{detail}')
                try:
                    reader, writer = await get_platform().ipc.open_connection(path)
                    break
                except (OSError, asyncio.IncompleteReadError):
                    await asyncio.sleep(.02)
            if mode == 'sensor':
                await reader.readexactly(1)  # Ack only after the worker is ready.
            else:
                await Connection(reader, writer).request('describe', timeout=options.startup_timeout_s)
            return reader, writer
    except BaseException:
        if writer is not None:
            writer.close()
        if process is not None:
            # Stop independently of the caller's startup lock on failed startup.
            (directory / 'supervisor.stop').touch()
            timeout = options.shutdown.grace_s + options.shutdown.kill_wait_s + 2
            try:
                await asyncio.to_thread(process.wait, timeout=timeout)
            except subprocess.TimeoutExpired:
                process.kill()
                await asyncio.to_thread(process.wait, timeout=options.shutdown.kill_wait_s)
        remove_tree(directory, ignore_errors=True)
        raise


async def open_provider(provider):
    options = provider.options
    path, lock_path = registry_paths(provider.component_key, provider.transport.domain_id, options)
    lock = await locked(lock_path, options.startup_timeout_s)
    try:
        try:
            provider.connection = await Connection.connect(path)
        except ProviderDisconnected:
            if provider.connection is not None:
                await provider.connection.close()
                provider.connection = None
        else:
            # An established instance failing readiness is a failed connection,
            # not permission to replace it while its supervisor is still stopping.
            await provider.connection.request('describe', timeout=options.startup_timeout_s)
            return
        directory = instance_directory('rsim-provider-', options)
        provider.directory = directory
        try:
            provider.log_path = worker_log(directory, options, provider._explicit_log_path)
            recipe = dict(component=provider.component, factory=provider.factory, placement=provider.provider_placement)
            (directory / 'recipe.pkl').write_bytes(cloudpickle.dumps(recipe))
            manifest = dict(protocol_version=PROTOCOL, component_key=provider.component_key,
                            instance_id=uuid.uuid4().hex, interface_version=provider.interface_version,
                            provider_version=provider.provider_version, host_id=host_id(options),
                            domain_id=provider.transport.domain_id,
                            ports={name: spec.manifest() for name, spec in provider.port_specs.items()})
            config = configuration(options, provider.transport, manifest=manifest)
            (directory / 'config.json').write_text(json.dumps(config), encoding='utf-8')
            provider.connection = Connection(*await launch(directory, path, lock_path, 'ports', options, provider.log_path))
        except BaseException:
            remove_tree(directory, ignore_errors=True)
            raise
    finally:
        lock.close()


async def open_sensor(sensor):
    options = sensor.options
    identity = f'{host_id(options)}:{sensor.transport.domain_id}:{sensor.source_key}'
    base = registry_directory(options) / hashlib.sha256(identity.encode()).hexdigest()[:32]
    path, lock_path, state_path = [base.with_suffix(suffix) for suffix in ('.sock', '.lock', '.json')]
    lock = await locked(lock_path, options.startup_timeout_s)
    signature = dict(version=sensor.version, history=sensor._history.maxlen)
    try:
        state = None
        if state_path.exists():
            try:
                sensor._reader, sensor._writer = await get_platform().ipc.open_connection(path)
                await asyncio.wait_for(sensor._reader.readexactly(1), options.startup_timeout_s)
            except (OSError, asyncio.IncompleteReadError):
                if sensor._writer is not None:
                    sensor._writer.close()
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
            directory = instance_directory('rsim-', options)
            try:
                topic = '/rsim/frames/p' + uuid.uuid4().hex
                (directory / 'factory.pkl').write_bytes(cloudpickle.dumps(sensor.factory))
                config = configuration(options, sensor.transport, history=sensor._history.maxlen, topic=topic)
                (directory / 'config.json').write_text(json.dumps(config), encoding='utf-8')
                state = dict(directory=str(directory), log_path=str(worker_log(directory, options)), topic=topic, signature=signature,
                             provider_version=sensor.provider_version)
                temporary = state_path.with_suffix('.tmp')
                temporary.write_text(json.dumps(state), encoding='utf-8')
                temporary.replace(state_path)
                sensor._reader, sensor._writer = await launch(directory, path, lock_path, 'sensor', options, state['log_path'])
            except BaseException:
                remove_tree(directory, ignore_errors=True)
                raise
        sensor.directory = Path(state['directory'])
        sensor.log_path = Path(state['log_path'])
        sensor._remote_sequence = 0
        sensor.pending.clear()
        sensor.queue_drops = sensor.evicted_descriptors = 0
        sensor.subscription = sensor.children[0].subscribe(state['topic'], sensor._enqueue)
        sensor.task('receive', sensor.receive, hz=sensor.hz)
    finally:
        lock.close()
