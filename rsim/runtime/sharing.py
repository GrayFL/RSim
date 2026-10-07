"""Connection-only multi-port views and explicit same-environment providers."""
import asyncio
from dataclasses import asdict, dataclass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import uuid

from rsim.core import Component, CommandSink, ComponentError, PortNotBound, ProviderDisconnected
from rsim.transport.descriptor import DescriptorTransport, transport_config
from rsim.transport.commands import CommandChannel, CommandClient
from .port_binding import DDSChannel, PortImporter, start_endpoint, stop_endpoint
from .registry import Connection, PROTOCOL, host_id, registry_paths


@dataclass(frozen=True)
class PortSpec:
    """Known wire contract. Schema names refer to the sealed shared codec."""
    schema: str = 'rsim.value.v1'
    direction: str = 'signal'
    clock: str | None = None
    history_capacity: int = 32
    max_ttl: float | None = None
    fallback: object = None

    def __post_init__(self):
        if self.direction not in ('signal', 'sink') or self.history_capacity < 1:
            raise ValueError('invalid port declaration')
        if self.direction == 'sink' and (self.max_ttl is None or self.max_ttl <= 0):
            raise ValueError('sink declarations require max_ttl')

    def manifest(self):
        return {name: getattr(self, name) for name in
                ('schema', 'direction', 'clock', 'history_capacity', 'max_ttl')}


def component_ports(component):
    ports = {name: PortSpec(clock=port.clock, history_capacity=port.history_size)
             for name, port in component.outputs.items()}
    ports.update({name: PortSpec(direction='sink', max_ttl=port.guard.max_ttl_ns / 1e9,
                                 fallback=port.fallback, clock='host:monotonic')
                  for name, port in component.sinks.items()})
    return ports


async def describe_shared(key, *, transport=None):
    """Explicitly discover metadata before constructing a dynamic logical view."""
    config = transport_config(transport)
    connection = await Connection.connect(registry_paths(key, config.domain_id)[0])
    try:
        return await connection.request('describe')
    finally:
        await connection.close()


class SharedComponent(Component):
    """Declare logical ports now; bind only Runtime demand when opened.

    This class never accepts a factory, starts a provider, imports driver code,
    or changes interpreters. Closing it releases this session's subscriptions.
    """
    def __init__(self, *, key, ports, interface_version='ports-v1', transport=None):
        self.transport = transport_config(transport)
        super().__init__(DescriptorTransport(self.transport))
        self.component_key, self.interface_version = key, interface_version
        self.port_specs = {name: spec if isinstance(spec, PortSpec) else PortSpec(**spec)
                           for name, spec in ports.items()}
        if not key:
            raise ValueError('a shared view needs a nonempty key')
        contract = json.dumps({name: spec.manifest() for name, spec in self.port_specs.items()}, sort_keys=True)
        digest = hashlib.sha256(contract.encode()).hexdigest()
        self.key = (f'shared-view:{type(self).__name__}:{self.transport}:{key}:{interface_version}:{digest}')
        self.connection = None
        self.manifest = None
        self.bindings, self.endpoints, self._port_users = {}, {}, {}
        for name, spec in self.port_specs.items():
            if hasattr(self, name):
                raise ValueError(f'port name conflicts with Component API: {name}')
            if spec.direction == 'signal':
                port = self.signal(name, history=spec.history_capacity, clock=spec.clock)
            else:
                async def apply(envelope, name=name):
                    endpoint = self.endpoints.get(name)
                    if endpoint is None:
                        raise PortNotBound(name)
                    return await endpoint.apply(envelope)
                async def safe(value, name=name):
                    if name in self.endpoints:
                        await self.endpoints[name].safe(value)
                port = CommandSink(self, name, apply, fallback=spec.fallback, safe=safe, max_ttl=spec.max_ttl)
            port._bound = False
            setattr(self, name, port)

    async def open(self):
        if self.connection is None:
            self.connection = await Connection.connect(registry_paths(self.component_key, self.transport.domain_id)[0])
        manifest = await self.connection.request('describe')
        if (manifest.get('protocol_version') != PROTOCOL or manifest['component_key'] != self.component_key
                or manifest['interface_version'] != self.interface_version
                or manifest['host_id'] != host_id() or manifest['domain_id'] != self.transport.domain_id):
            raise ComponentError('incompatible shared provider manifest')
        expected_version = getattr(self, 'provider_version', None)
        if isinstance(self, SharedProvider) and manifest['provider_version'] != expected_version:
            raise ComponentError('conflicting shared provider configuration')
        for name, spec in self.port_specs.items():
            actual = manifest['ports'].get(name, {})
            if any(actual.get(field) != getattr(spec, field) for field in ('schema', 'direction', 'clock', 'max_ttl')):
                raise ComponentError(f'incompatible schema for port {name}')
        self.manifest = manifest
        for port in (*self.outputs.values(), *self.sinks.values()):
            port._bound = False
        demanded = [port for port in self._runtime.demanded_ports if port.producer is self]
        for port in demanded:
            await self._acquire_port(port, 'runtime')
        self.task('provider-lease', self.check, hz=10)

    async def _acquire_port(self, port, token):
        """Explicit binding demand from a downstream provider, never from get()."""
        name, spec = port.name, self.port_specs[port.name]
        if name in self.endpoints:
            self._port_users[name].add(token)
            return
        descriptor = await self.connection.request('subscribe', port=name, instance_id=self.manifest['instance_id'])
        self.bindings[name] = descriptor
        directory = descriptor['storage_descriptor']['directory']
        if spec.direction == 'signal':
            channel = DDSChannel(directory, spec.history_capacity, spec.clock, descriptor['topic'])
            endpoint = PortImporter(channel, self.dependencies[0])
            # Import directly into the declared local cursor/history.
            endpoint.output = endpoint.primary = port
            endpoint.outputs = {}
            port._history.clear()
            port._port_error = None
        else:
            channel = CommandChannel(directory, descriptor['topic'], descriptor['instance_id'],
                                     descriptor['session_token'])
            endpoint = CommandClient(channel, self.dependencies[0])
        self.endpoints[name] = endpoint
        self._port_users[name] = {token}
        port._bound = True
        try:
            await start_endpoint(endpoint, self._runtime)
        except BaseException:
            await self._release_port(port, token)
            raise

    async def _release_port(self, port, token):
        name = port.name
        users = self._port_users.get(name)
        if users is None:
            return
        users.discard(token)
        if users:
            return
        endpoint = self.endpoints.pop(name)
        descriptor = self.bindings.pop(name)
        self._port_users.pop(name)
        port._bound = False
        await stop_endpoint(endpoint)
        if name in self.outputs:
            port._history.clear()
            await port._notify()
        try:
            await self.connection.request('unsubscribe', subscription_id=descriptor['subscription_id'])
        except ProviderDisconnected:
            pass

    async def check(self):
        if self.connection.reader.at_eof():
            raise ProviderDisconnected('provider instance ended; explicitly reopen this view')
        manifest = await self.connection.request('describe')
        if manifest['instance_id'] != self.manifest['instance_id']:
            raise ProviderDisconnected('provider instance changed')
        for name, endpoint in tuple(self.endpoints.items()):
            error = manifest.get('port_errors', {}).get(name) or endpoint._failure
            if error is not None and name in self.outputs:
                self.outputs[name]._port_error = ComponentError(str(error))
                await self.outputs[name]._notify()

    async def close(self):
        try:
            for endpoint in self.endpoints.values():
                await stop_endpoint(endpoint)
        finally:
            self.endpoints.clear()
            self._port_users.clear()
            if self.connection is not None:
                await self.connection.close()
                self.connection = None
            self.bindings.clear()
            for port in (*self.outputs.values(), *self.sinks.values()):
                port._bound = False


class SharedProvider(SharedComponent):
    """Explicit launcher. Only this environment serializes/executes the recipe.

    The supervised worker owns the graph; launcher and clients own independent
    socket leases. A launcher may exit while connected clients continue using it.
    """
    def __init__(self, component=None, *, factory=None, ports=None, key,
                 interface_version='ports-v1', provider_version='1', transport=None, placement=None):
        if (component is None) == (factory is None):
            raise ValueError('supply a component or a local provider factory')
        if component is not None:
            ports = component_ports(component) if ports is None else ports
        if ports is None:
            raise ValueError('a lazy provider factory requires known ports')
        super().__init__(key=key, ports=ports, interface_version=interface_version, transport=transport)
        self.component, self.factory = component, factory
        self.provider_version, self.provider_placement = provider_version, placement
        # Explicit launchers must not be merged with connection-only views.
        self.key = None
        self.directory = None

    async def open(self):
        path, lock_path = registry_paths(self.component_key, self.transport.domain_id)
        lock = lock_path.open('a+')
        acquired = False
        try:
            # Holding the lock for the worker lifetime excludes competing
            # launchers even while the provider graph is still starting.
            async with asyncio.timeout(30):
                while True:
                    try:
                        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        acquired = True
                        break
                    except BlockingIOError:
                        try:
                            self.connection = await Connection.connect(path)
                            break
                        except ProviderDisconnected:
                            await asyncio.sleep(.02)
            if acquired:
                await self._launch(path, lock)
        finally:
            # Do not LOCK_UN: the inherited open file description owns the lock.
            lock.close()
        await super().open()

    async def _launch(self, path, lock):
        import cloudpickle
        import shutil
        directory = Path(tempfile.mkdtemp(prefix=f'rsim-provider-{os.getuid()}-', dir='/dev/shm'))
        self.directory = directory
        listener = socket.socket(socket.AF_UNIX)
        initial_server, initial_client = socket.socketpair()
        try:
            path.unlink(missing_ok=True)
            listener.bind(str(path))
            listener.listen(128)
            recipe = dict(component=self.component, factory=self.factory, placement=self.provider_placement)
            (directory / 'recipe.pkl').write_bytes(cloudpickle.dumps(recipe))
            manifest = dict(protocol_version=PROTOCOL, component_key=self.component_key,
                            instance_id=uuid.uuid4().hex, interface_version=self.interface_version,
                            provider_version=self.provider_version, host_id=host_id(),
                            domain_id=self.transport.domain_id,
                            ports={name: spec.manifest() for name, spec in self.port_specs.items()})
            (directory / 'config.json').write_text(json.dumps(dict(manifest=manifest,
                transport=asdict(self.transport), sys_path=[str(Path(p).resolve()) for p in sys.path])))
            with (directory / 'worker.log').open('w') as log:
                process = subprocess.Popen([sys.executable, '-m', 'rsim.runtime.provider_supervisor',
                    str(directory), str(listener.fileno()), str(initial_server.fileno()),
                    str(lock.fileno()), str(path)], pass_fds=(listener.fileno(), initial_server.fileno(), lock.fileno()),
                    start_new_session=True, stdout=log, stderr=log)
            threading.Thread(target=process.wait, daemon=True, name='rsim-provider-reaper').start()
            self.connection = Connection(*await asyncio.open_unix_connection(sock=initial_client))
        except BaseException:
            initial_client.close()
            shutil.rmtree(directory, ignore_errors=True)
            raise
        finally:
            listener.close()
            initial_server.close()


async def serve_shared(provider):
    """Hold the launcher source lease without requesting any data ports."""
    from .graph import Runtime
    if not isinstance(provider, SharedProvider):
        raise TypeError('serve_shared requires an explicit SharedProvider')
    async with Runtime(provider, _root_ports=False) as runtime:
        await runtime.wait()
