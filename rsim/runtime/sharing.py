"""Connection-only multi-port views and explicit same-environment providers."""
from copy import deepcopy
from dataclasses import dataclass, fields, is_dataclass
import hashlib
import json
from pathlib import Path
import math
from typing import Protocol

from rsim.core import Component, CommandSink, ComponentError, PortNotBound, ProviderDisconnected
from rsim.transport.descriptor import DescriptorTransport, transport_config
from rsim.transport.commands import CommandChannel, CommandClient
from .port_binding import DDSChannel, PortImporter, start_endpoint, stop_endpoint
from .registry import Connection, PROTOCOL, host_id, registry_paths


class ComponentFactory(Protocol):
    """A lightweight recipe; build returns a Component without opening resources."""

    def build(self) -> Component: ...


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
        if (self.direction not in ('signal', 'sink') or not isinstance(self.history_capacity, int)
                or self.history_capacity < 1):
            raise ValueError('invalid port declaration')
        if self.direction == 'sink' and (self.max_ttl is None or not math.isfinite(self.max_ttl) or self.max_ttl <= 0):
            raise ValueError('sink declarations require max_ttl')

    def manifest(self):
        return {name: getattr(self, name) for name in
                ('schema', 'direction', 'clock', 'history_capacity', 'max_ttl')}


def component_ports(component):
    def declared(port):
        # Follow exposed aliases, but not placement proxies: dormant workers
        # have no requested proxy ports yet. Actual workers validate separately.
        seen = set()
        while getattr(port, '_target', None) is not None:
            if id(port) in seen:
                raise ComponentError('cyclic exposed port declaration')
            seen.add(id(port))
            port = port._target
        return port
    outputs = {name: declared(port) for name, port in component.outputs.items()}
    sinks = {name: declared(port) for name, port in component.sinks.items()}
    ports = {name: PortSpec(clock=port.clock, history_capacity=port.history_size)
             for name, port in outputs.items()}
    ports.update({name: PortSpec(direction='sink', max_ttl=port.guard.max_ttl_ns / 1e9,
                                 fallback=port.fallback, clock='host:monotonic')
                  for name, port in sinks.items()})
    return ports


def _same_value(expected, actual):
    if type(expected) is not type(actual):
        return False
    if is_dataclass(expected):
        return all(_same_value(getattr(expected, f.name), getattr(actual, f.name))
                   for f in fields(expected))
    if isinstance(expected, dict):
        return expected.keys() == actual.keys() and all(
            _same_value(value, actual[key]) for key, value in expected.items())
    if isinstance(expected, (list, tuple)):
        return len(expected) == len(actual) and all(map(lambda pair: _same_value(*pair), zip(expected, actual)))
    comparison = expected == actual
    return bool(comparison.all() if hasattr(comparison, 'all') else comparison)


def validate_component_ports(component, specs, *, check_fallback=True):
    """Validate a factory's complete declaration before open and before ready.

    This checks structural/codec and provider-side fallback configuration, not
    payload semantics. The latter require device-specific conformance tests.
    Fallback remains local trusted recipe data; the wire manifest is unchanged.
    """
    if not isinstance(component, Component):
        raise TypeError(f'provider factory must return Component, got {type(component).__name__}')
    actual = component_ports(component)
    if set(actual) != set(specs):
        raise ValueError(f'provider port set mismatch: expected {sorted(specs)}, actual {sorted(actual)}')
    for name, expected in specs.items():
        expected = expected if isinstance(expected, PortSpec) else PortSpec(**expected)
        observed = actual[name]
        for field in ('schema', 'direction', 'clock', 'history_capacity', 'max_ttl'):
            if getattr(expected, field) != getattr(observed, field):
                raise ValueError(f'provider port {name!r} {field}: expected '
                                 f'{getattr(expected, field)!r}, actual {getattr(observed, field)!r}')
        if check_fallback and expected.direction == 'sink' and not _same_value(expected.fallback, observed.fallback):
            raise ValueError(f'provider port {name!r} fallback: expected '
                             f'{expected.fallback!r}, actual {observed.fallback!r}')
    return actual


async def describe_shared(key, *, transport=None, options=None):
    """Explicitly discover metadata before constructing a dynamic logical view."""
    config = transport_config(transport)
    from .locks import locked
    from .options import RuntimeOptions
    options = (options or RuntimeOptions()).resolved()
    path, lock_path = registry_paths(key, config.domain_id, options)
    lock = await locked(lock_path, options.startup_timeout_s)
    connection = None
    try:
        connection = await Connection.connect(path)
        return await connection.request('describe')
    finally:
        if connection is not None:
            await connection.close()
        lock.close()


class SharedComponent(Component):
    """Declare logical ports now; bind only Runtime demand when opened.

    This class never accepts a factory, starts a provider, imports driver code,
    or changes interpreters. Closing it releases this session's subscriptions.
    """
    def __init__(self, *, key, ports, interface_version='ports-v1', transport=None, options=None):
        self.options = options
        self.transport = transport_config(transport)
        super().__init__(DescriptorTransport(self.transport))
        self.component_key, self.interface_version = key, interface_version
        self.port_specs = {name: deepcopy(spec) if isinstance(spec, PortSpec) else PortSpec(**deepcopy(spec))
                           for name, spec in ports.items()}
        if not key:
            raise ValueError('a shared view needs a nonempty key')
        contract = json.dumps({name: spec.manifest() for name, spec in self.port_specs.items()}, sort_keys=True)
        digest = hashlib.sha256(contract.encode()).hexdigest()
        # Explicit discovery roots are part of view identity, even when key and
        # port contracts match. Inherited options resolve once at Runtime open.
        scope = repr(options.paths.resolved()) if options is not None else 'runtime-options'
        self.key = (f'shared-view:{type(self).__name__}:{self.transport}:{key}:{interface_version}:{digest}:{scope}')
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
        from .locks import locked
        self.options = (self.options or self._runtime.options).resolved()
        if self.connection is None:
            path, lock_path = registry_paths(self.component_key, self.transport.domain_id, self.options)
            lock = await locked(lock_path, self.options.startup_timeout_s)
            try:
                self.connection = await Connection.connect(path)
                manifest = await self.connection.request('describe')
            finally:
                lock.close()
        else:
            manifest = await self.connection.request('describe')
        if (manifest.get('protocol_version') != PROTOCOL or manifest['component_key'] != self.component_key
                or manifest['interface_version'] != self.interface_version
                or manifest['host_id'] != host_id(self.options) or manifest['domain_id'] != self.transport.domain_id):
            raise ComponentError('incompatible shared provider manifest')
        expected_version = getattr(self, 'provider_version', None)
        if isinstance(self, SharedProvider) and manifest['provider_version'] != expected_version:
            raise ComponentError('conflicting shared provider configuration')
        for name, spec in self.port_specs.items():
            actual = manifest['ports'].get(name, {})
            for field in ('schema', 'direction', 'clock', 'max_ttl'):
                if actual.get(field) != getattr(spec, field):
                    raise ComponentError(f'incompatible schema for port {name!r}: {field} expected '
                                         f'{getattr(spec, field)!r}, actual {actual.get(field)!r}')
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
        # Liveness checks need identity and port failures, not a recursive
        # shared-file census. Scanning every retained sample at 10 Hz blocks
        # the provider event loop and can starve command acknowledgements.
        manifest = await self.connection.request('describe', diagnostics=False)
        if manifest['instance_id'] != self.manifest['instance_id']:
            raise ProviderDisconnected('provider instance changed')
        self.manifest = manifest
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
    An explicit log_path preserves worker diagnostics after lease cleanup.
    """
    def __init__(self, component=None, *, factory=None, ports=None, key,
                 interface_version='ports-v1', provider_version='1', transport=None, placement=None,
                 log_path=None, options=None):
        if (component is None) == (factory is None):
            raise ValueError('supply a component or a local provider factory')
        if component is not None:
            ports = component_ports(component) if ports is None else ports
            validate_component_ports(component, ports)
        if ports is None:
            raise ValueError('a lazy provider factory requires known ports')
        super().__init__(key=key, ports=ports, interface_version=interface_version, transport=transport, options=options)
        self.component, self.factory = component, factory
        self.provider_version, self.provider_placement = provider_version, placement
        # Explicit launchers must not be merged with connection-only views.
        self.key = None
        self.directory = None
        self.log_path = Path(log_path).expanduser().resolve() if log_path is not None else None
        self._explicit_log_path = self.log_path

    async def open(self):
        from .shared_launch import open_provider
        if self.transport.backend == 'inprocess':
            raise ValueError('SharedProvider requires a real cross-process DDS backend')
        self.options = (self.options or self._runtime.options).resolved()
        await open_provider(self)
        await super().open()


async def serve_shared(provider):
    """Hold the launcher source lease without requesting any data ports."""
    from .graph import Runtime
    if not isinstance(provider, SharedProvider):
        raise TypeError('serve_shared requires an explicit SharedProvider')
    async with Runtime(provider, _root_ports=False) as runtime:
        await runtime.wait()
