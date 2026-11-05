"""Publish a component in the calling process, with no supervisor or worker."""
import asyncio
from copy import deepcopy
import os
from uuid import uuid4

from rsim.core import Component, ComponentError, ProviderDisconnected
from rsim.transport.descriptor import DescriptorTransport, transport_config
from rsim.transport.files import remove_tree
from .graph import Runtime
from .locks import locked
from .options import RuntimeOptions
from .paths import instance_directory, host_id
from .port_binding import PortBinding
from .registry import Connection, PROTOCOL, RegistryServer, registry_paths
from .sharing import component_ports, validate_component_ports


async def serve_component(component, *, key, ports=None, interface_version='ports-v1',
                          provider_version='1', transport=None, options=None,
                          stopped=None, on_ready=None):
    """Serve native RSim Shared Ports until cancellation or ``stopped`` is set.

    The caller owns process lifetime; clients never keep an exited foreground
    owner alive. DDS, array storage, command ownership and discovery use exactly
    the same implementation as SharedProvider. Existing owners are not replaced.
    ``on_ready(manifest)`` runs after contracts and the registry are ready.
    """
    if not isinstance(component, Component) or not isinstance(key, str) or not key:
        raise ValueError('a Component and a nonempty provider key are required')
    config = transport_config(transport)
    if config.backend == 'inprocess':
        raise ValueError('a shared component requires a cross-process DDS transport')
    options = (options or RuntimeOptions()).resolved()
    specs = deepcopy(component_ports(component) if ports is None else ports)
    validate_component_ports(component, specs)
    component._provider_port_contract = (specs, True)
    path, lock_path = registry_paths(key, config.domain_id, options)
    startup = owner = directory = None
    owns_path = False
    try:
        startup = await locked(lock_path, options.startup_timeout_s)
        try:
            connection = await Connection.connect(path)
        except ProviderDisconnected:
            pass
        else:
            await connection.close()
            raise ComponentError(f'provider already running: {key}')
        owner = await locked(path.with_suffix('.owner.lock'), options.startup_timeout_s)
        path.unlink(missing_ok=True)
        owns_path = True
        directory = instance_directory('rsim-foreground-', options)
        descriptor = DescriptorTransport(config)
        lifetime = Component(component, descriptor)
        manifest = dict(protocol_version=PROTOCOL, component_key=key,
            instance_id=uuid4().hex, interface_version=interface_version,
            provider_version=provider_version, host_id=host_id(options), pid=os.getpid(),
            domain_id=config.domain_id, ports={name: spec.manifest() for name, spec in specs.items()})
        async with Runtime(lifetime, options=options, _root_ports=False) as runtime:
            validate_component_ports(component, specs)
            descriptor = descriptor._canonical or descriptor
            bindings = PortBinding(runtime, descriptor, directory/'ports', manifest['instance_id'])
            server = RegistryServer(manifest, {**component.outputs, **component.sinks}, bindings)
            pending = []
            try:
                await server.start(path=path)
                startup.close()
                startup = None
                if on_ready is not None:
                    on_ready(dict(manifest))
                pending.append(asyncio.create_task(runtime.wait()))
                if stopped is not None:
                    pending.append(asyncio.create_task(stopped.wait()))
                done, _ = await asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED)
                for task in done:
                    await task
            finally:
                for task in pending:
                    task.cancel()
                await asyncio.gather(*pending, return_exceptions=True)
                await server.close()
                await bindings.close()
    finally:
        if owns_path:
            path.unlink(missing_ok=True)
        if directory is not None:
            remove_tree(directory, ignore_errors=True)
        if owner is not None:
            owner.close()
        if startup is not None:
            startup.close()
