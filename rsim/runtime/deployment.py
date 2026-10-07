"""Same-host placement bindings. Logical ports remain independent of DDS/mmap.

Only declared Signals and CommandSinks are transportable component interfaces;
ordinary Python methods/attributes are not remote procedure calls. Process
placements with the same name share one worker and one asyncio event loop.
"""
from __future__ import annotations

import asyncio
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import uuid

import cloudpickle

from rsim.core.component import Component, ComponentError
from rsim.transport.shared import SharedStore
from rsim.transport.descriptor import DescriptorTransport, TransportConfig, transport_config
from rsim.core.commands import CommandSink
from rsim.transport.commands import CommandChannel, CommandClient, CommandServer
from .port_binding import (LocalReference, SharedMemoryChannel, DDSChannel,
                           PortExporter as _Export, PortImporter as _Import)


@dataclass(frozen=True)
class LocalPlacement:
    """Keep this component in the calling event loop."""


@dataclass(frozen=True)
class ProcessPlacement:
    name: str
    transport: TransportConfig | str | None = None

    def __post_init__(self):
        if not self.name:
            raise ValueError("process placement name must be nonempty")
        object.__setattr__(self, "transport", transport_config(self.transport))


class _View(Component):
    def __init__(self, original, ports, *, supervisor=None, inputs=(), commands=None):
        dependencies = tuple(dict.fromkeys(item.producer for item in ports.values()))
        dependencies += tuple((commands or {}).values())
        if supervisor is not None:
            dependencies = (supervisor,) + dependencies
        super().__init__(*dependencies, inputs=inputs)
        for name, target in ports.items():
            self.expose(name, target)
        self.original_type = type(original).__name__
        self.supervisor = supervisor
        for name, client in (commands or {}).items():
            original_sink = original.sinks[name]
            sink = CommandSink(self, name, client.apply, fallback=original_sink.fallback, safe=client.safe,
                               max_ttl=original_sink.guard.max_ttl_ns / 1e9, hz=original_sink.hz)
            setattr(self, name, sink)

    async def call(self, name, *args, **kwargs):
        if name in self._services:
            return await super().call(name, *args, **kwargs)
        raise ComponentError("ordinary component services do not cross placement boundaries; use ports")


class _ProcessHost(Component):
    def __init__(self, directory):
        super().__init__()
        self.directory = Path(directory)
        self.process = self._lease = self._log = self.worker_pid = None

    async def open(self):
        read, self._lease = os.pipe()
        self._log = (self.directory / "worker.log").open("w")
        try:
            self.process = await asyncio.create_subprocess_exec(
                sys.executable, "-m", "rsim.runtime.supervisor", str(self.directory), str(read),
                pass_fds=(read,), start_new_session=True, stdout=self._log, stderr=self._log)
        finally:
            os.close(read)
        self.task("supervise", self.check, hz=50)

    async def check(self):
        status = self.directory / "status.json"
        if status.exists():
            state = json.loads(status.read_text())
            self.worker_pid = state.get("pid")
            if "error" in state:
                raise ComponentError(state["error"])
        if self.process.returncode is not None:
            log = self.directory / "worker.log"
            detail = log.read_text()[-8000:] if log.exists() else ""
            raise ComponentError(f"placement supervisor exited: {self.process.returncode}\n{detail}")

    async def close(self):
        if self._lease is not None:
            os.close(self._lease)
            self._lease = None
        try:
            if self.process is not None:
                try:
                    await asyncio.wait_for(self.process.wait(), 10)
                except TimeoutError:
                    self.process.terminate()
                    await asyncio.wait_for(self.process.wait(), 7)
        finally:
            if self._log is not None:
                self._log.close()
            shutil.rmtree(self.directory, ignore_errors=True)


class _DirectoryLease(Component):
    def __init__(self, directory):
        super().__init__()
        self.directory = directory
        self.process = self.lease = None

    async def open(self):
        read, self.lease = os.pipe()
        try:
            self.process = await asyncio.create_subprocess_exec(
                sys.executable, "-m", "rsim.runtime.store_guard", str(self.directory), str(read),
                pass_fds=(read,), start_new_session=True)
        finally:
            os.close(read)
        self.task("store-guard", self.check, hz=20)

    async def check(self):
        if self.process.returncode is not None:
            raise ComponentError("deployment store guard exited")

    async def close(self):
        if self.lease is not None:
            os.close(self.lease)
            self.lease = None
        if self.process is not None:
            await asyncio.wait_for(self.process.wait(), 9)


class BindingPlan:
    def __init__(self, runtime):
        self.runtime = runtime
        self.components = tuple(runtime._order)
        self.assignments, self.channels, self.views = {}, {}, {}
        self.directory = None
        self.allocator = None
        self.exports = []
        self.hosts = {}
        self.bindings = {}
        self.command_channels = {}

    def _placements(self):
        explicit = dict(self.runtime.placement)
        if any(component not in self.components and component not in self.runtime._aliases
               for component in explicit):
            raise ValueError("placement target is not reachable from Runtime roots")
        for component in self.components:
            requested = explicit.get(component, component.placement)
            for alias in self.runtime._aliases:
                if alias._canonical is component and alias in explicit:
                    if requested is not None and requested != explicit[alias]:
                        raise ValueError("conflicting placements for one shared component")
                    requested = explicit[alias]
            if requested is not None:
                if not isinstance(requested, (LocalPlacement, ProcessPlacement)):
                    raise TypeError("placement must be LocalPlacement or ProcessPlacement")
                self.assignments[component] = requested

        # Ownership resources follow an explicitly placed owner unless a child
        # has an explicit placement. Data inputs keep their own placement.
        declared = set(self.assignments)
        def inherit(component, placement):
            for child in component.dependencies:
                child = child._canonical or child
                chosen = self.assignments.get(child)
                if chosen is None:
                    self.assignments[child] = placement
                    inherit(child, placement)
                elif child.process_local:
                    continue
                elif child not in declared and chosen != placement:
                    raise ValueError("shared ownership resource needs an explicit placement")
        for component, placement in tuple(self.assignments.items()):
            inherit(component, placement)
        for component in self.components:
            self.assignments.setdefault(component, LocalPlacement())
        specs = {}
        for placement in self.assignments.values():
            if isinstance(placement, ProcessPlacement):
                if placement.name in specs and specs[placement.name] != placement:
                    raise ValueError("one process name must have one transport configuration")
                specs[placement.name] = placement
        domains = {spec.transport.domain_id for spec in specs.values()}
        if len(domains) > 1:
            raise ValueError("placements in a Runtime must use one DDS domain")
        return specs

    def prepare(self):
        specs = self._placements()
        if not specs:
            return
        # Command bindings are installed separately from sample channels.
        self.directory = Path(tempfile.mkdtemp(prefix=f"rsim-deploy-{os.getuid()}-", dir="/dev/shm"))
        directories = {name: self.directory / ("p" + str(i)) for i, name in enumerate(specs)}
        local_directory = self.directory / "local"
        for directory in (*directories.values(), local_directory):
            directory.mkdir(mode=0o700)
        groups = {component: (placement.name if isinstance(placement, ProcessPlacement) else None)
                  for component, placement in self.assignments.items()}
        self.groups = groups
        self.canonical_ports = {port: port._resolved() for component in self.components
                                for port in (*component.outputs.values(), *component.sinks.values())}
        self.port_ids = {port: uuid.uuid4().hex for port in self.canonical_ports.values()}
        # Executors and DDS contexts are process resources, not physical data
        # sources. A graph may need one local instance in each owning process.
        replicas = {component: set() for component in self.components if component.process_local}
        for resource in replicas:
            if resource.outputs or resource.sinks or resource.inputs:
                raise ValueError("process-local resources cannot own data or command ports")
            if resource in self.runtime.placement or resource.placement is not None:
                raise ValueError("place the owners of a process-local context, not the context itself")
        for owner in self.components:
            for resource in owner.dependencies:
                if resource in replicas:
                    replicas[resource].add(groups[owner])
        for root in self.runtime.roots:
            if root in replicas:
                replicas[root].add(None)
        for resource, locations in replicas.items():
            if None in locations:
                groups[resource] = None
        self.destinations = {}
        self.command_destinations = {}
        for port, consumers in self.runtime.demanded_ports.items():
            source_group = groups[port.producer]
            destinations = {None if consumer is None else groups[consumer] for consumer in consumers}
            destinations.discard(source_group)
            if not destinations:
                continue
            table = self.command_destinations if isinstance(port, CommandSink) else self.destinations
            table[port] = destinations
        for component in self.components:
            for output in component.outputs.values():
                self.bindings[output] = LocalReference(output)
        for source in self.destinations:
            group = groups[source.producer]
            directory = local_directory if group is None else directories[group]
            identifier = uuid.uuid4().hex
            self.channels[source] = DDSChannel(str(directory / "frames" / identifier),
                                               source.history_size, source.clock,
                                               "/rsim/channels/p" + identifier)
            self.bindings[source] = self.channels[source]
        for sink in self.command_destinations:
            identifier = uuid.uuid4().hex
            self.command_channels[sink] = CommandChannel(str(self.directory / "commands" / identifier),
                                                          "/rsim/commands/p" + identifier)
        payload = {"components": self.components, "aliases": {alias: alias._canonical
                    for alias in self.runtime._aliases}, "groups": groups, "channels": self.channels,
                    "command_channels": self.command_channels,
                    "destinations": self.destinations, "command_destinations": self.command_destinations,
                    "port_ids": self.port_ids,
                    "claims": dict(self.runtime._command_claims), "replicas": replicas}
        # Serialize before starting any resource or installing parent-side views.
        recipe = cloudpickle.dumps(payload)
        for name, spec in specs.items():
            directory = directories[name]
            (directory / "graph.pkl").write_bytes(recipe)
            (directory / "config.json").write_text(json.dumps({
                "worker_module": "rsim.runtime.placement_worker", "group": name,
                "transport": asdict(spec.transport),
                "sys_path": [str(Path(path).resolve()) for path in sys.path]}))
            self.hosts[name] = _ProcessHost(directory)
        config = next(iter(specs.values())).transport
        transport = DescriptorTransport(config)
        self.allocator = SharedStore(local_directory / "allocation", reuse=True)
        imports = {}
        for source, channel in self.channels.items():
            if groups[source.producer] is None:
                self.exports.append(_Export(source, channel, transport, self.allocator))
            elif None in self.destinations[source]:
                imports[source] = _Import(channel, transport)
                # Keep cursors local to the public Signal across Runtime reopen,
                # even though its physical import endpoint is recreated.
                cursors = [output._sequence for component in self.components
                           for output in component.outputs.values() if output._resolved() is source]
                imports[source].output._sequence = max(cursors, default=0)
        for component in self.components:
            group = groups[component]
            if group is None:
                for sink in component.sinks.values():
                    if sink not in self.command_channels:
                        continue
                    writer = payload["claims"].get(sink)
                    if writer is not None and groups[writer] == group:
                        writer = None
                    self.exports.append(CommandServer(sink, self.command_channels[sink], transport,
                                                       writer=writer))
                continue
            ports = {name: imports[output._resolved()].output for name, output in component.outputs.items()
                     if output._resolved() in imports}
            commands = {name: CommandClient(self.command_channels[sink], transport)
                        for name, sink in component.sinks.items()
                        if sink in self.command_destinations and None in self.command_destinations[sink]}
            view = _View(component, ports, supervisor=self.hosts[group],
                         commands=commands)
            view.command_targets = tuple(sink for sink in component.command_targets
                                         if groups[sink._resolved().producer] is None)
            self.views[component] = view
        for component, view in self.views.items():
            component._binding = view
        guard = _DirectoryLease(self.directory)
        for host in self.hosts.values():
            host.dependencies += (guard,)
        for exporter in self.exports:
            exporter.dependencies += (guard,)
        self.runtime.roots = (guard,) + self.runtime.roots + tuple(self.exports) + tuple(self.hosts.values())

    def release(self):
        for component, view in self.views.items():
            for name, output in view.outputs.items():
                component.outputs[name]._sequence = output._resolved()._sequence
            component._binding = None
        for exporter in self.exports:
            if isinstance(exporter, _Export):
                exporter.source.remove_prepare_hook(exporter)
        if self.allocator is not None:
            self.allocator.close()
        if self.directory is not None:
            shutil.rmtree(self.directory, ignore_errors=True)


def worker_graph(payload, group, directory, transport):
    """Bind a deserialized logical graph to this worker's local/imported ports."""
    components, groups, channels = payload["components"], payload["groups"], payload["channels"]
    def local(component):
        return groups[component] == group or group in payload["replicas"].get(component, ())
    allocator = SharedStore(directory / "allocation", reuse=True)
    imports, views, exports = {}, {}, []
    for component in components:
        component.placement = None
    for source, channel in channels.items():
        if groups[source.producer] == group:
            exports.append(_Export(source, channel, transport, allocator))
        elif group in payload["destinations"][source]:
            imports[source] = _Import(channel, transport)
    for component in components:
        if local(component):
            for sink in component.sinks.values():
                if sink not in payload["command_channels"]:
                    continue
                writer = payload["claims"].get(sink)
                if writer is not None and groups[writer] == group:
                    writer = None
                exports.append(CommandServer(sink, payload["command_channels"][sink], transport,
                                              writer=writer))
            continue
        ports = {name: imports[output._resolved()].output for name, output in component.outputs.items()
                 if output._resolved() in imports}
        commands = {name: CommandClient(payload["command_channels"][sink], transport)
                    for name, sink in component.sinks.items()
                    if group in payload["command_destinations"].get(sink, ())}
        views[component] = _View(component, ports, commands=commands)
    for alias, canonical in payload["aliases"].items():
        alias._canonical = None
        alias._binding = canonical
    for component, view in views.items():
        component._binding = view
    roots = [component for component in components if local(component)]
    return roots + exports, allocator
