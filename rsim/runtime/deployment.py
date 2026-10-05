"""Same-host placement bindings. Logical ports remain independent of DDS/mmap.

Only declared Signals and CommandSinks are transportable component interfaces;
ordinary Python methods/attributes are not remote procedure calls. Process
placements with the same name share one worker and one asyncio event loop.
"""
from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import uuid

import cloudpickle

from rsim.core.component import Component, ComponentError, PrimaryComponent
from rsim.transport.shared import SharedStore, decode
from rsim.transport.descriptor import DescriptorTransport, TransportConfig, transport_config
from rsim.core.commands import CommandSink
from rsim.transport.commands import CommandChannel, CommandClient, CommandServer


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


@dataclass(frozen=True)
class LocalReference:
    signal: object


@dataclass(frozen=True)
class SharedMemoryChannel:
    directory: str
    history: int
    clock: str | None


@dataclass(frozen=True)
class DDSChannel(SharedMemoryChannel):
    topic: str


class _Export(Component):
    def __init__(self, source, channel, transport, allocator):
        super().__init__(transport, inputs=(source,))
        self.source, self.channel = source, channel
        self.store = SharedStore(channel.directory, history=channel.history)
        self.allocator = allocator
        self.previous, self.latest = 0, None
        # Installed before any producer.open callback (including seed frames).
        self.source._prepare = allocator.prepare

    async def open(self):
        self.publisher = self.dependencies[0].publisher(self.channel.topic)
        self.task("export", self.export, hz=500)
        self.task("announce", self.announce, hz=20)

    async def export(self):
        frame = await self.source.get(after=self.previous)
        self.latest = json.dumps(self.store.put(frame), allow_nan=False)
        self.previous = frame.sequence
        await self.announce()

    async def announce(self):
        if self.latest is not None:
            self.publisher.publish(self.latest)

    async def close(self):
        if hasattr(self, "publisher"):
            self.publisher.close()
        self.source._prepare = None
        self.store.close()


class _Import(PrimaryComponent):
    def __init__(self, channel, transport):
        super().__init__(transport, history=channel.history, clock=channel.clock)
        self.channel = channel
        self.pending = deque(maxlen=32)
        self.previous = 0
        self.subscription = None

    async def open(self):
        self.pending.clear()
        self.previous = 0
        self.subscription = self.dependencies[0].subscribe(self.channel.topic, self.pending.append)
        self.task("import", self.receive, hz=500)

    async def receive(self):
        while self.pending:
            descriptor = json.loads(self.pending.popleft())
            if descriptor["sequence"] <= self.previous:
                continue
            try:
                data = decode(descriptor["data"], self.channel.directory)
            except FileNotFoundError:
                continue  # Bounded history already evicted this generation.
            self.previous = descriptor["sequence"]
            await self.output.publish(data, stamp_ns=descriptor["stamp_ns"], clock=descriptor["clock"],
                                      received_ns=descriptor["received_ns"])

    async def close(self):
        if self.subscription is not None:
            self.subscription.close()
            self.subscription = None
        self.pending.clear()


class _View(Component):
    def __init__(self, original, ports, *, supervisor=None, inputs=(), commands=None):
        dependencies = tuple(dict.fromkeys(item.producer for item in ports.values()))
        dependencies += tuple((commands or {}).values())
        if supervisor is not None:
            dependencies = (supervisor,) + dependencies
        super().__init__(*dependencies, inputs=inputs)
        for name, target in ports.items():
            output = self.signal(name, history=target.history_size, clock=target.clock)
            output._target = target
            setattr(self, name, output)
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
        required = set()
        exposed = set()
        for root in self.runtime.roots:
            owner = root if isinstance(root, Component) else root.producer
            owner = owner._canonical or owner
            exposed.add(owner)
        for component in self.components:
            if groups[component] is not None and (component in exposed or component in self.runtime.placement
                                                   or component.placement is not None):
                required.update(output._resolved() for output in component.outputs.values())
            for signal in component.inputs:
                source = signal._resolved()
                if groups[source.producer] != groups[component]:
                    required.update(output._resolved() for output in signal.producer.outputs.values())
            for dependency in component.dependencies:
                if groups[dependency] != groups[component]:
                    required.update(signal._resolved() for signal in dependency.outputs.values())
            for output in component.outputs.values():
                self.bindings[output] = LocalReference(output)
        for source in required:
            group = groups[source.producer]
            directory = local_directory if group is None else directories[group]
            identifier = uuid.uuid4().hex
            self.channels[source] = DDSChannel(str(directory / "frames" / identifier),
                                               source.history_size, source.clock,
                                               "/rsim/channels/p" + identifier)
            self.bindings[source] = self.channels[source]
        for component in self.components:
            for sink in component.sinks.values():
                identifier = uuid.uuid4().hex
                self.command_channels[sink] = CommandChannel(str(self.directory / "commands" / identifier),
                                                              "/rsim/commands/p" + identifier)
        payload = {"components": self.components, "aliases": {alias: alias._canonical
                    for alias in self.runtime._aliases}, "groups": groups, "channels": self.channels,
                    "command_channels": self.command_channels,
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
            else:
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
                    writer = payload["claims"].get(sink)
                    if writer is not None and groups[writer] == group:
                        writer = None
                    self.exports.append(CommandServer(sink, self.command_channels[sink], transport,
                                                       writer=writer))
                continue
            ports = {name: imports[output._resolved()].output for name, output in component.outputs.items()
                     if output._resolved() in imports}
            commands = {name: CommandClient(self.command_channels[sink], transport)
                        for name, sink in component.sinks.items()}
            view = _View(component, ports, supervisor=self.hosts[group],
                         inputs=component.inputs, commands=commands)
            view.command_targets = component.command_targets
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
                exporter.source._prepare = None
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
        else:
            imports[source] = _Import(channel, transport)
    for component in components:
        if local(component):
            for sink in component.sinks.values():
                writer = payload["claims"].get(sink)
                if writer is not None and groups[writer] == group:
                    writer = None
                exports.append(CommandServer(sink, payload["command_channels"][sink], transport,
                                              writer=writer))
            continue
        ports = {name: imports[output._resolved()].output for name, output in component.outputs.items()
                 if output._resolved() in imports}
        commands = {name: CommandClient(payload["command_channels"][sink], transport)
                    for name, sink in component.sinks.items()}
        views[component] = _View(component, ports, commands=commands)
    for alias, canonical in payload["aliases"].items():
        alias._canonical = None
        alias._binding = canonical
    for component, view in views.items():
        component._binding = view
    roots = [component for component in components if local(component)]
    return roots + exports, allocator
