"""Reusable port endpoints; producer activation belongs to Runtime/SharedProvider."""
import asyncio
from collections import deque
from dataclasses import dataclass, replace
import json
from pathlib import Path
import uuid

from rsim.core.component import Component, PrimaryComponent
from rsim.core.model import SampleId
from rsim.core.commands import CommandSink
from rsim.transport.shared import SharedStore, decode
from rsim.transport.commands import CommandChannel, CommandClient, CommandServer


def sample_id(descriptor):
    value = descriptor.get("sample_id")
    return SampleId(**value) if value is not None else None


async def start_endpoint(endpoint, runtime):
    """Attach only transport tasks to an already running producer graph."""
    endpoint._runtime, endpoint._closed = runtime, False
    endpoint._closing, endpoint._failure = False, None
    try:
        await endpoint.open()
        endpoint._tasks.add(asyncio.create_task(endpoint._watchdog()))
    except BaseException:
        await stop_endpoint(endpoint)
        raise
    return endpoint


async def stop_endpoint(endpoint):
    endpoint._closing = True
    for task in endpoint._tasks:
        task.cancel()
    await asyncio.gather(*endpoint._tasks, return_exceptions=True)
    endpoint._tasks.clear()
    try:
        await endpoint.close()
    finally:
        endpoint._closed, endpoint._runtime = True, None
        for output in endpoint.outputs.values():
            await output._notify()


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


class PortExporter(Component):
    def __init__(self, source, channel, transport, allocator):
        super().__init__(transport, inputs=(source,))
        self.source, self.channel = source, channel
        self.store = SharedStore(channel.directory, history=channel.history)
        self.allocator = allocator
        self.previous, self.latest = 0, None
        # Installed before any producer.open callback (including seed frames).
        self.source.add_prepare_hook(self, allocator.prepare)

    async def open(self):
        self.publisher = self.dependencies[0].publisher(self.channel.topic)
        self.task("export", self.export, hz=500)
        self.task("announce", self.announce, hz=20)

    async def export(self):
        frame = await self.source.get(after=self.previous)
        self.latest = json.dumps(self.store.put(replace(frame, data=self.allocator.prepare(frame.data))), allow_nan=False)
        self.previous = frame.sequence
        await self.announce()

    async def announce(self):
        if self.latest is not None:
            self.publisher.publish(self.latest)

    async def close(self):
        if hasattr(self, "publisher"):
            self.publisher.close()
        self.source.remove_prepare_hook(self)
        self.store.close()


class PortImporter(PrimaryComponent):
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
                metadata = decode(descriptor["metadata"], self.channel.directory) if "metadata" in descriptor else {}
            except FileNotFoundError:
                continue  # Bounded history already evicted this generation.
            self.previous = descriptor["sequence"]
            await self.output.publish(data, stamp_ns=descriptor["stamp_ns"], clock=descriptor["clock"],
                                      received_ns=descriptor["received_ns"],
                                      sample_id=sample_id(descriptor),
                                      metadata=metadata)

    async def close(self):
        if self.subscription is not None:
            self.subscription.close()
            self.subscription = None
        self.pending.clear()


class PortBinding:
    """One exporter per canonical port, shared by all subscriber leases.

    Allocation reuse is by live array identity (not by contents). A dynamic
    export materializes a retained Frame separately, leaving its history intact.
    """
    def __init__(self, runtime, transport, directory, instance_id, *, allocator=None):
        self.runtime, self.transport = runtime, transport
        self.directory, self.instance_id = Path(directory), instance_id
        self._owns_allocator = allocator is None
        self.allocator = allocator or SharedStore(self.directory / "allocation", reuse=True)
        self.records, self.generations = {}, {}

    def adopt(self, endpoint):
        """Reuse a statically planned exporter; its graph owns the lifetime."""
        if not isinstance(endpoint, (PortExporter, CommandServer)):
            return
        channel = endpoint.channel
        if isinstance(endpoint, PortExporter):
            port, clock, history = endpoint.source, channel.clock, channel.history
        else:
            port, clock, history = endpoint.sink, 'host:monotonic', 0
            endpoint.v2_instance = self.instance_id
            if endpoint.tokens is None:
                endpoint.tokens = set()
        self.records[port] = {"endpoint": endpoint, "tokens": {"runtime"}, "adopted": True,
            "descriptor": {"binding_id": channel.topic.rsplit('/', 1)[-1], "channel_generation": 1,
                "topic": channel.topic, "clock": clock, "instance_id": self.instance_id,
                "storage_descriptor": {"kind": "npy-mmap-v1", "directory": channel.directory},
                "effective_history": {"provider": history, "dds": 1, "replay": "latest"}}}
        self.generations[port] = 1

    async def acquire(self, port, token):
        port = port._resolved()
        record = self.records.get(port)
        if record is None:
            identifier = uuid.uuid4().hex
            upstream = getattr(port.producer, '_acquire_port', None)
            if upstream is not None:
                await upstream(port, identifier)
            generation = self.generations.get(port, 0) + 1
            self.generations[port] = generation
            directory = str(self.directory / identifier)
            topic = "/rsim/ports/p" + identifier
            if isinstance(port, CommandSink):
                channel = CommandChannel(directory, topic, self.instance_id)
                endpoint = CommandServer(port, channel, self.transport)
                clock, history = "host:monotonic", 0
            else:
                channel = DDSChannel(directory, port.history_size, port.clock, topic)
                endpoint = PortExporter(port, channel, self.transport, self.allocator)
                clock, history = port.clock, port.history_size
            try:
                await start_endpoint(endpoint, self.runtime)
            except BaseException:
                if upstream is not None:
                    await port.producer._release_port(port, identifier)
                raise
            record = {"endpoint": endpoint, "tokens": set(), "descriptor": {
                "binding_id": identifier, "channel_generation": generation, "topic": topic,
                "storage_descriptor": {"kind": "npy-mmap-v1", "directory": directory},
                "effective_history": {"provider": history, "dds": 1, "replay": "latest"},
                "clock": clock, "instance_id": self.instance_id}}
            record['upstream'] = identifier if upstream is not None else None
            self.records[port] = record
        record["tokens"].add(token)
        endpoint = record["endpoint"]
        if isinstance(endpoint, CommandServer):
            endpoint.tokens.add(token)
        return dict(record["descriptor"], session_token=token)

    async def release(self, port, token):
        port = port._resolved()
        record = self.records.get(port)
        if record is None:
            return
        endpoint = record["endpoint"]
        record["tokens"].discard(token)
        if isinstance(endpoint, CommandServer):
            await endpoint.release_token(token)
        if not record["tokens"]:
            try:
                await stop_endpoint(endpoint)
            finally:
                self.records.pop(port, None)
                if record.get('upstream') is not None:
                    await port.producer._release_port(port, record['upstream'])

    def errors(self):
        return {port: str(record["endpoint"]._failure) for port, record in self.records.items()
                if record["endpoint"]._failure is not None}

    async def close(self):
        try:
            for port, record in tuple(self.records.items()):
                if not record.get("adopted"):
                    await stop_endpoint(record["endpoint"])
                    if record.get('upstream') is not None:
                        await port.producer._release_port(port, record['upstream'])
        finally:
            self.records.clear()
            if self._owns_allocator:
                self.allocator.close()
