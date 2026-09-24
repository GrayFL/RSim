"""Volatile same-host DDS command requests; validation stays at the provider."""
import asyncio
from collections import OrderedDict, deque
from dataclasses import dataclass
import json
from pathlib import Path
import time
import uuid

from .commands import CommandEnvelope, CommandRejected
from .core import Component, ComponentError
from .model import Frame
from .shared import SharedStore, decode


@dataclass(frozen=True)
class CommandChannel:
    directory: str
    topic: str


class CommandClient(Component):
    def __init__(self, channel, transport):
        super().__init__(transport)
        self.channel = channel
        self.pending, self.responses = {}, deque(maxlen=32)
        self.store = None
        self.last = None
        self.publisher = self.subscription = None
        self.ready = asyncio.Event()

    async def open(self):
        transport = self.dependencies[0]
        self.publisher = transport.publisher(self.channel.topic + "/request", durable=False, depth=16)
        self.subscription = transport.subscribe(self.channel.topic + "/reply", self.responses.append,
                                                 durable=False, depth=16)
        self.store = SharedStore(Path(self.channel.directory) / uuid.uuid4().hex, history=32)
        self.task("command-replies", self.receive, hz=500)
        self.task("discover-provider", self.discover, hz=20)

    async def discover(self):
        if self.ready.is_set():
            return
        try:
            await self.request({"op": "ping"}, timeout=.5)
        except TimeoutError:
            return
        self.ready.set()

    async def receive(self):
        while self.responses:
            response = json.loads(self.responses.popleft())
            future = self.pending.get(response["id"])
            if future is not None and not future.done():
                if "error" in response:
                    future.set_exception(CommandRejected(response["error"], reason=response.get("reason", "invalid")))
                else:
                    future.set_result(response["result"])

    async def request(self, packet, *, timeout=.6):
        if self._closed or self._failure is not None:
            raise ComponentError("command channel is closed")
        identifier = uuid.uuid4().hex
        packet = json.dumps(dict(packet, id=identifier), allow_nan=False)
        future = asyncio.get_running_loop().create_future()
        self.pending[identifier] = future
        try:
            async with asyncio.timeout(timeout):
                while True:
                    self.publisher.publish(packet)
                    try:
                        return await asyncio.wait_for(asyncio.shield(future), .04)
                    except TimeoutError:
                        continue
        finally:
            self.pending.pop(identifier, None)
            future.cancel()

    async def apply(self, envelope):
        previous = self.last
        self.last = envelope
        try:
            async with asyncio.timeout(max(0, (envelope.deadline_ns - time.monotonic_ns()) / 1e9)):
                await self.ready.wait()
                descriptor = self.store.put(Frame(envelope, time.monotonic_ns(), "host:monotonic"))
                return await self.request({"op": "set", "data": descriptor["data"]})
        except TimeoutError as error:
            # Discovery or lost acknowledgements cannot renew a command. Its
            # provider deadline has now passed; a fresh command can try again.
            raise CommandRejected("command deadline passed before acknowledgement", reason="expired") from error
        except CommandRejected:
            self.last = previous
            raise

    async def safe(self, _):
        if self.last is not None and self.last.deadline_ns > time.monotonic_ns():
            try:
                await self.request({"op": "stop", "controller_id": self.last.controller_id,
                                    "controller_epoch": self.last.controller_epoch},
                                   timeout=max(.01, (self.last.deadline_ns - time.monotonic_ns()) / 1e9))
            except (TimeoutError, CommandRejected):
                # Either TTL has elapsed, or this session never/ no longer owns
                # the provider. Never stop a different controller's lease.
                pass

    async def close(self):
        for future in self.pending.values():
            if not future.done():
                future.set_exception(ComponentError("command channel closed"))
        for endpoint in (self.publisher, self.subscription):
            if endpoint is not None:
                endpoint.close()
        if self.store is not None:
            self.store.close()


class CommandServer(Component):
    def __init__(self, sink, channel, transport, *, writer=None):
        super().__init__(sink.producer, transport)
        self.sink, self.channel = sink, channel
        self.writer = writer
        self.pending = deque(maxlen=32)
        self.replies = OrderedDict()
        self.publisher = self.subscription = None

    async def open(self):
        transport = self.dependencies[1]
        self.publisher = transport.publisher(self.channel.topic + "/reply", durable=False, depth=16)
        self.subscription = transport.subscribe(self.channel.topic + "/request", self.pending.append,
                                                 durable=False, depth=16)
        self.task("command-requests", self.dispatch, hz=500)

    async def dispatch(self):
        if not self.pending:
            return
        request = json.loads(self.pending.popleft())
        identifier = request["id"]
        if identifier in self.replies:
            self.publisher.publish(self.replies[identifier])
            return
        try:
            if request["op"] == "ping":
                result = {"ready": True}
            elif request["op"] == "set":
                envelope = decode(request["data"], self.channel.directory)
                if not isinstance(envelope, CommandEnvelope):
                    raise CommandRejected("expected command envelope")
                writer = self.writer
                if writer is not None:
                    writer = writer._binding or writer._canonical or writer
                result = await self.sink.set(envelope, _writer=writer)
            elif request["op"] == "stop":
                identity = request["controller_id"], request["controller_epoch"]
                if self.sink.guard.owner != identity:
                    raise CommandRejected("stop request does not own this command session")
                await self.sink._safe()
                result = {"stopped": True}
            else:
                raise CommandRejected("unknown command operation")
            response = {"id": identifier, "result": result}
        except (CommandRejected, ValueError, FileNotFoundError) as error:
            response = {"id": identifier, "error": str(error), "reason": getattr(error, "reason", "invalid")}
        encoded = json.dumps(response, allow_nan=False)
        self.replies[identifier] = encoded
        while len(self.replies) > 128:
            self.replies.popitem(last=False)
        self.publisher.publish(encoded)

    async def close(self):
        for endpoint in (self.publisher, self.subscription):
            if endpoint is not None:
                endpoint.close()
        self.pending.clear()
