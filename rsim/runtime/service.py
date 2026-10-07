"""Small JSON DDS requests and telemetry, usable without shared files or ROS.

Only JSON values cross this boundary; no pickle or executable methods travel.
Applications own lease, replay and physical command validation.
"""

import asyncio
from collections import deque, OrderedDict
import json
import re
import time
import uuid

from rsim.core.component import Component, ComponentError
from rsim.transport.descriptor import DescriptorTransport


class RemoteError(ComponentError):
    pass


def topic_prefix(name):
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,63}", name):
        raise ValueError("service name must be a short DDS identifier")
    return "/rsim/services/" + name


class RPCServer(Component):
    def __init__(self, name, *dependencies, transport=None):
        self.bus = DescriptorTransport(transport)
        super().__init__(self.bus, *dependencies)
        self.prefix = topic_prefix(name)
        self.pending = deque(maxlen=128)
        self.replies = OrderedDict()

    async def open(self):
        self.bus = self.dependencies[0]
        self.generation = uuid.uuid4().hex
        self.pending.clear()
        self.replies.clear()
        self.publisher = self.bus.publisher(
            self.prefix + "/reply", durable=False, depth=128
        )
        self.telemetry = self.bus.publisher(
            self.prefix + "/state", durable=False, depth=8
        )
        self.subscription = self.bus.subscribe(
            self.prefix + "/request", self.pending.append, durable=False, depth=128
        )
        self.task("requests", self.dispatch, hz=500)

    async def dispatch(self):
        if not self.pending:
            return
        raw = self.pending.popleft()
        try:
            if len(raw) > 65536:
                return
            request = json.loads(raw)
            identifier = request["id"]
            if not isinstance(identifier, str) or len(identifier) > 128:
                return
        except (ValueError, KeyError, TypeError):
            return
        if identifier in self.replies:
            self.publisher.publish(self.replies[identifier])
            return
        try:
            if request.get("op") == "hello":
                result = {
                    "generation": self.generation,
                    "server_ns": time.monotonic_ns(),
                }
            else:
                if request.get("generation") != self.generation:
                    raise RemoteError("provider restarted; reconnect explicitly")
                result = await self.handle(request)
            response = {"id": identifier, "result": result}
        except (ValueError, TypeError, KeyError, ComponentError, TimeoutError) as error:
            response = {"id": identifier, "error": str(error)}
        encoded = json.dumps(response, allow_nan=False)
        self.replies[identifier] = encoded
        while len(self.replies) > 512:
            self.replies.popitem(last=False)
        self.publisher.publish(encoded)

    async def handle(self, request):
        raise RemoteError("unsupported request")

    async def close(self):
        for name in ("subscription", "publisher", "telemetry"):
            endpoint = getattr(self, name, None)
            if endpoint is not None:
                endpoint.close()


class RPCClient(Component):
    def __init__(self, name, *, transport=None):
        self.bus = DescriptorTransport(transport)
        super().__init__(self.bus)
        self.prefix = topic_prefix(name)
        self.responses = deque(maxlen=128)
        self.states = deque(maxlen=8)
        self.pending = {}

    async def open(self):
        self.bus = self.dependencies[0]
        self.pending.clear()
        self.responses.clear()
        self.states.clear()
        self.publisher = self.bus.publisher(
            self.prefix + "/request", durable=False, depth=128
        )
        self.subscription = self.bus.subscribe(
            self.prefix + "/reply", self.responses.append, durable=False, depth=128
        )
        self.state_subscription = self.bus.subscribe(
            self.prefix + "/state", self.states.append, durable=False, depth=8
        )
        self.task("responses", self.receive, hz=500)

    async def receive(self):
        while self.responses:
            try:
                packet = json.loads(self.responses.popleft())
                future = self.pending.get(packet["id"])
                if future is not None and not future.done():
                    if "error" in packet:
                        future.set_exception(RemoteError(packet["error"]))
                    else:
                        future.set_result(packet["result"])
            except (ValueError, KeyError, TypeError):
                continue

    async def request(self, packet, *, timeout=0.5):
        identifier = uuid.uuid4().hex
        encoded = json.dumps(dict(packet, id=identifier), allow_nan=False)
        if len(encoded) > 65536:
            raise ValueError("request too large")
        future = asyncio.get_running_loop().create_future()
        self.pending[identifier] = future
        try:
            async with asyncio.timeout(timeout):
                while True:
                    # Runtime stops our metered receiver before close(). The
                    # dependency bus remains live while a final release awaits.
                    await self.receive()
                    if future.done():
                        return future.result()
                    self.publisher.publish(encoded)
                    try:
                        return await asyncio.wait_for(asyncio.shield(future), 0.05)
                    except TimeoutError:
                        continue
        finally:
            self.pending.pop(identifier, None)
            future.cancel()

    async def close(self):
        for future in self.pending.values():
            if not future.done():
                future.set_exception(ComponentError("service client closed"))
        for name in ("subscription", "state_subscription", "publisher"):
            endpoint = getattr(self, name, None)
            if endpoint is not None:
                endpoint.close()
