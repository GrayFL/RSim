"""Bounded descriptor bus for one process/event loop; no ROS or native DDS.

This implements the same endpoint protocol used by RPCServer/RPCClient. It is
explicitly local: domain IDs isolate buses, not connect separate processes.
"""
import asyncio
from collections import deque
from weakref import WeakKeyDictionary


_LOOPS = WeakKeyDictionary()


class Endpoint:
    def __init__(self, owner, topic, callback, *, durable, depth):
        if not isinstance(depth, int) or depth < 1:
            raise ValueError("endpoint depth must be positive")
        self.owner, self.topic, self.callback = owner, topic, callback
        self.durable, self.closed = durable, False
        self.messages = deque(maxlen=depth)

    def publish(self, text):
        if self.closed or self.callback is not None:
            raise RuntimeError("endpoint is not an open publisher")
        if not isinstance(text, str):
            raise TypeError("descriptors must be strings")
        if self.durable:
            self.messages.append(text)
        for peer in tuple(self.owner.bus):
            if peer.callback is not None and peer.topic == self.topic:
                peer.messages.append(text)

    def close(self):
        if not self.closed:
            self.closed = True
            self.owner.bus.remove(self)
            self.owner.endpoints.remove(self)
            self.messages.clear()


class InProcessTransport:
    def __init__(self, domain_id):
        self.domain_id = domain_id
        self.endpoints = []

    def open(self):
        domains = _LOOPS.setdefault(asyncio.get_running_loop(), {})
        self.bus = domains.setdefault(self.domain_id, [])

    def _endpoint(self, topic, callback=None, *, durable=True, depth=16):
        endpoint = Endpoint(self, topic, callback, durable=durable, depth=depth)
        if callback is not None and durable:
            for writer in self.bus:
                if writer.topic == topic and writer.callback is None and writer.durable:
                    endpoint.messages.extend(writer.messages)
        self.endpoints.append(endpoint)
        self.bus.append(endpoint)
        return endpoint

    def publisher(self, topic, **options):
        return self._endpoint(topic, **options)

    def subscribe(self, topic, callback, **options):
        return self._endpoint(topic, callback, **options)

    def poll(self):
        for endpoint in tuple(self.endpoints):
            if endpoint.callback is not None:
                for _ in range(min(32, len(endpoint.messages))):
                    if endpoint.closed or not endpoint.messages:
                        break
                    endpoint.callback(endpoint.messages.popleft())

    def close(self):
        for endpoint in tuple(self.endpoints):
            endpoint.close()
