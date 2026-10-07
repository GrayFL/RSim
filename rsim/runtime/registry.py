"""Versioned, finite same-host lease protocol. No executable recipes on the wire."""
import asyncio
from collections import OrderedDict
import hashlib
import json
import os
from pathlib import Path
import uuid

from rsim.core.errors import ComponentError, ProviderDisconnected, PortNotBound
from .host import registry_directory

PROTOCOL = 2
LIMIT = 65536


def host_id():
    # The boot identity also scopes CLOCK_MONOTONIC and stale registry entries.
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def registry_paths(key, domain_id):
    digest = hashlib.sha256(f'{host_id()}:{os.getuid()}:{domain_id}:{key}'.encode()).hexdigest()[:32]
    base = registry_directory() / ('v2-' + digest)
    return base.with_suffix('.sock'), base.with_suffix('.lock')


class Connection:
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer
        self.lock = asyncio.Lock()

    @classmethod
    async def connect(cls, path):
        try:
            return cls(*await asyncio.open_unix_connection(str(path), limit=LIMIT))
        except (FileNotFoundError, ConnectionRefusedError) as error:
            raise ProviderDisconnected('provider is not running; start it in its own environment') from error

    async def request(self, op, *, request_id=None, **fields):
        identifier = request_id or uuid.uuid4().hex
        packet = json.dumps(dict(protocol=PROTOCOL, id=identifier, op=op, **fields), allow_nan=False).encode() + b'\n'
        if len(packet) > LIMIT:
            raise ValueError('registry request too large')
        async with self.lock:
            try:
                async with asyncio.timeout(10):
                    self.writer.write(packet)
                    await self.writer.drain()
                    line = await self.reader.readline()
                if not line:
                    raise ProviderDisconnected('provider instance disconnected; explicitly reopen to rebind')
                response = json.loads(line)
                if response.get('id') != identifier or response.get('protocol') != PROTOCOL:
                    raise ProviderDisconnected('invalid registry response')
                if 'error' in response:
                    raise ComponentError(response['error'])
                return response['result']
            except (ConnectionError, OSError, TimeoutError) as error:
                raise ProviderDisconnected('provider connection ended') from error

    async def close(self):
        self.writer.close()
        try:
            await self.writer.wait_closed()
        except ConnectionError:
            pass


class RegistryServer:
    """Each connection owns one source lease and its exact port subscriptions."""
    def __init__(self, manifest, ports, bindings, *, on_empty=None):
        self.manifest, self.ports, self.bindings = manifest, ports, bindings
        self.on_empty = on_empty
        self.lock = asyncio.Lock()
        self.sessions = set()
        self.tasks = set()
        self.server = None

    async def start(self, *, path=None, sock=None):
        self.server = await asyncio.start_unix_server(self.accept, path=path, sock=sock, limit=LIMIT)
        return self

    def describe(self):
        errors = self.bindings.errors()
        reported = {}
        for name, port in self.ports.items():
            error = errors.get(port)
            if error is None:
                try:
                    error = errors.get(port._resolved())
                except PortNotBound:
                    pass
            if error is not None:
                reported[name] = error
        return dict(self.manifest, port_errors=reported)

    async def accept(self, reader, writer):
        task = asyncio.current_task()
        self.tasks.add(task)
        self.sessions.add(writer)
        subscriptions, replies = {}, OrderedDict()
        try:
            while line := await reader.readline():
                request = json.loads(line)
                identifier = request.get('id')
                if not isinstance(identifier, str) or not 0 < len(identifier) <= 128:
                    break
                async with self.lock:
                    if identifier in replies:
                        encoded = replies[identifier]
                    else:
                        try:
                            if request.get('protocol') != PROTOCOL:
                                raise ValueError('unsupported registry protocol')
                            operation = request.get('op')
                            if operation == 'describe':
                                refresh = getattr(self.bindings, 'refresh', None)
                                if refresh is not None:
                                    await refresh()
                                result = self.describe()
                            elif operation == 'subscribe':
                                if request.get('instance_id') != self.manifest['instance_id']:
                                    raise ValueError('provider instance changed; describe and explicitly rebind')
                                name = request['port']
                                if name not in self.ports:
                                    raise ValueError('unknown port: ' + str(name))
                                # Idempotent even when the response cache has evicted an older request.
                                if identifier not in subscriptions:
                                    token = uuid.uuid4().hex
                                    descriptor = await self.bindings.acquire(self.ports[name], token)
                                    subscriptions[identifier] = (self.ports[name], token, descriptor)
                                result = dict(subscriptions[identifier][2], subscription_id=identifier)
                            elif operation == 'unsubscribe':
                                subscription = subscriptions.pop(request['subscription_id'], None)
                                if subscription:
                                    await self.bindings.release(*subscription[:2])
                                result = {'released': True}
                            else:
                                raise ValueError('unsupported registry operation')
                            response = dict(protocol=PROTOCOL, id=identifier, result=result)
                        except (ValueError, KeyError, ComponentError) as error:
                            response = dict(protocol=PROTOCOL, id=identifier, error=str(error))
                        encoded = json.dumps(response, allow_nan=False).encode() + b'\n'
                        replies[identifier] = encoded
                        while len(replies) > 128:
                            replies.popitem(last=False)
                writer.write(encoded)
                await writer.drain()
        except (ConnectionError, ValueError):
            pass
        finally:
            async with self.lock:
                for port, token, _ in subscriptions.values():
                    await self.bindings.release(port, token)
                self.sessions.discard(writer)
                if not self.sessions and self.on_empty is not None:
                    self.on_empty()
            writer.close()
            self.tasks.discard(task)

    async def close(self):
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()
        for writer in tuple(self.sessions):
            writer.close()
        await asyncio.gather(*tuple(self.tasks), return_exceptions=True)
