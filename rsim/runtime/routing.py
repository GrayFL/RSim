"""Route provider subscriptions to the actual placement, without a payload relay."""
import asyncio
from rsim.core import ProviderDisconnected
from .registry import Connection


class RoutedBindings:
    def __init__(self, local, plan):
        self.local, self.plan = local, plan if plan and plan.directory else None
        self.connections, self.remote, self.remote_errors = {}, {}, {}

    def route(self, port):
        if self.plan is None or not self.plan.directory:
            return port._resolved(), None
        source = self.plan.canonical_ports[port]
        return source, self.plan.groups[source.producer]

    async def acquire(self, port, token):
        source, group = self.route(port)
        if group is None:
            return await self.local.acquire(source, token)
        if group not in self.connections:
            host = self.plan.hosts[group]
            async with asyncio.timeout(20):
                while host.worker_pid is None:
                    if host._failure:
                        raise ProviderDisconnected(str(host._failure))
                    await asyncio.sleep(.02)
            connection = await Connection.connect(host.directory / 'ports.sock')
            manifest = await connection.request('describe')
            self.connections[group] = (connection, manifest)
        connection, manifest = self.connections[group]
        descriptor = await connection.request('subscribe', request_id=token,
            instance_id=manifest['instance_id'], port=self.plan.port_ids[source])
        self.remote[token] = (group, descriptor['subscription_id'])
        return descriptor

    async def release(self, port, token):
        source, group = self.route(port)
        if group is None:
            return await self.local.release(source, token)
        subscription = self.remote.pop(token, None)
        if subscription is not None:
            try:
                await self.connections[group][0].request('unsubscribe', subscription_id=subscription[1])
            except ProviderDisconnected:
                pass

    async def refresh(self):
        for group, (connection, _) in self.connections.items():
            try:
                state = await connection.request('describe')
                errors = state.get('port_errors', {})
                for port, identifier in self.plan.port_ids.items():
                    if identifier in errors:
                        self.remote_errors[port] = errors[identifier]
            except ProviderDisconnected as error:
                for port in self.plan.port_ids:
                    if self.plan.groups[port.producer] == group:
                        self.remote_errors[port] = str(error)

    def errors(self):
        errors = {**self.local.errors(), **self.remote_errors}
        if self.plan:
            return {port: errors[source] for port, source in self.plan.canonical_ports.items() if source in errors}
        return errors

    async def close(self):
        for connection, _ in self.connections.values():
            await connection.close()
        await self.local.close()
