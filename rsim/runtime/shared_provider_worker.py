"""Execute a trusted recipe only in the explicitly chosen provider environment."""
import asyncio
import json
import os
from pathlib import Path
import signal
import socket
import sys

import cloudpickle

from rsim.core import Component
from rsim.transport.descriptor import DescriptorTransport, TransportConfig
from .graph import Runtime
from .port_binding import PortBinding
from .registry import RegistryServer
from .routing import RoutedBindings
from .sharing import component_ports
from .deployment import ProcessPlacement


async def run(directory, listener):
    config = json.loads((directory / 'config.json').read_text())
    sys.path[:] = config['sys_path']
    os.environ['ROS_DOMAIN_ID'] = str(config['transport']['domain_id'])
    recipe = cloudpickle.loads((directory / 'recipe.pkl').read_bytes())
    component = recipe['component'] if recipe['component'] is not None else recipe['factory']()
    ports = {**component.outputs, **component.sinks}
    if set(ports) != set(config['manifest']['ports']):
        raise ValueError('provider recipe ports do not match manifest')
    for name, spec in component_ports(component).items():
        declared = config['manifest']['ports'][name]
        if any(declared[field] != getattr(spec, field) for field in ('direction', 'clock', 'max_ttl')):
            raise ValueError('provider disagrees with declared port interface: ' + name)
        declared['history_capacity'] = spec.history_capacity
    transport = DescriptorTransport(TransportConfig(**config['transport']))
    lifetime = Component(component, transport)
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, stopped.set)
    async with Runtime(lifetime, placement=recipe['placement'], _root_ports=False) as runtime:
        transport = transport._canonical or transport
        plan = runtime._binding_plan
        if plan and any(isinstance(spec, ProcessPlacement) and spec.transport.domain_id != transport.config.domain_id
                        for spec in plan.assignments.values()):
            raise ValueError('shared provider placements must use its advertised DDS domain')
        bindings = PortBinding(runtime, transport, directory / 'ports', config['manifest']['instance_id'],
                               allocator=plan.allocator if plan else None)
        if plan:
            for endpoint in plan.exports:
                bindings.adopt(endpoint)
        routes = RoutedBindings(bindings, plan)
        config['manifest']['pid'] = os.getpid()
        server = RegistryServer(config['manifest'], ports, routes)
        await server.start(sock=socket.socket(fileno=listener))
        stop_task, failure = asyncio.create_task(stopped.wait()), asyncio.create_task(runtime.wait())
        try:
            done, _ = await asyncio.wait((stop_task, failure), return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                await task
        finally:
            stop_task.cancel()
            failure.cancel()
            await asyncio.gather(stop_task, failure, return_exceptions=True)
            await server.close()
            await routes.close()


if __name__ == '__main__':
    asyncio.run(run(Path(sys.argv[1]), int(sys.argv[2])))
