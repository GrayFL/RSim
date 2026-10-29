"""Supervised process entrypoint for a bound Component graph."""
import asyncio
import json
import os
from pathlib import Path
from rsim.platforms import get_platform
import sys
import traceback
import uuid

import cloudpickle

from .graph import Runtime
from .options import RuntimeOptions
from .deployment import worker_graph
from rsim.transport.descriptor import DescriptorTransport, TransportConfig
from .worker import write_status
from .port_binding import PortBinding
from .registry import RegistryServer
from .sharing import validate_component_ports


async def run(directory):
    config = json.loads((directory / "config.json").read_text())
    sys.path[:] = config["sys_path"]
    os.environ["ROS_DOMAIN_ID"] = str(config["transport"]["domain_id"])
    os.environ["RSIM_TRANSPORT"] = config["transport"]["backend"]
    payload = cloudpickle.loads((directory / "graph.pkl").read_bytes())
    transport = DescriptorTransport(TransportConfig(**config["transport"]))
    roots, allocator = worker_graph(payload, config["group"], directory, transport)
    contracts = [(component, component._provider_port_contract)
                 for component, group in payload['groups'].items()
                 if group == config['group'] and hasattr(component, '_provider_port_contract')]
    stopped = asyncio.Event()
    get_platform().process.install_shutdown(stopped, directory)
    try:
        for component, (specs, check_fallback) in contracts:
            validate_component_ports(component, specs, check_fallback=check_fallback)
        async with Runtime(*roots, options=RuntimeOptions.from_dict(config.get('options')), _root_ports=False) as runtime:
            for component, (specs, check_fallback) in contracts:
                validate_component_ports(component, specs, check_fallback=check_fallback)
            # Dynamic shared clients bind here, at the computation location.
            local_ports = {identifier: port for port, identifier in payload['port_ids'].items()
                           if payload['groups'][port.producer] == config['group']}
            # A dormant placement still needs a descriptor context when its
            # first external subscriber arrives.
            from .port_binding import start_endpoint, stop_endpoint
            own_transport = transport._runtime is None and transport._canonical is None
            if own_transport:
                await start_endpoint(transport, runtime)
            actual_transport = transport._canonical or transport
            bindings = PortBinding(runtime, actual_transport, directory / 'external', uuid.uuid4().hex,
                                   allocator=allocator)
            for endpoint in roots:
                bindings.adopt(endpoint)
            server = RegistryServer({'instance_id': bindings.instance_id}, local_ports, bindings)
            await server.start(path=str(directory / 'ports.sock'))
            write_status(directory, {"pid": os.getpid(), "ready": True})
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
                await bindings.close()
                if own_transport:
                    await stop_endpoint(transport)
    finally:
        allocator.close()


def main():
    directory = Path(sys.argv[1])
    try:
        asyncio.run(run(directory))
    except BaseException:
        write_status(directory, {"pid": os.getpid(), "error": traceback.format_exc()})
        raise


if __name__ == "__main__":
    main()
