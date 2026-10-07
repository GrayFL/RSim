"""Composition helpers shared by CLI and Notebook applications."""

import asyncio
from pathlib import Path

from rsim.components.vehicle import VehicleParameters


def vehicle_parameters(path):
    import yaml

    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {"vehicle"}:
        raise ValueError("keyboard configuration requires a vehicle mapping")
    return VehicleParameters(**data["vehicle"])


def connection_arguments(parser):
    parser.add_argument("--name", default="chassis")
    parser.add_argument("--domain", type=int, default=0)


def transport(args):
    from rsim.transport.descriptor import TransportConfig

    return TransportConfig(backend="cyclonedds", domain_id=args.domain)


async def until_closed(component, event):
    tasks = [asyncio.create_task(component.wait()), asyncio.create_task(event.wait())]
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for task in done:
            task.result()
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
