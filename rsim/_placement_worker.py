"""Supervised process entrypoint for a bound Component graph."""
import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import traceback

import cloudpickle

from .core import Runtime
from .deployment import worker_graph
from .transport import DescriptorTransport, TransportConfig
from ._worker import write_status


async def run(directory):
    config = json.loads((directory / "config.json").read_text())
    sys.path[:] = config["sys_path"]
    os.environ["ROS_DOMAIN_ID"] = str(config["transport"]["domain_id"])
    os.environ["RSIM_TRANSPORT"] = config["transport"]["backend"]
    payload = cloudpickle.loads((directory / "graph.pkl").read_bytes())
    transport = DescriptorTransport(TransportConfig(**config["transport"]))
    roots, allocator = worker_graph(payload, config["group"], directory, transport)
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)
    try:
        async with Runtime(*roots) as runtime:
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
