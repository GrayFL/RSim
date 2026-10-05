"""Internal child entrypoint; public callers use ProcessSensor(factory)."""
import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import traceback

import cloudpickle

from .graph import Runtime
from rsim.core.component import Component
from rsim.core.signal import as_signal
from rsim.transport.descriptor import DescriptorTransport, TransportConfig
from rsim.transport.shared import SharedStore, current_store


class Export(Component):
    def __init__(self, source, directory, config):
        self.source = as_signal(source)
        super().__init__(DescriptorTransport(TransportConfig(**config["transport"])), inputs=(self.source,))
        self.store = SharedStore(directory / "frames", history=config["history"])
        self._prepared_source = self.source._resolved()
        self._prepared_source._prepare = self.store.prepare
        self.topic = config["topic"]
        self.previous = 0
        self.latest = None

    async def open(self):
        self.publisher = self.children[0].publisher(self.topic)
        self.task("export", self.export, hz=200)
        self.task("announce", self.announce, hz=20)

    async def export(self):
        frame = await self.source.get(after=self.previous)
        self.latest = self.store.put(frame)
        self.previous = frame.sequence
        await self.announce()

    async def announce(self):
        if self.latest is not None:
            self.publisher.publish(json.dumps(self.latest, allow_nan=False))

    async def close(self):
        if hasattr(self, "publisher"):
            self.publisher.close()
        self._prepared_source._prepare = None
        self.store.close()


async def run(directory):
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, stopped.set)
    loop.add_signal_handler(signal.SIGINT, stopped.set)
    config = json.loads((directory / "config.json").read_text())
    # Descendant factories and ROS ingress inherit the same domain/backend.
    os.environ["ROS_DOMAIN_ID"] = str(config["transport"]["domain_id"])
    os.environ["RSIM_TRANSPORT"] = config["transport"]["backend"]
    sys.path[:] = config["sys_path"]
    factory = cloudpickle.loads((directory / "factory.pkl").read_bytes())
    source = factory()
    as_signal(source)
    exported = Export(source, directory, config)
    current_store.set(exported.store)
    async with Runtime(exported):
        write_status(directory, {"pid": os.getpid(), "ready": True})
        while not stopped.is_set():
            if exported._failure is not None:
                raise RuntimeError("worker sensor failed") from exported._failure
            try:
                await asyncio.wait_for(stopped.wait(), 0.05)
            except TimeoutError:
                pass


def write_status(directory, state):
    temporary = directory / "status.tmp"
    temporary.write_text(json.dumps(state))
    temporary.replace(directory / "status.json")


def main():
    directory = Path(sys.argv[1])
    try:
        asyncio.run(run(directory))
    except BaseException:
        write_status(directory, {"pid": os.getpid(), "error": traceback.format_exc()})
        raise


if __name__ == "__main__":
    main()
