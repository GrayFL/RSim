"""Internal child entrypoint; public callers use ProcessSensor(factory)."""
import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import traceback

import cloudpickle

from .core import Runtime, Sensor
from .process import descriptor_qos
from .ros import RosContext
from .shared import SharedStore, current_store


class Export(Sensor):
    def __init__(self, source, directory, config):
        super().__init__(source, RosContext(), history=1)
        self.store = SharedStore(directory / "frames", history=config["history"])
        self.topic = config["topic"]
        self.previous = 0
        self.latest = None

    async def open(self):
        from std_msgs.msg import String
        self.publisher = self.children[1].node.create_publisher(String, self.topic, descriptor_qos())
        self.task("export", self.export, hz=200)
        self.task("announce", self.announce, hz=20)

    async def export(self):
        frame = await self.children[0].get(after=self.previous)
        self.latest = self.store.put(frame)
        self.previous = frame.sequence
        await self.announce()

    async def announce(self):
        from std_msgs.msg import String
        if self.latest is not None:
            self.publisher.publish(String(data=json.dumps(self.latest, allow_nan=False)))

    async def close(self):
        if hasattr(self, "publisher"):
            self.children[1].node.destroy_publisher(self.publisher)
        self.store.close()


async def run(directory):
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, stopped.set)
    loop.add_signal_handler(signal.SIGINT, stopped.set)
    config = json.loads((directory / "config.json").read_text())
    sys.path[:] = config["sys_path"]
    factory = cloudpickle.loads((directory / "factory.pkl").read_bytes())
    source = factory()
    if not isinstance(source, Sensor):
        raise TypeError("ProcessSensor factory must return a Sensor")
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
