"""Run an arbitrary sensor factory in a supervised child asyncio loop."""
from __future__ import annotations

import asyncio
from collections import deque
import json
import os
from pathlib import Path
import sys
import tempfile
import uuid

import cloudpickle

from .core import Sensor, SensorError
from .ros import RosContext
from .shared import decode


def descriptor_qos():
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    return QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ProcessSensor(Sensor):
    def __init__(self, factory, *, history=16, hz=200):
        super().__init__(RosContext(), history=history)
        self.factory, self.hz = factory, hz
        self.process = None
        self.directory = None
        self._lease = None
        self.subscription = None
        self.pending = deque(maxlen=32)
        self._remote_sequence = 0
        self.worker_pid = None
        self._log = None

    async def open(self):
        from std_msgs.msg import String
        self.pending.clear()
        self._remote_sequence = 0
        self.directory = Path(tempfile.mkdtemp(prefix=f"rsim-{os.getuid()}-", dir="/dev/shm"))
        topic = "/rsim/frames/" + "p" + uuid.uuid4().hex
        (self.directory / "factory.pkl").write_bytes(cloudpickle.dumps(self.factory))
        (self.directory / "config.json").write_text(json.dumps({
            "history": self._history.maxlen, "topic": topic,
            "sys_path": [str(Path(p).resolve()) for p in sys.path]}))
        self.subscription = self.children[0].node.create_subscription(
            String, topic, lambda msg: self.pending.append(msg.data), descriptor_qos())
        lease_read, self._lease = os.pipe()
        self._log = (self.directory / "worker.log").open("w")
        try:
            self.process = await asyncio.create_subprocess_exec(
                sys.executable, "-m", "rsim._supervisor", str(self.directory), str(lease_read),
                pass_fds=(lease_read,), start_new_session=True,
                stdout=self._log, stderr=self._log)
        finally:
            os.close(lease_read)
        self.task("receive", self.receive, hz=self.hz)

    async def receive(self):
        status = self.directory / "status.json"
        if status.exists():
            state = json.loads(status.read_text())
            self.worker_pid = state.get("pid")
            if "error" in state:
                raise SensorError(state["error"])
        if self.process is not None and self.process.returncode is not None:
            raise SensorError(f"sensor supervisor exited: {self.process.returncode}")
        if not self.pending:
            return
        descriptor = json.loads(self.pending.popleft())
        if descriptor["sequence"] <= self._remote_sequence:
            return
        try:
            data = decode(descriptor["data"], self.directory / "frames")
        except FileNotFoundError:
            # Bounded history can evict a frame before a delayed subscriber maps
            # it. No reused slot can be mistaken for the old generation.
            return
        self._remote_sequence = descriptor["sequence"]
        await self.publish(data, stamp_ns=descriptor["stamp_ns"], clock=descriptor["clock"],
                           received_ns=descriptor["received_ns"])

    async def close(self):
        import shutil
        if self._lease is not None:
            os.close(self._lease)
            self._lease = None
        if self.process is not None:
            try:
                await asyncio.wait_for(self.process.wait(), 10)
            except TimeoutError:
                self.process.terminate()
                await asyncio.wait_for(self.process.wait(), 7)
        if self.subscription is not None:
            self.children[0].node.destroy_subscription(self.subscription)
        if self._log is not None:
            self._log.close()
        if self.directory is not None:
            shutil.rmtree(self.directory, ignore_errors=True)
        self.pending.clear()
