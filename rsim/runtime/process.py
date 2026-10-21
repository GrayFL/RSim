"""Run an arbitrary sensor factory in a supervised child asyncio loop."""
from __future__ import annotations

import asyncio
from collections import deque
import json
import os
from pathlib import Path
from rsim.platforms.process import spawn_leased
from .paths import instance_directory, worker_log
from .supervision import wait_ready, read_status, check_cleanup_report
from rsim.transport.files import remove_tree, open_shared
import sys
import uuid
from dataclasses import asdict

import cloudpickle

from rsim.core.component import PrimaryComponent, ComponentError
from rsim.transport.shared import decode
from .port_binding import sample_id
from rsim.transport.descriptor import DescriptorTransport, transport_config


class ProcessSensor(PrimaryComponent):
    def __init__(self, factory, *, history=16, hz=200, transport=None, output_name="output", options=None):
        self.options = options
        self.transport = transport_config(transport)
        super().__init__(DescriptorTransport(self.transport), history=history, output_name=output_name)
        self.factory, self.hz = factory, hz
        self.process = None
        self.directory = None
        self._lease = None
        self.subscription = None
        self.pending = deque(maxlen=32)
        self._remote_sequence = 0
        self.worker_pid = None
        self._log = None
        self.queue_drops = self.evicted_descriptors = 0

    def _enqueue(self, value):
        self.queue_drops += len(self.pending) == self.pending.maxlen
        self.pending.append(value)

    def __getstate__(self):
        state = super().__getstate__()
        state.update(process=None, directory=None, _lease=None, subscription=None,
                     worker_pid=None, _log=None)
        return state

    async def open(self):
        if self.transport.backend == 'inprocess':
            raise ValueError('ProcessSensor requires a real cross-process DDS backend')
        self.options = (self.options or self._runtime.options).resolved()
        self.pending.clear()
        self.queue_drops = self.evicted_descriptors = 0
        self._remote_sequence = 0
        self.directory = instance_directory('rsim-', self.options)
        self.log_path = worker_log(self.directory, self.options)
        topic = "/rsim/frames/" + "p" + uuid.uuid4().hex
        (self.directory / "factory.pkl").write_bytes(cloudpickle.dumps(self.factory))
        (self.directory / "config.json").write_text(json.dumps({
            "history": self._history.maxlen, "topic": topic,
            "options": self.options.to_dict(), "log_path": str(self.log_path),
            "transport": asdict(self.transport),
            "sys_path": [str(Path(p).resolve()) for p in sys.path]}))
        self.subscription = self.children[0].subscribe(topic, self._enqueue)
        self._log = open_shared(self.log_path, write=True, create=True)
        self.process, self._lease = await spawn_leased('rsim.runtime.supervisor', self.directory,
                                                       stdout=self._log, stderr=self._log)
        self.worker_pid = (await wait_ready(self.process, self.directory, self.options.startup_timeout_s))['pid']
        self.task("receive", self.receive, hz=self.hz)

    async def receive(self):
        state = read_status(self.directory)
        if state:
            self.worker_pid = state.get("pid")
            if "error" in state:
                raise ComponentError(state["error"])
        if self.process is not None and self.process.returncode is not None:
            raise ComponentError(f"sensor supervisor exited: {self.process.returncode}")
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
            self.evicted_descriptors += 1
            return
        self._remote_sequence = descriptor["sequence"]
        await self.publish(data, stamp_ns=descriptor["stamp_ns"], clock=descriptor["clock"],
                           received_ns=descriptor["received_ns"],
                           sample_id=sample_id(descriptor),
                           metadata=decode(descriptor["metadata"], self.directory / "frames") if "metadata" in descriptor else {})

    async def close(self):
        if self._lease is not None:
            os.close(self._lease)
            self._lease = None
        try:
            if self.process is not None:
                try:
                    await asyncio.wait_for(self.process.wait(), self.options.shutdown.grace_s + self.options.shutdown.kill_wait_s + 2)
                except TimeoutError:
                    self.process.terminate()
                    await asyncio.wait_for(self.process.wait(), self.options.shutdown.kill_wait_s)
        finally:
            if self.subscription is not None:
                self.subscription.close()
                self.subscription = None
            if self._log is not None:
                self._log.close()
            if self.directory is not None:
                remove_tree(self.directory)
            self.pending.clear()
        check_cleanup_report(getattr(self, 'log_path', None))
