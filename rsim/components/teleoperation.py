"""Metered keyboard-to-command simulation, independent of input and transport."""

import asyncio
import logging
import time

from rsim.core.component import Component
from rsim.core.commands import VelocityCommand
from rsim.core.signal import as_signal
from .vehicle import VehicleDynamics

logger = logging.getLogger(__name__)


class Teleoperation(Component):
    def __init__(self, keyboard, velocity, *, parameters=None, dry_run=False):
        self.keyboard, self.velocity = as_signal(keyboard), velocity
        super().__init__(velocity.producer, inputs=(self.keyboard,))
        self.command_targets = (velocity,)
        self.dynamics = VehicleDynamics(parameters)
        self.dry_run = bool(dry_run)
        self.state = self.signal("state")
        self.finished = asyncio.Event()
        self.acknowledged = asyncio.Event()

    async def open(self):
        self.dynamics.reset()
        self.finished.clear()
        self.acknowledged.clear()
        self.previous = time.monotonic()
        self.pending = None
        self.command_latency_ms = None
        self.task("model", self.step, hz=self.dynamics.parameters.hz)
        self.task("drive", self.send, hz=self.dynamics.parameters.hz)

    async def step(self):
        p = self.dynamics.parameters
        frame = await self.keyboard.get(timeout=p.max_loop_gap)
        now = time.monotonic()
        dt, self.previous = now - self.previous, now
        if time.time_ns() - frame.received_ns > p.max_loop_gap * 1e9:
            raise TimeoutError("keyboard source is stale")
        if frame.data["quit"]:
            self.finished.set()
        if frame.data["brake"] or self.finished.is_set():
            self.dynamics.reset()
            command = VelocityCommand()
        else:
            command = self.dynamics.step(frame.data["keys"], dt)
        # One latest value, not a queue. Waiting for a remote acknowledgement
        # must not pace input sampling or the vehicle model.
        self.pending = (VelocityCommand() if self.dry_run else command, now)
        await self.state.publish(
            dict(
                self.dynamics.state(),
                dry_run=self.dry_run,
                keys=frame.data["keys"],
                brake=frame.data["brake"],
                command_latency_ms=self.command_latency_ms,
            ),
            stamp_ns=time.monotonic_ns(),
            clock="host:monotonic",
        )

    async def send(self):
        if self.pending is None:
            return
        command, sampled = self.pending
        self.pending = None
        p = self.dynamics.parameters
        remaining = p.command_ttl - (time.monotonic() - sampled)
        if remaining <= 0:
            raise TimeoutError("keyboard command expired before transmission")
        started = time.monotonic()
        try:
            await self.velocity.set(command, ttl=remaining, _writer=self)
        except Exception:
            logger.exception("Velocity acknowledgement failed after %.1fms (remaining TTL %.1fms)",
                             (time.monotonic() - started) * 1000, remaining * 1000)
            raise
        self.command_latency_ms = (time.monotonic() - started) * 1000
        self.acknowledged.set()
        if self.command_latency_ms > p.max_loop_gap * 1000:
            logger.warning("Slow velocity acknowledgement %.1fms; model runs independently",
                           self.command_latency_ms)

    async def close(self):
        self.finished.set()
        self.pending = None
        self.dynamics.reset()
        await self.velocity.set(
            VelocityCommand(), ttl=self.dynamics.parameters.command_ttl, _writer=self
        )
