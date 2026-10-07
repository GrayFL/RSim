"""Metered keyboard-to-command simulation, independent of input and transport."""

import asyncio
import time

from rsim.core.component import Component
from rsim.core.commands import VelocityCommand
from rsim.core.signal import as_signal
from .vehicle import VehicleDynamics


class Teleoperation(Component):
    def __init__(self, keyboard, velocity, *, parameters=None, dry_run=False):
        self.keyboard, self.velocity = as_signal(keyboard), velocity
        super().__init__(velocity.producer, inputs=(self.keyboard,))
        self.command_targets = (velocity,)
        self.dynamics = VehicleDynamics(parameters)
        self.dry_run = bool(dry_run)
        self.state = self.signal("state")
        self.finished = asyncio.Event()

    async def open(self):
        self.dynamics.reset()
        self.finished.clear()
        self.previous = time.monotonic()
        self.task("drive", self.step, hz=self.dynamics.parameters.hz)

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
        await self.velocity.set(
            VelocityCommand() if self.dry_run else command,
            ttl=p.command_ttl,
            _writer=self,
        )
        await self.state.publish(
            dict(
                self.dynamics.state(),
                dry_run=self.dry_run,
                keys=frame.data["keys"],
                brake=frame.data["brake"],
            ),
            stamp_ns=time.monotonic_ns(),
            clock="host:monotonic",
        )

    async def close(self):
        self.finished.set()
        self.dynamics.reset()
        await self.velocity.set(
            VelocityCommand(), ttl=self.dynamics.parameters.command_ttl, _writer=self
        )
