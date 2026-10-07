"""Connection-only chassis client. No ROS, serial driver or server factory import."""

import asyncio
import json
import math
import time
import uuid

from rsim.core.commands import CommandSink, CommandRejected, VelocityCommand
from rsim.runtime.service import RPCClient, RemoteError


class Chassis(RPCClient):
    def __init__(self, name="chassis", *, transport=None, history=128, lease_ttl=0.3):
        if not 0.15 <= lease_ttl <= 0.5:
            raise ValueError("lease_ttl must be in [0.15, 0.5] seconds")
        super().__init__(name, transport=transport)
        self.pose = self.signal("pose", history=history)
        self.state = self.signal("state", history=32)
        self.velocity = CommandSink(
            self,
            "velocity",
            self._velocity,
            fallback=VelocityCommand(),
            safe=self._safe,
            max_ttl=0.5,
            hz=100,
        )
        self.lease_ttl = lease_ttl

    async def open(self):
        # Resolve the optional pose dependency before starting time-sensitive
        # exchanges; first-time SciPy imports can otherwise stall a heartbeat.
        from graphmap.pose import Pose

        self.pose_type = Pose
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.owns = False
        self.generation = None
        self.pose_sequence = 0
        self.source_sequence = 0
        self.offset = 0
        self.clock_sample_ns = 0
        self.manual = False
        self.action_active = False
        await super().open()
        async with asyncio.timeout(15):
            while self.generation is None:
                try:
                    await self.synchronize()
                except TimeoutError:
                    await asyncio.sleep(0.05)
        self.task("clock-bound", self.synchronize, hz=1)
        self.task("control-lease", self.renew, hz=20)
        self.task("chassis-feedback", self.feedback, hz=200)

    async def synchronize(self):
        sent = time.monotonic_ns()
        result = await self.request({"op": "hello"}, timeout=0.5)
        received = time.monotonic_ns()
        if self.generation is not None and self.generation != result["generation"]:
            raise RemoteError("provider restarted; reconnect explicitly")
        if received - sent > 50_000_000:
            # A queued reply is still a valid lower clock bound, but can consume
            # nearly the whole lease. Keep a recent good sample or fail closed.
            if (
                not self.clock_sample_ns
                or received - self.clock_sample_ns > 3_000_000_000
            ):
                raise TimeoutError("no sufficiently fresh clock sample")
            return
        self.generation = result["generation"]
        # Conservative lower bound: network time consumes the client's TTL.
        self.offset = result["server_ns"] - received
        self.clock_sample_ns = received

    async def _call(self, op, *, deadline=None, **values):
        if self._closed or getattr(self, "generation", None) is None:
            raise RemoteError("chassis client is not running")
        if self._failure is not None:
            raise RemoteError("chassis client failed") from self._failure
        self.sequence += 1
        deadline = (
            time.monotonic_ns() + int(self.lease_ttl * 1e9)
            if deadline is None
            else deadline
        )
        return await self.request(
            dict(
                op=op,
                generation=self.generation,
                session=self.session,
                sequence=self.sequence,
                deadline_ns=deadline + self.offset,
                **values,
            ),
            timeout=max(0.001, (deadline - time.monotonic_ns()) / 1e9),
        )

    async def renew(self):
        if self.owns:
            try:
                await self._call("renew")
            except (RemoteError, TimeoutError):
                if self.owns:
                    raise

    async def feedback(self):
        while self.states:
            packet = json.loads(self.states.popleft())
            if packet["generation"] != self.generation:
                raise RemoteError("provider identity changed")
            if packet["sequence"] <= self.pose_sequence:
                continue
            self.pose_sequence = packet["sequence"]
            metadata = dict(
                stamp_ns=packet["stamp_ns"],
                clock="provider:" + self.generation + ":" + packet["clock"],
            )
            if packet["source_sequence"] > self.source_sequence:
                self.source_sequence = packet["source_sequence"]
                await self.pose.publish(
                    self.pose_type(**packet["pose"]),
                    received_ns=time.time_ns() - packet["source_age_ns"],
                    **metadata,
                )
            await self.state.publish(packet["state"], **metadata)

    async def _velocity(self, envelope):
        if not isinstance(envelope.value, VelocityCommand):
            raise CommandRejected("expected VelocityCommand")
        if self.action_active:
            raise CommandRejected("a movement is already active")
        try:
            result = await self._call(
                "velocity",
                deadline=envelope.deadline_ns,
                linear_x=envelope.value.linear_x,
                angular_z=envelope.value.angular_z,
            )
            self.owns = True
            self.manual = True
            return result
        except (RemoteError, TimeoutError) as error:
            raise CommandRejected(
                str(error) or "command acknowledgement expired"
            ) from error

    async def _safe(self, _):
        if self.owns and self.manual:
            await self.stop()

    async def drive(self, command, *, ttl=0.25):
        return await self.velocity.set(command, ttl=ttl)

    async def stop(self):
        self.owns = False
        self.manual = False
        return await self._call("release")

    async def _movement(self, op, value, timeout):
        if self._closed:
            raise RemoteError("chassis client is not running")
        if self.action_active:
            raise RemoteError("a movement is already active")
        self.action_active = True
        try:
            return await self._perform_movement(op, value, timeout)
        finally:
            self.action_active = False

    async def _perform_movement(self, op, value, timeout):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("movement must be finite")
        timeout = 60.0 if timeout is None else float(timeout)
        if self.manual:
            await self.stop()
        action = await self._call(op, value=value, timeout=timeout)
        self.owns = True
        try:
            async with asyncio.timeout(timeout + 1):
                while True:
                    result = await self._call("result", action=action["action"])
                    if result["done"]:
                        if "error" in result:
                            raise RemoteError(result["error"])
                        return self.pose_type(**result["pose"])
                    await asyncio.sleep(0.03)
        finally:
            self.owns = False
            try:
                await self._call("release")
            except (RemoteError, TimeoutError):
                pass

    async def move(self, distance_m, *, timeout=None):
        return await self._movement("move", distance_m, timeout)

    async def rotate(self, yaw_deg=None, *, yaw_rad=None, timeout=None):
        if (yaw_deg is None) == (yaw_rad is None):
            raise ValueError("specify exactly one angle unit")
        value = math.radians(float(yaw_deg)) if yaw_rad is None else yaw_rad
        return await self._movement("rotate", value, timeout)

    async def close(self):
        try:
            if self.owns:
                try:
                    await self._call("release")
                except (RemoteError, TimeoutError):
                    pass  # provider still owns the independent lease expiry
        finally:
            self.owns = False
            await super().close()
