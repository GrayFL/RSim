"""DDS control protocol adapter around an injected, protocol-free controller."""

import asyncio
from collections import OrderedDict
import json
import math
import time
import uuid

from rsim.core.commands import VelocityCommand
from rsim.runtime.service import RPCServer, RemoteError
from rsim.runtime.locks import acquire_device


def pose_packet(pose):
    return dict(
        position=pose.position.tolist(),
        rotation=pose.quat.tolist(),
        scale=float(pose.scale),
        wrd_frame=pose.wrd_frame,
        ego_frame=pose.ego_frame,
    )


class MotionService(RPCServer):
    def __init__(self, controller, *, name="chassis", state=None, transport=None):
        super().__init__(name, controller, transport=transport)
        self.name, self.controller, self.hardware_state = name, controller, state
        self.inputs = () if state is None else (state,)
        self.pose = self.signal("pose", history=128)
        self.pose._target = controller.pose
        self.state = self.signal("state", history=32)
        self._lease = None

    async def open(self):
        self._lease = acquire_device(
            "control-service:" + str(self.bus.config.domain_id) + ":" + self.name
        )
        self.owner = None
        self.deadline = 0
        self.cursors = {}
        self.job = None
        self.job_id = None
        self.results = OrderedDict()
        self.telemetry_sequence = 0
        self.control_lock = asyncio.Lock()
        await super().open()
        self.task("controller-lease", self.expire, hz=100)
        self.task("telemetry", self.publish_state, hz=30)

    async def _halt(self, *, release=False):
        if self.job is not None and not self.job.done():
            self.job.cancel()
            await asyncio.gather(self.job, return_exceptions=True)
            result = self.results[self.job_id]
            if not result["done"]:
                result.update(done=True, error="movement cancelled before starting")
        await self.controller.stop()
        if release:
            self.owner = None
            self.deadline = 0

    async def expire(self):
        async with self.control_lock:
            await self._expire_locked()

    async def _expire_locked(self):
        if self.owner is not None and time.monotonic_ns() >= self.deadline:
            await self._halt(release=True)

    async def handle(self, request):
        async with self.control_lock:
            return await self._handle_locked(request)

    async def _handle_locked(self, request):
        now = time.monotonic_ns()
        session = request["session"]
        sequence = request["sequence"]
        deadline = request["deadline_ns"]
        if (
            not isinstance(session, str)
            or not 1 <= len(session) <= 128
            or type(sequence) is not int
            or sequence <= self.cursors.get(session, 0)
            or type(deadline) is not int
            or not 0 < deadline - now <= 500_000_000
        ):
            raise RemoteError("invalid, replayed or expired request")
        if session not in self.cursors and len(self.cursors) >= 1024:
            raise RemoteError("session history full; restart provider")
        self.cursors[session] = sequence
        op = request["op"]
        if op == "result":
            result = self.results.get(request["action"])
            if result is None or result["session"] != session:
                raise RemoteError("unknown action")
            return result
        if op in ("stop", "release"):
            if self.owner == session:
                await self._halt(release=op == "release")
            return {"stopped": self.owner in (None, session)}
        await self._expire_locked()
        if self.owner is not None and self.owner != session:
            raise RemoteError("another client owns chassis control")
        if op == "renew":
            if self.owner != session:
                raise RemoteError("control lease expired")
            self.deadline = deadline
            return {"renewed": True}
        if op not in ("velocity", "move", "rotate"):
            raise RemoteError("unsupported control operation")
        if self.job is not None and not self.job.done():
            raise RemoteError("a movement is already active")
        if op == "velocity":
            command = VelocityCommand(
                float(request["linear_x"]), float(request["angular_z"])
            )
            # Validate/execute before granting ownership on rejected commands.
            await self.controller.drive(
                command,
                ttl=min(
                    self.controller.command_ttl, (deadline - time.monotonic_ns()) / 1e9
                ),
            )
            self.owner, self.deadline = session, deadline
            return {"accepted": True}
        value = float(request["value"])
        timeout = float(request["timeout"])
        if (
            not math.isfinite(value)
            or not math.isfinite(timeout)
            or timeout <= 0
            or timeout > 3600
        ):
            raise RemoteError("invalid movement target or timeout")
        if value != 0 and not self.controller.motion_enabled:
            raise RemoteError("motion disabled")
        self.owner, self.deadline = session, deadline
        identifier = uuid.uuid4().hex
        self.results[identifier] = {"session": session, "done": False}
        while len(self.results) > 64:
            self.results.popitem(last=False)
        self.job_id = identifier

        async def move():
            result = self.results[identifier]
            try:
                pose = await (
                    self.controller.move(value, timeout=timeout)
                    if op == "move"
                    else self.controller.rotate(yaw_rad=value, timeout=timeout)
                )
                result["pose"] = pose_packet(pose)
            except asyncio.CancelledError:
                result["error"] = "movement cancelled or control lease expired"
            except Exception as error:
                result["error"] = str(error)
            finally:
                result["done"] = True

        self.job = asyncio.create_task(move(), name="rsim:remote-movement")
        return {"action": identifier}

    async def publish_state(self):
        try:
            frame = await self.controller.pose.get(timeout=0.01)
        except TimeoutError:
            return
        self.telemetry_sequence += 1
        hardware = None
        if self.hardware_state is not None and self.hardware_state.frames:
            hardware = (await self.hardware_state.get(timeout=0.01)).data
        state = dict(
            motion_enabled=self.controller.motion_enabled,
            owner=self.owner,
            moving=self.job is not None and not self.job.done(),
            max_linear=self.controller.max_linear,
            max_angular=self.controller.max_angular,
            hardware=hardware,
        )
        packet = dict(
            generation=self.generation,
            sequence=self.telemetry_sequence,
            pose=pose_packet(frame.data),
            stamp_ns=frame.stamp_ns,
            clock=frame.clock,
            source_sequence=frame.sequence,
            source_age_ns=max(0, time.time_ns() - frame.received_ns),
            state=state,
        )
        self.telemetry.publish(json.dumps(packet, allow_nan=False))
        await self.state.publish(
            state, stamp_ns=time.monotonic_ns(), clock="host:monotonic"
        )

    async def close(self):
        try:
            if hasattr(self, "job"):
                await self._halt(release=True)
        finally:
            if hasattr(self, "subscription"):
                await super().close()
            if self._lease is not None:
                self._lease.close()
                self._lease = None

    async def _watchdog(self):
        await super()._watchdog()
        if self._failure is not None and hasattr(self, "job"):
            # An RPC/telemetry failure must also end the separately awaited
            # action; otherwise the healthy controller could keep refreshing it.
            try:
                await self._halt(release=True)
            except Exception:
                pass  # hardware provider retains its own TTL
