"""ROS-free, asynchronous ROS1 topic ingress and egress over managed SSH."""
from collections import deque
from dataclasses import dataclass
import asyncio
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import sys
import time

import numpy as np

from .core import Sensor, SensorError
from .compose import Bundle

MAX_MESSAGE = 8 * 1024 * 1024
ARRAY_DTYPES = {"?": "?", "b": "i1", "B": "u1", "h": "<i2", "H": "<u2", "i": "<i4",
                "I": "<u4", "q": "<i8", "Q": "<u8", "f": "<f4", "d": "<f8"}


def decode_message(value):
    if isinstance(value, dict):
        if "__array__" in value:
            dtype = np.dtype(ARRAY_DTYPES[value["__array__"]])
            raw = base64.b64decode(value["data"], validate=True)
            return np.frombuffer(raw, dtype=dtype)
        if "__float__" in value:
            return float(value["__float__"])
        return {key: decode_message(item) for key, item in value.items()}
    if isinstance(value, list):
        return [decode_message(item) for item in value]
    return value


@dataclass(frozen=True)
class SSHConfig:
    host: str
    remote_script: str = "Projects/RSim/compat/ros1_agent.py"
    python: str = "python2"
    setup: tuple[str, ...] = ("/opt/ros/kinetic/setup.bash",)
    master_uri: str = "http://127.0.0.1:11311"
    ros_ip: str | None = None

    def command(self):
        if not self.host or self.host.startswith("-"):
            raise ValueError("SSH host must be a hostname or SSH alias")
        lines = ["set -e"]
        lines += ["source " + shlex.quote(path) + " >&2" for path in self.setup]
        lines.append("export ROS_MASTER_URI=" + shlex.quote(self.master_uri))
        if self.ros_ip:
            lines.append("export ROS_IP=" + shlex.quote(self.ros_ip))
        lines.append("exec " + shlex.join([self.python, "-u", self.remote_script]))
        return [shutil.which("ssh") or "ssh", "-T", "-o", "BatchMode=yes",
                "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=3",
                "-o", "ServerAliveCountMax=3", "--", self.host,
                "bash -c " + shlex.quote("\n".join(lines))]


class Ros1Bridge(Sensor):
    """One SSH session shared by topic sensors in a Runtime.

    Frames contain plain dictionaries and read-only NumPy arrays. Publishing
    acknowledges a call to rospy with live subscribers, not physical actuation.
    An interrupted session fails pending requests; commands are never replayed.
    """
    def __init__(self, connection, *, log_path=None):
        if isinstance(connection, str):
            connection = SSHConfig(connection)
        self.connection = connection
        key = hashlib.sha256(repr(connection).encode()).hexdigest()
        super().__init__(key="ros1:ssh:" + key, history=1)
        self.log_path = Path(log_path) if log_path is not None else None
        self.process = self._log = None
        self._pending, self._queues = {}, {}
        self._request_id = 0
        self._write_lock = asyncio.Lock()
        self.diagnostics = deque(maxlen=20)
        self.remote_info = None

    def configuration(self):
        return super().configuration(), self.connection

    def topic(self, name, message_type=None, *, hz=100, history=32):
        return Ros1Topic(self, name, message_type, hz=hz, history=history)

    async def open(self):
        self._pending.clear()
        self._queues.clear()
        self.diagnostics.clear()
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log = self.log_path.open("ab")
        self.process = await asyncio.create_subprocess_exec(
            sys.executable, "-m", "rsim._exec", str(os.getpid()), *self.connection.command(),
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE, limit=MAX_MESSAGE)
        self.task("ssh-stderr", self._stderr, hz=1000)
        async with asyncio.timeout(20):
            packet = await self._packet()
        if packet.get("op") != "ready" or packet.get("version") != 1:
            raise SensorError("incompatible ROS1 endpoint handshake")
        self.remote_info = packet
        self.task("ssh-receive", self._receive, hz=1000)
        self.task("ssh-heartbeat", self._heartbeat, hz=1)
        self.service("publish-message", self._publish, hz=100, capacity=8)

    async def _stderr(self):
        data = await self.process.stderr.readline()
        if data:
            self.diagnostics.append(data.decode(errors="replace").rstrip())
            if self._log is not None:
                self._log.write(data)
                self._log.flush()

    async def _packet(self):
        data = await self.process.stdout.readline()
        if not data:
            raise SensorError("ROS1 SSH session closed: " + "\n".join(self.diagnostics))
        return json.loads(data)

    async def _receive(self):
        try:
            packet = await self._packet()
            if packet["op"] == "reply":
                future = self._pending.get(packet["id"])
                if future is not None and not future.done():
                    if "error" in packet:
                        future.set_exception(SensorError(packet["error"]))
                    else:
                        future.set_result(packet["result"])
            elif packet["op"] == "sample":
                queue = self._queues.get(packet["topic"])
                if queue is not None:
                    queue.append((packet, time.time_ns()))
            else:
                raise SensorError("unexpected ROS1 endpoint operation")
        except Exception as error:
            for future in tuple(self._pending.values()):
                if not future.done():
                    future.set_exception(error)
            raise

    async def _request(self, op, *, timeout=5, **fields):
        if self._canonical is not None:
            return await self._canonical._request(op, timeout=timeout, **fields)
        if self._failure is not None:
            raise SensorError("ROS1 connection failed") from self._failure
        if self._closed or self.process is None or self.process.returncode is not None:
            raise SensorError("ROS1 connection is not running")
        if len(self._pending) >= 32:
            raise SensorError("too many pending ROS1 requests")
        self._request_id += 1
        request_id = str(self._request_id)
        data = json.dumps(dict(fields, op=op, id=request_id), allow_nan=False).encode() + b"\n"
        if len(data) > MAX_MESSAGE:
            raise ValueError("ROS1 request exceeds size limit")
        future = asyncio.get_running_loop().create_future()
        self._pending[request_id] = future
        try:
            async with asyncio.timeout(timeout):
                async with self._write_lock:
                    self.process.stdin.write(data)
                    await self.process.stdin.drain()
                return await future
        finally:
            self._pending.pop(request_id, None)
            if not future.done():
                future.cancel()

    async def _heartbeat(self):
        await self._request("ping", timeout=5)

    async def topics(self):
        return await self._request("topics")

    async def _publish(self, topic, message_type, data):
        return await self._request("publish", topic=topic, type=message_type, data=data)

    async def publish_message(self, topic, message_type, data, *, timeout=5):
        return await self.call("publish-message", topic, message_type, data, timeout=timeout)

    async def close(self):
        for future in tuple(self._pending.values()):
            if not future.done():
                future.set_exception(SensorError("ROS1 connection closed"))
        if self.process is not None:
            if self.process.stdin is not None:
                self.process.stdin.close()
            try:
                await asyncio.wait_for(self.process.wait(), 4)
            except TimeoutError:
                self.process.kill()
                await self.process.wait()
        if self._log is not None:
            self._log.close()
            self._log = None
        self._queues.clear()


class Ros1Topic(Sensor):
    def __init__(self, bridge, topic, message_type=None, *, hz=100, history=32):
        if not topic.startswith("/") or not math.isfinite(hz) or not 0 < hz <= 1000:
            raise ValueError("topic must be absolute and hz in (0, 1000]")
        super().__init__(bridge, key=f"{bridge.key}:{topic}", history=history)
        self.topic_name, self.message_type, self.hz = topic, message_type, hz
        self.pending = deque(maxlen=1)
        self.actual_type = None

    def configuration(self):
        return super().configuration(), self.message_type, self.hz

    @property
    def bridge(self):
        return (self._canonical or self).children[0]

    async def open(self):
        bridge = self.children[0]
        self.pending.clear()
        bridge._queues[self.topic_name] = self.pending
        result = await bridge._request("subscribe", topic=self.topic_name,
                                       type=self.message_type, hz=self.hz)
        self.actual_type = result["type"]
        self.task("topic-ingress", self._ingress, hz=self.hz * 2)

    async def _ingress(self):
        if self.pending:
            packet, received = self.pending.popleft()
            await self.publish(decode_message(packet["data"]), stamp_ns=packet["stamp_ns"],
                               clock="ros1:" + self.children[0].connection.host, received_ns=received)

    async def close(self):
        bridge = self.children[0]
        bridge._queues.pop(self.topic_name, None)
        if bridge.process is not None and bridge.process.returncode is None:
            try:
                await bridge._request("unsubscribe", topic=self.topic_name, timeout=1)
            except (SensorError, OSError, TimeoutError):
                pass
        self.pending.clear()


class Chassis(Bundle):
    """Composable IMU / odometry / planar scan and a velocity command port."""
    def __init__(self, connection, *, imu_topic="/imu_data", odom_topic="/odom",
                 scan_topic="/scan", cmd_vel_topic="/cmd_vel", history=32, hz=30, log_path=None):
        bridge = Ros1Bridge(connection, log_path=log_path)
        self.imu = bridge.topic(imu_topic, "sensor_msgs/Imu", hz=100, history=history)
        self.odom = bridge.topic(odom_topic, "nav_msgs/Odometry", hz=50, history=history)
        self.scan = bridge.topic(scan_topic, "sensor_msgs/LaserScan", hz=30, history=history)
        self.cmd_vel_topic = cmd_vel_topic
        super().__init__(imu=self.imu, odom=self.odom, scan=self.scan, hz=hz, history=history)

    @property
    def bridge(self):
        return self.imu.bridge

    async def set_velocity(self, linear=0.0, angular=0.0):
        linear, angular = float(linear), float(angular)
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError("velocity must be finite")
        return await self.bridge.publish_message(self.cmd_vel_topic, "geometry_msgs/Twist", {
            "linear": {"x": linear, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular}})

    async def stop(self):
        return await self.set_velocity()
