"""Optional ROS 2 ingress. ROS messages never escape through Signal.get()."""
from __future__ import annotations

import asyncio
from collections import deque
from pathlib import Path
import os
import signal
import sys
import time
import uuid

import numpy as np

from .core import Component, PrimaryComponent, ComponentError
from .model import Image, PointCloud
from ._driver_config import d435_setup, robin_setup
from ._ros_args import RosArguments


def pointcloud_array(msg):
    types = {
        1: "i1",
        2: "u1",
        3: "i2",
        4: "u2",
        5: "i4",
        6: "u4",
        7: "f4",
        8: "f8"
        }
    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for field in msg.fields:
        dtype = np.dtype(endian + types[field.datatype])
        names.append(field.name)
        formats.append(
            dtype if field.count == 1 else (dtype, (field.count, ))
            )
        offsets.append(field.offset)
    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": msg.point_step
        })
    result = np.ndarray((msg.height, msg.width),
                        dtype=dtype,
                        buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    result.flags.writeable = False
    return PointCloud(result, msg.header.frame_id)


def image_array(msg):
    formats = {
        "rgb8": ("u1", 3),
        "bgr8": ("u1", 3),
        "rgba8": ("u1", 4),
        "bgra8": ("u1", 4),
        "mono8": ("u1", 1),
        "8UC1": ("u1", 1),
        "mono16": ("u2", 1),
        "16UC1": ("u2", 1),
        "32FC1": ("f4", 1)
        }
    if msg.encoding not in formats:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    scalar, channels = formats[msg.encoding]
    dtype = np.dtype((">" if msg.is_bigendian else "<") + scalar)
    shape = (msg.height, msg.width) + ((channels, ) if channels > 1 else
                                        ())
    strides = (msg.step, channels *
                dtype.itemsize) + ((dtype.itemsize, ) if channels > 1 else
                                    ())
    result = np.ndarray(
        shape, dtype=dtype, buffer=msg.data, strides=strides
        )
    result.flags.writeable = False
    return Image(result, msg.encoding, msg.header.frame_id)


class RosContext(Component):
    process_local = True

    def __init__(self, *, hz=1000):
        super().__init__(key="rsim:ros-context")
        self.hz = hz
        self.node = self.context = self.executor = None

    def configuration(self):
        return super().configuration(), self.hz

    async def open(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        self.context = Context()
        rclpy.init(
            args=[],
            context=self.context,
            signal_handler_options=SignalHandlerOptions.NO
            )
        self.node = rclpy.create_node(
            "rsim_" + uuid.uuid4().hex, context=self.context
            )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.task("dds-poll", self.poll, hz=self.hz)

    async def poll(self):
        self.executor.spin_once(timeout_sec=0)

    async def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None:
            self.context.try_shutdown()
        self.node = self.context = self.executor = None


class Driver(Component):
    """Direct native executable launch, guarded against parent death on Linux."""

    def __init__(
            self,
            package,
            executable,
            parameters=None,
            *,
            key,
            log_path=None,
            remappings=None,
            ros_args=None
        ):
        super().__init__(key=key)
        self.package, self.executable = package, executable
        self.options = parameters if isinstance(parameters, RosArguments) else RosArguments(
            parameters, ros_args, remappings)
        self.parameters = self.options.parameters
        self.log_path = log_path
        self.remappings = self.options.remappings
        self.process = self._log = None
        self._device_lease = None

    def configuration(self):
        return super().configuration(), self.package, self.executable, self.options.signature()

    def __getstate__(self):
        state = super().__getstate__()
        state.update(process=None, _log=None, _device_lease=None)
        return state

    async def open(self):
        from ament_index_python.packages import get_package_prefix
        from .lifecycle import acquire_device
        executable = Path(
            get_package_prefix(self.package)
            ) / "lib" / self.package / self.executable
        args = [str(executable), *self.options.arguments()]
        self._device_lease = acquire_device(self.key)
        if self.log_path:
            self._log = open(self.log_path, "a")
        self.process = await asyncio.create_subprocess_exec(
            sys.executable,
            "-m",
            "rsim._exec",
            str(os.getpid()),
            *args,
            stdout=self._log or asyncio.subprocess.DEVNULL,
            stderr=self._log or asyncio.subprocess.DEVNULL
            )
        self.task("driver-health", self.check, hz=20)

    async def check(self):
        if self.process.returncode is not None:
            raise ComponentError(
                f"{self.package} driver exited: {self.process.returncode}"
                )

    async def close(self):
        try:
            if self.process is not None and self.process.returncode is None:
                try:
                    self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(self.process.wait(), 5)
                except TimeoutError:
                    self.process.kill()
                    await self.process.wait()
        finally:
            if self._log is not None:
                self._log.close()
                self._log = None
            if self._device_lease is not None:
                self._device_lease.close()
                self._device_lease = None


class RosSensor(PrimaryComponent):

    def __init__(
            self, topic, kind, *, clock, driver=None, hz=200, history=16
        ):
        children = (RosContext(), ) + ((driver, ) if driver else ())
        super().__init__(
            *children, key=f"ros:{kind}:{topic}", history=history, output_name=kind, clock=clock
            )
        setattr(self, kind, self.output)
        self.topic, self.kind, self.clock, self.hz = topic, kind, clock, hz
        self.pending = deque(maxlen=2)
        self.subscription = None
        self.dropped = 0

    def configuration(self):
        return (
            super().configuration(),
            self.topic,
            self.kind,
            self.clock,
            self.hz,
            tuple(c.configuration() for c in self.children)
            )

    async def open(self):
        from sensor_msgs.msg import Image as RosImage, PointCloud2
        from rclpy.qos import qos_profile_sensor_data
        self.pending.clear()

        def receive(msg):
            if len(self.pending) == self.pending.maxlen:
                self.dropped += 1
            self.pending.append((msg, time.time_ns()))

        self.subscription = self.children[0].node.create_subscription(
            PointCloud2 if self.kind == "points" else RosImage,
            self.topic,
            receive,
            qos_profile_sensor_data
            )
        self.task("convert", self.convert, hz=self.hz)

    async def convert(self):
        if not self.pending:
            return
        msg, received_ns = self.pending.popleft()
        data = pointcloud_array(
            msg
            ) if self.kind == "points" else image_array(msg)
        await self.publish(
            data,
            stamp_ns=msg.header.stamp.sec * 10**9
            + msg.header.stamp.nanosec,
            clock=self.clock,
            received_ns=received_ns
            )

    async def close(self):
        if self.subscription is not None:
            self.children[0].node.destroy_subscription(self.subscription)
            self.subscription = None
        self.pending.clear()


class RobinW(RosSensor):

    def __init__(
            self,
            ip="192.168.199.97",
            *,
            start_driver=True,
            topic="/iv_points",
            history=16,
            log_path=None,
            parameters=None,
            ros_args=None,
            _setup=None
        ):
        setup = _setup or robin_setup(ip, topic, parameters, ros_args)
        ip, topic = setup["ip"], setup["topic"]
        driver = Driver(
            "seyond",
            "seyond_node", setup["options"],
            key=f"driver:robin:{ip}",
            log_path=log_path
            ) if start_driver else None
        super().__init__(
            topic,
            "points",
            clock=f"device:robin:{ip}",
            driver=driver,
            history=history
            )



class D435(RosSensor):

    def __init__(
            self,
            *,
            stream="depth",
            serial="",
            start_driver=True,
            history=16,
            log_path=None,
            depth_profile="640x480x15",
            color_profile="640x480x15",
            parameters=None,
            ros_args=None,
            _setup=None
        ):
        if stream not in ("depth", "color"):
            raise ValueError("stream must be depth or color")
        setup = _setup or d435_setup(serial, depth_profile, color_profile, parameters, ros_args)
        serial = setup["config"]["serial"]
        if stream not in setup["enabled"]:
            raise ValueError(f"D435 {stream} stream is disabled by ROS parameters")
        driver = Driver(
            "realsense2_camera",
            "realsense2_camera_node",
            setup["options"],
            key=f"driver:d435:{serial}",
            log_path=log_path
            ) if start_driver else None
        topic = setup["topics"][stream]
        super().__init__(
            topic,
            "image",
            clock=f"device:d435:{serial}",
            driver=driver,
            history=history
            )
