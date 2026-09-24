"""ROS-free device views. Start providers explicitly through rsim.drivers."""
import json

from ._device_config import d435_config, d435_version
from .core import Sensor, SensorError
from .host import SharedSensor


def RobinW(ip="192.168.199.97", *, history=8, transport=None):
    return SharedSensor(
        key=f"robin:{ip}",
        version="robin-v1",
        history=history,
        transport=transport
        )


class _ImageStream(Sensor):
    """Select fresh source frames; repeated bundle snapshots are not new images."""

    def __init__(self, source, stream, *, hz, history):
        super().__init__(source, history=history)
        self.stream, self.hz = stream, hz
        self.previous = 0
        self.identity = None

    async def open(self):
        self.previous, self.identity = 0, None
        self.task("select-image", self.select, hz=self.hz)

    async def select(self):
        frame = await self.children[0].get(after=self.previous)
        self.previous = frame.sequence
        if self.stream not in frame.data:
            raise SensorError(
                f"D435 provider has no enabled {self.stream} stream"
                )
        sample = frame.data[self.stream]
        identity = sample["clock"], sample["stamp_ns"], sample["received_ns"]
        if identity == self.identity:
            return
        await self.publish(
            sample["data"],
            stamp_ns=sample["stamp_ns"],
            clock=sample["clock"],
            received_ns=sample["received_ns"]
            )
        self.identity = identity


def D435(
        *,
        serial="",
        stream="depth",
        history=8,
        depth_profile="640x480x15",
        color_profile="640x480x15",
        transport=None
    ):
    """Connect a depth or color view to an already running RGB-D provider.

    Both views must agree on profiles/history. Explicit profiles avoid relying
    on USB-dependent driver defaults. The streams are not pixel-aligned.
    """
    config = d435_config(serial, depth_profile, color_profile)
    return _d435_view(config, stream, history, transport=transport)


def _d435_view(
        config,
        stream,
        history,
        *,
        factory=None,
        transport=None,
        provider_version=None
    ):
    if stream not in ("color", "depth"):
        raise ValueError("stream must be color or depth")
    depth_fps = int(config["depth_profile"].split("x")[-1])
    color_fps = int(config["color_profile"].split("x")[-1])
    shared = SharedSensor(
        factory,
        key=f"d435:{config['serial']}",
        version=d435_version(config),
        history=history,
        transport=transport,
        provider_version=provider_version
        )
    return _ImageStream(
        shared,
        stream,
        hz=2 * (depth_fps if stream == "depth" else color_fps),
        history=history
        )


def Camera(
        device="/dev/video0",
        *,
        width=640,
        height=640,
        fps=15,
        history=8,
        transport=None
    ):
    config = {
        "device": device, "width": width, "height": height, "fps": fps
        }
    return SharedSensor(
        key=f"uvc:{device}",
        version=json.dumps(config, sort_keys=True),
        history=history,
        transport=transport
        )
