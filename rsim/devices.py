"""Public device factories with host-wide source reuse and isolated ingress."""
import json
from pathlib import Path

from .compose import Bundle
from .core import Sensor
from .host import SharedSensor


def RobinW(ip="192.168.199.97", *, history=8):
    from .ros import RobinW as Source
    return SharedSensor(lambda: Source(ip, history=history), key=f"robin:{ip}",
                        version="robin-v1", history=history)


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
        sample = frame.data[self.stream]
        identity = sample["clock"], sample["stamp_ns"], sample["received_ns"]
        if identity == self.identity:
            return
        await self.publish(sample["data"], stamp_ns=sample["stamp_ns"],
                           clock=sample["clock"], received_ns=sample["received_ns"])
        self.identity = identity


def D435(*, serial="", stream="depth", history=8,
         depth_profile="640x480x15", color_profile="640x480x15", log_path=None):
    """A depth or color view over one shared RGB-D capture process.

    Both views must agree on profiles/history. Explicit profiles avoid relying
    on USB-dependent driver defaults. The streams are not pixel-aligned.
    """
    from .ros import D435 as Source, d435_profile
    if stream not in ("color", "depth"):
        raise ValueError("stream must be color or depth")
    depth_profile, color_profile = d435_profile(depth_profile), d435_profile(color_profile)
    config = {"serial": serial, "depth_profile": depth_profile, "color_profile": color_profile}
    log_path = str(Path(log_path).resolve()) if log_path is not None else None
    depth_fps, color_fps = int(depth_profile.split("x")[-1]), int(color_profile.split("x")[-1])
    def factory():
        return Bundle(color=Source(**config, stream="color", history=history, log_path=log_path),
                      depth=Source(**config, stream="depth", history=history, log_path=log_path),
                      hz=2 * max(depth_fps, color_fps), history=history)
    shared = SharedSensor(factory, key=f"d435:{serial}",
                          version="d435-v2:" + json.dumps(config, sort_keys=True), history=history)
    return _ImageStream(shared, stream,
                        hz=2 * (depth_fps if stream == "depth" else color_fps), history=history)


def Camera(device="/dev/video0", *, width=640, height=640, fps=15, history=8):
    from .uvc import UvcCamera
    config = {"device": device, "width": width, "height": height, "fps": fps}
    return SharedSensor(lambda: UvcCamera(**config, history=history), key=f"uvc:{device}",
                        version=json.dumps(config, sort_keys=True), history=history)
