"""ROS-free RGB-D configuration and stream views."""
import json
import re
from rsim.core import PrimaryComponent, ComponentError
from rsim.core.signal import as_signal
from rsim.runtime.host import SharedSensor

def d435_profile(value):
    pattern = r"\s*(\d+)\s*[xX,]\s*(\d+)\s*[xX,]\s*(\d+)\s*"
    match = re.fullmatch(pattern, value) if isinstance(value, str) else None
    if match is None or any(int(n) <= 0 for n in match.groups()):
        raise ValueError("camera profile must be WIDTHxHEIGHTxFPS with positive integers")
    return "x".join(str(int(n)) for n in match.groups())


def d435_config(serial, depth_profile, color_profile):
    if not isinstance(serial, str) or (serial and not re.fullmatch(r"[0-9]+", serial)):
        raise ValueError("serial must contain digits only (without the ROS '_' prefix)")
    return {"serial": serial, "depth_profile": d435_profile(depth_profile),
            "color_profile": d435_profile(color_profile)}


def d435_version(config):
    return "d435-v2:" + json.dumps(config, sort_keys=True)


class _ImageStream(PrimaryComponent):
    """Select fresh source frames; repeated bundle snapshots are not new images."""

    def __init__(self, source, stream, *, hz, history):
        self.source = as_signal(source)
        super().__init__(inputs=(self.source,), history=history, output_name="image")
        self.image = self.output
        self.stream, self.hz = stream, hz
        self.previous = 0
        self.identity = None

    async def open(self):
        self.previous, self.identity = 0, None
        self.task("select-image", self.select, hz=self.hz)

    async def select(self):
        frame = await self.source.get(after=self.previous)
        self.previous = frame.sequence
        if self.stream not in frame.data:
            raise ComponentError(
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
    return d435_view(config, stream, history, transport=transport)


def d435_view(
        config,
        stream,
        history,
        *,
        factory=None,
        transport=None,
        provider_version=None
    ):
    """Shared stream construction used by connection and provider factories."""
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
