"""Public device factories with host-wide source reuse and isolated ingress."""
import json

from .compose import Bundle, Map
from .core import Frame
from .host import SharedSensor


def RobinW(ip="192.168.199.97", *, history=8):
    from .ros import RobinW as Source
    return SharedSensor(lambda: Source(ip, history=history), key=f"robin:{ip}",
                        version="robin-v1", history=history)


def D435(*, serial="", stream="depth", history=8):
    from .ros import D435 as Source
    if stream not in ("color", "depth"):
        raise ValueError("stream must be color or depth")
    def factory():
        return Bundle(color=Source(serial=serial, stream="color", history=history),
                      depth=Source(serial=serial, stream="depth", history=history), history=history)
    shared = SharedSensor(factory, key=f"d435:{serial}", version="d435-v1", history=history)
    def select(data):
        sample = data[stream]
        return Frame(sample["data"], sample["stamp_ns"], sample["clock"], sample["received_ns"])
    return Map(shared, select)


def Camera(device="/dev/video0", *, width=640, height=640, fps=15, history=8):
    from .uvc import UvcCamera
    config = {"device": device, "width": width, "height": height, "fps": fps}
    return SharedSensor(lambda: UvcCamera(**config, history=history), key=f"uvc:{device}",
                        version=json.dumps(config, sort_keys=True), history=history)
