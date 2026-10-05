import json
from rsim.runtime.host import SharedSensor

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

    def factory():
        from rsim.adapters.uvc import UvcCamera
        return UvcCamera(**config, history=history)

    source = SharedSensor(
        factory,
        key=f"uvc:{device}",
        version=json.dumps(config, sort_keys=True),
        history=history,
        transport=transport, output_name="image"
        )
    source.image = source.output
    return source
