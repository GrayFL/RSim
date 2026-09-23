"""Use the currently attached UVC camera through the ROS-backed sensor API."""
import asyncio
import json
from pathlib import Path

from rsim import Camera, Runtime


async def main():
    camera = Camera()
    async with Runtime(camera):
        frame = await camera.get(timeout=20)
        frame = await camera.get(after=frame.sequence, timeout=10)
        image = frame.data
        result = {
            "shape": image.pixels.shape,
            "encoding": image.encoding,
            "stamp_ns": frame.stamp_ns,
            "clock": frame.clock,
            "mean": float(image.pixels.mean()),
            "std": float(image.pixels.std()),
            "read_only": not image.pixels.flags.writeable
            }
        import cv2
        assert cv2.imwrite("assets/camera.jpg", image.pixels)
    Path("assets/camera.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    asyncio.run(main())
