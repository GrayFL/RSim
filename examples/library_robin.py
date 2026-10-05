"""python -m examples.library_robin, or await main() in a notebook."""
import asyncio
import json
from pathlib import Path

import numpy as np

from rsim import Runtime
from rsim.drivers.seyond import RobinWSource as RobinW


async def main():
    sensor = RobinW(log_path="assets/robin-library-driver.log")
    async with Runtime(sensor):
        frames = []
        sequence = None
        for _ in range(5):
            frame = await sensor.get(after=sequence, timeout=30)
            sequence = frame.sequence
            points = frame.data.points
            assert not points.flags.writeable
            assert await sensor.get(
                timestamp_ns=frame.stamp_ns, clock=frame.clock
                ) is frame
            frames.append({
                "sequence": sequence,
                "stamp_ns": frame.stamp_ns,
                "clock": frame.clock,
                "received_ns": frame.received_ns,
                "shape": points.shape,
                "bytes": points.nbytes,
                "finite_x": int(np.isfinite(points["x"]).sum())
                })
    Path("assets/robin-library-frames.json").write_text(
        json.dumps(frames, indent=2)
        )
    print(json.dumps(frames[-1], indent=2))


if __name__ == "__main__":
    asyncio.run(main())
