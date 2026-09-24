"""Read one shared lidar source; place point-cloud computation in a worker."""
import asyncio
import json
import os
from pathlib import Path
import time

import numpy as np

from rsim import Runtime, ProcessPlacement, allocate
from rsim.compose import Map
from rsim.drivers import RobinW


def summarize(cloud):
    points = cloud.points
    xyz = np.stack([points[name].ravel() for name in ("x", "y", "z")],
                    axis=1)
    finite = xyz[np.isfinite(xyz).all(axis=1)]
    # Voxel aggregation: a representative compute load, executed in a worker.
    voxels, counts = np.unique(np.floor(finite / 0.1).astype(np.int32), axis=0,
                               return_counts=True)
    ranges = allocate(points.shape, np.float32)
    np.square(points["x"], out=ranges)
    ranges += np.square(points["y"])
    ranges += np.square(points["z"])
    np.sqrt(ranges, out=ranges)
    return {
        "voxels": voxels,
        "counts": counts,
        "ranges": ranges,
        "worker_pid": os.getpid(),
        "input_points": int(len(finite))
        }


async def main(ip):
    lidar = RobinW(ip=ip, history=8)
    sensor = Map(lidar.points, summarize, hz=10, history=8)
    delays = []

    async def heartbeat():
        while True:
            start = time.monotonic()
            await asyncio.sleep(0.01)
            delays.append(time.monotonic() - start)

    beat = asyncio.create_task(heartbeat())
    try:
        async with Runtime(sensor.output, placement={sensor: ProcessPlacement("points")}) as runtime:
            frame = await sensor.get(timeout=30)
            following = await sensor.get(after=frame.sequence, timeout=10)
            assert isinstance(following.data["voxels"], np.memmap)
            assert not following.data["voxels"].flags.writeable
            result = {
                "input_points":
                    following.data["input_points"],
                "voxels":
                    len(following.data["voxels"]),
                "main_pid":
                    os.getpid(),
                "compute_pid":
                    following.data["worker_pid"],
                "clock":
                    following.clock,
                "stamp_ns":
                    following.stamp_ns,
                "heartbeat_p95_ms":
                    float(np.percentile(delays, 95) * 1000),
                "heartbeat_max_ms":
                    max(delays) * 1000
                }
        result["closed"] = not runtime._active
        assets = Path(__file__).resolve().parents[1] / "assets"
        assets.mkdir(parents=True, exist_ok=True)
        (assets / "process-robin.json").write_text(
            json.dumps(result, indent=2)
            )
        print(json.dumps(result, indent=2))
    finally:
        beat.cancel()
        await asyncio.gather(beat, return_exceptions=True)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    asyncio.run(main(parser.parse_args().ip))
