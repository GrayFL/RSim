"""Camera + shared lidar + nested point processing, all accessed as sensors."""
import asyncio
import json
import os
from pathlib import Path

from rsim import Bundle, Camera, Map, ProcessSensor, RobinW, Runtime
from examples.process_robin import summarize


def processing():
    source = RobinW()

    def compute(cloud):
        data = summarize(cloud)
        data["source_pid"] = source.worker_pid
        return data

    return Map(source, compute, hz=10)


async def main():
    lidar = RobinW()
    stack = Bundle(
        camera=Camera(),
        lidar=lidar,
        voxels=ProcessSensor(processing),
        hz=10
        )
    async with Runtime(stack):
        frame = await stack.get(timeout=30)
        samples = frame.data
        assert lidar.worker_pid == samples["voxels"]["data"]["source_pid"]
        drivers = []
        for path in Path("/proc").glob("[0-9]*/cmdline"):
            try:
                args = path.read_bytes().split(b"\0")
            except FileNotFoundError:
                continue
            if args and args[0].endswith(b"/seyond_node"):
                drivers.append(int(path.parent.name))
        assert len(drivers) == 1, drivers
        result = {
            "camera_shape": samples["camera"]["data"].pixels.shape,
            "lidar_points": samples["lidar"]["data"].points.size,
            "voxels": len(samples["voxels"]["data"]["voxels"]),
            "source_pid": lidar.worker_pid,
            "compute_pid": samples["voxels"]["data"]["worker_pid"],
            "main_pid": os.getpid(),
            "seyond_driver_pids": drivers,
            "sample_clocks": {
                name: item["clock"]
                for name, item in samples.items()
                }
            }
    Path("assets/stack.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    asyncio.run(main())
