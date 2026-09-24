"""Camera / lidar Signals and a process-placed computation feed one snapshot."""
import asyncio
import json
import os
from pathlib import Path

from rsim import Bundle, Map, ProcessPlacement, Runtime
from rsim.drivers import Camera, RobinW as RobinWDriver
from examples.process_robin import summarize


async def main(ip, camera_device):
    lidar = RobinWDriver(ip=ip)
    camera = Camera(device=camera_device)
    voxels = Map(lidar.points, summarize, hz=10)
    stack = Bundle(camera=camera.image, lidar=lidar.points, voxels=voxels.output, hz=10)
    async with Runtime(stack.output, placement={voxels: ProcessPlacement("points")}):
        frame = await stack.get(timeout=30)
        samples = frame.data
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
    assets = Path(__file__).resolve().parents[1] / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    (assets / "stack.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    parser.add_argument("--camera-device", default="/dev/video0")
    args = parser.parse_args()
    asyncio.run(main(args.ip, args.camera_device))
