"""Read a configured sensor assembly; default selection opens only the IMU."""
import argparse
import asyncio
import json
from pathlib import Path
import time

import numpy as np

from rsim import Runtime
from rsim.config import load_rig
from rsim.model import Image, PointCloud


async def capture(signal, count, timeout):
    frame = await signal.get(timeout=timeout)
    samples = []
    started = time.monotonic()
    for _ in range(count):
        frame = await signal.get(after=frame.sequence, timeout=timeout)
        samples.append(frame)
    result = {"frames": count, "unique_timestamps": len({f.stamp_ns for f in samples}),
              "clock": frame.clock, "sample_rate_hz": count / (time.monotonic() - started)}
    assert await signal.get(timestamp_ns=frame.stamp_ns, clock=frame.clock) is frame
    if isinstance(frame.data, Image):
        result.update(shape=list(frame.data.pixels.shape), encoding=frame.data.encoding)
    elif isinstance(frame.data, PointCloud):
        result["points"] = frame.data.points.size
    else:
        result["last"] = frame.data
    return result, samples


async def run(args):
    out = Path(__file__).resolve().parents[1] / "assets" / "hipnuc"
    out.mkdir(parents=True, exist_ok=True)
    overrides = {"imu": {"mode": args.mode, "log_path": str(out / "imu-driver.log")}}
    if args.port:
        overrides["imu"]["port"] = args.port
    rig = load_rig(args.config, select=args.select, overrides=overrides)
    report = {"mode": args.mode, "sensors": {}}
    async with Runtime(rig):
        streams = [(name, port, signal) for name, sensor in rig.sensors.items()
                   for port, signal in sensor.outputs.items()]
        results = await asyncio.gather(*(capture(signal, args.frames, 30) for _, _, signal in streams))
        for (name, port, _), (result, frames) in zip(streams, results):
            report["sensors"][name + "." + port] = result
            if port == "imu":
                values = np.array([[f.stamp_ns * 1e-9,
                                    *[f.data["linear_acceleration"][k] for k in "xyz"],
                                    *[f.data["angular_velocity"][k] for k in "xyz"],
                                    *[f.data["orientation"][k] for k in "xyzw"]] for f in frames])
                np.save(out / (args.mode + "-samples.npy"), values)
        if {"rgb", "lidar3d"} <= rig.sensors.keys():
            report["T_rgb_lidar3d"] = rig.transform("rgb", "lidar3d").matrix.tolist()
    (out / (args.mode + "-capture.json")).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1]/"configs/sensors.yaml")
    parser.add_argument("--select", nargs="+", default=["imu"])
    parser.add_argument("--mode", choices=["serial", "ros2"], default="serial")
    parser.add_argument("--port")
    parser.add_argument("--frames", type=int, default=200)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("frames must be positive")
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
