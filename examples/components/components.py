"""Multi-output computation, process placement and control using simulated data.

No hardware is opened. The centroid calculation is a demonstration, not SLAM;
the controller drives only the in-memory actuator defined in this file.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import time

import numpy as np

from rsim import (Component, PrimaryComponent, Runtime, Bundle, CommandSink, VelocityCommand,
                  CommandMux, CommandInput, Connect, ProcessPlacement)


class CloudSource(PrimaryComponent):
    def __init__(self, *, size=40000):
        super().__init__(output_name="points", clock="simulation", history=8)
        self.points = self.output
        self.size, self.index = size, 0
        self.rng = np.random.default_rng(7)

    async def open(self):
        self.task("sample", self.sample, hz=20)

    async def sample(self):
        self.index += 1
        points = self.rng.normal(size=(self.size, 3)).astype("f4")
        points[:, 0] += np.sin(self.index / 20)
        await self.points.publish(points, stamp_ns=self.index * 50_000_000, clock="simulation")


class CloudAnalysis(Component):
    def __init__(self, points):
        super().__init__(inputs=(points,))
        self.centroid = self.signal("centroid", history=8, clock="simulation")
        self.voxels = self.signal("voxels", history=8, clock="simulation")
        self.metrics = self.signal("metrics", history=8, clock="simulation")
        self.previous = 0

    async def open(self):
        self.task("analyse", self.analyse, hz=10)

    async def analyse(self):
        frame = await self.inputs[0].get(after=self.previous)
        self.previous = frame.sequence
        started = time.monotonic_ns()
        points = frame.data
        valid = points[np.isfinite(points).all(axis=1)]
        voxels, counts = np.unique(np.floor(valid / .1).astype("i4"), axis=0, return_counts=True)
        await self.centroid.publish(valid.mean(axis=0), stamp_ns=frame.stamp_ns, clock=frame.clock)
        await self.voxels.publish({"cells": voxels, "counts": counts},
                                  stamp_ns=frame.stamp_ns, clock=frame.clock)
        await self.metrics.publish({"pid": os.getpid(), "input_points": len(points),
                                    "compute_ms": (time.monotonic_ns() - started) / 1e6},
                                   stamp_ns=frame.stamp_ns, clock=frame.clock)


class ToyNavigation(Component):
    def __init__(self, position):
        super().__init__(inputs=(position,))
        self.path = self.signal("path", history=8, clock="simulation")
        self.velocity_command = self.signal("velocity_command", history=8, clock="simulation")
        self.previous = 0

    async def open(self):
        self.task("plan", self.plan, hz=20)

    async def plan(self):
        frame = await self.inputs[0].get(after=self.previous)
        self.previous = frame.sequence
        goal = np.array([0., 0., 0.])
        await self.path.publish(np.stack((frame.data, goal)), stamp_ns=frame.stamp_ns, clock=frame.clock)
        command = VelocityCommand(float(np.clip(-frame.data[0], -.2, .2)), 0.)
        await self.velocity_command.publish(command, stamp_ns=frame.stamp_ns, clock=frame.clock)


class SimulatedChassis(Component):
    def __init__(self):
        super().__init__()
        self.state = self.signal("state", history=64, clock="host:monotonic")
        self.velocity = CommandSink(self, "velocity", self.apply, fallback=VelocityCommand())
        self.commands = []

    async def apply(self, envelope):
        value = envelope.value
        self.commands.append(value.linear_x)
        await self.state.publish({"linear_x": value.linear_x, "angular_z": value.angular_z},
                                  stamp_ns=time.monotonic_ns(), clock="host:monotonic")


async def demo(*, use_process=True, frames=10):
    cloud = CloudSource()
    analysis = CloudAnalysis(cloud.points)
    navigation = ToyNavigation(analysis.centroid)
    manual = Component()
    joystick = manual.signal("joystick")
    mux = CommandMux(navigation=CommandInput(navigation.velocity_command, 10, .4),
                     manual=CommandInput(joystick, 50, .2), fallback=VelocityCommand())
    chassis = SimulatedChassis()
    drive = Connect(mux.output, chassis.velocity)
    snapshot = Bundle(centroid=analysis.centroid, voxels=analysis.voxels,
                      metrics=analysis.metrics, path=navigation.path, state=chassis.state)
    placement = {analysis: ProcessPlacement("perception")} if use_process else None
    async with Runtime(snapshot.output, drive, placement=placement):
        frame = await snapshot.get(timeout=20)
        for _ in range(frames):
            frame = await snapshot.get(after=frame.sequence, timeout=5)
        # An explicit manual zero command overrides navigation in this simulator.
        mux.override("manual")
        await joystick.publish(VelocityCommand(), stamp_ns=time.monotonic_ns(), clock="host:monotonic")
        await asyncio.sleep(.08)
        selected = mux.selected
        report = {"main_pid": os.getpid(), "worker": frame.data["metrics"]["data"],
                  "centroid": frame.data["centroid"]["data"].tolist(),
                  "voxel_count": len(frame.data["voxels"]["data"]["cells"]),
                  "path": frame.data["path"]["data"].tolist(), "manual_selected": selected,
                  "commands": list(chassis.commands), "process": use_process}
    assert chassis.commands[-1] == 0.
    report["stopped_on_close"] = True
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local", action="store_true", help="keep exactly the same graph in one event loop")
    parser.add_argument("--frames", type=int, default=10)
    args = parser.parse_args()
    report = asyncio.run(demo(use_process=not args.local, frames=args.frames))
    assets = Path(__file__).resolve().parents[2] / "assets" / "components"
    assets.mkdir(parents=True, exist_ok=True)
    (assets / ("local.json" if args.local else "process.json")).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
