"""Native chassis + local IMU composition from decoded settings."""

import asyncio
from dataclasses import dataclass
import json
import time

from graphmap.pose import Pose

from rsim.runtime.graph import Runtime
from rsim.core.commands import VelocityCommand
from rsim.components.imu_calibration import (
    PlanarIMU,
    PlanarIMUCalibration,
    calibrate_planar_imu,
)
from rsim.components.motion import ChassisController
from rsim.components.odometry import PlanarOdometry
from .stm32 import STM32
from .hipnuc import Hipnuc


def sources(config, *, motion_enabled=False):
    chassis_options = dict(config["chassis"])
    parameters = dict(chassis_options.pop("parameters", {}))
    # One explicit flag controls both control layers.
    parameters["motion_enabled"] = bool(motion_enabled)
    chassis_options.pop("motion_enabled", None)
    chassis = STM32(**chassis_options, parameters=parameters)
    imu_options = dict(config["imu"])
    imu_options["mode"] = "ros2"  # both inputs carry the host ROS system clock
    return chassis, Hipnuc(**imu_options)


@dataclass
class LocalRobot:
    chassis: object
    raw_imu: object
    imu: PlanarIMU
    odometry: PlanarOdometry
    control: ChassisController

    @property
    def pose(self):
        return self.control.pose


def build(config, calibration, *, motion_enabled=False):
    """Build components only; Runtime owns starting and stopping all resources."""
    calibration = PlanarIMUCalibration.from_dict(calibration)
    chassis, raw = sources(config, motion_enabled=motion_enabled)
    imu = PlanarIMU(raw, calibration)
    odometry = PlanarOdometry(chassis.odom, imu, **config.get("ekf", {}))
    options = {**config.get("control", {}), "motion_enabled": bool(motion_enabled)}
    control = ChassisController(
        pose=odometry.pose, velocity=chassis.velocity, **options
    )
    return LocalRobot(chassis, raw, imu, odometry, control)


async def calibrate(config, calibration_path, *, seconds=10.0):
    """Read both IMUs while holding zero; save the limited planar calibration.

    The reference connection is used only here. build() never opens SSH.
    """
    from rsim.adapters.ros1 import Ros1Bridge, SSHConfig

    if not 5 <= seconds <= 120:
        raise ValueError("calibration duration must be between 5 and 120 seconds")
    chassis, local = sources(config)
    reference_config = config["reference"]
    connection = dict(reference_config["connection"])
    if "setup" in connection:
        connection["setup"] = tuple(connection["setup"])
    bridge = Ros1Bridge(SSHConfig(**connection))
    reference = bridge.topic(
        reference_config["topic"], "sensor_msgs/Imu", hz=500, history=2048
    )
    mount = Pose(**reference_config["body_imu"])
    captured = {"local": [], "reference": []}
    previous = {name: 0 for name in captured}
    async with Runtime(chassis, local, reference):
        await asyncio.gather(
            local.get(timeout=10),
            reference.get(timeout=10),
            chassis.odom.get(timeout=10),
        )
        # Discard startup history; only capture the common stationary window.
        previous.update(
            local=(await local.get()).sequence,
            reference=(await reference.get()).sequence,
        )
        until, last_zero = time.monotonic() + seconds, 0.0
        while time.monotonic() < until:
            if time.monotonic() - last_zero > 0.08:
                await chassis.velocity.set(VelocityCommand(), ttl=0.2)
                last_zero = time.monotonic()
            wheel = await chassis.odom.get(timeout=0.5)
            if time.time_ns() - wheel.received_ns > 250_000_000:
                raise ValueError("wheel feedback is stale")
            twist = wheel.data["twist"]["twist"]
            if any(
                abs(twist[field][axis]) > 0.003
                for field in ("linear", "angular")
                for axis in "xyz"
            ):
                raise ValueError("robot moved during stationary calibration")
            for name, source in (("local", local), ("reference", reference)):
                latest = await source.get(timeout=0.5)
                if time.time_ns() - latest.received_ns > 500_000_000:
                    raise ValueError(name + " IMU reception is stale")
                for frame in source.output.frames:
                    if frame.sequence > previous[name]:
                        captured[name].append(frame.data)
                        previous[name] = frame.sequence
            await asyncio.sleep(0.005)
    result = calibrate_planar_imu(
        captured["local"], captured["reference"], reference_to_body=mount
    )
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    calibration_path.write_text(json.dumps(result.to_dict(), indent=2) + "\n")
    return result
