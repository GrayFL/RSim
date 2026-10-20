"""MJCF assembly. +X forward, +Y left, +Z up; wheel axes are +Y."""
from pathlib import Path
import os
import tempfile
import xml.etree.ElementTree as ET

from .config import RobotConfig, SensorConfig
from .scene import _numbers


def build_model_xml(scene_dir, robot: RobotConfig, sensors: SensorConfig, *, return_text=False):
    scene_dir = Path(scene_dir).resolve()
    root = ET.Element("mujoco", model="indoor_diffdrive")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", meshdir=".", texturedir=".")
    ET.SubElement(root, "option", timestep=str(robot.timestep), gravity="0 0 -9.81",
                  integrator="implicitfast", cone="elliptic", iterations="50")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(sensors.width), offheight=str(sensors.height))
    ET.SubElement(visual, "map", znear="0.001", zfar="5")
    ET.SubElement(visual, "headlight", ambient="0.6 0.6 0.6", diffuse="0.6 0.6 0.6", specular="0 0 0")
    ET.SubElement(root, "statistic", extent="10", meansize="0.1")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "geom", solref="0.01 1", solimp="0.95 0.99 0.001", margin="0.001")
    robot_defaults = ET.SubElement(defaults, "default", **{"class": "robot"})
    ET.SubElement(robot_defaults, "geom", contype="2", conaffinity="1", group="1",
                  friction="1 0.005 0.0001", condim="4")
    scene = ET.parse(scene_dir / "scene.xml").getroot()
    root.append(scene.find("asset"))
    world = scene.find("worldbody")
    root.append(world)
    body = ET.SubElement(world, "body", name="base_footprint", pos="0 0 0.005", childclass="robot")
    ET.SubElement(body, "freejoint", name="base_free")
    ET.SubElement(body, "site", name="imu", pos="0 0 0", size="0.005", rgba="0 0 0 0")
    ET.SubElement(body, "geom", name="chassis", type="box", pos="0 0 0.235",
                  size=_numbers([robot.side/2, robot.side/2, 0.065]), mass="12", rgba="0.08 0.35 0.65 1")
    ET.SubElement(body, "geom", name="front_mark", type="box", pos="0.299 0 0.25",
                  size="0.001 0.10 0.025", mass="0.001", contype="0", conaffinity="0", rgba="1 0.7 0.1 1")
    for sign, side in ((1, "left"), (-1, "right")):
        wheel = ET.SubElement(body, "body", name=f"{side}_wheel", pos=_numbers([0, sign*robot.wheel_track/2, robot.wheel_radius]))
        ET.SubElement(wheel, "joint", name=f"{side}_wheel_joint", type="hinge", axis="0 1 0", damping="0.05", armature="0.01")
        ET.SubElement(wheel, "geom", name=f"{side}_tire", type="cylinder",
                      size=_numbers([robot.wheel_radius, robot.wheel_width/2]),
                      euler="1.5707963267948966 0 0", mass="0.6", rgba="0.055 0.06 0.07 1",
                      friction="1.3 0.0001 0.00001", priority="1", condim="3")
    # Two low-friction ball supports, not driven wheels, stabilise pitch.
    for x in (-0.235, 0.235):
        ET.SubElement(body, "geom", type="sphere", pos=f"{x} 0 0.04", size="0.04", mass="0.1",
                      rgba="0.55 0.55 0.55 1", friction="0.001 0.00001 0.00001", priority="1", condim="3")
    ET.SubElement(body, "geom", name="mast", type="cylinder", pos="-0.16 0 0.63", size="0.018 0.33",
                  mass="0.25", rgba="0.15 0.18 0.2 1")
    ET.SubElement(body, "geom", name="camera_mount", type="capsule", fromto="-0.16 0 0.97 0 0 0.97",
                  size="0.015", mass="0.04", rgba="0.15 0.18 0.2 1")
    ET.SubElement(body, "geom", name="lidar_housing", type="cylinder",
                  pos=f"0 0 {sensors.lidar_height-0.015}", size="0.055 0.025", mass="0.15", rgba="0.12 0.15 0.16 1")
    ET.SubElement(body, "site", name="lidar", pos=f"0 0 {sensors.lidar_height}", size="0.006", rgba="1 0.1 0.1 1")
    ET.SubElement(body, "geom", name="camera_housing", type="box",
                  pos=f"-0.025 0 {sensors.camera_height}", size="0.02 0.06 0.025", mass="0.12", rgba="0.1 0.1 0.1 1")
    ET.SubElement(body, "camera", name="rgbd", pos=f"0 0 {sensors.camera_height}",
                  xyaxes="0 -1 0 0 0 1", fovy=str(sensors.vertical_fov))
    actuator = ET.SubElement(root, "actuator")
    for side in ("left", "right"):
        ET.SubElement(actuator, "velocity", name=f"{side}_motor", joint=f"{side}_wheel_joint", kv="12",
                      ctrlrange=f"-{robot.max_wheel_speed} {robot.max_wheel_speed}", forcerange="-8 8")
    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu")
    ET.SubElement(sensor, "accelerometer", name="imu_accel", site="imu")
    ET.indent(root)
    path = scene_dir / "world.xml"
    xml = ET.tostring(root, encoding="unicode", xml_declaration=True)
    # world.xml is an inspectable export. Concurrent simulations may have
    # different sensors; each must compile its own text, never reread this file.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=scene_dir,
                                         prefix=".world-", suffix=".xml", delete=False) as f:
            temporary = Path(f.name)
            f.write(xml)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return xml if return_text else path

