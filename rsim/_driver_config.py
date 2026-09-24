"""Provider-side hardware setup, separate from ROS-free client contracts."""
from ._device_config import d435_config
from ._ros_args import RosArguments, effective_settings


def d435_setup(serial, depth_profile, color_profile, parameters=None, ros_args=None):
    config = d435_config(serial, depth_profile, color_profile)
    node_name = "d435" + ("_" + serial if serial else "")
    params = {
        "camera_name": node_name, "device_type": "d435(?!i)",
        "enable_depth": True, "enable_color": True,
        "enable_infra1": False, "enable_infra2": False,
        "depth_module.depth_profile": config["depth_profile"],
        "rgb_camera.color_profile": config["color_profile"],
        "wait_for_device_timeout": 15.0,
    }
    if serial:
        params["serial_no"] = "_" + serial
    params.update(parameters or {})
    options = RosArguments(params, ros_args, {"__ns": "/rsim", "__node": node_name})
    values, topics = effective_settings(options, "camera", "/camera", lambda _: {
        "depth": "~/depth/image_rect_raw", "color": "~/color/image_raw"})
    serial = values.get("serial_no", "")
    if not isinstance(serial, str):
        raise ValueError("serial_no must be a ROS string (quote numeric serials)")
    config = d435_config(serial.removeprefix("_"), values["depth_module.depth_profile"],
                         values["rgb_camera.color_profile"])
    enabled = []
    for stream in ("color", "depth"):
        value = values["enable_" + stream]
        if type(value) is not bool:
            raise ValueError(f"enable_{stream} must be a ROS boolean")
        if value:
            enabled.append(stream)
    if not enabled:
        raise ValueError("D435 requires at least one enabled color/depth stream")
    return {"config": config, "options": options, "topics": topics, "enabled": enabled}


def robin_setup(ip, topic="/iv_points", parameters=None, ros_args=None):
    options = RosArguments({"lidar_ip": ip, "frame_topic": topic, **(parameters or {})}, ros_args)
    values, topics = effective_settings(options, "seyond", "/", lambda p: {"points": p["frame_topic"]})
    if not isinstance(values["lidar_ip"], str) or not values["lidar_ip"]:
        raise ValueError("lidar_ip must be a nonempty ROS string")
    return {"ip": values["lidar_ip"], "options": options, "topic": topics["points"]}
