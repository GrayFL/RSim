from pathlib import Path
from rsim.adapters.ros2 import Driver, RosSensor
from rsim.adapters.ros2.arguments import RosArguments, effective_settings

from rsim.devices.realsense import d435_config, d435_view
from rsim.core.compose import Bundle

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


class D435Source(RosSensor):

    def __init__(
            self,
            *,
            stream="depth",
            serial="",
            start_driver=True,
            history=16,
            log_path=None,
            depth_profile="640x480x15",
            color_profile="640x480x15",
            parameters=None,
            ros_args=None,
            _setup=None
        ):
        if stream not in ("depth", "color"):
            raise ValueError("stream must be depth or color")
        setup = _setup or d435_setup(serial, depth_profile, color_profile, parameters, ros_args)
        serial = setup["config"]["serial"]
        if stream not in setup["enabled"]:
            raise ValueError(f"D435 {stream} stream is disabled by ROS parameters")
        driver = Driver(
            "realsense2_camera",
            "realsense2_camera_node",
            setup["options"],
            key=f"driver:d435:{serial}",
            log_path=log_path
            ) if start_driver else None
        topic = setup["topics"][stream]
        super().__init__(
            topic,
            "image",
            clock=f"device:d435:{serial}",
            driver=driver,
            history=history
            )


def D435(
        *,
        serial="",
        stream="depth",
        history=8,
        depth_profile="640x480x15",
        color_profile="640x480x15",
        log_path=None,
        transport=None,
        parameters=None,
        ros_args=None
    ):
    """Start RGB-D with arbitrary native ROS parameters and argv options.

    Precedence: convenience options < parameters < ros_args assignments.
    Only enabled image streams are collected. Application views must match the
    effective serial/profiles/history, but need no driver-specific parameters.
    """
    setup = d435_setup(
        serial, depth_profile, color_profile, parameters, ros_args
        )
    config = setup["config"]
    if stream not in ("color", "depth"):
        raise ValueError("stream must be color or depth")
    if stream not in setup["enabled"]:
        raise ValueError(
            f"D435 {stream} stream is disabled by ROS parameters"
            )
    log_path = str(
        Path(log_path).resolve()
        ) if log_path is not None else None

    def factory():
        setup["options"].arguments(
        )  # Verify parameter files before sharing a recipe.
        return Bundle(
            **{
                name:
                    D435Source(
                        stream=name,
                        history=history,
                        log_path=log_path,
                        _setup=setup
                        )
                for name in setup["enabled"]
                },
            hz=2 * max(
                int(config[k].split("x")[-1])
                for k in ("depth_profile", "color_profile")
                ),
            history=history
            )

    return d435_view(
        config,
        stream,
        history,
        factory=factory,
        transport=transport,
        provider_version=setup["options"].signature()
        )
