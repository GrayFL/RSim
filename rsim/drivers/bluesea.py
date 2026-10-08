"""BlueSea LDS-50C-2 2D lidar provider using the vendor ROS 2 executable."""

from pathlib import Path

from rsim.runtime.host import SharedSensor
from rsim.adapters.ros2.arguments import RosArguments, effective_settings


def BlueSea(port, *, topic="/rsim/chassis/scan_raw", history=8,
            transport=None, parameters=None, ros_args=None, log_path=None):
    """Start the BlueSea ROS 2 driver and expose its LaserScan as a Signal.

    Requires the vendor ``bluesea2`` ROS 2 package in the driver workspace.
    Parameters are native bluesea2 node parameters; no laser commands are sent
    by this factory itself.
    """
    from rsim.adapters.ros2 import Driver, RosSensor

    resolved = str(Path(port).expanduser().resolve())
    if not Path(resolved).exists():
        raise ValueError("BlueSea serial port does not exist: " + resolved)
    defaults = {
        "type": "uart", "port": resolved, "baud_rate": 500000,
        "scan_topic": topic, "frame_id": "laser_frame",
        "raw_bytes": 3, "output_360": True, "output_scan": True,
        "output_cloud": False, "output_cloud2": False,
        "with_angle_filter": False, "max_dist": 50.0,
        "inverted": True,
    }
    options = RosArguments({**defaults, **(parameters or {})}, ros_args)
    values, topics = effective_settings(
        options, "bluesea_node", "/",
        lambda value: {"scan": value["scan_topic"]},
    )
    if values["type"] != "uart" or not values["output_scan"]:
        raise ValueError("BlueSea requires UART mode and scan output")
    configured_port = str(Path(values["port"]).expanduser().resolve())
    if configured_port != resolved:
        raise ValueError("BlueSea port override conflicts with source identity")
    if log_path is not None:
        log_path = str(Path(log_path).expanduser().resolve())
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)

    def factory():
        driver = Driver("bluesea2", "bluesea2_node", options,
                        key="bluesea-port:" + resolved, log_path=log_path)
        return RosSensor(topics["scan"], "scan", clock="ros:system",
                         driver=driver, hz=100, history=history)

    source = SharedSensor(
        factory, key="bluesea:" + resolved, version="bluesea-v1",
        history=history, transport=transport,
        provider_version=options.signature(), output_name="scan",
    )
    source.scan = source.output
    return source
