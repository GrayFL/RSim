"""Construct a direct native ROS2 chassis provider without SSH."""
from pathlib import Path


def STM32(port=None, *, namespace="/rsim/chassis", motion_enabled=False,
          history=128, parameters=None, ros_args=None, log_path=None, start_driver=True):
    """Expose odom/state Signals and a velocity CommandSink.

    Native ROS parameters and remaps pass through unchanged. Commands and the
    native driver must run on the same host (shared CLOCK_MONOTONIC). Place this
    component in a ProcessPlacement to isolate ROS from the application loop.
    An external IMU is composed separately; this MCU protocol contains no IMU.
    """
    from rsim.adapters.ros2 import Driver
    from rsim.adapters.ros2.arguments import RosArguments, effective_settings
    from rsim.adapters.ros2.stm32 import Stm32Chassis
    defaults = dict(port=str(port or ""), motion_enabled=motion_enabled)
    options = RosArguments({**defaults, **(parameters or {})}, ros_args,
                           {"__ns": namespace, "__node": "stm32_driver"})
    values, topics = effective_settings(options, "stm32_driver", "/", lambda _: {
        name: name for name in ("odom", "diagnostics", "set_velocity", "stop")})
    if not values["port"]:
        raise ValueError("STM32 requires an explicit serial port")
    resolved_port = str(Path(values["port"]).expanduser().resolve())
    if log_path is not None:
        log_path = Path(log_path).expanduser().resolve()
        log_path.parent.mkdir(parents=True, exist_ok=True)
    driver = Driver("rsim_stm32", "stm32_node", options,
                    key="stm32-port:" + resolved_port, log_path=log_path) if start_driver else None
    return Stm32Chassis(topics=topics, driver=driver, history=history,
                       max_ttl=values.get("max_command_ttl", .5))
