from pathlib import Path
from rsim.runtime.host import SharedSensor
from rsim.adapters.ros2 import Driver, RosSensor
from rsim.adapters.ros2.arguments import RosArguments, effective_settings

def robin_setup(ip, topic="/iv_points", parameters=None, ros_args=None):
    options = RosArguments({"lidar_ip": ip, "frame_topic": topic, **(parameters or {})}, ros_args)
    values, topics = effective_settings(options, "seyond", "/", lambda p: {"points": p["frame_topic"]})
    if not isinstance(values["lidar_ip"], str) or not values["lidar_ip"]:
        raise ValueError("lidar_ip must be a nonempty ROS string")
    return {"ip": values["lidar_ip"], "options": options, "topic": topics["points"]}


class RobinWSource(RosSensor):

    def __init__(
            self,
            ip="192.168.199.97",
            *,
            start_driver=True,
            topic="/iv_points",
            history=16,
            log_path=None,
            parameters=None,
            ros_args=None,
            _setup=None
        ):
        setup = _setup or robin_setup(ip, topic, parameters, ros_args)
        ip, topic = setup["ip"], setup["topic"]
        driver = Driver(
            "seyond",
            "seyond_node", setup["options"],
            key=f"driver:robin:{ip}",
            log_path=log_path
            ) if start_driver else None
        super().__init__(
            topic,
            "points",
            clock=f"device:robin:{ip}",
            driver=driver,
            history=history
            )


def RobinW(
        ip="192.168.199.97",
        *,
        history=8,
        transport=None,
        parameters=None,
        ros_args=None,
        log_path=None
    ):
    """Start Seyond with native parameters/ROS argv; follow frame_topic/remaps."""
    setup = robin_setup(ip, parameters=parameters, ros_args=ros_args)
    log_path = str(
        Path(log_path).resolve()
        ) if log_path is not None else None

    def factory():
        setup["options"].arguments()
        return RobinWSource(history=history, log_path=log_path, _setup=setup)

    source = SharedSensor(
        factory,
        key=f"robin:{setup['ip']}",
        version="robin-v1",
        history=history,
        transport=transport,
        provider_version=setup["options"].signature(), output_name="points"
        )
    source.points = source.output
    return source
