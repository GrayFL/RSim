"""ROS-free views of locally relayed ROS 2 topics."""

from rsim.runtime.host import SharedSensor


def ROS2Topic(topic, kind, *, history=8, transport=None):
    """Connect to a relay started by rsim.drivers.ROS2Topic on this host."""
    if kind not in ("imu", "scan", "odom", "state"):
        raise ValueError("unsupported ROS 2 topic kind")
    if not isinstance(topic, str) or not topic.startswith("/") or "//" in topic:
        raise ValueError("topic must be an absolute ROS 2 name")
    source = SharedSensor(
        key=f"ros2-topic:{kind}:{topic}",
        version="ros2-topic-v1",
        history=history,
        transport=transport,
        output_name=kind,
    )
    setattr(source, kind, source.output)
    return source
