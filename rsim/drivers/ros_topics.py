"""Expose a live ROS 2 topic as a local, connection-only RSim sensor."""

import json

from rsim.runtime.host import SharedSensor


def ROS2Topic(topic, kind, *, history=8, transport=None, clock="ros:system"):
    """Start a local relay for a native ROS 2 publisher on either host.

    The relay receives ROS 2 over DDS and writes its arrays to local shared
    storage. Clients on this host connect with rsim.devices.ROS2Topic.
    """
    if kind not in ("imu", "scan", "odom", "state"):
        raise ValueError("unsupported ROS 2 topic kind")
    if not isinstance(topic, str) or not topic.startswith("/") or "//" in topic:
        raise ValueError("topic must be an absolute ROS 2 name")

    def factory():
        from rsim.adapters.ros2.sensor import RosSensor
        return RosSensor(topic, kind, clock=clock, history=history)

    source = SharedSensor(
        factory,
        key=f"ros2-topic:{kind}:{topic}",
        version="ros2-topic-v1",
        history=history,
        transport=transport,
        provider_version=json.dumps({"clock": clock}, sort_keys=True),
        output_name=kind,
    )
    setattr(source, kind, source.output)
    return source
