"""Optional standard ROS2 topic mirror for a ROS1-backed Chassis."""
from collections import deque

import numpy as np

from .core import Sensor
from .ros import RosContext


def fill_ros2(message, values):
    """Adapt ROS1 Header/time names; retain all other standard message fields."""
    for name, value in values.items():
        if name == "seq" and hasattr(message, "stamp") and hasattr(message, "frame_id"):
            continue  # ROS2 Header has no sequence field.
        name = {"secs": "sec", "nsecs": "nanosec"}.get(name, name)
        current = getattr(message, name)
        if isinstance(value, dict):
            fill_ros2(current, value)
        else:
            setattr(message, name, value.tolist() if isinstance(value, np.ndarray) else value)
    return message


class ChassisROS2(Sensor):
    """Mirror sensor_msgs/Imu, nav_msgs/Odometry, LaserScan and reverse Twist.

    Command subscriptions are volatile, depth one. No command is latched or
    replayed on reconnect. Use a distinct prefix to avoid feedback topic loops.
    """
    def __init__(self, chassis, *, prefix="/chassis", hz=200):
        if not prefix.startswith("/") or prefix == "/":
            raise ValueError("prefix must be a non-root absolute ROS namespace")
        super().__init__(chassis, RosContext(), history=1)
        self.prefix, self.hz = prefix.rstrip("/"), hz
        self.publishers, self.previous = {}, {}
        self.pending = deque(maxlen=1)
        self.subscription = None
        self.frame_sequence = 0

    async def open(self):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        self.types = {"imu": Imu, "odom": Odometry, "scan": LaserScan}
        node = self.children[1].node
        self.previous.clear()
        self.frame_sequence = 0
        self.pending.clear()
        for name, cls in self.types.items():
            self.publishers[name] = node.create_publisher(cls, self.prefix + "/" + name,
                                                         qos_profile_sensor_data)
            self.task("mirror-" + name, self._mirror_callback(name), hz=self.hz)
        self.subscription = node.create_subscription(
            Twist, self.prefix + "/cmd_vel", lambda message: self.pending.append(message),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.task("forward-command", self._command, hz=100)
        self.task("relay-frame", self._frame, hz=self.hz)

    async def _frame(self):
        frame = await self.children[0].get(after=self.frame_sequence)
        await self.publish(frame.data, stamp_ns=frame.stamp_ns, clock=frame.clock,
                           received_ns=frame.received_ns)
        self.frame_sequence = frame.sequence

    def _mirror_callback(self, name):
        async def mirror():
            source = getattr(self.children[0], name)
            frame = await source.get(after=self.previous.get(name, 0))
            message = fill_ros2(self.types[name](), frame.data)
            self.publishers[name].publish(message)
            self.previous[name] = frame.sequence
        return mirror

    async def _command(self):
        if self.pending:
            message = self.pending.popleft()
            data = {name: {axis: getattr(getattr(message, name), axis) for axis in ("x", "y", "z")}
                    for name in ("linear", "angular")}
            chassis = self.children[0]
            await chassis.bridge.publish_message(chassis.cmd_vel_topic, "geometry_msgs/Twist", data)

    async def close(self):
        node = self.children[1].node
        if self.subscription is not None:
            node.destroy_subscription(self.subscription)
            self.subscription = None
        for publisher in self.publishers.values():
            node.destroy_publisher(publisher)
        self.publishers.clear()
        self.pending.clear()
