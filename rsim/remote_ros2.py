"""Optional standard ROS2 topic mirror for a ROS1-backed Chassis."""
from collections import deque

import numpy as np

from .core import Component
from .commands import Connect, VelocityCommand
from .ros import RosContext
import time


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


class ChassisROS2(Component):
    """Mirror sensor_msgs/Imu, nav_msgs/Odometry, LaserScan and reverse Twist.

    Command subscriptions are volatile, depth one. No command is latched or
    replayed on reconnect. Use a distinct prefix to avoid feedback topic loops.
    """
    def __init__(self, chassis, *, prefix="/chassis", hz=200, forward_commands=True):
        if not prefix.startswith("/") or prefix == "/":
            raise ValueError("prefix must be a non-root absolute ROS namespace")
        super().__init__(chassis, RosContext())
        self.velocity_command = self.signal("velocity_command", history=1, clock="host:monotonic")
        if forward_commands:
            self.dependencies += (Connect(self.velocity_command, chassis.velocity),)
        for name in ("imu", "odom", "scan", "state", "velocity_feedback"):
            output = self.signal(name)
            output._target = getattr(chassis, name)
            setattr(self, name, output)
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
            if any((message.linear.y, message.linear.z, message.angular.x, message.angular.y)):
                raise ValueError("planar chassis accepts only linear.x and angular.z")
            await self.velocity_command.publish(VelocityCommand(message.linear.x, message.angular.z),
                                                 stamp_ns=time.monotonic_ns(), clock="host:monotonic")

    async def close(self):
        node = self.children[1].node
        if self.subscription is not None:
            node.destroy_subscription(self.subscription)
            self.subscription = None
        for publisher in self.publishers.values():
            node.destroy_publisher(publisher)
        self.publishers.clear()
        self.pending.clear()
