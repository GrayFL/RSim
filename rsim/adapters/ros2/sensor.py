from collections import deque
import time
from rsim.core import PrimaryComponent
from .context import RosContext
from .conversions import (
    pointcloud_array, image_array, imu_data, scan_data, odom_data,
    diagnostics_data,
)

class RosSensor(PrimaryComponent):

    def __init__(
            self, topic, kind, *, clock, driver=None, hz=200, history=16, ros=None
        ):
        children = (ros if ros is not None else RosContext(), ) + ((driver, ) if driver else ())
        super().__init__(
            *children, key=f"ros:{kind}:{topic}", history=history, output_name=kind, clock=clock
            )
        setattr(self, kind, self.output)
        self.topic, self.kind, self.clock, self.hz = topic, kind, clock, hz
        self.pending = deque(maxlen=2)
        self.subscription = None
        self.dropped = 0

    def configuration(self):
        return (
            super().configuration(),
            self.topic,
            self.kind,
            self.clock,
            self.hz,
            tuple(c.configuration() for c in self.children)
            )

    async def open(self):
        from sensor_msgs.msg import Image as RosImage, PointCloud2, Imu, LaserScan
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import qos_profile_sensor_data
        self.pending.clear()

        def receive(msg):
            if len(self.pending) == self.pending.maxlen:
                self.dropped += 1
            self.pending.append((msg, time.time_ns()))

        self.subscription = self.children[0].node.create_subscription(
            {"points": PointCloud2, "image": RosImage, "imu": Imu,
             "scan": LaserScan, "odom": Odometry,
             "state": DiagnosticArray}[self.kind],
            self.topic,
            receive,
            qos_profile_sensor_data
            )
        self.task("convert", self.convert, hz=self.hz)

    async def convert(self):
        if not self.pending:
            return
        msg, received_ns = self.pending.popleft()
        data = {"points": pointcloud_array, "image": image_array,
                "imu": imu_data, "scan": scan_data, "odom": odom_data,
                "state": diagnostics_data}[self.kind](msg)
        await self.publish(
            data,
            stamp_ns=msg.header.stamp.sec * 10**9
            + msg.header.stamp.nanosec,
            clock=self.clock,
            received_ns=received_ns
            )

    async def close(self):
        if self.subscription is not None:
            self.children[0].node.destroy_subscription(self.subscription)
            self.subscription = None
        self.pending.clear()
