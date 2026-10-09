"""Expose a processed local scan Signal on a native ROS topic."""
from rsim.core import Component
from rsim.core.signal import as_signal
from .chassis import fill_ros2


class RosScanFilter(Component):
    def __init__(self, source, *, ros, topic):
        self.source = as_signal(source)
        if not topic.startswith('/'):
            raise ValueError('filtered scan topic must be absolute')
        self.topic = topic
        super().__init__(ros, inputs=(self.source,), key='scan-filter:'+topic)
        self.expose('scan', self.source)

    def configuration(self):
        return super().configuration(), self.topic, self.source.producer.configuration()

    async def open(self):
        from sensor_msgs.msg import LaserScan
        self.node = self.dependencies[0].node
        self.publisher = self.node.create_publisher(LaserScan, self.topic, 32)
        self.previous = 0
        self.task('native-scan', self.publish, hz=100)

    async def publish(self):
        from sensor_msgs.msg import LaserScan
        await self.source.get()
        for frame in self.source.frames:
            if frame.sequence <= self.previous:
                continue
            self.previous = frame.sequence
            message = fill_ros2(LaserScan(), frame.data)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(frame.stamp_ns, 10**9)
            self.publisher.publish(message)

    async def close(self):
        if hasattr(self, 'publisher'):
            self.node.destroy_publisher(self.publisher)
