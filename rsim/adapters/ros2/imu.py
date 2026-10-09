"""Optional ROS2 publisher for the shared HiPNUC serial decoder."""
import asyncio
import signal

from rsim.core.component import Component
from rsim.runtime.graph import Runtime
from rsim.adapters.hipnuc import SerialIMU


class ImuPublisher(Component):
    def __init__(self, node):
        self.node = node
        defaults = dict(port="", baudrate=460800, frame_id="hipnuc_imu",
                        navigation_frame="device_navigation", gravity=9.8,
                        hz=400., timeout=3., history=128)
        values = {}
        for name, default in defaults.items():
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            values[name] = node.get_parameter(name).value
        self.source = SerialIMU(**values)
        super().__init__(self.source)
        self.previous = 0

    async def open(self):
        from sensor_msgs.msg import Imu
        from rclpy.qos import qos_profile_sensor_data
        self.publisher = self.node.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self.task("publish", self.publish, hz=400)
        self.task("ros", self.spin, hz=200)
        self.node.get_logger().info(f"Reading {self.source.port} at {self.source.baudrate}; reception timestamps")

    async def spin(self):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0)

    async def publish(self):
        from sensor_msgs.msg import Imu
        # Forward all retained frames, not just the latest packet in a read.
        frames = self.source.imu.frames
        for frame in frames:
            if frame.sequence <= self.previous:
                continue
            self.previous = frame.sequence
            msg = Imu()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = frame.data["header"]["frame_id"]
            for field, axes in (("orientation", "xyzw"), ("angular_velocity", "xyz"),
                                ("linear_acceleration", "xyz")):
                for axis in axes:
                    setattr(getattr(msg, field), axis, float(frame.data[field][axis]))
                setattr(msg, field + "_covariance", frame.data[field + "_covariance"])
            self.publisher.publish(msg)

    async def close(self):
        self.node.destroy_publisher(self.publisher)


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("hipnuc_imu", automatically_declare_parameters_from_overrides=True)

    async def run():
        task = asyncio.current_task()
        loop = asyncio.get_running_loop()
        for signum in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(signum, task.cancel)
        try:
            async with Runtime(ImuPublisher(node)) as runtime:
                await runtime.wait()
        except asyncio.CancelledError:
            pass

    try:
        asyncio.run(run())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
