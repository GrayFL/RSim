"""Optional UVC-to-ROS adapter, run inside a worker because capture.read blocks."""
import time

from .core import Component, ComponentError
from .ros import RosContext, RosSensor


class UvcPublisher(Component):
    def __init__(self, device, width, height, fps, topic):
        super().__init__(RosContext(), key=f"driver:uvc:{device}")
        self.device, self.width, self.height, self.fps, self.topic = device, width, height, fps, topic
        self.capture = self.publisher = None
        self._device_lease = None

    def configuration(self):
        return super().configuration(), self.device, self.width, self.height, self.fps, self.topic

    async def open(self):
        import cv2
        from sensor_msgs.msg import Image
        from rclpy.qos import qos_profile_sensor_data
        from .lifecycle import acquire_device
        self._device_lease = acquire_device(self.key)
        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise ComponentError(f"cannot open UVC camera: {self.device}")
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.publisher = self.children[0].node.create_publisher(Image, self.topic, qos_profile_sensor_data)
        self.task("capture", self.read, hz=self.fps)

    async def read(self):
        from sensor_msgs.msg import Image
        ok, pixels = self.capture.read()
        if not ok:
            raise ComponentError("UVC capture failed")
        now = time.time_ns()
        message = Image()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(now, 10**9)
        message.header.frame_id = "uvc_camera"
        message.height, message.width = pixels.shape[:2]
        message.encoding = "bgr8"
        message.step = message.width * 3
        message.data = pixels.tobytes()
        self.publisher.publish(message)

    async def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if self.publisher is not None:
            self.children[0].node.destroy_publisher(self.publisher)
            self.publisher = None
        if self._device_lease is not None:
            self._device_lease.close()
            self._device_lease = None


class UvcCamera(RosSensor):
    def __init__(self, device="/dev/video0", *, width=640, height=640, fps=15, history=8):
        topic = "/rsim/uvc/" + device.rsplit("/", 1)[-1] + "/image_raw"
        super().__init__(topic, "image", clock="host:unix", history=history,
                         driver=UvcPublisher(device, width, height, fps, topic))
