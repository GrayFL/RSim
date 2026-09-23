"""Optional rclpy descriptor backend, sharing the native backend's DDS wire type."""
import uuid

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


def descriptor_qos():
    return QoSProfile(
        depth=16,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
        )


class Endpoint:

    def __init__(self, owner, entity, reader):
        self.owner, self.entity, self.reader = owner, entity, reader

    def publish(self, text):
        self.entity.publish(String(data=text))

    def close(self):
        if self.entity is not None:
            destroy = self.owner.node.destroy_subscription if self.reader else self.owner.node.destroy_publisher
            destroy(self.entity)
            self.entity = None
            self.owner.endpoints.remove(self)


class Ros2Transport:

    def __init__(self, domain_id):
        self.domain_id = domain_id
        self.context = self.node = self.executor = None
        self.endpoints = []

    def open(self):
        self.context = Context()
        rclpy.init(
            args=[],
            context=self.context,
            domain_id=self.domain_id,
            signal_handler_options=SignalHandlerOptions.NO
            )
        self.node = rclpy.create_node(
            "rsim_transport_" + uuid.uuid4().hex, context=self.context
            )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def subscribe(self, topic, callback):
        entity = self.node.create_subscription(
            String,
            topic, lambda msg: callback(msg.data),
            descriptor_qos()
            )
        endpoint = Endpoint(self, entity, True)
        self.endpoints.append(endpoint)
        return endpoint

    def publisher(self, topic):
        endpoint = Endpoint(
            self,
            self.node.create_publisher(String, topic, descriptor_qos()),
            False
            )
        self.endpoints.append(endpoint)
        return endpoint

    def poll(self):
        self.executor.spin_once(timeout_sec=0)

    def close(self):
        for endpoint in tuple(self.endpoints):
            endpoint.close()
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None:
            self.context.try_shutdown()
