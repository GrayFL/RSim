"""Explicit provider recipes for existing native ROS 2 topics."""
from dataclasses import dataclass
import json
from typing import ClassVar

from rsim.runtime.host import SharedSensor
from rsim.runtime.sharing import PortSpec, validate_component_ports
from .configuration import fingerprint


@dataclass(frozen=True)
class ROS2TopicFactory:
    topic: str
    kind: str
    history: int = 8
    clock: str = 'ros:system'
    FACTORY_VERSION: ClassVar[str] = 'ros2-topic-factory-v1'

    def __post_init__(self):
        if self.kind not in ('imu', 'scan', 'odom', 'state'):
            raise ValueError('unsupported ROS 2 topic kind')
        if not isinstance(self.topic, str) or not self.topic.startswith('/') or '//' in self.topic:
            raise ValueError('topic must be an absolute ROS 2 name')
        if not isinstance(self.clock, str) or not self.clock:
            raise ValueError('clock must name the source time domain')
        self.port_specs(self.kind, clock=self.clock, history=self.history)

    @staticmethod
    def port_specs(kind, *, clock='ros:system', history=8):
        return {kind: PortSpec(clock=clock, history_capacity=history)}

    def fingerprint(self):
        return fingerprint(self)

    def build(self):
        from rsim.adapters.ros2.sensor import RosSensor
        specs = self.port_specs(self.kind, clock=self.clock, history=self.history)
        spec = specs[self.kind]
        source = RosSensor(self.topic, self.kind, clock=spec.clock, history=spec.history_capacity)
        validate_component_ports(source, specs)
        return source

    def shared(self, *, transport=None):
        source = SharedSensor(self.build, key=f'ros2-topic:{self.kind}:{self.topic}',
                              version='ros2-topic-v1', history=self.history, transport=transport,
                              provider_version=json.dumps({'clock': self.clock}, sort_keys=True),
                              output_name=self.kind)
        setattr(source, self.kind, source.output)
        return source

    def device(self, *, transport=None):
        from rsim.devices.ros_topics import ROS2Topic as TopicDevice
        return TopicDevice(self.topic, self.kind, history=self.history, transport=transport)


def ROS2Topic(topic, kind, *, history=8, transport=None, clock='ros:system'):
    """Compatibility wrapper starting a relay only when its Runtime is opened."""
    return ROS2TopicFactory(topic, kind, history, clock).shared(transport=transport)
