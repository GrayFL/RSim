"""Native DDS backend with ROS-compatible String wire format, without ROS."""
from dataclasses import dataclass

from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import final, cdrv0
from cyclonedds.pub import DataWriter
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration


@dataclass
@final
@cdrv0
class DescriptorString(IdlStruct, typename="std_msgs::msg::dds_::String_"):
    data: str


def descriptor_qos():
    return Qos(
        Policy.Reliability.Reliable(
            max_blocking_time=duration(milliseconds=50)
            ),
        Policy.Durability.TransientLocal,
        Policy.History.KeepLast(16),
        Policy.DataRepresentation(use_cdrv0_representation=True)
        )


class Endpoint:

    def __init__(self, owner, topic, entity, callback=None):
        self.owner, self.topic, self.entity, self.callback = owner, topic, entity, callback

    def publish(self, text):
        self.entity.write(DescriptorString(text))

    def close(self):
        if self.entity is not None:
            # Cyclone Python exposes deterministic entity deletion via __del__;
            # its guard makes the later GC invocation a no-op.
            self.entity.__del__()
            self.topic.__del__()
            self.entity = self.topic = None
            self.owner.endpoints.remove(self)


class CycloneTransport:

    def __init__(self, domain_id):
        self.domain_id = domain_id
        self.participant = None
        self.endpoints = []

    def open(self):
        self.participant = DomainParticipant(self.domain_id)

    def _endpoint(self, name, callback=None):
        topic = Topic(self.participant, "rt" + name, DescriptorString)
        try:
            entity = (DataReader if callback is not None else DataWriter
                     )(self.participant, topic, qos=descriptor_qos())
        except BaseException:
            topic.__del__()
            raise
        endpoint = Endpoint(self, topic, entity, callback)
        self.endpoints.append(endpoint)
        return endpoint

    def subscribe(self, topic, callback):
        return self._endpoint(topic, callback)

    def publisher(self, topic):
        return self._endpoint(topic)

    def poll(self):
        for endpoint in tuple(self.endpoints):
            if endpoint.callback is not None:
                for message in endpoint.entity.take(32):
                    if isinstance(message, DescriptorString):
                        endpoint.callback(message.data)

    def close(self):
        for endpoint in tuple(self.endpoints):
            endpoint.close()
        if self.participant is not None:
            self.participant.__del__()
            self.participant = None
