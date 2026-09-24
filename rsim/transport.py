"""Metered descriptor transport, independent of ROS and array storage.

Both backends use the same DDS domain, topic and CDR String wire type.
Backend dependencies are loaded only when a Runtime opens the transport.
"""
from dataclasses import dataclass, field
import os

from .core import Component


@dataclass(frozen=True)
class TransportConfig:
    backend: str = field(default_factory=lambda: os.environ.get("RSIM_TRANSPORT", "cyclonedds"))
    domain_id: int = field(default_factory=lambda: int(os.environ.get("ROS_DOMAIN_ID", "0")))

    def __post_init__(self):
        if self.backend not in ("cyclonedds", "ros2"):
            raise ValueError("transport backend must be cyclonedds or ros2")
        if not isinstance(self.domain_id, int) or not 0 <= self.domain_id <= 232:
            raise ValueError("domain_id must be an integer between 0 and 232")


def transport_config(value=None):
    if value is None:
        return TransportConfig()
    if isinstance(value, str):
        return TransportConfig(backend=value)
    if not isinstance(value, TransportConfig):
        raise TypeError("transport must be a backend name or TransportConfig")
    return value


class DescriptorTransport(Component):
    """One backend context shared by consumers in a Runtime and DDS domain."""
    process_local = True

    def __init__(self, config=None, *, hz=1000):
        self.config = transport_config(config)
        super().__init__(key=f"rsim:transport:{self.config.backend}:{self.config.domain_id}")
        self.hz, self.backend = hz, None

    def configuration(self):
        return super().configuration(), self.config, self.hz

    async def open(self):
        if self.config.backend == "cyclonedds":
            from .transports.cyclone import CycloneTransport
            self.backend = CycloneTransport(self.config.domain_id)
        else:
            from .transports.ros2 import Ros2Transport
            self.backend = Ros2Transport(self.config.domain_id)
        self.backend.open()
        self.task("descriptor-poll", self.poll, hz=self.hz)

    async def poll(self):
        self.backend.poll()

    def subscribe(self, topic, callback, **options):
        return self.backend.subscribe(topic, callback, **options)

    def publisher(self, topic, **options):
        return self.backend.publisher(topic, **options)

    async def close(self):
        if self.backend is not None:
            self.backend.close()
            self.backend = None
