"""Descriptor transport and local payload storage; backends load on open."""
from .descriptor import DescriptorTransport, TransportConfig, transport_config
from .shared import allocate

__all__ = ['DescriptorTransport', 'TransportConfig', 'transport_config', 'allocate']
