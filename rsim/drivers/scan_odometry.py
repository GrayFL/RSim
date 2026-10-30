"""Immutable scan estimator recipe with one local port declaration."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar

from rsim.runtime.sharing import PortSpec, SharedComponent, SharedProvider, validate_component_ports
from .configuration import (absolute_name, fingerprint, freeze_fields, from_config,
                            connect_source, mount_data, plain, positive, require_keys, source_reference, shared_ros_context)


@dataclass(frozen=True)
class ScanOdometryFactory:
    mount: object
    directory: str = field(default='.', metadata={'fingerprint': False})
    prefix: str = '/rsim/odometry'
    wheel_topic: str | None = None
    scan_topic: str | None = None
    odom_frame: str | None = None
    parameters: object = None
    history: int = 512
    sources: object = None
    schema_version: int = 1
    FACTORY_VERSION: ClassVar[str] = 'scan-odometry-factory-v1'
    INTERFACE_VERSION: ClassVar[str] = 'scan-odometry-ports-v1'

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError('unsupported scan odometry schema_version')
        absolute_name(self.prefix, 'prefix')
        for name in ('wheel_topic', 'scan_topic'):
            if getattr(self, name) is not None:
                absolute_name(getattr(self, name), name)
        if self.odom_frame is not None and (not isinstance(self.odom_frame, str) or not self.odom_frame):
            raise ValueError('odom_frame must be a nonempty frame')
        positive(self.history, 'history', integer=True)
        object.__setattr__(self, 'mount', mount_data(self.mount, base='base_footprint'))
        object.__setattr__(self, 'directory', str(Path(self.directory).expanduser().resolve()))
        from rsim.adapters.ros2.arguments import GeneratedParameters
        from rsim.adapters.ros2.scan_odometry import icp_parameters
        GeneratedParameters(icp_parameters(prefix=self.prefix, body_frame=self.mount['wrd_frame'],
            odom_frame=self.odom_frame, parameters=plain(self.parameters)))
        sources = dict(self.sources or {})
        require_keys(sources, ('wheel', 'scan'), 'scan odometry sources')
        for name, value in sources.items():
            sources[name] = source_reference(value, kind='odom' if name == 'wheel' else 'scan')
        object.__setattr__(self, 'sources', sources)
        freeze_fields(self)

    @classmethod
    def from_config(cls, path, overrides=None):
        return from_config(cls, path, section='scan_odometry', overrides=overrides, path_fields=('directory',))

    @staticmethod
    def port_specs(*, history=512):
        positive(history, 'history', integer=True)
        return {'odometry': PortSpec(clock='ros:system', history_capacity=history)}

    def fingerprint(self):
        return fingerprint(self)

    def build(self, *, ros=None, wheel=None, scan=None):
        from rsim.adapters.ros2.scan_odometry import ScanOdometry
        specs = self.port_specs(history=self.history)
        ros = shared_ros_context(ros, wheel, scan)
        if wheel is None and self.sources.get('wheel') is not None:
            wheel = connect_source(self.sources['wheel'], kind='odom', ros=ros)
        if scan is None and self.sources.get('scan') is not None:
            scan = connect_source(self.sources['scan'], kind='scan', ros=ros)
        result = ScanOdometry(ros, mount=plain(self.mount),
            directory=self.directory, prefix=self.prefix, wheel_topic=self.wheel_topic,
            scan_topic=self.scan_topic, odom_frame=self.odom_frame, parameters=plain(self.parameters), ports=specs,
            wheel=wheel, scan=scan)
        validate_component_ports(result, specs)
        return result

    def shared(self, *, key=None, transport=None, options=None):
        return SharedProvider(factory=self.build, key=key or 'scan-odometry:' + self.prefix,
            ports=self.port_specs(history=self.history), interface_version=self.INTERFACE_VERSION,
            provider_version=self.fingerprint(), transport=transport, options=options)

    def device(self, *, key=None, transport=None, options=None):
        return SharedComponent(key=key or 'scan-odometry:' + self.prefix,
            ports=self.port_specs(history=self.history), interface_version=self.INTERFACE_VERSION,
            transport=transport, options=options)
