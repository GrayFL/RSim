"""Wheel/IMU/optional scan fusion with pure recipes and local injection."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar

from rsim.runtime.sharing import PortSpec, SharedComponent, SharedProvider, validate_component_ports
from .configuration import (absolute_name, connect_source, fingerprint, freeze, freeze_fields,
                            from_config, mount_data, plain, positive, require_keys, source_reference, shared_ros_context)


@dataclass(frozen=True)
class OdometryFactory:
    imu_mount: object
    directory: str = field(default='.', metadata={'fingerprint': False})
    sources: object = None
    wheel_topic: str = '/rsim/chassis/odom'
    imu_topic: str = '/rsim/chassis/imu/data'
    scan_topic: str | None = None
    scan_mount: object = None
    prefix: str = '/rsim/odometry'
    world_frame: str = 'odom_fused'
    imu_options: object = None
    scan_parameters: object = None
    parameters: object = None
    input_timeout: float = .3
    history: int = 512
    use_orientation: bool = True
    use_acceleration: bool = True
    lateral_acceleration: bool = False
    schema_version: int = 1
    FACTORY_VERSION: ClassVar[str] = 'odometry-factory-v1'
    INTERFACE_VERSION: ClassVar[str] = 'odometry-ports-v1'

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError('unsupported odometry schema_version')
        absolute_name(self.prefix, 'prefix')
        for name in ('wheel_topic', 'imu_topic'):
            absolute_name(getattr(self, name), name)
        positive(self.history, 'history', integer=True)
        positive(self.input_timeout, 'input_timeout')
        for name in ('use_orientation', 'use_acceleration', 'lateral_acceleration'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} must be boolean')
        object.__setattr__(self, 'imu_mount', mount_data(self.imu_mount, name='imu_mount'))
        if not isinstance(self.world_frame, str) or not self.world_frame or self.world_frame == self.imu_mount['wrd_frame']:
            raise ValueError('world_frame must differ from body frame')
        object.__setattr__(self, 'directory', str(Path(self.directory).expanduser().resolve()))
        inputs = dict(self.sources or {})
        require_keys(inputs, ('wheel', 'imu', 'scan'), 'odometry sources')
        inputs['wheel'] = source_reference(inputs.get('wheel'), kind='odom', default_topic=self.wheel_topic)
        inputs['imu'] = source_reference(inputs.get('imu'), kind='imu', default_topic=self.imu_topic)
        if 'scan' in inputs:
            inputs['scan'] = source_reference(inputs['scan'], kind='odometry')
            if inputs['scan']['kind'] not in ('shared_device', 'shared_sensor'):
                raise ValueError('scan estimator reference must connect a shared provider')
            if self.scan_topic is not None:
                raise ValueError('choose a scan estimator reference or a raw scan topic')
        if self.scan_topic is not None:
            absolute_name(self.scan_topic, 'scan_topic')
            if self.scan_mount is None:
                raise ValueError('scan odometry requires scan_mount')
        if self.scan_mount is not None:
            object.__setattr__(self, 'scan_mount', mount_data(self.scan_mount, name='scan_mount', base='base_footprint'))
        object.__setattr__(self, 'sources', freeze(inputs))
        require_keys(self.imu_options or {}, ('gravity', 'remove_gravity', 'gyro_bias', 'acceleration_bias',
            'orientation_std', 'gyro_std', 'acceleration_std', 'history', 'hz'), 'imu_options')
        for name, value in (self.imu_options or {}).items():
            if name in ('gyro_bias', 'acceleration_bias'):
                if not isinstance(value, (list, tuple)) or len(value) != 3:
                    raise ValueError(f'{name} must contain three finite numbers')
                freeze(value)
                if any(type(item) not in (int, float) for item in value):
                    raise ValueError(f'{name} must contain three finite numbers')
            elif name == 'remove_gravity':
                if type(value) is not bool:
                    raise ValueError('remove_gravity must be boolean')
            else:
                positive(value, name, integer=name == 'history')
        for name in ('frequency', 'sensor_timeout', 'history_length'):
            if name in (self.parameters or {}):
                positive(self.parameters[name], name)
        from rsim.adapters.ros2.arguments import GeneratedParameters
        from rsim.adapters.ros2.odometry import filter_parameters
        GeneratedParameters(filter_parameters(prefix=self.prefix, world_frame=self.world_frame,
            body_frame=self.imu_mount['wrd_frame'], scan=self.scan_topic is not None or 'scan' in inputs,
            parameters=plain(self.parameters), use_orientation=self.use_orientation,
            use_acceleration=self.use_acceleration, lateral_acceleration=self.lateral_acceleration))
        GeneratedParameters(self.scan_parameters or {})
        if self.scan_topic is not None:
            from .scan_odometry import ScanOdometryFactory
            ScanOdometryFactory(mount=plain(self.scan_mount), prefix=self.prefix, odom_frame=self.world_frame,
                                parameters=plain(self.scan_parameters))
        elif self.scan_parameters:
            raise ValueError('scan_parameters require a raw scan_topic')
        freeze_fields(self)

    @classmethod
    def from_config(cls, path, overrides=None):
        return from_config(cls, path, section='odometry', overrides=overrides, path_fields=('directory',))

    @staticmethod
    def port_specs(*, history=512):
        positive(history, 'history', integer=True)
        return {'pose': PortSpec(clock='ros:system', history_capacity=history),
                'odometry': PortSpec(clock='ros:system', history_capacity=history),
                'status': PortSpec(clock='ros:system', history_capacity=32)}

    def fingerprint(self):
        return fingerprint(self)

    def build(self, *, wheel=None, imu=None, scan=None, ros=None,
              pose_history=None, covariance_history=None, scan_input=None):
        from rsim.core import Component
        from rsim.adapters.ros2.odometry import InertialOdometry
        from rsim.components.inertial import BodyIMU
        from .scan_odometry import ScanOdometryFactory
        ros = shared_ros_context(ros, wheel, imu, scan, scan_input)
        wheel = wheel if wheel is not None else connect_source(self.sources['wheel'], kind='odom', ros=ros)
        imu = imu if imu is not None else connect_source(self.sources['imu'], kind='imu', ros=ros)
        body_imu = BodyIMU(imu, plain(self.imu_mount), **(plain(self.imu_options) or {}))
        if scan is None and self.sources.get('scan') is not None:
            scan = Component()
            scan.expose('odometry', connect_source(self.sources['scan'], kind='odometry'))
        if scan is None and self.scan_topic is not None:
            scan = ScanOdometryFactory(mount=plain(self.scan_mount), directory=self.directory,
                prefix=self.prefix, wheel_topic=self.wheel_topic, scan_topic=self.scan_topic,
                odom_frame=self.world_frame, parameters=plain(self.scan_parameters)).build(ros=ros, wheel=wheel, scan=scan_input)
        specs = self.port_specs(history=self.history)
        result = InertialOdometry(wheel, body_imu.imu, ros=ros, scan=scan,
            prefix=self.prefix, world_frame=self.world_frame, body_frame=self.imu_mount['wrd_frame'],
            directory=self.directory, parameters=plain(self.parameters), input_timeout=self.input_timeout,
            use_orientation=self.use_orientation, use_acceleration=self.use_acceleration,
            lateral_acceleration=self.lateral_acceleration, pose_history=pose_history,
            covariance_history=covariance_history, ports=specs)
        result.body_imu = body_imu
        validate_component_ports(result, specs)
        return result

    def shared(self, *, key=None, transport=None, options=None):
        return SharedProvider(factory=self.build, key=key or 'odometry:' + self.prefix,
            ports=self.port_specs(history=self.history), interface_version=self.INTERFACE_VERSION,
            provider_version=self.fingerprint(), transport=transport, options=options)

    def device(self, *, key=None, transport=None, options=None):
        return SharedComponent(key=key or 'odometry:' + self.prefix,
            ports=self.port_specs(history=self.history), interface_version=self.INTERFACE_VERSION,
            transport=transport, options=options)


def Odometry(*, imu_mount, directory, wheel=None, imu=None, scan=None, ros=None,
             pose_history=None, covariance_history=None, scan_input=None, **settings):
    """Build a local estimator; new applications may retain OdometryFactory."""
    if scan is not None and settings.get('scan_topic') is not None:
        raise ValueError('pass a scan estimator or a scan topic')
    return OdometryFactory(imu_mount=imu_mount, directory=directory, **settings).build(
        wheel=wheel, imu=imu, scan=scan, ros=ros,
        pose_history=pose_history, covariance_history=covariance_history, scan_input=scan_input)
