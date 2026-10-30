"""Native chassis compositions with one ROS owner and immutable worker recipes."""
from dataclasses import dataclass, field, fields, replace
from pathlib import Path
from typing import ClassVar

from rsim.core import Component, CommandSink, CommandRejected, VelocityCommand
from rsim.runtime.sharing import PortSpec, SharedComponent, SharedProvider, validate_component_ports
from .configuration import (absolute_name, connect_source, fingerprint, freeze, freeze_fields,
                            from_config, mount_data, plain, positive, require_keys,
                            resource_digests, restore_setup, snapshot_setup, source_reference, validate_resources, shared_ros_context)


class NativeRobot(Component):
    """Composite ports; the optional local controller has a separate lifetime.

    Runtime(robot) exposes the velocity sink for an external writer. To use
    move/rotate locally open Runtime(robot.control); it claims that same sink.
    Both paths share exactly one lower-level actuator owner.
    """
    def __init__(self, chassis, odometry, *, motion_enabled=False, control=None,
                 scan=None, scan_filter=None):
        super().__init__(chassis, odometry, *((scan.producer,) if scan is not None else ()))
        self.chassis, self.odometry = chassis, odometry
        self.scan, self.scan_filter = scan, scan_filter
        self.motion_enabled = motion_enabled
        self.expose('pose', odometry.pose)
        self.expose('odom', odometry.odometry)
        self.expose('state', chassis.state)
        self.velocity = CommandSink(self, 'velocity', self._apply, safe=self._safe,
            fallback=VelocityCommand(), max_ttl=chassis.velocity.guard.max_ttl_ns / 1e9)
        self.command_targets = (chassis.velocity,)
        from rsim.components.motion import ChassisController
        self.control = ChassisController(pose=self.pose, velocity=self.velocity,
                                         motion_enabled=motion_enabled, **(control or {}))

    async def _apply(self, envelope):
        if envelope.source_bound is not None:
            raise CommandRejected('native chassis has no simulation source context for source_bound')
        if not isinstance(envelope.value, VelocityCommand):
            raise CommandRejected('velocity requires VelocityCommand')
        if not self.motion_enabled and envelope.value != VelocityCommand():
            raise CommandRejected('nonzero velocity requires motion_enabled=True')
        return await self.chassis.velocity.set(envelope, _writer=self)

    async def _safe(self, _):
        await self.chassis.velocity._safe()


_SENSOR_FIELDS = {'topic', 'start_driver', 'port', 'parameters', 'ros_args', 'filter'}
_CONTROL_FIELDS = {'hz', 'max_linear', 'max_angular', 'min_angular', 'linear_acceleration',
                   'angular_acceleration', 'distance_tolerance', 'angle_tolerance',
                   'pose_timeout', 'command_ttl', 'settle_samples'}


def _sensor_settings(value, kind, *, topic_only=False):
    if value is None:
        return None
    settings = dict(value)
    require_keys(settings, {'topic', 'start_driver', 'filter'} if topic_only else _SENSOR_FIELDS,
                 kind + (' topic' if topic_only else ''))
    absolute_name(settings.get('topic'), kind + '.topic')
    start = settings.setdefault('start_driver', False)
    if type(start) is not bool:
        raise ValueError(kind + '.start_driver must be boolean')
    if topic_only and start:
        raise ValueError(kind + '.start_driver is forbidden in a topic service')
    if start and not settings.get('port'):
        raise ValueError(kind + ' native launch requires a serial port')
    if not start and (settings.get('parameters') or settings.get('ros_args')):
        raise ValueError(kind + ' parameters/ros_args require an owned driver; configure the existing owner separately')
    if settings.get('port') is not None:
        settings['port'] = str(Path(settings['port']).expanduser().resolve())
    if kind == 'imu' and 'filter' in settings:
        raise ValueError('filter is only supported for scan')
    if 'filter' in settings:
        require_keys(settings['filter'], {'topic', 'angle_masks', 'history'}, 'scan filter')
        absolute_name(settings['filter'].get('topic'), 'filtered scan topic')
        if settings['filter']['topic'] == settings['topic']:
            raise ValueError('filtered scan topic must differ from raw input')
        import math
        masks = settings['filter'].get('angle_masks')
        if (not isinstance(masks, (list, tuple)) or not masks or any(
                not isinstance(pair, (list, tuple)) or len(pair) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in pair)
                or not -math.pi <= pair[0] < pair[1] <= math.pi for pair in masks)):
            raise ValueError('angle_masks must be ordered radian pairs within [-pi, pi]')
        positive(settings['filter'].get('history', 32), 'scan filter history', integer=True)
    from rsim.adapters.ros2.arguments import RosArguments
    RosArguments(settings.get('parameters'), settings.get('ros_args'))
    return settings


def _sensor_setup(settings, kind, frame):
    from rsim.adapters.ros2.arguments import RosArguments, effective_settings
    topic = settings['topic']
    defaults = (dict(baudrate=460800, frame_id=frame, navigation_frame='enu') if kind == 'imu' else
        dict(type='uart', baud_rate=500000, frame_id=frame, raw_bytes=3,
             output_360=True, output_scan=True, output_cloud=False, output_cloud2=False,
             with_angle_filter=False, max_dist=50., inverted=True, scan_topic=topic))
    parameters = {**defaults, **settings.get('parameters', {})}
    if settings.get('port') is not None:
        parameters['port'] = settings['port']
    options = RosArguments(parameters, settings.get('ros_args'), {'imu/data': topic} if kind == 'imu' else {})
    values, topics = effective_settings(options, 'hipnuc_imu' if kind == 'imu' else 'bluesea_node', '/',
                                        lambda p: {kind: 'imu/data' if kind == 'imu' else p['scan_topic']})
    if settings['start_driver'] and str(Path(values.get('port', '')).expanduser().resolve()) != settings['port']:
        raise ValueError(kind + ' port override conflicts with its explicit owner')
    if values.get('use_sim_time', False):
        raise ValueError('native chassis requires ROS system time')
    if values.get('frame_id') != frame:
        raise ValueError(kind + ' frame override differs from its mount')
    return dict(options=options, values=values, topics=topics)


@dataclass(frozen=True)
class NativeChassisFactory:
    stm32: object
    imu: object
    imu_mount: object
    directory: str = field(default='.', metadata={'fingerprint': False})
    scan: object = None
    scan_mount: object = None
    estimator: str = 'wheel_imu'
    odometry: object = None
    control: object = None
    motion_enabled: bool = False
    sources: object = None
    name: str = 'chassis'
    schema_version: int = 1
    _prepared: object = field(default=None, repr=False)
    _resources: object = field(init=False, repr=False)
    FACTORY_VERSION: ClassVar[str] = 'native-chassis-factory-v1'
    INTERFACE_VERSION: ClassVar[str] = 'native-chassis-ports-v1'
    TOPIC_ONLY: ClassVar[bool] = False

    def __post_init__(self):
        from .stm32 import STM32Factory
        from .odometry import OdometryFactory
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError('unsupported chassis schema_version')
        if type(self.motion_enabled) is not bool:
            raise ValueError('motion_enabled must be boolean')
        if not isinstance(self.name, str) or not self.name or '/' in self.name:
            raise ValueError('name must be a nonempty identifier without slash')
        if self.estimator not in ('wheel_imu', 'scan_imu'):
            raise ValueError('estimator must be wheel_imu or scan_imu')
        object.__setattr__(self, 'imu_mount', mount_data(self.imu_mount, name='imu_mount', base='base_footprint'))
        object.__setattr__(self, 'directory', str(Path(self.directory).expanduser().resolve()))
        if self.scan_mount is not None:
            object.__setattr__(self, 'scan_mount', mount_data(self.scan_mount, name='scan_mount', base='base_footprint'))
        if self.estimator == 'scan_imu' and (self.scan is None or self.scan_mount is None):
            raise ValueError('scan_imu requires scan settings and scan_mount')
        motor = dict(self.stm32)
        allowed = ({'namespace', 'start_driver', 'remote_clock', 'history'} if self.TOPIC_ONLY else
                   {f.name for f in fields(STM32Factory)} - {'motion_enabled', 'log_path', '_prepared'})
        require_keys(motor, allowed, 'stm32')
        if self.TOPIC_ONLY:
            if motor.get('start_driver', False):
                raise ValueError('stm32.start_driver is forbidden in a topic service')
            motor.update(start_driver=False, remote_clock=motor.get('remote_clock', True))
        if type(motor.get('start_driver', True)) is not bool or type(motor.get('remote_clock', False)) is not bool:
            raise ValueError('stm32 start_driver and remote_clock must be boolean')
        absolute_name(motor.get('namespace', '/rsim/chassis'), 'stm32.namespace')
        positive(motor.get('history', 128), 'stm32.history', integer=True)
        if motor.get('port') is not None:
            motor['port'] = str(Path(motor['port']).expanduser().resolve())
        parameters = motor.get('parameters') or {}
        if 'motion_enabled' in parameters and (type(parameters['motion_enabled']) is not bool
                                               or parameters['motion_enabled'] != self.motion_enabled):
            raise ValueError('stm32 motion_enabled must match the composite permission')
        if motor.get('port') and parameters.get('port') and str(Path(parameters['port']).expanduser().resolve()) != motor['port']:
            raise ValueError('STM32 port override conflicts with its explicit owner')
        if parameters.get('use_sim_time', False):
            raise ValueError('native chassis requires ROS system time')
        positive(parameters.get('max_command_ttl', .5), 'max_command_ttl')
        if motor.get('start_driver', True) and not (motor.get('port') or parameters.get('port')):
            raise ValueError('STM32 requires an explicit serial port')
        object.__setattr__(self, 'stm32', motor)
        object.__setattr__(self, 'imu', _sensor_settings(self.imu, 'imu', topic_only=self.TOPIC_ONLY))
        object.__setattr__(self, 'scan', _sensor_settings(self.scan, 'scan', topic_only=self.TOPIC_ONLY))
        if self.imu is None:
            raise ValueError('chassis requires an IMU connection recipe')
        require_keys(self.control or {}, _CONTROL_FIELDS, 'control')
        for name, value in (self.control or {}).items():
            if name == 'min_angular':
                if type(value) not in (float, int) or not 0 <= value <= (self.control or {}).get('max_angular', .5):
                    raise ValueError('min_angular must be between zero and max_angular')
            else:
                positive(value, name, integer=name == 'settle_samples')
        require_keys(self.odometry or {}, {f.name for f in fields(OdometryFactory)} -
                     {'imu_mount', 'directory', 'sources', 'wheel_topic', 'imu_topic', 'scan_topic', 'scan_mount', 'schema_version'},
                     'odometry')
        # Validate all estimator settings without constructing or opening its DAG.
        self._odometry_recipe()
        references = dict(self.sources or {})
        require_keys(references, ('wheel', 'state', 'imu', 'scan'), 'chassis sources')
        if ('wheel' in references) != ('state' in references):
            raise ValueError('chassis sources require wheel and state together')
        for name, value in references.items():
            references[name] = source_reference(value, kind='odom' if name == 'wheel' else name)
            settings = self.imu if name == 'imu' else self.scan if name == 'scan' else None
            if settings is not None and settings.get('start_driver'):
                raise ValueError(name + ' source reference conflicts with owned hardware startup')
        object.__setattr__(self, 'sources', references)
        object.__setattr__(self, '_resources', resource_digests(dict(stm32=self.stm32, imu=self.imu, scan=self.scan)))
        freeze_fields(self)

    @classmethod
    def from_config(cls, path, overrides=None):
        return from_config(cls, path, section='chassis', overrides=overrides,
                           path_fields=('directory', 'stm32.port', 'stm32.parameters.port', 'imu.port', 'scan.port'))

    def _motor_recipe(self):
        from .stm32 import STM32Factory
        return STM32Factory(**plain(self.stm32), motion_enabled=self.motion_enabled,
                            log_path=str(Path(self.directory) / 'stm32.log'),
                            _prepared=(self._prepared or {}).get('stm32'))

    def _odometry_recipe(self, *, wheel_topic=None, scan_topic=None):
        from .odometry import OdometryFactory
        options = plain(self.odometry) or {}
        if self.estimator == 'scan_imu':
            options.update(scan_topic=scan_topic or (self.scan.get('filter') or {}).get('topic', self.scan['topic']),
                           scan_mount=plain(self.scan_mount))
        return OdometryFactory(imu_mount=plain(self.imu_mount), directory=self.directory,
            wheel_topic=wheel_topic or self.stm32.get('namespace', '/rsim/chassis').rstrip('/') + '/odom',
            imu_topic=self.imu['topic'], **options)

    def prepare(self):
        """Resolve native ROS argv in this environment; retain only pure snapshots."""
        validate_resources(self._resources)
        prepared = {'stm32': self._motor_recipe().prepare()._prepared}
        motor = restore_setup(prepared['stm32'])
        if motor['values']['motion_enabled'] != self.motion_enabled:
            raise ValueError('native argv motion_enabled conflicts with composite permission')
        if motor['values'].get('use_sim_time', False):
            raise ValueError('native chassis requires ROS system time')
        expected_port = self.stm32.get('port') or (self.stm32.get('parameters') or {}).get('port')
        if expected_port and str(Path(expected_port).expanduser().resolve()) != motor['port']:
            raise ValueError('native argv port conflicts with the explicit STM32 owner')
        for kind, settings, mount in (('imu', self.imu, self.imu_mount), ('scan', self.scan, self.scan_mount)):
            if settings is not None and settings.get('start_driver'):
                prepared[kind] = snapshot_setup(_sensor_setup(plain(settings), kind,
                    mount['ego_frame'] if mount is not None else 'laser_frame'))
        return replace(self, _prepared=prepared)

    def _require_prepared(self):
        validate_resources(self._resources)
        for name, settings in (('stm32', self.stm32), ('imu', self.imu), ('scan', self.scan)):
            if settings and settings.get('ros_args') and name not in (self._prepared or {}):
                raise ValueError('native ros_args require explicit factory.prepare() before build/shared')

    @staticmethod
    def port_specs(*, history=512, state_history=128, max_ttl=.5):
        positive(history, 'history', integer=True)
        positive(state_history, 'state_history', integer=True)
        return {'pose': PortSpec(clock='ros:system', history_capacity=history),
                'odom': PortSpec(clock='ros:system', history_capacity=history),
                'state': PortSpec(clock='ros:system', history_capacity=state_history),
                'velocity': PortSpec(direction='sink', clock='host:monotonic', max_ttl=max_ttl,
                                     fallback=VelocityCommand())}

    def _specs(self):
        motor = (restore_setup(self._prepared['stm32'])['values'] if self._prepared and self._prepared.get('stm32')
                 else self.stm32.get('parameters') or {})
        return self.port_specs(history=(self.odometry or {}).get('history', 512),
            state_history=self.stm32.get('history', 128),
            max_ttl=motor.get('max_command_ttl', .5))

    def fingerprint(self):
        self._require_prepared()
        return fingerprint(self)

    def build(self, *, ros=None, chassis=None, imu=None, scan=None, odometry=None):
        from rsim.adapters.ros2 import RosSensor, Driver
        from rsim.core.signal import as_signal
        self._require_prepared()
        ros = shared_ros_context(ros, chassis, imu, scan, odometry)
        if chassis is None:
            feedback = ({name: connect_source(self.sources[key], kind=name, ros=ros)
                         for name, key in (('odom', 'wheel'), ('state', 'state'))}
                        if 'wheel' in self.sources else None)
            chassis = self._motor_recipe().build(ros=ros, feedback_sources=feedback)

        def sensor(kind, settings, mount, injected):
            if injected is not None:
                return as_signal(injected)
            if kind in self.sources:
                return connect_source(self.sources[kind], kind=kind, ros=ros)
            settings = plain(settings)
            driver = None
            topic = settings['topic']
            if settings['start_driver']:
                frame = mount['ego_frame'] if mount is not None else 'laser_frame'
                snapshot = (self._prepared or {}).get(kind)
                setup = restore_setup(snapshot) if snapshot is not None else _sensor_setup(settings, kind, frame)
                topic = setup['topics'][kind]
                driver = Driver('rsim_hipnuc' if kind == 'imu' else 'bluesea2',
                    'serial_node' if kind == 'imu' else 'bluesea2_node', setup['options'],
                    key=('hipnuc-node:' if kind == 'imu' else 'bluesea-port:') + settings['port'],
                    log_path=Path(self.directory) / (kind + '.log'))
            return getattr(RosSensor(topic, kind, ros=ros, clock='ros:system', driver=driver,
                           history=512, hz=400 if kind == 'imu' else 100), kind)

        raw_imu = sensor('imu', self.imu, self.imu_mount, imu)
        scan_signal = scan_filter = None
        scan_topic = None
        if self.scan is not None:
            scan_signal = sensor('scan', self.scan, self.scan_mount, scan)
            scan_topic = self.scan['topic']
            if self.scan.get('filter') is not None:
                from rsim.components.scan_filter import AngularScanMask
                from rsim.adapters.ros2.scan_filter import RosScanFilter
                filtering = plain(self.scan['filter'])
                scan_topic = filtering.pop('topic')
                masked = AngularScanMask(scan_signal, **filtering)
                scan_filter = RosScanFilter(masked.scan, ros=ros, topic=scan_topic)
                scan_signal = scan_filter.scan
        if odometry is None:
            odometry = self._odometry_recipe(wheel_topic=chassis.topics['odom'], scan_topic=scan_topic).build(
                wheel=chassis.odom, imu=raw_imu, ros=ros, scan_input=scan_signal)
            if self.estimator == 'scan_imu' and scan_signal is not None:
                odometry.scan.dependencies += (scan_signal.producer,)
        result = NativeRobot(chassis, odometry, motion_enabled=self.motion_enabled,
            control=plain(self.control), scan=scan_signal, scan_filter=scan_filter)
        validate_component_ports(result, self._specs())
        return result

    def shared(self, *, key=None, transport=None, options=None):
        return SharedProvider(factory=self.build, key=key or 'native-chassis:' + self.name,
            ports=self._specs(), interface_version=self.INTERFACE_VERSION, provider_version=self.fingerprint(),
            transport=transport, options=options)

    def device(self, *, key=None, transport=None, options=None):
        return SharedComponent(key=key or 'native-chassis:' + self.name, ports=self._specs(),
                              interface_version=self.INTERFACE_VERSION, transport=transport, options=options)

    def service(self, *, source=None, transport=None):
        """Serve local or existing shared ports without rebuilding their hardware."""
        from .chassis import ChassisFactory
        source = self.build() if source is None else source
        return ChassisFactory(name=self.name, motion_enabled=self.motion_enabled, control=plain(self.control)).build(
            pose=source.pose, velocity=source.velocity, state=source.state, transport=transport)


@dataclass(frozen=True)
class TopicChassisFactory(NativeChassisFactory):
    """Connect existing ROS topics/providers; never create hardware launchers."""
    FACTORY_VERSION: ClassVar[str] = 'topic-chassis-factory-v1'
    TOPIC_ONLY: ClassVar[bool] = True


def NativeChassis(**settings):
    """Build a local composite; use a Factory for configuration and placement."""
    return NativeChassisFactory(**settings).build()


def TopicChassis(**settings):
    """Build a connection-only composite; old automatic relay startup was removed."""
    if settings.pop('relay_transport', None) is not None:
        raise ValueError('relay_transport was removed; declare existing shared_device sources')
    return TopicChassisFactory(**settings).build()
