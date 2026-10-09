"""Native ROS chassis composition; only the final service crosses RSim DDS."""
from dataclasses import dataclass
from pathlib import Path


@dataclass
class NativeRobot:
    chassis: object
    odometry: object
    control: object
    scan: object = None
    scan_filter: object = None

    @property
    def pose(self):
        return self.odometry.pose

    @property
    def velocity(self):
        return self.chassis.velocity


def NativeChassis(*, stm32, imu, imu_mount, directory, scan=None, scan_mount=None,
                  estimator='wheel_imu', odometry=None, control=None, motion_enabled=False):
    """Assemble an estimator and native STM32 sink in one event loop.

    Hardware settings use start_driver=False to attach to existing native nodes.
    IMU and optional scan settings accept parameters/ros_args for native launch.
    The motion flag applies independently to the controller and owned MCU node.
    """
    from rsim.adapters.ros2 import RosContext, RosSensor, Driver
    from rsim.components.motion import ChassisController
    from .stm32 import STM32
    from .odometry import Odometry

    if estimator not in ('wheel_imu', 'scan_imu'):
        raise ValueError('estimator must be wheel_imu or scan_imu')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    ros = RosContext(max_callbacks=8)
    chassis = STM32(**stm32, ros=ros, motion_enabled=motion_enabled,
                    log_path=directory/'stm32.log')

    def sensor(settings, kind, package, executable, defaults, remappings):
        settings = dict(settings)
        topic = settings.pop('topic')
        start = settings.pop('start_driver', False)
        parameters = {**defaults, **settings.pop('parameters', {})}
        ros_args = settings.pop('ros_args', None)
        if start:
            port = settings.pop('port', None)
            if not port:
                raise ValueError(kind+' native launch requires a serial port')
            parameters['port'] = str(Path(port).expanduser().resolve())
        else:
            settings.pop('port', None)
        if settings:
            raise ValueError('unknown '+kind+' settings: '+str(sorted(settings)))
        driver = Driver(package, executable, parameters,
            remappings=remappings(topic), ros_args=ros_args,
            key=('hipnuc-node:' if kind == 'imu' else 'bluesea-port:')+parameters['port'],
            log_path=directory/(kind+'.log')) if start else None
        return RosSensor(topic, kind, ros=ros, clock='ros:system', driver=driver,
                         history=512, hz=400 if kind == 'imu' else 100)

    frame = imu_mount['ego_frame'] if isinstance(imu_mount, dict) else imu_mount.ego_frame
    raw_imu = sensor(imu, 'imu', 'rsim_hipnuc', 'serial_node',
        dict(baudrate=460800, frame_id=frame, navigation_frame='enu'),
        lambda topic: {'imu/data': topic})
    settings = dict(odometry or {})
    scan_signal = scan_filter = scan_owner = None
    if estimator == 'scan_imu' and (scan is None or scan_mount is None):
        raise ValueError('scan_imu requires scan settings and scan_mount')
    if scan is not None:
        scan_settings = dict(scan)
        filter_settings = scan_settings.pop('filter', None)
        frame = (scan_mount['ego_frame'] if isinstance(scan_mount, dict) else scan_mount.ego_frame) if scan_mount is not None else 'laser_frame'
        raw_scan = sensor(scan_settings, 'scan', 'bluesea2', 'bluesea2_node',
            dict(type='uart', baud_rate=500000, frame_id=frame, raw_bytes=3,
                 output_360=True, output_scan=True, output_cloud=False, output_cloud2=False,
                 with_angle_filter=False, max_dist=50., inverted=True, scan_topic=scan['topic']),
            lambda topic: {})
        scan_signal, scan_owner, scan_topic = raw_scan.scan, raw_scan, scan['topic']
        if filter_settings is not None:
            from rsim.components.scan_filter import AngularScanMask
            from rsim.adapters.ros2.scan_filter import RosScanFilter
            filter_settings = dict(filter_settings)
            scan_topic = filter_settings.pop('topic')
            if scan_topic == scan['topic']:
                raise ValueError('filtered scan topic must differ from raw input')
            masked = AngularScanMask(raw_scan.scan, **filter_settings)
            scan_filter = RosScanFilter(masked.scan, ros=ros, topic=scan_topic)
            scan_signal, scan_owner = scan_filter.scan, scan_filter
        if estimator == 'scan_imu':
            settings.update(scan_topic=scan_topic, scan_mount=scan_mount)
    estimate = Odometry(wheel=chassis.odom, imu=raw_imu.imu, imu_mount=imu_mount,
        wheel_topic=chassis.topics['odom'], ros=ros, directory=directory, **settings)
    if estimator == 'scan_imu':
        estimate.scan.dependencies += (scan_owner,)
    controller = ChassisController(pose=estimate.pose, velocity=chassis.velocity,
        motion_enabled=motion_enabled, **(control or {}))
    return NativeRobot(chassis, estimate, controller, scan_signal, scan_filter)
