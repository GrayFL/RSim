"""Super-LIO/RTAB recipes with local schema and explicit persistent ownership."""
from dataclasses import asdict, dataclass, field, fields, is_dataclass
import hashlib
from pathlib import Path
import re
from typing import ClassVar

from rsim.devices.mapping import MappingSave, MappingClient
from rsim.runtime.sharing import PortSpec, SharedProvider, validate_component_ports
from .configuration import (absolute_name, connect_source, fingerprint, freeze_fields, from_config,
                            mount_data, plain, positive, read_config, require_keys,
                            source_reference, validate_resources, shared_ros_context)


class MappingProvider(MappingSave, SharedProvider):
    pass


@dataclass(frozen=True)
class MappingFactory:
    database: str
    mounts: object = None
    name: str = 'mapping'
    history: int = 3
    sources: object = None
    connection: object = None
    lidar_ip: str | None = None
    timing: object = None
    allow_estimated_timing: bool = False
    topics: object = None
    camera_parameters: object = None
    lidar_parameters: object = None
    lio_parameters: object = None
    rtabmap_parameters: object = None
    cloud_filter: object = None
    map_options: object = None
    scan2d_parameters: object = None
    fusion_parameters: object = None
    start_drivers: bool = False
    database_policy: str = 'new'
    directory: str | None = field(default=None, metadata={'fingerprint': False})
    geometry_file: str | None = None
    schema_version: int = 1
    _resources: object = field(init=False, repr=False)
    FACTORY_VERSION: ClassVar[str] = 'mapping-factory-v2'
    INTERFACE_VERSION: ClassVar[str] = 'mapping-ports-v2'

    def __post_init__(self):
        from rsim.adapters.ros1 import SSHConfig
        from rsim.adapters.ros2.arguments import GeneratedParameters
        from rsim.adapters.ros2.mapping_input import ClockAlignment
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError('unsupported mapping schema_version')
        if not isinstance(self.name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', self.name):
            raise ValueError('mapping name must be a ROS-compatible identifier')
        positive(self.history, 'history', integer=True)
        for name in ('start_drivers', 'allow_estimated_timing'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(name + ' must be boolean')
        if self.database_policy != 'new':
            raise ValueError('database_policy must be new; native-map relocalization is not implicit')
        if not isinstance(self.database, (str, Path)) or not str(self.database):
            raise ValueError('database must be an explicit persistent asset path')
        database = str(Path(self.database).expanduser().resolve())
        object.__setattr__(self, 'database', database)
        object.__setattr__(self, 'directory', str(Path(self.directory or Path(database).parent).expanduser().resolve()))
        connection = asdict(self.connection) if is_dataclass(self.connection) else self.connection
        if connection is not None:
            require_keys(connection, (f.name for f in fields(SSHConfig)), 'connection')
            SSHConfig(**plain(connection)).command()
        object.__setattr__(self, 'connection', connection)
        require_keys(self.timing or {}, ('lidar', 'chassis', 'camera'), 'timing')
        camera_timing = dict((self.timing or {}).get('camera', {}))
        timing = {name: dict((self.timing or {}).get(name, {})) for name in ('lidar', 'chassis')}
        if connection is None:
            timing['chassis'].setdefault('offset_s', 0.)
        if not self.allow_estimated_timing and any(value.get('offset_s') is None for value in timing.values()):
            raise ValueError('supply measured clock offsets or explicitly allow_estimated_timing')
        for name, values in timing.items():
            require_keys(values, ('offset_s', 'samples'), name + ' timing')
            positive(values.get('samples', 20), name + ' samples', integer=True)
            ClockAlignment(**values)
        if connection is None and timing['chassis']['offset_s'] != 0:
            raise ValueError('native ROS chassis timestamps require offset_s=0')
        object.__setattr__(self, 'timing', timing)
        resources = {}
        mounts = dict(self.mounts or {})
        if self.geometry_file is not None:
            path = Path(self.geometry_file).expanduser().resolve(strict=True)
            geometry = read_config(path)['sensor_params']
            resources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            object.__setattr__(self, 'geometry_file', str(path))
            for name, source, child in (('body_lidar', 'lidar3d', 'mapping_lidar'), ('body_camera', 'rgb', 'camera_link')):
                original = geometry[source]
                mounts.setdefault(name, dict(position=original['position'], rotation=original['rotation'],
                    degrees=True, wrd_frame='base_footprint', ego_frame=child))
        required = {'body_lidar', 'body_imu', 'body_camera'} | ({'body_scan'} if self.scan2d_parameters is not None else set())
        if set(mounts) != required:
            raise ValueError(f'mapping needs mounts: {sorted(required)}')
        mounts = {name: mount_data(value, name=name, base='base_footprint') for name, value in mounts.items()}
        object.__setattr__(self, 'mounts', mounts)
        object.__setattr__(self, '_resources', resources)
        if self.fusion_parameters is not None and self.scan2d_parameters is None:
            raise ValueError('mapping fusion requires scan2d_parameters and body_scan')
        if not self.start_drivers and (self.camera_parameters or self.lidar_parameters):
            raise ValueError('configure camera/lidar parameters in hardware launchers, not the mapping service')
        if self.start_drivers and not self.lidar_ip:
            raise ValueError('explicit hardware startup requires lidar_ip')
        require_keys(self.topics or {}, ('points', 'imu', 'odom', 'scan', 'camera_prefix'), 'topics')
        for name, value in (self.topics or {}).items():
            absolute_name(value, name)
        sources = dict(self.sources or {})
        require_keys(sources, ('lidar', 'imu', 'wheel', 'scan', 'camera', 'camera_info'), 'mapping sources')
        if ('camera' in sources) != ('camera_info' in sources):
            raise ValueError('shared camera input requires camera and camera_info references')
        for name, value in sources.items():
            sources[name] = source_reference(value, kind='odom' if name == 'wheel' else name,
                clocks=None if name in ('lidar', 'camera', 'camera_info') else ('ros:system',))
            if name in ('lidar', 'camera', 'camera_info') and sources[name]['kind'] not in ('shared_device', 'shared_sensor'):
                raise ValueError(name + ' reference requires a shared provider; native ROS sources use topics')
        if 'camera' in sources:
            if sources['camera']['clock'] != sources['camera_info']['clock']:
                raise ValueError('camera and camera_info must use one source clock')
            if sources['camera']['clock'] != 'ros:system' and camera_timing.get('offset_s') is None:
                raise ValueError('device camera clock requires an explicit camera offset_s')
        require_keys(camera_timing, ('offset_s',), 'camera timing')
        if camera_timing:
            if camera_timing.get('offset_s') is None:
                raise ValueError('camera offset_s must be measured, not reception estimated')
            ClockAlignment(**camera_timing)
            timing['camera'] = camera_timing
        if self.start_drivers and set(sources) & {'lidar', 'camera', 'camera_info'}:
            raise ValueError('owned hardware startup conflicts with existing camera/lidar sources')
        object.__setattr__(self, 'sources', sources)
        for name in ('camera_parameters', 'lidar_parameters', 'lio_parameters', 'rtabmap_parameters',
                     'scan2d_parameters', 'fusion_parameters'):
            GeneratedParameters(getattr(self, name) or {})
        if any(key.startswith(('lio.wheel.', 'lio.planar.')) for key in (self.lio_parameters or {})):
            raise ValueError('native wheel/planar experiments were removed; use fusion_parameters instead')
        rtab = self.rtabmap_parameters or {}
        if (float(rtab.get('Mem/RehearsalSimilarity', '1')) != 1.0
                or str(rtab.get('Mem/ReduceGraph', 'false')).lower() != 'false'):
            raise ValueError('stable laser provenance requires Mem/RehearsalSimilarity=1 and Mem/ReduceGraph=false')
        if rtab.get('database_path', database) != database:
            raise ValueError('RTAB database_path cannot override the owned persistent asset')
        require_keys(self.cloud_filter or {}, ('min_range', 'max_range', 'stride'), 'cloud_filter')
        filtering = self.cloud_filter or {}
        positive(filtering.get('stride', 1), 'stride', integer=True)
        lo, hi = filtering.get('min_range', .3), filtering.get('max_range', 50.)
        if type(lo) not in (int, float) or type(hi) not in (int, float) or not 0 <= lo < hi:
            raise ValueError('cloud_filter requires 0 <= min_range < max_range')
        require_keys(self.map_options or {}, ('resolution', 'adaptive_keyframes', 'max_rgb_dt_s', 'keyframe_hz',
            'occlusion_cell', 'occlusion_tolerance_m', 'max_wait_s', 'scan_capacity', 'image_capacity'), 'map_options')
        options = self.map_options or {}
        for name, value in options.items():
            if name not in ('adaptive_keyframes', 'occlusion_tolerance_m'):
                positive(value, name, integer=name in ('occlusion_cell', 'scan_capacity', 'image_capacity'))
        if options.get('occlusion_tolerance_m', .05) < 0:
            raise ValueError('occlusion_tolerance_m must be nonnegative')
        if options.get('adaptive_keyframes') is not None:
            from rsim.components.keyframe_rate import MotionKeyframeRate
            require_keys(options['adaptive_keyframes'], ('min_hz', 'linear_speed_m_s', 'angular_speed_rad_s', 'window_s',
                         'linear_deadband_m_s', 'angular_deadband_rad_s'), 'adaptive_keyframes')
            MotionKeyframeRate(None, max_hz=options.get('keyframe_hz', 2.), **plain(options['adaptive_keyframes']))
        freeze_fields(self)

    @classmethod
    def from_config(cls, path, overrides=None):
        return from_config(cls, path, section='mapping', overrides=overrides,
                           path_fields=('database', 'directory', 'geometry_file'))

    @staticmethod
    def port_specs(*, history=3):
        positive(history, 'history', integer=True)
        return {name: PortSpec(clock='ros:system', history_capacity=history)
                for name in ('pose', 'odometry', 'map', 'rgb_map', 'status')}

    def prepare(self):
        validate_resources(self._resources)
        return self

    def fingerprint(self):
        validate_resources(self._resources)
        return fingerprint(self)

    def build(self, *, input_source=None, ros=None, sources=None):
        validate_resources(self._resources)
        if input_source is not None and sources:
            raise ValueError('choose input_source or individual injected sources')
        if input_source is not None:
            from rsim.core import Component
            if not isinstance(input_source, Component) or not hasattr(input_source, 'ros'):
                raise ValueError('input_source must be a Component with a RosContext')
            if ros is not None and ros is not input_source.ros:
                raise ValueError('mapping input and build must share one RosContext')
            ros = input_source.ros
        injected = dict(sources or {})
        require_keys(injected, ('lidar', 'imu', 'wheel', 'scan', 'camera', 'camera_info'), 'injected mapping sources')
        ros = shared_ros_context(ros, input_source, *injected.values())
        if input_source is None:
            for name, reference in self.sources.items():
                if name not in injected:
                    injected[name] = connect_source(reference, kind='odom' if name == 'wheel' else name, ros=ros)
        if ('camera' in injected) != ('camera_info' in injected):
            raise ValueError('injected camera requires both image and calibration Signals')
        settings = {name: plain(getattr(self, name)) for name in ('connection', 'lidar_ip', 'mounts', 'database', 'name',
            'timing', 'topics', 'camera_parameters', 'lidar_parameters', 'lio_parameters', 'rtabmap_parameters',
            'cloud_filter', 'map_options', 'scan2d_parameters', 'fusion_parameters', 'start_drivers')}
        for name in ('topics', 'camera_parameters', 'lidar_parameters', 'lio_parameters', 'rtabmap_parameters', 'cloud_filter'):
            settings[name] = settings[name] or {}
        specs = self.port_specs(history=self.history)
        result = mapping_graph(**settings, input_source=input_source, ros=ros, sources=injected,
                               ports=specs, directory=self.directory, database_policy=self.database_policy)
        validate_component_ports(result, specs)
        return result

    def shared(self, *, transport=None, options=None):
        return MappingProvider(factory=self.build, key='mapper:' + self.name, ports=self.port_specs(history=self.history),
            interface_version=self.INTERFACE_VERSION, provider_version=self.fingerprint(), transport=transport,
            log_path=Path(self.directory) / 'provider.log', options=options)

    def device(self, *, transport=None, options=None):
        return MappingClient(key='mapper:' + self.name, ports=self.port_specs(history=self.history),
                             interface_version=self.INTERFACE_VERSION, transport=transport, options=options)


def Mapper(*, transport=None, **settings):
    """Construct an explicit launcher; MappingFactory retains reusable configuration."""
    return MappingFactory(**settings).shared(transport=transport)


def mapping_graph(*, connection, lidar_ip, mounts, database, name, timing, topics,
                  camera_parameters, lidar_parameters, lio_parameters, rtabmap_parameters, cloud_filter,
                  map_options=None, input_source=None, scan2d_parameters=None, fusion_parameters=None,
                  start_drivers=False, ros=None, sources=None, ports=None, directory=None, database_policy='new'):
    """Assemble the native backend with live inputs or an injected Component.

    An injected source owns its RosContext, exposes prefix/diagnostics(), and
    publishes the same aligned ROS input topics. It must use the backend's
    wall-clock timestamp domain; no hardware drivers are created in this mode.
    """
    from graphmap.pose import Pose
    from rsim.adapters.ros1 import Ros1Bridge, SSHConfig
    from rsim.adapters.ros2 import RosContext, Driver
    from rsim.adapters.ros2.mapping_input import MappingInputs
    from rsim.adapters.ros2.mapping_output import MappingOutput
    from rsim.adapters.ros2.laser_mapping import LaserMappingIO
    from rsim.components.mapping import MapLedger, LaserKeyframe
    from rsim.components.keyframe_rate import MotionKeyframeRate
    from rsim.components.projection import PoseHistory, PinholeCamera, colorize_laser
    import numpy as np

    prefix = '/rsim/' + name
    frames = {'map': name + '_map', 'odom': name + '_odom', 'base': 'base_footprint'}
    frames['lio'] = name + '_lio_odom' if fusion_parameters is not None else frames['odom']
    if fusion_parameters is not None and scan2d_parameters is None:
        raise ValueError('mapping fusion requires scan2d_parameters and body_scan')
    if any(key.startswith(('lio.wheel.', 'lio.planar.')) for key in lio_parameters):
        raise ValueError('native wheel/planar experiments were removed; use fusion_parameters instead')
    geometry = {key: Pose(**value) for key, value in mounts.items()}
    if any(pose.wrd_frame != frames['base'] or pose.scale != 1 for pose in geometry.values()):
        raise ValueError('mapping mounts must be SE(3) poses with parent base_footprint')
    T_base_lidar, T_base_imu = geometry['body_lidar'], geometry['body_imu']
    T_imu_lidar = ~T_base_imu * T_base_lidar
    camera_prefix = topics.get('camera_prefix', prefix + '/camera/d435' if start_drivers else '/rsim/d435')
    if 'camera' in (sources or {}):
        camera_prefix = prefix + '/inputs/camera'
    defaults = ({'imu': '/imu_data', 'odom': '/odom_raw', 'scan': '/scan'} if connection else
                {'imu': '/rsim/chassis/imu/data', 'odom': '/rsim/chassis/odom', 'scan': '/rsim/chassis/scan'})
    topics = {'points': prefix+'/raw/points' if start_drivers else '/iv_points', **defaults, **topics}
    topics.update(lidar_frame=T_base_lidar.ego_frame, imu_frame=T_base_imu.ego_frame)
    directory = Path(directory) if directory is not None else Path(database).parent
    from rsim.adapters.ros2.mapping_resources import MappingDatabase
    database_session = MappingDatabase(database, policy=database_policy)
    if input_source is not None:
        from rsim.core import Component
        if (not isinstance(input_source, Component) or getattr(input_source, 'prefix', None) != prefix
                or not isinstance(getattr(input_source, 'ros', None), RosContext)
                or not callable(getattr(input_source, 'diagnostics', None))):
            raise ValueError('injected mapping input must own a RosContext and match the topic prefix')
        inputs = input_source
        if ros is not None and ros is not inputs.ros:
            raise ValueError('mapping input and backend must share one RosContext')
    else:
        ros = ros if ros is not None else RosContext(max_callbacks=8)
        camera = Driver('realsense2_camera', 'realsense2_camera_node', {
            'enable_depth': False, 'enable_color': True, 'align_depth.enable': False,
            'enable_sync': False, 'publish_tf': True,
            'depth_module.depth_profile': '640x480x15', 'rgb_camera.color_profile': '640x480x15',
            **camera_parameters}, key='driver:d435:' + str(camera_parameters.get('serial_no', '')).lstrip('_'),
            remappings={'__ns': prefix + '/camera', '__node': 'd435'}, log_path=directory/'camera.log') if start_drivers else None
        lidar = Driver('seyond', 'seyond_node', {
            'lidar_ip': lidar_ip, 'frame_topic': prefix + '/raw/points',
            'frame_id': T_base_lidar.ego_frame, 'coordinate_mode': 3, **lidar_parameters},
            key='driver:robin:' + lidar_ip, log_path=directory/'lidar.log') if start_drivers else None
        bridge = Ros1Bridge(SSHConfig(**connection), log_path=directory/'chassis.log') if connection is not None else None
        inputs = MappingInputs(ros, bridge,
            prefix=prefix, lidar_driver=lidar, camera_driver=camera,
            mounts=list(geometry.values()), topics=topics, timing=timing, cloud_filter=cloud_filter,
            sources=sources, camera_prefix=camera_prefix)
    scan2d = None
    if scan2d_parameters is not None:
        from rsim.adapters.ros2.scan_odometry import ScanOdometry
        if 'body_scan' not in geometry:
            raise ValueError('scan odometry requires an explicit body_scan mount')
        from .scan_odometry import ScanOdometryFactory
        scan2d = ScanOdometry(inputs.ros, prefix=prefix, mount=geometry['body_scan'], ports=ScanOdometryFactory.port_specs(),
                             directory=directory, parameters=scan2d_parameters,
                             odom_frame=frames['odom'] if fusion_parameters is not None else None)
    native_lio = {
        'lio.ros.lidar_topic': prefix + '/lidar',
        # Native ROS IMU is already in the system clock domain. Its integration
        # must not depend on a Python decode/re-encode round trip competing with
        # map assembly. Recorded/ROS1 ingress still needs the aligned relay.
        'lio.ros.imu_topic': topics['imu'] if connection is None and input_source is None and not (sources or {}).get('imu') else prefix + '/imu',
        'lio.ros.reliable_lidar': True,
        # VELO16 selects the xyz/intensity/time wire layout; hardware remains Seyond.
        'lio.sensor.lidar_type': 3, 'lio.sensor.imu_type': 1,
        'lio.sensor.blind': .3, 'lio.sensor.maxrange': 50., 'lio.sensor.filter_rate': 1,
        'lio.sensor.enable_downsample': True, 'lio.sensor.voxel_fliter_size': .15,
        'lio.sensor.gravity_norm': 9.80665, 'lio.sensor.imu_na': .1, 'lio.sensor.imu_ng': .01,
        'lio.sensor.imu_nba': .0001, 'lio.sensor.imu_nbg': .0001,
        'lio.extrinsic.lidar_imu': T_imu_lidar.position.tolist() + T_imu_lidar.matrix[:3, :3].ravel(order='F').tolist(),
        # The upstream field seeds the IMU position in the local odometry frame.
        'lio.extrinsic.odom_robo': T_base_imu.position.tolist() + T_base_imu.euler.tolist(),
        'lio.hash_map.hash_capacity': 100000, 'lio.hash_map.vox_resolution': .5,
        'lio.kf.kf_type': 0, 'lio.kf.kf_max_iterations': 4, 'lio.kf.kf_align_gravity': True,
        'lio.kf.kf_quit_eps': .001, 'lio.map.save_map': False,
        'lio.output.robot': False, 'lio.output.map': False, 'lio.output.dense': False,
        'lio.output.cloud_pose': True,
        'lio.output.pub_step': 1, 'lio.eva.timer': False,
        **lio_parameters,
    }
    lio = Driver('super_lio', 'super_lio_node', native_lio, key='mapping:lio:' + name,
        remappings={'/lio/odom': prefix + '/lio/corrected_odom',
                    '/lio/body/cloud_pose': prefix + '/lio/cloud_pose',
                    '/lio/imu/odom': prefix + '/lio/odom', '/tf': prefix + '/lio/tf'},
        log_path=directory/'lio.log')
    native_rtabmap = {
        'frame_id': frames['base'], 'map_frame_id': frames['map'],
        'subscribe_depth': False, 'subscribe_rgbd': False, 'subscribe_scan': False,
        'subscribe_rgb': True, 'subscribe_scan_cloud': True,
        'approx_sync': False,
        'topic_queue_size': 10, 'sync_queue_size': 10, 'qos_image': 1, 'qos_camera_info': 1,
        'qos_scan': 1, 'qos_odom': 1,
        'publish_tf': True, 'wait_for_transform': .2,
        'database_path': database, 'map_always_update': True,
        'Rtabmap/DetectionRate': '0', 'RGBD/LinearUpdate': '0.1', 'RGBD/AngularUpdate': '0.05',
        'Rtabmap/ImagesAlreadyRectified': 'true',
        # Metric visual features come from projected laser depth, not camera geometry.
        'gen_depth': True, 'gen_depth_decimation': 2, 'gen_depth_fill_holes_size': 0,
        'Mem/DepthCompressionFormat': '.png',
        'RGBD/CreateOccupancyGrid': 'false', 'Grid/Sensor': '0', 'Grid/3D': 'true',
        'Reg/Strategy': '2', 'Mem/IncrementalMemory': 'true', 'Mem/ReduceGraph': 'false',
        # Rehearsal can replace a node's sensor data and retire its original
        # identity. Keep immutable source IDs tied to their own optimized pose.
        'Mem/RehearsalSimilarity': '1.0',
        'Reg/Force3DoF': 'true' if fusion_parameters is not None else 'false',
        'Optimizer/Slam2D': 'true' if fusion_parameters is not None else 'false',
        'Mem/STMSize': '10', 'Rtabmap/MemoryThr': '0', 'Rtabmap/TimeThr': '0',
        'RGBD/ProximityBySpace': 'true', 'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1', 'RGBD/OptimizeFromGraphEnd': 'false',
        'Icp/PointToPlane': 'true', 'Icp/PointToPlaneK': '20', 'Icp/VoxelSize': '0.15',
        'Icp/MaxCorrespondenceDistance': '0.5', 'Icp/CorrespondenceRatio': '0.2',
        **rtabmap_parameters}
    if (float(native_rtabmap['Mem/RehearsalSimilarity']) != 1.0
            or str(native_rtabmap['Mem/ReduceGraph']).lower() != 'false'):
        raise ValueError('stable laser provenance requires Mem/RehearsalSimilarity=1 and Mem/ReduceGraph=false')
    rectify_rgb = str(native_rtabmap['Rtabmap/ImagesAlreadyRectified']).lower() == 'true'
    if native_rtabmap['gen_depth'] and not rectify_rgb:
        raise ValueError('laser depth generation requires rectified RTAB RGB; enable ImagesAlreadyRectified')
    # RTAB-Map uses names such as Grid/3D, which the ROS CLI -p lexer rejects.
    # Native YAML parameter files preserve these names and their string values.
    rtabmap = Driver('rtabmap_slam', 'rtabmap', key='mapping:rtabmap:' + name,
        generated_parameters=native_rtabmap,
        remappings={'__ns': prefix, 'rgb/image': prefix + '/keyframe/rgb',
            'rgb/camera_info': prefix + '/keyframe/camera_info', 'odom': prefix + '/keyframe/odom',
            'scan_cloud': prefix + '/keyframe/scan'},
        log_path=directory/'rtabmap.log')
    assumptions = {
        'time_alignment': 'reception offsets include unknown transport latency',
        'mounts': mounts,
        'odometry_covariance': ('interpolated EKF covariance; not calibrated ground-truth uncertainty'
                                if fusion_parameters is not None else 'configured weights, not Super-LIO covariance'),
        'external_imu': 'unused: mounting transform unknown',
        'wheel_and_scan': ('independent robot_localization planar fusion' if fusion_parameters is not None
                           else 'not fused; Super-LIO supplies odometry'),
        'fusion_covariance': 'configured noise floors; correlated scan/wheel errors are not fully modeled',
        'map_geometry': 'Super-LIO deskewed laser points, optimized per RTAB-Map keyframe',
        'map_color': 'RGB projection with lidar z-buffer; no D435 depth',
        'rtab_registration': native_rtabmap['Reg/Strategy'],
        'rtab_feature_depth': 'projected lidar' if native_rtabmap['gen_depth'] else 'none',
    }
    options = dict(map_options or {})
    pose_history = PoseHistory(capacity=4096)
    adaptive = options.pop('adaptive_keyframes', None)
    rate_policy = (MotionKeyframeRate(pose_history, max_hz=options.get('keyframe_hz', 2.), **adaptive)
                   if adaptive is not None else None)
    fusion = None
    if fusion_parameters is not None:
        from .odometry import Odometry
        from rsim.components.synchronization import SampleHistory
        fusion = Odometry(ros=inputs.ros, scan=scan2d, prefix=prefix+'/fusion',
            wheel_topic=prefix+'/wheel_odom', imu_topic=prefix+'/imu',
            world_frame=frames['odom'], imu_mount=T_base_imu,
            directory=directory, parameters=fusion_parameters, pose_history=PoseHistory(capacity=4096),
            covariance_history=SampleHistory())
    ledger = MapLedger(resolution=options.pop('resolution', .05), frame_id=frames['map'])
    mapping = LaserMappingIO(ledger=ledger, pose_history=pose_history, keyframe_type=LaserKeyframe,
        camera_type=PinholeCamera, colorize=colorize_laser, T_base_imu=T_base_imu,
        camera_prefix=camera_prefix, allocator=np.empty, fusion=fusion, rectify_rgb=rectify_rgb,
        rate_policy=rate_policy, **options)
    result = MappingOutput(inputs, lio, rtabmap, frames=frames, T_base_imu=T_base_imu,
                         database=database, assumptions=assumptions, allocator=np.empty, mapping=mapping,
                         ports=ports if ports is not None else MappingFactory.port_specs())
    result.dependencies = (database_session, *result.dependencies)
    result.database_session = database_session

    result.scan_odometry = scan2d
    result.fusion = fusion
    if fusion is not None:
        result.children = (*result.children, fusion)
    elif scan2d is not None:
        result.children = (*result.children, scan2d)
    return result
