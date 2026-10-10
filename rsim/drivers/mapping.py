"""Super-LIO laser geometry, RGB coloring and RTAB-Map pose-graph optimization."""
from dataclasses import asdict
import json
from pathlib import Path
import re

from rsim.devices.mapping import MappingSave, mapping_ports
from rsim.runtime.sharing import SharedProvider


class MappingProvider(MappingSave, SharedProvider):
    pass


def Mapper(*, connection=None, lidar_ip=None, mounts, database, name='mapping', history=3,
           transport=None, timing=None, allow_estimated_timing=False,
           topics=None, camera_parameters=None, lidar_parameters=None,
           lio_parameters=None, rtabmap_parameters=None, cloud_filter=None, map_options=None,
           scan2d_parameters=None, fusion_parameters=None, start_drivers=False):
    """Start a read-only mapping stack; no chassis velocity writer is created.

    Mounts are graphmap Pose constructor dictionaries for body_lidar, body_imu,
    body_camera; integrated scan odometry additionally needs body_scan. Explicitly opt into reception-derived clock offsets for a
    prototype, or supply measured offsets for both source clocks.
    """
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
        raise ValueError('mapping name must be a ROS-compatible identifier')
    from rsim.adapters.ros1 import SSHConfig
    if isinstance(connection, dict):
        connection = SSHConfig(**connection)
    if connection is not None and not isinstance(connection, SSHConfig):
        raise TypeError('mapping connection must be SSHConfig or its keyword mapping')
    required_mounts = {'body_lidar', 'body_imu', 'body_camera'}
    if scan2d_parameters is not None:
        required_mounts.add('body_scan')
    if set(mounts) != required_mounts:
        raise ValueError(f'mapping needs mounts: {sorted(required_mounts)}')
    timing = {key: dict((timing or {}).get(key, {})) for key in ('lidar', 'chassis')}
    if connection is None:
        timing['chassis'].setdefault('offset_s', 0.)
    if not allow_estimated_timing and any(value.get('offset_s') is None for value in timing.values()):
        raise ValueError('supply measured clock offsets or explicitly allow_estimated_timing')
    database = str(Path(database).expanduser().resolve())
    if fusion_parameters is not None and scan2d_parameters is None:
        raise ValueError('mapping fusion requires scan2d_parameters and body_scan')
    if not start_drivers and (camera_parameters or lidar_parameters):
        raise ValueError('configure camera/lidar parameters in their hardware launchers, not the mapping service')
    if start_drivers and not lidar_ip:
        raise ValueError('explicit hardware startup requires lidar_ip')
    settings = dict(start_drivers=start_drivers, fusion_parameters=fusion_parameters, scan2d_parameters=scan2d_parameters, connection=asdict(connection) if connection else None, lidar_ip=lidar_ip, mounts=mounts, database=database,
        name=name, timing=timing, topics=topics or {}, camera_parameters=camera_parameters or {},
        lidar_parameters=lidar_parameters or {}, lio_parameters=lio_parameters or {},
        rtabmap_parameters=rtabmap_parameters or {}, cloud_filter=cloud_filter or {}, map_options=map_options or {})
    # A JSON snapshot gives independent callers a deterministic conflict check.
    signature = json.dumps(settings, sort_keys=True, allow_nan=False)
    settings = json.loads(signature)
    def factory():
        return mapping_graph(**settings)
    return MappingProvider(factory=factory, ports=mapping_ports(history), key='mapper:' + name,
        interface_version='mapping-ports-v1', transport=transport, provider_version=signature)


def mapping_graph(*, connection, lidar_ip, mounts, database, name, timing, topics,
                  camera_parameters, lidar_parameters, lio_parameters, rtabmap_parameters, cloud_filter,
                  map_options=None, input_source=None, scan2d_parameters=None, fusion_parameters=None,
                  start_drivers=False):
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
    defaults = ({'imu': '/imu_data', 'odom': '/odom_raw', 'scan': '/scan'} if connection else
                {'imu': '/rsim/chassis/imu/data', 'odom': '/rsim/chassis/odom', 'scan': '/rsim/chassis/scan'})
    topics = {'points': prefix+'/raw/points' if start_drivers else '/iv_points', **defaults, **topics}
    topics.update(lidar_frame=T_base_lidar.ego_frame, imu_frame=T_base_imu.ego_frame)
    directory = Path(database).parent
    directory.mkdir(parents=True, exist_ok=True)
    if Path(database).exists() and Path(database).stat().st_size:
        raise ValueError('start a new database for a new laser map session; native-map relocalization is not implicit')
    if input_source is not None:
        from rsim.core import Component
        if (not isinstance(input_source, Component) or getattr(input_source, 'prefix', None) != prefix
                or not isinstance(getattr(input_source, 'ros', None), RosContext)
                or not callable(getattr(input_source, 'diagnostics', None))):
            raise ValueError('injected mapping input must own a RosContext and match the topic prefix')
        inputs = input_source
    else:
        ros = RosContext(max_callbacks=8)
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
            mounts=list(geometry.values()), topics=topics, timing=timing, cloud_filter=cloud_filter)
    scan2d = None
    if scan2d_parameters is not None:
        from rsim.adapters.ros2.scan_odometry import ScanOdometry
        if 'body_scan' not in geometry:
            raise ValueError('scan odometry requires an explicit body_scan mount')
        scan2d = ScanOdometry(inputs.ros, prefix=prefix, mount=geometry['body_scan'],
                             directory=directory, parameters=scan2d_parameters,
                             odom_frame=frames['odom'] if fusion_parameters is not None else None)
    native_lio = {
        'lio.ros.lidar_topic': prefix + '/lidar', 'lio.ros.imu_topic': prefix + '/imu',
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
    import yaml
    parameter_file = directory / 'rtabmap-parameters.yaml'
    parameter_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': native_rtabmap}}))
    rtabmap = Driver('rtabmap_slam', 'rtabmap', key='mapping:rtabmap:' + name,
        ros_args=['--params-file', str(parameter_file)],
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
    mapping = LaserMappingIO(ledger=ledger, pose_history=PoseHistory(capacity=4096), keyframe_type=LaserKeyframe,
        camera_type=PinholeCamera, colorize=colorize_laser, T_base_imu=T_base_imu,
        camera_prefix=camera_prefix, allocator=np.empty, fusion=fusion, rectify_rgb=rectify_rgb, **options)
    result = MappingOutput(inputs, lio, rtabmap, frames=frames, T_base_imu=T_base_imu,
                         database=database, assumptions=assumptions, allocator=np.empty, mapping=mapping)

    result.scan_odometry = scan2d
    result.fusion = fusion
    if fusion is not None:
        result.children = (*result.children, fusion)
    elif scan2d is not None:
        result.children = (*result.children, scan2d)
    return result
