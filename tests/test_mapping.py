import asyncio
import time
from types import SimpleNamespace

import numpy as np
import pytest

from rsim import Frame, PointCloud, Runtime
from rsim.adapters.ros2.mapping_input import ClockAlignment, timed_points
from rsim.devices.mapping import MappingView


def test_mapping_clocks_freeze_offset_and_reject_resets():
    clock = ClockAlignment(samples=3)
    assert clock.observe(1., 101.1) is None
    assert clock.observe(2., 102.02) is None
    assert clock.observe(3., 103.04) == pytest.approx(103.02)
    assert clock.observe(4., 104.5) == pytest.approx(104.02)
    assert clock.report()['estimated'] is True
    with pytest.raises(ValueError, match='backwards'):
        clock.observe(3., 105.)
    exact = ClockAlignment(offset_s=10.)
    assert exact.observe(4., 100.) == 14.
    assert not exact.report()['estimated']


def test_seyond_scan_retains_acquisition_times_and_sorts():
    dtype = [(key, 'f4') for key in ('x', 'y', 'z', 'intensity')] + [('timestamp', 'f8')]
    points = np.zeros((1, 5), dtype=dtype)
    points['x'] = [[2, 3, 4, 0, np.nan]]
    points['timestamp'] = [[100.07, 100.0, 100.09, 100.05, 100.02]]
    data, start, end = timed_points(points)
    assert (start, end) == (100., 100.09)
    np.testing.assert_allclose(data['x'], [3, 2, 4])
    np.testing.assert_allclose(data['time'], [0, .07, .09])
    with pytest.raises(ValueError, match='per-point'):
        timed_points(points[['x', 'y', 'z', 'intensity']])
    points['timestamp'] = 1.
    with pytest.raises(ValueError, match='timed scan'):
        timed_points(points)


def test_rgb_cloud_conversion_and_pose_frames():
    pytest.importorskip('graphmap.pose')
    from graphmap.pose import Pose
    from rsim.adapters.ros2.mapping_output import rgb_points, pose_from_ros
    points = np.zeros(3, dtype=[(name, '<f4') for name in ('x', 'y', 'z', 'rgb')])
    points['x'] = [1, 2, np.nan]
    points['rgb'] = np.array([0xFF102030, 0xFFFF0080, 0], dtype='<u4').view('<f4')
    result = rgb_points(points)
    assert result.dtype.names == ('x', 'y', 'z', 'r', 'g', 'b')
    assert result[['r', 'g', 'b']].tolist() == [(16, 32, 48), (255, 0, 128)]
    assert not result.flags.writeable
    native = SimpleNamespace(position=SimpleNamespace(x=2., y=0., z=.04),
                             orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    T_world_imu = pose_from_ros(native, 'odom', 'imu')
    T_base_imu = Pose(position=[0, 0, .04], wrd_frame='base', ego_frame='imu')
    robot = T_world_imu * ~T_base_imu
    assert robot.wrd_frame == 'odom' and robot.ego_frame == 'base'
    np.testing.assert_allclose(robot.position, [2., 0., 0.])


def test_timed_points_stride_precedes_stable_time_order_and_preserves_fields():
    points = np.zeros(7, dtype=[(name, 'f4') for name in ('x', 'y', 'z', 'intensity')]
                      + [('unused_vendor_payload', 'V32'), ('timestamp', 'f8')])
    points['x'] = [0., 1., 2., 3., 4., 5., np.nan]
    points['y'] = [.01, .1, .2, .3, .4, .5, 0.]
    points['intensity'] = np.arange(7)*10
    points['timestamp'] = [100., 100.07, 100.02, 100.01, 100.03, 100.01, 100.09]
    result, start, end = timed_points(points, stride=2)
    # Valid original indices are 1..5; stride retains 1,3,5. Equal times
    # keep their original order, and unused vendor bytes are never published.
    assert (start, end) == (100., 100.09)
    np.testing.assert_array_equal(result['x'], [3., 5., 1.])
    np.testing.assert_allclose(result['y'], [.3, .5, .1])
    np.testing.assert_array_equal(result['intensity'], [30, 50, 10])
    np.testing.assert_allclose(result['time'], [.01, .01, .07])


def test_mapping_ports_do_not_republish_old_maps_and_export_ply(tmp_path):
    from rsim.core import Component
    class Source(Component):
        def __init__(self):
            super().__init__()
            for name in ("pose", "odometry", "map", "rgb_map", "status"):
                setattr(self, name, self.signal(name))
        async def open(self):
            data = np.array([(1., 2., 3., 10, 20, 30)],
                dtype=[(name, '<f4') for name in 'xyz'] + [(name, 'u1') for name in 'rgb'])
            await self.rgb_map.publish(PointCloud(data, 'map'), stamp_ns=10, clock='test')
    async def run():
        source = Source()
        mapper = MappingView(source)
        async with Runtime(mapper):
            frame = await mapper.rgb_map.get(timeout=1)
            # Updating status must not invent a newer physical map.
            await source.status.publish({}, stamp_ns=20, clock='test')
            await mapper.status.get(timeout=1)
            assert await mapper.rgb_map.get() is frame
            path = await mapper.save(tmp_path/'map.ply')
            header, payload = path.read_bytes().split(b'end_header\n')
            assert b'element vertex 1\n' in header
            assert payload == frame.data.points.tobytes()
            with pytest.raises(ValueError, match='ply'):
                await mapper.save(tmp_path/'map.obj')
    asyncio.run(run())


def test_mapping_configuration_requires_explicit_timing_and_keeps_clients_light(tmp_path):
    pytest.importorskip('yaml')
    pytest.importorskip('graphmap.pose')
    from rsim.config import load_mapper
    from rsim.drivers.mapping import Mapper
    with pytest.raises(ValueError, match='clock offsets'):
        Mapper(connection={'host': 'test'}, lidar_ip='test',
               mounts=dict.fromkeys(('body_lidar', 'body_imu', 'body_camera'), {}),
               database=tmp_path/'map.db')
    recipe = tmp_path/'mapping.yaml'
    recipe.write_text('mapping:\n  name: test\n  database: session/map.db\n')
    client = load_mapper(recipe, providers=False)
    assert not hasattr(client, 'factory')
    assert client.component_key == 'mapper:test'
    assert set(client.outputs) == {'pose', 'odometry', 'rgb_map', 'map', 'status'}


def test_mapping_input_failure_and_gap_diagnostics(monkeypatch):
    from rsim.core import Component, PrimaryComponent
    from rsim.adapters.ros2 import mapping_input
    bridge = SimpleNamespace(topic=lambda *args, **kwargs: PrimaryComponent())
    inputs = mapping_input.MappingInputs(Component(), bridge, prefix='/test',
        lidar_driver=Component(), camera_driver=Component(), mounts=[],
        topics={'imu': '/imu', 'odom': '/odom', 'scan': '/scan'},
        timing={'lidar': {'offset_s': 0}, 'chassis': {'offset_s': 0}}, cloud_filter={})
    inputs.started = 0.
    monkeypatch.setattr(mapping_input.time, 'monotonic', lambda: 31.)
    inputs.record_received('lidar')
    inputs.record_received('imu')
    asyncio.run(inputs.health())
    monkeypatch.setattr(mapping_input.time, 'monotonic', lambda: 31.1)
    inputs.record_received('lidar')
    assert inputs.diagnostics()['max_input_gap_s']['lidar'] == pytest.approx(.1)
    monkeypatch.setattr(mapping_input.time, 'monotonic', lambda: 34.2)
    with pytest.raises(RuntimeError, match='lidar'):
        asyncio.run(inputs.health())
    inputs.record_received('lidar')
    with pytest.raises(RuntimeError, match='imu'):
        asyncio.run(inputs.health())


def test_mapping_output_composes_global_pose_and_detects_lost_corrections():
    pytest.importorskip('graphmap.pose')
    Odometry = pytest.importorskip('nav_msgs.msg').Odometry
    from graphmap.pose import Pose
    from rsim.core import Component
    from rsim.adapters.ros2.mapping_output import MappingOutput
    inputs = Component()
    inputs.ros, inputs.prefix = None, '/test'
    inputs.diagnostics = lambda: {}
    T_base_imu = Pose(position=[.1, 0, .04], rotation=[0, 0, 90], degrees=True,
                      wrd_frame='base', ego_frame='imu')
    output = MappingOutput(inputs, frames={'map': 'map', 'odom': 'odom', 'base': 'base'},
        T_base_imu=T_base_imu, database='test.db', assumptions={})
    output._closed = False
    published, transforms = [], []
    output.odom_publisher = SimpleNamespace(publish=published.append)
    output.broadcaster = SimpleNamespace(sendTransform=transforms.append)
    output.correction = Pose(position=[10, 0, 0], wrd_frame='map', ego_frame='odom')
    stamp = time.time_ns()
    native = Odometry()
    native.header.stamp.sec, native.header.stamp.nanosec = divmod(stamp, 10**9)
    native.pose.pose.position.x = 1.
    native.pose.pose.position.z = .04
    native.pose.pose.orientation.w = 1.
    output.odometry_queue.append(native)
    output.corrected(native)
    cloud = PointCloud(np.zeros(1, dtype=[(name, 'f4') for name in 'xyz']), 'map')
    output.latest['rgb_map'] = Frame(cloud, stamp, 'ros:system')
    async def run():
        await output.convert()
        T_odom_imu = Pose(position=[1, 0, .04], wrd_frame='odom', ego_frame='imu')
        expected = T_odom_imu * ~T_base_imu
        np.testing.assert_allclose(output.latest['odometry'].data.matrix, expected.matrix)
        np.testing.assert_allclose(output.latest['pose'].data.matrix, (output.correction * expected).matrix)
        assert transforms[0].child_frame_id == 'base'
        assert published[0].header.frame_id == 'odom'
        # A replayed prediction must not invent a newer pose.
        previous = output.latest['pose']
        output.odometry_queue.append(native)
        await output.convert()
        assert output.latest['pose'] is previous
        await output.emit_status()
        assert output.latest['status'].data['lidar_correction_age_s'] < 1
        output.last_correction_progress -= 6
        with pytest.raises(RuntimeError, match='corrections'):
            await output.emit_status()
    asyncio.run(run())


def test_atomic_laser_correction_health_ignores_delayed_replays(monkeypatch):
    from graphmap.pose import Pose
    from rsim.core import Component
    from rsim.adapters.ros2.mapping_output import MappingOutput
    from rsim.adapters.ros2.laser_mapping import LaserMappingIO
    from rsim.components.mapping import MapLedger

    inputs = Component()
    inputs.ros, inputs.prefix = None, '/test'
    inputs.diagnostics = lambda: {}
    io = LaserMappingIO(ledger=MapLedger(), pose_history=None, keyframe_type=None,
        camera_type=None, colorize=None, T_base_imu=Pose(), camera_prefix='/camera')
    output = MappingOutput(inputs, frames={}, T_base_imu=Pose(), database='test.db',
        assumptions={}, mapping=io)
    io.owner = output
    now = 10 * 10**9
    monkeypatch.setattr(time, 'time_ns', lambda: now)
    monkeypatch.setattr(time, 'monotonic', lambda: now*1e-9)

    def message(stamp):
        sec, nanosec = divmod(stamp, 10**9)
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))

    output._closed = False

    async def run():
        nonlocal now
        output.corrected(message(now - 4 * 10**9))
        # Old acquisition time is acceptable while processing progresses.
        await output.emit_status()
        fresh_stamp = now - 100_000_000
        io.scan(SimpleNamespace(cloud=message(fresh_stamp)))
        # Separate odometry and scan queues can deliver older data afterwards.
        output.corrected(message(now - 5 * 10**9))
        io.scan(SimpleNamespace(cloud=message(now - 4 * 10**9)))
        await output.emit_status()
        status = output.latest['status'].data
        assert status['lidar_correction_age_s'] == pytest.approx(.1)
        assert status['corrected_odometry_age_s'] == pytest.approx(4.)
        assert status['mapping']['cloud_pose_age_s'] == pytest.approx(.1)
        # Replaying a scan never refreshes acquisition time. Once both native
        # correction outputs stop, the original watchdog still rejects it.
        now += 6 * 10**9
        io.scan(SimpleNamespace(cloud=message(fresh_stamp)))
        with pytest.raises(RuntimeError, match='corrections'):
            await output.emit_status()
    asyncio.run(run())


@pytest.mark.parametrize('recorded', [False, True])
def test_mapping_recipe_passes_native_parameters_and_transform(tmp_path, recorded):
    pytest.importorskip('graphmap.pose')
    yaml = pytest.importorskip('yaml')
    from graphmap.pose import Pose
    from rsim.drivers.mapping import mapping_graph
    from rsim.core import Component
    from rsim.adapters.ros2 import RosContext
    class Recorded(Component):
        def __init__(self):
            self.ros = RosContext()
            super().__init__(self.ros)
            self.prefix = '/rsim/test'

        def diagnostics(self):
            return {'source': 'recorded'}

    source = Recorded() if recorded else None
    mounts = {
        'body_imu': dict(position=[0, 0, .04], rotation=[0, 0, 30], degrees=True,
                         wrd_frame='base_footprint', ego_frame='imu'),
        'body_lidar': dict(position=[.1, 0, 1.], rotation=[0, 0, 45], degrees=True,
                           wrd_frame='base_footprint', ego_frame='lidar'),
        'body_camera': dict(wrd_frame='base_footprint', ego_frame='camera_link'),
    }
    graph = mapping_graph(connection={} if recorded else {'host': 'test'}, lidar_ip='test', mounts=mounts,
        database=str(tmp_path/'map.db'), name='test', timing={'lidar': {}, 'chassis': {}},
        topics={}, camera_parameters={}, lidar_parameters={}, cloud_filter={},
        lio_parameters={'lio.sensor.filter_rate': 7}, rtabmap_parameters={'Grid/3D': 'false'},
        input_source=source, start_drivers=not recorded)
    lio = next(child for child in graph.children if getattr(child, 'package', None) == 'super_lio')
    assert lio.parameters['lio.sensor.filter_rate'] == 7
    assert not any(k.startswith(('lio.wheel.', 'lio.planar.')) for k in lio.parameters)
    T_imu_lidar = ~Pose(**mounts['body_imu']) * Pose(**mounts['body_lidar'])
    np.testing.assert_allclose(lio.parameters['lio.extrinsic.lidar_imu'][:3], T_imu_lidar.position)
    np.testing.assert_allclose(lio.parameters['lio.extrinsic.lidar_imu'][3:], T_imu_lidar.matrix[:3, :3].ravel(order='F'))
    assert lio.parameters['lio.extrinsic.odom_robo'][-1] == pytest.approx(30.)
    params = yaml.safe_load((tmp_path/'rtabmap-parameters.yaml').read_text())['/**']['ros__parameters']
    assert params['Grid/3D'] == 'false'
    assert params['database_path'] == str(tmp_path/'map.db')
    assert params['subscribe_rgb'] and params['subscribe_scan_cloud']
    assert params['qos_scan'] == params['qos_odom'] == params['qos_image'] == params['qos_camera_info'] == 1
    assert float(params['Mem/RehearsalSimilarity']) == 1.0
    assert params['Mem/ReduceGraph'] == 'false'
    assert not params['subscribe_depth'] and not params['subscribe_rgbd']
    assert params['Rtabmap/ImagesAlreadyRectified'] == 'true'
    assert params['gen_depth'] and params['Reg/Strategy'] == '2'
    assert lio.parameters['lio.output.cloud_pose']
    if recorded:
        assert graph.ingress is source and graph.ros is source.ros
        assert list(source.children) == [source.ros]
        assert {getattr(child, 'package', None) for child in graph.children} == {None, 'super_lio', 'rtabmap_slam'}
    else:
        camera = next(child for child in graph.ingress.children if getattr(child, 'package', None) == 'realsense2_camera')
        assert not camera.parameters['enable_depth']


@pytest.mark.parametrize('override', [{'Mem/RehearsalSimilarity': '0.6'}, {'Mem/ReduceGraph': 'true'}])
def test_native_keyframe_merging_cannot_silently_change_laser_source_identity(tmp_path, override):
    pytest.importorskip('graphmap.pose')
    from rsim.drivers.mapping import mapping_graph
    mounts = {name: dict(wrd_frame='base_footprint', ego_frame=child)
              for name, child in [('body_imu', 'imu'), ('body_lidar', 'lidar'), ('body_camera', 'camera_link')]}
    with pytest.raises(ValueError, match='stable laser provenance'):
        mapping_graph(connection={'host': 'test'}, lidar_ip='test', mounts=mounts,
            database=str(tmp_path/'map.db'), name='test', timing={'lidar': {}, 'chassis': {}},
            topics={}, camera_parameters={}, lidar_parameters={}, cloud_filter={},
            lio_parameters={}, rtabmap_parameters=override)


def test_native_graph_acknowledges_sources_and_applies_individual_pose_updates(tmp_path):
    pytest.importorskip('graphmap.pose')
    messages = pytest.importorskip('rtabmap_msgs.msg')
    from geometry_msgs.msg import Pose as ROSPose
    from graphmap.pose import Pose
    from rsim.components.mapping import MapLedger, LaserKeyframe, GraphMap
    from rsim.components.projection import PoseHistory, PinholeCamera, colorize_laser
    from rsim.adapters.ros2.laser_mapping import LaserMappingIO
    ledger = MapLedger()
    io = LaserMappingIO(ledger=ledger, pose_history=PoseHistory(), keyframe_type=LaserKeyframe,
        camera_type=PinholeCamera, colorize=colorize_laser,
        T_base_imu=Pose(wrd_frame='base', ego_frame='imu'), camera_prefix='/camera')
    owner = SimpleNamespace(frames=dict(map='map', odom='odom', base='base'),
        database=str(tmp_path/'map.db'), latest={}, correction=None,
        correction_revision={})
    async def publish_port(name, data, stamp, *, metadata=None):
        owner.latest[name] = Frame(data, stamp, 'ros:system', metadata=metadata or {})
    owner.publish_port = publish_port
    io.owner = owner
    stamp = time.time_ns()
    io.pending[stamp] = dict(stamp_ns=stamp, scan_stamp_ns=stamp-10000000,
        xyz=np.array([[1., 0, 0]]), rgba=np.array([[1, 2, 3, 255]], np.uint8),
        pixels=np.array([[3, 4]], np.int32), odometry=Pose(wrd_frame='odom', ego_frame='base'))
    def graph(node_id, x, *, with_node=False):
        message = messages.MapData()
        message.header.frame_id = 'map'
        message.graph.poses_id = [node_id]
        pose = ROSPose()
        pose.position.x, pose.orientation.w = float(x), 1.
        message.graph.poses = [pose]
        message.graph.map_to_odom.rotation.w = 1.
        if with_node:
            node = messages.Node()
            node.id, node.stamp = node_id, stamp/1e9
            message.nodes = [node]
        return message
    async def run():
        io.graphs.append(graph(1, 5, with_node=True))
        await io.process()
        view = GraphMap().update(owner.latest['map'].data)
        source = int(view.source_ids[0])
        view.set_features('test', [source], ['retained'])
        np.testing.assert_allclose(view.infopoints.xyz, [[6, 0, 0]])
        assert not io.pending and io.diagnostics()['map_revision'] == view.revision
        io.last_snapshot = 0
        io.graphs.append(graph(1, 8))  # Complete graph, no repeated sensor payload needed.
        await io.process()
        view.update(owner.latest['map'].data)
        np.testing.assert_allclose(view.infopoints.xyz, [[9, 0, 0]])
        assert view.index_db('test').get(view.voxel_for(source))[0]['value'] == 'retained'
        assert view.snapshot['source_pixels'].tolist() == [[3, 4]]
        assert owner.correction.position[0] == pytest.approx(8.)
        second_stamp = stamp+100_000_000
        io.pending[second_stamp] = dict(stamp_ns=second_stamp, scan_stamp_ns=second_stamp,
            xyz=np.array([[2., 0, 0]]), rgba=np.array([[4, 5, 6, 255]], np.uint8),
            pixels=np.array([[1, 2]], np.int32), odometry=Pose(wrd_frame='odom', ego_frame='base'))
        complete = graph(2, 10, with_node=True)
        complete.nodes[0].stamp = second_stamp/1e9
        complete.graph.poses_id = [1, 2]
        complete.graph.poses = [graph(1, 8).graph.poses[0], complete.graph.poses[0]]
        io.graphs.append(complete)
        await io.process()
        assert ledger.active == {1, 2}
        io.graphs.append(graph(1, 8))  # Explicitly complete: node 2 was retired.
        io.last_snapshot = 0
        await io.process()
        assert ledger.active == {1} and set(ledger.keyframes) == {1, 2}
        view.update(owner.latest['map'].data)
        assert set(view.snapshot['keyframes']) == {'1'}
        assert view.index_db('test').get(view.voxel_for(source))[0]['value'] == 'retained'
        # A node acknowledged by the complete graph still needs its source.
        io.graphs.append(graph(3, 10, with_node=True))
        with pytest.raises(RuntimeError, match='no retained laser observation'):
            await io.process()
    asyncio.run(run())


def test_streaming_graph_is_only_a_notification_until_complete_service_replies():
    from concurrent.futures import Future
    from graphmap.pose import Pose
    from rsim.adapters.ros2.laser_mapping import LaserMappingIO
    from rsim.components.mapping import MapLedger
    io = LaserMappingIO(ledger=MapLedger(), pose_history=None, keyframe_type=None,
        camera_type=None, colorize=None, T_base_imu=Pose(), camera_prefix='/camera')
    future, requests = Future(), []
    def call(request):
        requests.append(request)
        return future
    io.graph_client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    io.graph_request_type = SimpleNamespace
    io.graph('temporary pose and temporary edge')
    assert not io.graphs
    io.poll_graph()
    assert not io.graphs and len(requests) == 1
    assert requests[0].global_map and requests[0].optimized and requests[0].graph_only
    io.poll_graph()
    assert len(requests) == 1  # One in-flight request, without blocking the loop.
    future.set_result(SimpleNamespace(data='authoritative complete graph'))
    io.poll_graph()
    assert list(io.graphs) == ['authoritative complete graph']
    assert io.graph_completed == io.graph_generation == 1
    assert io.stats['graph_responses'] == 1
