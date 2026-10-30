"""Recipes reconstruct real DAGs in spawn; ROS activation requires ROS elsewhere."""
import asyncio
from dataclasses import dataclass, replace
import multiprocessing as mp
from pathlib import Path
import time
import uuid

import pytest

from rsim import Component, Runtime, CommandSink, CommandEnvelope, CommandRejected, VelocityCommand
from rsim.drivers import OdometryFactory, ScanOdometryFactory, NativeChassisFactory, TopicChassisFactory, MappingFactory
from rsim.runtime import PortSpec, SharedProvider, component_ports


IMU_MOUNT = dict(wrd_frame='base_footprint', ego_frame='imu')
SCAN_MOUNT = dict(wrd_frame='base_footprint', ego_frame='laser')
MAP_MOUNTS = {name: dict(wrd_frame='base_footprint', ego_frame=child) for name, child in
              (('body_imu', 'imu'), ('body_lidar', 'lidar'), ('body_camera', 'camera_optical'))}


def graph_components(root):
    seen = set()
    def visit(node):
        if node in seen:
            return
        seen.add(node)
        for dependency in (*node.dependencies, *(port.producer for port in node.inputs)):
            visit(dependency)
    visit(root)
    return seen


def spawned_build(recipe, queue):
    try:
        from rsim.adapters.ros2 import RosContext
        graph = recipe.build()
        nodes = graph_components(graph)
        queue.put(dict(ports=component_ports(graph), fingerprint=recipe.fingerprint(),
                       contexts=sum(isinstance(c, RosContext) for c in nodes),
                       unopened=all(c._closed and c._runtime is None for c in nodes),
                       files=[str(path) for path in Path(recipe.directory).rglob('*')]))
    except BaseException as error:
        queue.put(error)


@pytest.mark.parametrize('recipe', [OdometryFactory(IMU_MOUNT, history=7),
    ScanOdometryFactory(SCAN_MOUNT, history=9),
    OdometryFactory(IMU_MOUNT, scan_topic='/scan', scan_mount=SCAN_MOUNT),
    NativeChassisFactory(stm32={'start_driver': False}, imu={'topic': '/imu'}, imu_mount=IMU_MOUNT),
    TopicChassisFactory(stm32={}, imu={'topic': '/imu'}, imu_mount=IMU_MOUNT),
    NativeChassisFactory(stm32={'port': '/dev/tty-test'},
        imu={'topic': '/imu', 'port': '/dev/imu-test', 'start_driver': True}, imu_mount=IMU_MOUNT,
        scan={'topic': '/scan', 'port': '/dev/scan-test', 'start_driver': True,
              'filter': {'topic': '/scan-filtered', 'angle_masks': [[-1., -.5]]}},
        scan_mount=SCAN_MOUNT, estimator='scan_imu'),
    MappingFactory(database='/tmp/rsim-unopened-test.db', mounts=MAP_MOUNTS, timing={'lidar': {'offset_s': 0.}}, history=7)])
def test_factory_spawn_build_and_complete_local_spec(tmp_path, recipe):
    recipe = replace(recipe, directory=str(tmp_path / 'not-created'))
    shared = recipe.shared()
    device = recipe.device()
    assert shared.factory.__self__ is recipe
    assert device.interface_version == recipe.INTERFACE_VERSION
    assert shared.provider_version == recipe.fingerprint()
    context = mp.get_context('spawn')
    queue = context.Queue()
    process = context.Process(target=spawned_build, args=(recipe, queue))
    process.start()
    try:
        result = queue.get(timeout=20)
        if isinstance(result, BaseException):
            raise result
        assert result['ports'] == shared.port_specs
        assert result['fingerprint'] == recipe.fingerprint()
        assert result['contexts'] == 1
        assert result['unopened'] and result['files'] == []
        process.join(timeout=5)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5)
        queue.close()


def test_strict_yaml_toml_relative_paths_and_immutable_defaults(tmp_path, monkeypatch):
    yaml = tmp_path / 'recipe.yaml'
    yaml.write_text('schema_version: 1\nodometry:\n  directory: ./logs\n  history: 7\n'
                    '  imu_mount: {wrd_frame: base_footprint, ego_frame: imu}\n'
                    '  parameters: {frequency: 40.0}\n')
    toml = tmp_path / 'recipe.toml'
    toml.write_text('schema_version = 1\n[odometry]\ndirectory = "./logs"\nhistory = 7\n'
                    '[odometry.imu_mount]\nwrd_frame = "base_footprint"\nego_frame = "imu"\n'
                    '[odometry.parameters]\nfrequency = 40.0\n')
    monkeypatch.chdir('/')
    first = OdometryFactory.from_config(yaml)
    second = OdometryFactory.from_config(toml)
    assert first == second
    assert first.directory == str(tmp_path / 'logs')
    assert not Path(first.directory).exists()
    assert first.fingerprint() == replace(first, directory=str(tmp_path / 'elsewhere')).fingerprint()
    assert first.fingerprint() != replace(first, history=8).fingerprint()
    assert first.fingerprint() != OdometryFactory.from_config(yaml, {'parameters': {'frequency': 50.}}).fingerprint()
    yaml.write_text('odometry: {unexpected: 1}\n')
    with pytest.raises(ValueError, match='unknown'):
        OdometryFactory.from_config(yaml)
    for text in ('odometry: {}\nodometry: {}\n', 'odometry: !!python/object/apply:os.system ["false"]\n'):
        yaml.write_text(text)
        with pytest.raises(Exception):
            OdometryFactory.from_config(yaml)
    toml.write_text('schema_version = 1\nschema_version = 1\n')
    with pytest.raises(ValueError):
        OdometryFactory.from_config(toml)


@pytest.mark.parametrize('settings', [dict(history=True), dict(history=0), dict(input_timeout=float('nan')),
    dict(use_orientation='false'), dict(schema_version=2), dict(imu_options={'misspelled': 2}),
    dict(sources={'imu': {'kind': 'unknown'}}), dict(parameters={'invalid': None}),
    dict(parameters={'publish_tf': True}), dict(parameters={'frequency': -1}),
    dict(scan_parameters={'unused': 1}), dict(imu_options={'gyro_bias': ['bad', 0, 0]})])
def test_odometry_invalid_config_rejected(settings):
    with pytest.raises((TypeError, ValueError)):
        OdometryFactory(IMU_MOUNT, **settings)


def test_live_dependencies_are_injected_and_never_retained_by_recipe(tmp_path):
    from rsim.adapters.ros2 import RosContext
    source = Component()
    wheel, imu = source.signal('wheel', clock='ros:system'), source.signal('imu', clock='ros:system')
    ros = RosContext()
    recipe = OdometryFactory(IMU_MOUNT, directory=str(tmp_path / 'logs'))
    graph = recipe.build(wheel=wheel, imu=imu, ros=ros)
    assert graph.ros is ros and graph.wheel is wheel and graph.body_imu.source is imu
    assert len([c for c in graph_components(graph) if isinstance(c, RosContext)]) == 1
    with pytest.raises((ValueError, TypeError)):
        OdometryFactory(IMU_MOUNT, sources={'imu': imu})
    assert recipe.shared().factory.__self__ is recipe
    assert not Path(recipe.directory).exists()


class ImuProvider(Component):
    def __init__(self):
        super().__init__()
        self.imu = self.signal('imu', history=8, clock='ros:system')

    async def open(self):
        await self.emit()
        self.task('imu', self.emit, hz=50)

    async def emit(self):
        await self.imu.publish(dict(header={'frame_id': 'imu'},
            orientation=dict(x=0., y=0., z=0., w=1.), orientation_covariance=[0.]*9,
            angular_velocity=dict(x=0., y=0., z=.3), angular_velocity_covariance=[0.]*9,
            linear_acceleration=dict(x=0., y=0., z=9.8), linear_acceleration_covariance=[0.]*9),
            stamp_ns=time.time_ns(), clock='ros:system')


from rsim.core import PrimaryComponent


class LegacyImuProvider(PrimaryComponent):
    def __init__(self):
        super().__init__(output_name='imu', history=8, clock='ros:system')
        self.imu = self.output

    open = ImuProvider.open
    emit = ImuProvider.emit


def spawned_reference_consumer(recipe, queue):
    async def run():
        graph = recipe.build()
        # Activate the real ROS-independent mounted-IMU branch of the built DAG.
        # The native EKF itself needs ROS and is intentionally not activated here.
        async with Runtime(graph.body_imu):
            frame = await graph.body_imu.imu.get(timeout=10)
            queue.put((frame.data, frame.clock))
    try:
        asyncio.run(run())
    except BaseException as error:
        queue.put(error)


@pytest.mark.parametrize('protocol', ['ports', 'sensor'])
def test_worker_reconnects_source_reference_and_transforms_real_shared_samples(tmp_path, protocol):
    pytest.importorskip('cyclonedds')
    key = 'factory-imu:' + uuid.uuid4().hex
    if protocol == 'ports':
        reference = dict(kind='shared_device', key=key, interface_version='imu-probe-v1', port='imu', history=8)
        provider = SharedProvider(factory=ImuProvider, key=key, interface_version='imu-probe-v1',
                                   ports={'imu': PortSpec(clock='ros:system', history_capacity=8)})
    else:
        from rsim.runtime.host import SharedSensor
        reference = dict(kind='shared_sensor', key=key, version='imu-probe-v1', port='imu', history=8)
        provider = SharedSensor(factory=LegacyImuProvider, key=key, version='imu-probe-v1', history=8, output_name='imu')
    recipe = OdometryFactory(IMU_MOUNT, directory=str(tmp_path / 'logs'), sources={'imu': reference})
    context = mp.get_context('spawn')
    queue = context.Queue()
    process = context.Process(target=spawned_reference_consumer, args=(recipe, queue))

    async def run():
        async with Runtime(provider, _root_ports=False):
            process.start()
            result = await asyncio.to_thread(queue.get, True, 20)
            if isinstance(result, BaseException):
                raise result
            data, clock = result
            assert clock == 'ros:system'
            assert data['header']['frame_id'] == 'base_footprint'
            assert data['angular_velocity']['z'] == pytest.approx(.3)
            assert data['linear_acceleration']['z'] == pytest.approx(0)
            await asyncio.to_thread(process.join, 5)
            assert process.exitcode == 0
        async with asyncio.timeout(5):
            while provider.directory.exists():
                await asyncio.sleep(.02)
    try:
        asyncio.run(run())
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5)
        queue.close()


def test_native_config_paths_digests_and_no_constructor_outputs(tmp_path, monkeypatch):
    params = tmp_path / 'driver.yaml'
    params.write_text('/**:\n  ros__parameters:\n    max_command_ttl: 0.5\n')
    config = tmp_path / 'robot.yaml'
    config.write_text('schema_version: 1\nchassis:\n  directory: ./logs\n'
        '  stm32: {port: ./motor, ros_args: [--params-file, ./driver.yaml]}\n'
        '  imu: {topic: /imu}\n  imu_mount: {wrd_frame: base_footprint, ego_frame: imu}\n')
    monkeypatch.chdir('/')
    recipe = NativeChassisFactory.from_config(config)
    assert recipe.stm32['port'] == str(tmp_path / 'motor')
    assert recipe.stm32['ros_args'][-1] == str(params)
    assert str(params) in recipe._resources
    assert not (tmp_path / 'logs').exists()
    with pytest.raises(ValueError, match='prepare'):
        recipe.build()
    params.write_text('/changed: {}\n')
    with pytest.raises(ValueError, match='resource changed'):
        recipe.build()
    assert params.read_text() == '/changed: {}\n'


@pytest.mark.parametrize('settings', [dict(stm32={'start_driver': True, 'port': '/dev/test'}),
    dict(stm32={'port': '/dev/test'}), dict(imu={'topic': '/imu', 'start_driver': True}),
    dict(imu={'topic': '/imu', 'parameters': {'baudrate': 1}}),
    dict(scan={'topic': '/scan', 'port': '/dev/test'}), dict(motion_enabled='true')])
def test_topic_factory_rejects_hardware_and_invalid_permission(settings):
    values = dict(stm32={}, imu={'topic': '/imu'}, imu_mount=IMU_MOUNT)
    values.update(settings)
    with pytest.raises((TypeError, ValueError)):
        TopicChassisFactory(**values)


def test_topic_graph_contains_no_hardware_owner_and_one_ros_context(tmp_path):
    from rsim.adapters.ros2 import Driver, RosContext
    recipe = TopicChassisFactory(stm32={}, imu={'topic': '/imu'}, imu_mount=IMU_MOUNT,
        scan={'topic': '/scan'}, scan_mount=SCAN_MOUNT, estimator='scan_imu', directory=str(tmp_path / 'logs'))
    graph = recipe.build()
    nodes = graph_components(graph)
    assert {node.package for node in nodes if isinstance(node, Driver)} == {'robot_localization', 'rtabmap_odom'}
    assert sum(isinstance(node, RosContext) for node in nodes) == 1
    assert all(node._closed and node._runtime is None for node in nodes)
    assert not Path(recipe.directory).exists()


class LocalMotor(Component):
    def __init__(self):
        super().__init__()
        self.odom = self.signal('odom', history=128, clock='ros:system')
        self.state = self.signal('state', history=128, clock='ros:system')
        self.velocity = CommandSink(self, 'velocity', self.apply, safe=self.safe,
                                    fallback=VelocityCommand(), max_ttl=.5)
        self.topics = {'odom': '/wheel'}
        self.received = []
        self.stopped = 0

    async def apply(self, envelope):
        self.received.append(envelope)
        data = {name: getattr(envelope, name) for name in ('controller_id', 'controller_epoch', 'sequence', 'deadline_ns')}
        data['speed'] = envelope.value.linear_x
        await self.state.publish(data, stamp_ns=time.time_ns(), clock='ros:system')
        return {'accepted': True}

    async def safe(self, value):
        self.stopped += 1


class LocalEstimate(Component):
    def __init__(self):
        super().__init__()
        self.pose = self.signal('pose', history=512, clock='ros:system')
        self.odometry = self.signal('odometry', history=512, clock='ros:system')


def injected_robot(recipe):
    return recipe.build(chassis=LocalMotor(), odometry=LocalEstimate(),
                        imu=Component().signal('imu', clock='ros:system'))


@dataclass(frozen=True)
class NativeFixtureRecipe:
    """Only native hardware/estimator dependencies are replaced in this test."""
    recipe: NativeChassisFactory

    def build(self):
        return injected_robot(self.recipe)


@pytest.mark.parametrize('motion_enabled', [False, True])
def test_native_composite_permissions_ownership_deadline_and_stop(motion_enabled):
    recipe = NativeChassisFactory(stm32={'start_driver': False}, imu={'topic': '/imu'},
                                  imu_mount=IMU_MOUNT, motion_enabled=motion_enabled)
    robot = injected_robot(recipe)
    async def run():
        async with Runtime(robot):
            assert robot.control._closed  # Public sink has no hidden competing controller.
            envelope = CommandEnvelope(VelocityCommand(.1), 'test-owner', 'test-epoch', 1,
                                       time.monotonic_ns() + 300_000_000)
            if motion_enabled:
                await robot.velocity.set(envelope)
                assert robot.chassis.received == [envelope]
                with pytest.raises(CommandRejected, match='owned'):
                    await robot.chassis.velocity.set(VelocityCommand(.2), ttl=.1)
            else:
                with pytest.raises(CommandRejected, match='motion_enabled'):
                    await robot.velocity.set(envelope)
                assert robot.chassis.received == []
        assert robot.chassis.stopped == int(motion_enabled)
        controlled = injected_robot(recipe)
        async with Runtime(controlled.control):
            with pytest.raises(CommandRejected, match='owned'):
                await controlled.velocity.set(VelocityCommand(), ttl=.1)
    asyncio.run(run())


def test_native_shared_composite_forwards_original_envelope_and_service_reuses_owner():
    pytest.importorskip('cyclonedds')
    recipe = NativeChassisFactory(stm32={'start_driver': False}, imu={'topic': '/imu'},
                                  imu_mount=IMU_MOUNT, motion_enabled=True)
    key = 'native-composite:' + uuid.uuid4().hex
    provider = SharedProvider(factory=NativeFixtureRecipe(recipe).build, key=key,
                               ports=recipe._specs(), interface_version=recipe.INTERFACE_VERSION)
    client = recipe.device(key=key)
    service = recipe.service(source=client)
    assert service.controller.velocity is client.velocity
    assert service.controller.pose._target is client.pose

    async def run():
        async with Runtime(provider, _root_ports=False), Runtime(client, client.velocity):
            envelope = CommandEnvelope(VelocityCommand(.2), 'source', 'epoch', 7,
                                       time.monotonic_ns() + 400_000_000)
            await client.velocity.set(envelope)
            state = (await client.state.get(timeout=5)).data
            for name in ('controller_id', 'controller_epoch', 'sequence', 'deadline_ns'):
                assert state[name] == getattr(envelope, name)
            assert state['speed'] == .2
        async with asyncio.timeout(5):
            while provider.directory.exists():
                await asyncio.sleep(.02)
    asyncio.run(run())


def test_native_rejects_two_injected_ros_contexts():
    from rsim.adapters.ros2 import RosContext
    recipe = NativeChassisFactory(stm32={'start_driver': False}, imu={'topic': '/imu'}, imu_mount=IMU_MOUNT)
    first, second = RosContext(), RosContext()
    imu = Component(first).signal('imu', clock='ros:system')
    with pytest.raises(ValueError, match='one RosContext'):
        recipe.build(ros=second, imu=imu)


def test_native_scan_estimator_consumes_the_injected_wheel_and_scan_sources():
    recipe = NativeChassisFactory(stm32={'start_driver': False}, imu={'topic': '/imu'},
        imu_mount=IMU_MOUNT, scan={'topic': '/scan'}, scan_mount=SCAN_MOUNT, estimator='scan_imu')
    chassis = LocalMotor()
    source = Component()
    imu = source.signal('imu', clock='ros:system')
    scan = source.signal('scan', clock='ros:system')
    graph = recipe.build(chassis=chassis, imu=imu, scan=scan)
    assert graph.odometry.scan.sources == {'wheel': chassis.odom, 'scan': scan}
    scan_recipe = ScanOdometryFactory(SCAN_MOUNT, sources={'scan': {
        'kind': 'shared_device', 'key': 'scan:existing', 'interface_version': 'scan-v1', 'port': 'scan'}})
    rebuilt = scan_recipe.shared().factory()
    assert rebuilt.sources['scan'].producer.component_key == 'scan:existing'
    assert not hasattr(rebuilt.sources['scan'].producer, 'factory')


def mapping_recipe(tmp_path, **settings):
    return MappingFactory(database=str(tmp_path / 'persistent' / 'map.db'), mounts=MAP_MOUNTS,
                          timing={'lidar': {'offset_s': 0.}}, **settings)


def test_mapping_config_snapshot_digest_and_relative_user_assets(tmp_path, monkeypatch):
    geometry = tmp_path / 'geometry.yaml'
    geometry.write_text('sensor_params:\n  lidar3d: {position: [0, 0, 1], rotation: [0, 0, 0]}\n'
                        '  rgb: {position: [0, 0, 0], rotation: [0, 0, 0]}\n')
    config = tmp_path / 'mapping.yaml'
    config.write_text('schema_version: 1\nmapping:\n  name: test\n  database: data/map.db\n'
        '  directory: logs\n  geometry_file: geometry.yaml\n  timing: {lidar: {offset_s: 0.0}}\n'
        '  mounts: {body_imu: {wrd_frame: base_footprint, ego_frame: imu}}\n')
    monkeypatch.chdir('/')
    recipe = MappingFactory.from_config(config)
    assert recipe.database == str(tmp_path / 'data' / 'map.db')
    assert recipe.directory == str(tmp_path / 'logs')
    assert recipe.geometry_file == str(geometry)
    graph = recipe.build()
    assert not (tmp_path / 'data').exists() and not Path(recipe.directory).exists()
    assert set(graph.outputs) == set(recipe.port_specs())
    assert recipe.fingerprint() == replace(recipe, directory=str(tmp_path / 'different')).fingerprint()
    geometry.write_text(geometry.read_text() + '# changed revision\n')
    with pytest.raises(ValueError, match='resource changed'):
        recipe.build()
    assert geometry.exists()


@pytest.mark.parametrize('settings', [dict(history=0), dict(schema_version=2), dict(database_policy='reuse'),
    dict(topics={'typo': '/imu'}), dict(map_options={'misspelled': 2}), dict(cloud_filter={'stride': True}),
    dict(rtabmap_parameters={'database_path': '/another.db'}), dict(map_options={'keyframe_hz': 0}),
    dict(sources={'lidar': {'kind': 'ros_topic', 'topic': '/cloud'}}),
    dict(sources={'camera': {'kind': 'shared_device', 'key': 'c', 'interface_version': 'c-v1'}})])
def test_mapping_rejects_invalid_or_ambiguous_configuration(tmp_path, settings):
    with pytest.raises((ValueError, TypeError)):
        mapping_recipe(tmp_path, **settings)


def test_mapping_reconstructs_explicit_device_refs_without_launching_owners(tmp_path):
    from rsim.runtime import SharedComponent
    refs = {name: dict(kind='shared_device', key='existing:' + name, port=name,
                       interface_version='fixture-v1', history=8) for name in
            ('imu', 'wheel', 'scan', 'lidar', 'camera', 'camera_info')}
    recipe = mapping_recipe(tmp_path, sources=refs, history=11)
    graph = recipe.shared().factory()
    refs['imu']['key'] = 'mutated'
    nodes = graph_components(graph)
    devices = [c for c in nodes if isinstance(c, SharedComponent)]
    assert {d.component_key for d in devices} == {'existing:' + name for name in refs}
    assert all(not hasattr(d, 'factory') for d in devices)
    assert all(port.history_size == 11 for port in graph.outputs.values())
    assert graph.mapping.camera_prefix == '/rsim/mapping/inputs/camera'
    lio = next(c for c in nodes if getattr(c, 'package', None) == 'super_lio')
    assert lio.parameters['lio.ros.imu_topic'] == '/rsim/mapping/imu'
    assert not Path(recipe.database).parent.exists()
    assert recipe.device().interface_version == 'mapping-ports-v2'


def test_mapping_preserves_device_clocks_for_existing_sensor_providers(tmp_path):
    from rsim.runtime.host import SharedSensor
    refs = {
        'lidar': dict(kind='shared_sensor', key='robin:front', version='robin-v1', port='points',
                      clock='device:robin:front', history=8),
        'camera': dict(kind='shared_sensor', key='d435:serial', version='camera-config-v1', port='output',
                       clock='device:d435:serial', history=8, bundle_stream='color'),
        'camera_info': dict(kind='shared_device', key='camera-info', interface_version='calibration-v1',
                            port='info', clock='device:d435:serial'),
    }
    with pytest.raises(ValueError, match='camera offset_s'):
        mapping_recipe(tmp_path, sources=refs)
    recipe = MappingFactory(database=str(tmp_path / 'map.db'), mounts=MAP_MOUNTS, sources=refs,
        timing={'lidar': {'offset_s': .2}, 'camera': {'offset_s': .1}})
    graph = recipe.build()
    assert graph.ingress.extra_sources['lidar'].clock == 'device:robin:front'
    assert graph.ingress.extra_sources['camera'].clock == 'device:d435:serial'
    assert graph.ingress.camera_offset_ns == 100_000_000
    sensors = [node for node in graph_components(graph) if isinstance(node, SharedSensor)]
    assert {node.source_key for node in sensors} == {'robin:front', 'd435:serial'}
    assert all(node.factory is None for node in sensors)


def test_mapping_reuses_injected_inputs_and_ros_owner(tmp_path):
    from rsim.adapters.ros2 import RosContext, RosSensor
    ros = RosContext()
    imu = RosSensor('/custom/imu', 'imu', ros=ros, clock='ros:system')
    recipe = mapping_recipe(tmp_path)
    graph = recipe.build(sources={'imu': imu.imu})
    assert graph.ros is ros
    assert graph.ingress.remote['imu'] is imu.imu
    assert len([node for node in graph_components(graph) if isinstance(node, RosContext)]) == 1
    with pytest.raises(ValueError, match='one RosContext'):
        recipe.build(ros=RosContext(), sources={'imu': imu.imu})
    with pytest.raises((ValueError, TypeError)):
        mapping_recipe(tmp_path, sources={'imu': imu.imu})


def database_competitor(path, queue):
    from rsim.adapters.ros2.mapping_resources import MappingDatabase
    async def run():
        async with Runtime(MappingDatabase(path)):
            queue.put('acquired')
    try:
        asyncio.run(run())
    except BaseException as error:
        queue.put(str(error))


def test_mapping_database_ownership_failure_close_and_persistent_assets(tmp_path):
    from rsim.adapters.ros2.mapping_resources import MappingDatabase
    recipe = mapping_recipe(tmp_path)
    graph = recipe.build()
    asset = graph.database_session
    assert graph.dependencies[0] is asset
    assert not asset.path.parent.exists()
    context = mp.get_context('spawn')
    queue = context.Queue()
    process = context.Process(target=database_competitor, args=(recipe.database, queue))
    async def run():
        async with Runtime(asset):
            assert asset.path.parent.is_dir()
            process.start()
            message = await asyncio.to_thread(queue.get, True, 10)
            assert 'already owned' in message
            await asyncio.to_thread(process.join, 5)
            assert process.exitcode == 0
            asset.path.write_bytes(b'user database content')
            archive = asset.path.with_suffix('.laser')
            archive.mkdir()
            (archive / 'owned-by-user').write_text('preserve')
        await asset.close()
        await asset.close()
        assert asset.path.read_bytes() == b'user database content'
        assert (archive / 'owned-by-user').read_text() == 'preserve'
        with pytest.raises(ValueError, match='new database'):
            await asset.open()
        assert asset.lease is None
        assert asset.path.read_bytes() == b'user database content'
        # A failed owner has released its lease; a second attempt reaches the
        # same content policy, rather than reporting an orphaned ownership lock.
        fresh = MappingDatabase(asset.path)
        with pytest.raises(ValueError, match='new database'):
            await fresh.open()
        assert fresh.lease is None
    try:
        asyncio.run(run())
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5)
        queue.close()


def test_mapping_resource_rollback_releases_owner_after_later_open_failure(tmp_path):
    from rsim.adapters.ros2.mapping_resources import MappingDatabase
    asset = MappingDatabase(tmp_path / 'session' / 'map.db')
    class Failing(Component):
        async def open(self):
            raise RuntimeError('downstream failed')
    async def run():
        with pytest.raises(RuntimeError, match='downstream failed'):
            async with Runtime(Failing(asset)):
                pass
        assert asset.lease is None
        async with Runtime(MappingDatabase(asset.path)):
            pass
        assert asset.path.parent.exists() and not asset.path.exists()
    asyncio.run(run())


def test_mapping_protects_orphan_archive_and_same_stem_database_alias(tmp_path):
    from rsim.core import ComponentError
    from rsim.adapters.ros2.mapping_resources import MappingDatabase
    first, second = MappingDatabase(tmp_path / 'map.db'), MappingDatabase(tmp_path / 'map.sqlite')
    async def run():
        async with Runtime(first):
            with pytest.raises(ComponentError, match='already owned'):
                await second.open()
            assert second.lease is second.archive_lease is None
        first.archive.mkdir()
        (first.archive / 'retained').write_bytes(b'preserve')
        with pytest.raises(ValueError, match='new archive'):
            await second.open()
        assert second.lease is second.archive_lease is None
        await second.close()
        assert (first.archive / 'retained').read_bytes() == b'preserve'
        assert not first.path.exists() and not second.path.exists()
    asyncio.run(run())


def test_mapping_shared_lidar_branch_preserves_points_and_reception_time(tmp_path):
    import numpy as np
    from rsim import PointCloud, Image
    from rsim.adapters.ros2.mapping_input import rgb_message_fields, timed_points
    source = Component()
    points = source.signal('points', clock='device:robin:front')
    graph = mapping_recipe(tmp_path).build(sources={'lidar': points})
    data = np.zeros(40, dtype=[(name, 'f4') for name in ('x', 'y', 'z', 'intensity')] + [('timestamp', 'f8')])
    data['x'] = 1
    data['timestamp'] = np.linspace(100., 100.1, len(data))
    payload = PointCloud(data, 'lidar')
    async def run():
        async with Runtime(source):
            await points.publish(payload, stamp_ns=100_100_000_000, clock=points.clock, received_ns=200_000_000_000)
            await graph.ingress.shared_inputs()
            sample, received = graph.ingress.pending.popleft()
            assert sample is payload and received == 200.
            converted, start, end = timed_points(sample.points)
            assert (start, end) == (100., 100.1)
            assert converted['time'][-1] == pytest.approx(.1)
    asyncio.run(run())
    pixels = np.arange(72, dtype=np.uint8).reshape(4, 6, 3)[:, ::2]
    fields = rgb_message_fields(Image(pixels, 'rgb8', 'camera_optical'))
    assert fields['width'] == 3 and fields['height'] == 4 and fields['step'] == 9
    assert fields['data'] == np.ascontiguousarray(pixels).tobytes()
    with pytest.raises(ValueError, match='RGB/BGR'):
        rgb_message_fields(Image(pixels, 'mono8', 'camera_optical'))


def test_native_hardware_rejects_source_bound_instead_of_dropping_it():
    from rsim.core.commands import SourceCommandBound
    from rsim.core.lockstep import StepContext
    from rsim.adapters.ros2.stm32 import Stm32Chassis
    recipe = NativeChassisFactory(stm32={'start_driver': False}, imu={'topic': '/imu'},
                                  imu_mount=IMU_MOUNT, motion_enabled=True)
    robot = injected_robot(recipe)
    envelope = CommandEnvelope(VelocityCommand(.1), 'controller', 'epoch', 1,
        time.monotonic_ns() + 400_000_000, SourceCommandBound(StepContext('sim', 0, 0), 'sim', 10))
    async def run():
        async with Runtime(robot):
            with pytest.raises(CommandRejected, match='source context'):
                await robot.velocity.set(envelope)
            assert robot.chassis.received == []
        owner = object.__new__(Stm32Chassis)
        with pytest.raises(CommandRejected, match='source context'):
            await owner._apply_velocity(envelope)
    asyncio.run(run())
