"""Recipes reconstruct real DAGs in spawn; ROS activation requires ROS elsewhere."""
import asyncio
from dataclasses import replace
import multiprocessing as mp
from pathlib import Path
import time
import uuid

import pytest

from rsim import Component, Runtime
from rsim.drivers import OdometryFactory, ScanOdometryFactory
from rsim.runtime import PortSpec, SharedProvider, component_ports


IMU_MOUNT = dict(wrd_frame='base_footprint', ego_frame='imu')
SCAN_MOUNT = dict(wrd_frame='base_footprint', ego_frame='laser')


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
    OdometryFactory(IMU_MOUNT, scan_topic='/scan', scan_mount=SCAN_MOUNT)])
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
        assert result['ports'] == recipe.port_specs(history=recipe.history)
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
    dict(sources={'imu': {'kind': 'unknown'}}), dict(parameters={'invalid': None})])
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


def test_worker_reconnects_source_reference_and_transforms_real_shared_samples(tmp_path):
    pytest.importorskip('cyclonedds')
    key = 'factory-imu:' + uuid.uuid4().hex
    recipe = OdometryFactory(IMU_MOUNT, directory=str(tmp_path / 'logs'), sources={'imu': {
        'kind': 'shared_device', 'key': key, 'interface_version': 'imu-probe-v1', 'port': 'imu', 'history': 8}})
    provider = SharedProvider(factory=ImuProvider, key=key, interface_version='imu-probe-v1',
                               ports={'imu': PortSpec(clock='ros:system', history_capacity=8)})
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
