"""Configuration and ownership checks without native ROS or physical devices."""
from contextlib import contextmanager
from dataclasses import FrozenInstanceError, replace
import multiprocessing
import os
import subprocess
import sys
from types import SimpleNamespace

import pytest

from rsim.core import Component
from rsim.core.commands import CommandSink, VelocityCommand
from rsim.drivers import (
    Camera, CameraFactory, D435, D435Factory, RobinW, RobinWFactory,
    ROS2Topic, ROS2TopicFactory, Hipnuc, HipnucFactory,
    BlueSea, BlueSeaFactory, STM32, STM32Factory, Chassis, ChassisFactory,
)
from rsim.runtime.sharing import validate_component_ports


def test_recipes_build_without_importing_optional_hardware_or_opening_resources():
    code = '''
import sys
from importlib.abc import MetaPathFinder
class RejectHardware(MetaPathFinder):
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'rclpy', 'serial', 'cv2', 'mujoco'}:
            raise AssertionError('hardware imported during build: ' + fullname)
sys.meta_path.insert(0, RejectHardware())
from rsim.drivers import (CameraFactory, D435Factory, RobinWFactory,
                         HipnucFactory, ROS2TopicFactory, STM32Factory)
recipes = [CameraFactory(), D435Factory(), RobinWFactory(),
           HipnucFactory('/unused/serial'), ROS2TopicFactory('/imu', 'imu'),
           STM32Factory(start_driver=False)]
for recipe in recipes:
    first, second = recipe.build(), recipe.build()
    assert first is not second
    assert first._runtime is None
    assert second._runtime is None
'''
    result = subprocess.run([sys.executable, '-c', code], text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr


def test_port_specs_match_every_built_component_and_are_independent(tmp_path):
    port = tmp_path / 'serial'
    port.touch()
    cases = [
        (CameraFactory(history=7), CameraFactory.port_specs(history=7)),
        (D435Factory(history=7), D435Factory.port_specs(history=7)),
        (RobinWFactory(ip='lidar.example', history=7), RobinWFactory.port_specs(ip='lidar.example', history=7)),
        (HipnucFactory(str(port), history=7), HipnucFactory.port_specs(history=7)),
        (BlueSeaFactory(str(port), history=7), BlueSeaFactory.port_specs(history=7)),
        (ROS2TopicFactory('/scan', 'scan', history=7), ROS2TopicFactory.port_specs('scan', history=7)),
        (STM32Factory(start_driver=False, history=7, parameters={'max_command_ttl': .2}),
         STM32Factory.port_specs(history=7, max_ttl=.2)),
    ]
    for recipe, specs in cases:
        validate_component_ports(recipe.build(), specs)
        specs.clear()
        assert recipe.port_specs('scan') if isinstance(recipe, ROS2TopicFactory) else recipe.port_specs()


def test_nested_configuration_is_detached_immutable_and_not_a_resource_container():
    parameters = {'publish_tf': False, 'indices': [1, 2]}
    recipe = D435Factory(parameters=parameters)
    before = recipe.fingerprint()
    parameters['indices'].append(3)
    parameters['publish_tf'] = True
    assert recipe.fingerprint() == before
    assert recipe.parameters['indices'] == (1, 2)
    with pytest.raises(FrozenInstanceError):
        recipe.history = 33
    with pytest.raises(TypeError):
        recipe.parameters['publish_tf'] = True
    with pytest.raises(TypeError, match='inject resources'):
        ChassisFactory(control={'resource': Component()})
    assert replace(recipe, history=9).fingerprint() != before
    assert replace(recipe, log_path='different.log').fingerprint() == before
    assert STM32Factory(motion_enabled=True).fingerprint() != STM32Factory().fingerprint()
    assert STM32Factory(parameters={'max_command_ttl': .1}).fingerprint() != STM32Factory().fingerprint()


def test_fingerprint_is_stable_in_an_independent_interpreter():
    recipe = D435Factory(parameters={'indices': [1, 2], 'publish_tf': False})
    code = "from rsim.drivers import D435Factory; print(D435Factory(parameters={'publish_tf': False, 'indices': [1, 2]}).fingerprint())"
    environment = dict(os.environ, PYTHONHASHSEED='random')
    result = subprocess.check_output([sys.executable, '-c', code], env=environment, text=True, timeout=20)
    assert result.strip() == recipe.fingerprint()


def test_shared_and_device_do_not_build_and_keep_existing_client_keys(monkeypatch, tmp_path):
    port = tmp_path / 'serial'
    port.touch()
    cases = [CameraFactory(), D435Factory(), RobinWFactory(), HipnucFactory(str(port)),
             BlueSeaFactory(str(port)), ROS2TopicFactory('/imu', 'imu')]
    for recipe in cases:
        def forbidden(*args, **kwargs):
            raise AssertionError('build executed during declaration/connection')
        monkeypatch.setattr(type(recipe), 'build', forbidden)
        provider, client = recipe.shared(), recipe.device()
        if isinstance(recipe, D435Factory):
            provider, client = provider.source.producer, client.source.producer
        assert provider.source_key == client.source_key
        assert provider.version == client.version
        assert provider.factory is not None and client.factory is None


def test_compatibility_functions_delegate_to_recipe_implementations(tmp_path):
    port = tmp_path / 'serial'
    port.touch()
    cases = [
        (Camera(), CameraFactory), (D435(), D435Factory), (RobinW(), RobinWFactory),
        (Hipnuc(str(port)), HipnucFactory), (BlueSea(str(port)), BlueSeaFactory),
        (ROS2Topic('/imu', 'imu'), ROS2TopicFactory),
    ]
    for provider, recipe_type in cases:
        if recipe_type is D435Factory:
            provider = provider.source.producer
        assert isinstance(provider.factory.__self__, recipe_type)
    source = STM32(start_driver=False)
    validate_component_ports(source, STM32Factory.port_specs())


def test_native_preparation_is_explicit_and_legacy_wrapper_prepares(monkeypatch, tmp_path):
    from rsim.adapters.ros2.arguments import RosArguments
    calls = []

    @contextmanager
    def resolve(arguments, node_name, namespace='/'):
        calls.append(node_name)
        values = dict(arguments.parameters)
        values['serial_no'] = '_012345'
        yield SimpleNamespace(
            list_parameters=lambda *args, **kwargs: SimpleNamespace(names=list(values)),
            get_parameter=lambda name: SimpleNamespace(value=values[name]),
            resolve_topic_name=lambda topic: '/prepared/' + topic.removeprefix('~/'),
        )

    monkeypatch.setattr(RosArguments, 'resolve', resolve)
    recipe = D435Factory(ros_args=['-p', "serial_no:='_012345'"])
    assert calls == []
    with pytest.raises(ValueError, match='prepare'):
        recipe.build()
    assert calls == []
    prepared = recipe.prepare()
    assert calls == ['camera']
    graph = prepared.build()
    assert graph.sources['depth'].producer.topic == '/prepared/depth/image_rect_raw'
    assert calls == ['camera']
    provider = D435(ros_args=['-p', "serial_no:='_012345'"])
    assert provider.source.producer.source_key == 'd435:012345'
    assert calls == ['camera', 'camera']
    path = tmp_path / 'parameters.yaml'
    path.write_text('/**:\n  ros__parameters:\n    publish_tf: false\n')
    snapshot = D435Factory(ros_args=['--params-file', str(path)]).prepare()
    path.write_text('/**:\n  ros__parameters:\n    publish_tf: true\n')
    with pytest.raises(ValueError, match='file changed'):
        snapshot.build()


def test_rpc_chassis_injects_live_ports_without_storing_them_in_recipe():
    owner = Component()
    pose = owner.signal('pose', clock='test:pose')
    velocity = CommandSink(owner, 'velocity', lambda envelope: None,
                           fallback=VelocityCommand(), max_ttl=.5)
    recipe = ChassisFactory(control={'command_ttl': .2})
    service = recipe.build(pose=pose, velocity=velocity)
    assert service.controller.pose._target is pose
    assert service.controller.velocity is velocity
    assert not service.controller.motion_enabled
    validate_component_ports(service, recipe.port_specs())
    compatible = Chassis(pose=pose, velocity=velocity, control={'command_ttl': .2})
    assert type(compatible) is type(service)
    assert type(recipe.device()).__module__ == 'rsim.devices.chassis'
    with pytest.raises(ValueError):
        recipe.build(pose=pose, velocity=velocity, robot=object())


def _build_worker(recipe, queue):
    source = recipe.build()
    queue.put((os.getpid(), recipe.fingerprint(), sorted(source.outputs)))


def test_module_recipe_build_survives_real_spawn_without_resources():
    recipe = D435Factory(history=5)
    context = multiprocessing.get_context('spawn')
    queue = context.Queue()
    process = context.Process(target=_build_worker, args=(recipe, queue))
    process.start()
    try:
        pid, digest, outputs = queue.get(timeout=15)
        process.join(timeout=15)
        assert process.exitcode == 0
        assert pid != os.getpid()
        assert digest == recipe.fingerprint()
        assert outputs == ['output']
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5)
        queue.close()
        queue.join_thread()
