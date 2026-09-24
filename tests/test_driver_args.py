import asyncio
import json
from pathlib import Path
import sys

import pytest

from rsim import D435 as ClientD435, Runtime, SensorError
from rsim._ros_args import RosArguments
from rsim.drivers import D435, RobinW, _parse_args, _sensor_from_args
from rsim.ros import Driver


def require_ros():
    pytest.importorskip("rclpy", exc_type=ImportError)


def test_native_parameter_types_and_precedence(tmp_path):
    require_ros()
    yaml = tmp_path / "settings with spaces.yaml"
    yaml.write_text('/**:\n  ros__parameters:\n    number: 20\n    from_file: true\n'
                    '/rig/renamed:\n  ros__parameters:\n    targeted: [1, 2, 3]\n')
    values = {"number": 10, "numeric_string": "0012", "boolean_string": "true",
              "text": "a: b 'quoted' \"double\" \\slash\n中文", "bool": False,
              "real": 2e-6, "strings": ["true", "12"], "flags": [False, True]}
    options = RosArguments(values, ["--params-file", str(yaml), "-p", "number:=30",
                                    "-r", "__ns:=/rig", "-r", "__node:=renamed"],
                           {"__ns": "/fallback", "__node": "fallback"})
    values["strings"].append("mutation")
    with options.resolve("probe") as node:
        assert node.get_fully_qualified_name() == "/rig/renamed"
        expected = dict(values, number=30, strings=["true", "12"], from_file=True, targeted=[1, 2, 3])
        for name, value in expected.items():
            actual = node.get_parameter(name).value
            assert actual == value
            assert type(actual) is type(value)


def test_camera_routing_enabled_streams_and_client_contract():
    require_ros()
    provider = D435(parameters={"publish_tf": False}, ros_args=[
        "-p", "serial_no:='_012345'", "-p", "enable_color:=false",
        "-p", "depth_module.depth_profile:=480,270,30",
        "-r", "__ns:=/rig", "-r", "__node:=front",
        "-r", "~/depth/image_rect_raw:=/measurements/depth"])
    shared = provider.source.producer
    client = ClientD435(serial="012345", depth_profile="480x270x30").source.producer
    assert (shared.source_key, shared.version) == (client.source_key, client.version)
    graph = shared.factory()
    assert graph.names == ("depth",)
    assert graph.sources["depth"].producer.topic == "/measurements/depth"
    driver = graph.sources["depth"].producer.children[1]
    assert driver.key == "driver:d435:012345"
    assert driver.parameters["publish_tf"] is False
    with pytest.raises(ValueError, match="disabled"):
        D435(stream="color", parameters={"enable_color": False})


def test_robin_parameters_and_remaps_follow_effective_device():
    require_ros()
    provider = RobinW("old", parameters={"frame_id": "lidar_custom"}, ros_args=[
        "-p", "lidar_ip:=lidar.example", "-p", "frame_topic:=cloud",
        "-r", "__ns:=/rig", "-r", "cloud:=filtered"])
    assert provider.source_key == "robin:lidar.example"
    source = provider.factory()
    assert source.topic == "/rig/filtered"
    assert source.children[1].parameters["frame_id"] == "lidar_custom"


def test_cli_preserves_native_argv_and_matches_function():
    require_ros()
    native = ["--ros-args", "--log-level", "warn", "-p", "enable_depth:=false",
              "-p", "rgb_camera.enable_auto_exposure:=false"]
    cli = _sensor_from_args(_parse_args(["d435", "--stream", "color", *native]))
    function = D435(stream="color", ros_args=native)
    assert cli.source.producer.provider_version == function.source.producer.provider_version
    assert cli.source.producer.factory().names == ("color",)
    with pytest.raises(SystemExit):
        _parse_args(["camera", *native])
    with pytest.raises(SystemExit):
        _parse_args(["d435", "--misspelled-option"])
    robin = _sensor_from_args(_parse_args(["robin", "--ros-args", "-p", "lidar_ip:=lidar.example"]))
    assert robin.source_key == "robin:lidar.example"


def test_parameter_file_changes_are_not_silently_reused(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    path = Path("parameters.yaml")
    path.write_text('/**:\n  ros__parameters:\n    value: 1\n')
    first = RosArguments(ros_args=["--params-file", str(path)])
    assert str(path.resolve()) in first.arguments()
    path.write_text('/**:\n  ros__parameters:\n    value: 2\n')
    assert first.signature() != RosArguments(ros_args=["--params-file", str(path)]).signature()
    with pytest.raises(ValueError, match="file changed"):
        first.arguments()


@pytest.mark.parametrize("value", [None, {"nested": 1}, [1, "2"], float("nan"), 2**63])
def test_invalid_ros_types_fail_before_launch(value):
    with pytest.raises((ValueError, TypeError)):
        D435(parameters={"custom": value})


def test_ros_args_reject_shell_strings_and_unknown_flags():
    with pytest.raises(TypeError, match="sequence"):
        D435(ros_args="-p publish_tf:=false")
    require_ros()
    from rclpy.impl.implementation_singleton import rclpy_implementation
    with pytest.raises(rclpy_implementation.UnknownROSArgsError, match="--not-a-ros-option"):
        D435(ros_args=["--not-a-ros-option"])


def test_driver_executes_native_arguments_with_exact_types(tmp_path, monkeypatch):
    require_ros()
    from ament_index_python import packages
    executable = tmp_path / "lib" / "probe" / "node"
    executable.parent.mkdir(parents=True)
    output = tmp_path / "received.json"
    executable.write_text(f"#!{sys.executable}\n" + '''
import json, os, time
from pathlib import Path
import rclpy
rclpy.init()
node = rclpy.create_node('probe', automatically_declare_parameters_from_overrides=True)
Path(os.environ['RSIM_PROBE_OUTPUT']).write_text(json.dumps({
    'name': node.get_fully_qualified_name(),
    'values': {p: node.get_parameter(p).value for p in node.list_parameters([], 0).names}}))
try:
    while True: time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
''')
    executable.chmod(0o755)
    monkeypatch.setattr(packages, "get_package_prefix", lambda _: str(tmp_path))
    monkeypatch.setenv("RSIM_PROBE_OUTPUT", str(output))
    driver = Driver("probe", "node", {"number": 1, "text": "false", "array": [1.0, 2.0]},
                    key="driver:probe:" + str(tmp_path),
                    ros_args=["-p", "number:=2", "-r", "__node:=native"],
                    remappings={"__node": "fallback"})

    async def run():
        async with Runtime(driver):
            async with asyncio.timeout(10):
                while not output.exists():
                    await asyncio.sleep(0.02)
            received = json.loads(output.read_text())
            assert received["name"] == "/native"
            assert received["values"]["number"] == 2
            assert received["values"]["text"] == "false"
            assert received["values"]["array"] == [1.0, 2.0]
        assert driver.process.returncode is not None
    asyncio.run(run())


def test_disabled_client_stream_reports_error_without_waiting_forever():
    from rsim.core import Sensor
    from rsim.devices import _ImageStream

    async def run():
        source = Sensor()
        color = _ImageStream(source, "color", hz=100, history=2)
        async with Runtime(color):
            await source.publish({"depth": {}}, stamp_ns=1, clock="test")
            with pytest.raises(SensorError) as error:
                await color.get(timeout=1)
            assert "enabled color" in str(error.value.__cause__)
    asyncio.run(run())
