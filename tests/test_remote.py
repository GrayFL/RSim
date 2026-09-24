import asyncio
from dataclasses import dataclass
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import types

import numpy as np
import pytest

from rsim import Chassis, ProcessSensor, Ros1Bridge, Runtime, SSHConfig, SensorError
from rsim.model import Frame
from rsim.remote import decode_message
from rsim.shared import SharedStore, decode


spec = importlib.util.spec_from_file_location("ros1_agent", Path(__file__).parents[1] / "compat/ros1_agent.py")
agent_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agent_module)


@pytest.mark.parametrize("kind, values", [
    ("float32[]", [1.0, float("inf"), float("-inf"), float("nan")]),
    ("float64[2]", [1.234567890123, -1e100]), ("bool[]", [True, False]),
    ("uint8[]", bytes([0, 128, 255])), ("int64[]", [-2**63, 2**63-1]),
    ("uint64[]", [0, 2**64-1]), ("float32[]", []),
])
def test_python2_wire_arrays_preserve_values_and_are_readonly(kind, values):
    wire = json.loads(json.dumps(agent_module.encode(values, kind), allow_nan=False))
    result = decode_message(wire)
    np.testing.assert_equal(result, list(values))
    assert not result.flags.writeable
    if kind == "bool[]":
        assert result.dtype == np.bool_


def test_nonfinite_scalars_survive_wire_and_local_dds_storage(tmp_path):
    source = {name: agent_module.encode(value) for name, value in
              {"positive": float("inf"), "negative": float("-inf"), "invalid": float("nan")}.items()}
    data = decode_message(json.loads(json.dumps(source, allow_nan=False)))
    store = SharedStore(tmp_path)
    wire = json.loads(json.dumps(store.put(Frame(data, 1, "test")), allow_nan=False))
    decoded = decode(wire["data"], tmp_path)
    assert np.isposinf(decoded["positive"]) and np.isneginf(decoded["negative"])
    assert np.isnan(decoded["invalid"])


def test_velocity_watchdog_and_validation_without_hardware(monkeypatch):
    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0., y=0., z=0.)
            self.angular = types.SimpleNamespace(x=0., y=0., z=0.)

        def serialize(self, buffer):
            buffer.write(b"validated")

    messages = []

    class Publisher:
        def get_num_connections(self):
            return 1

        def publish(self, msg):
            messages.append(msg)

    def fill(msg, arguments):
        for name, fields in arguments[0].items():
            for axis, value in fields.items():
                setattr(getattr(msg, name), axis, value)

    monkeypatch.setitem(sys.modules, "roslib.message", types.SimpleNamespace(get_message_class=lambda _: Twist))
    monkeypatch.setitem(sys.modules, "genpy.message", types.SimpleNamespace(fill_message_args=fill))
    rospy = types.SimpleNamespace(Publisher=lambda *a, **kw: Publisher())
    agent = agent_module.Agent(io.BytesIO(), rospy)
    request = {"topic": "/cmd_vel", "type": "geometry_msgs/Twist", "data": {"linear": {"x": 1.0}}}
    assert agent.publish(request)["connections"] == 1
    assert messages[-1].linear.x == 1.0
    agent.commands["/cmd_vel"] = (0, Twist)
    agent.stop_expired()
    assert messages[-1].linear.x == messages[-1].angular.z == 0
    assert not agent.commands
    before = len(messages)
    request["data"]["linear"]["x"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        agent.publish(request)
    assert len(messages) == before


FAKE_ENDPOINT = '''
import base64, json, os, select, struct, sys, time
def send(value):
    print(json.dumps(value), flush=True)
send(dict(op='ready', version=1, pid=os.getpid(), python='test', node='/fake'))
topics = {}
counter = 0
while True:
    if select.select([sys.stdin], [], [], .01)[0]:
        line=sys.stdin.readline()
        if not line: break
        request=json.loads(line)
        op=request['op']
        if op=='subscribe':
            topics[request['topic']]=request.get('type') or 'sensor_msgs/LaserScan'
            result={'type':topics[request['topic']]}
        elif op=='unsubscribe':
            topics.pop(request['topic'],None)
            result={}
        elif op=='topics': result=[{'topic':k,'type':v} for k,v in topics.items()]
        elif op=='publish': result={'published':True,'echo':request['data']}
        else: result={'alive':True}
        send(dict(op='reply', id=request['id'], result=result))
    counter+=1
    for topic,kind in topics.items():
        data={'header':{'stamp':{'secs':0,'nsecs':counter},'frame_id':'test','seq':counter}}
        if kind=='sensor_msgs/LaserScan':
            data['ranges']={'__array__':'f','data':base64.b64encode(struct.pack('<2f',1.,float('inf'))).decode()}
        send(dict(op='sample',topic=topic,type=kind,stamp_ns=counter,data=data))
'''


@dataclass(frozen=True)
class LocalEndpoint:
    script: str
    host: str = "fake"

    def command(self):
        return [sys.executable, "-u", self.script]


@pytest.fixture
def endpoint(tmp_path):
    script = tmp_path / "endpoint.py"
    script.write_text(FAKE_ENDPOINT)
    return LocalEndpoint(str(script))


def test_async_topics_sharing_history_commands_and_reopen(endpoint):
    async def run():
        bridge = Ros1Bridge(endpoint)
        equivalent = Ros1Bridge(endpoint)
        first = bridge.topic("/scan")
        second = equivalent.topic("/scan")
        for _ in range(2):
            async with Runtime(first, second):
                a, b = await asyncio.gather(first.get(timeout=5), second.get(timeout=5))
                assert a is b
                assert second.bridge is bridge
                assert await first.get(timestamp_ns=a.stamp_ns, clock=a.clock) is a
                assert (await bridge.topics())[0]["topic"] == "/scan"
                response = await second.bridge.publish_message("/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": 0.}})
                assert response["echo"]["linear"]["x"] == 0
                fresh = await first.get(after=a.sequence, timeout=2)
                assert fresh.stamp_ns > a.stamp_ns
            assert bridge.process.returncode == 0
            assert not bridge._pending and not bridge._tasks
    asyncio.run(run())


def test_connection_loss_propagates_to_sensor_get(endpoint):
    async def run():
        bridge = Ros1Bridge(endpoint)
        sensor = bridge.topic("/scan")
        async with Runtime(sensor):
            await sensor.get(timeout=5)
            bridge.process.kill()
            await bridge.process.wait()
            async with asyncio.timeout(2):
                while sensor._failure is None:
                    await asyncio.sleep(.01)
            with pytest.raises(SensorError):
                await sensor.get(timeout=1)
    asyncio.run(run())


def test_endpoint_does_not_leak_while_waiting_for_an_unavailable_master(tmp_path):
    (tmp_path / "rospy.py").write_text("import time\ndef init_node(*a, **kw): time.sleep(60)\n")
    async def run():
        script = Path(__file__).parents[1] / "compat/ros1_agent.py"
        process = await asyncio.create_subprocess_exec(sys.executable, str(script),
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE, env=dict(os.environ, PYTHONPATH=str(tmp_path)))
        try:
            process.stdin.close()
            stdout, stderr = await asyncio.wait_for(process.communicate(), 3)
            assert process.returncode == 1
            assert b"startup aborted" in stderr and not stdout
        finally:
            if process.returncode is None:
                process.kill()
                await process.wait()
    asyncio.run(run())


def test_chassis_can_run_in_a_process_with_local_shared_arrays(endpoint):
    async def run():
        source = ProcessSensor(lambda: Chassis(endpoint, history=3), history=3)
        async with Runtime(source):
            frame = await source.get(timeout=15)
            ranges = frame.data["scan"]["data"]["ranges"]
            assert isinstance(ranges, np.memmap) and not ranges.flags.writeable
            assert np.isposinf(ranges[1])
            assert frame.data["imu"]["clock"] == "ros1:fake"
    asyncio.run(run())


def test_ssh_configuration_uses_argv_and_quotes_remote_paths():
    config = SSHConfig("robot", remote_script="path with spaces/agent.py", setup=("ROS setup.bash",))
    command = config.command()
    assert command[-2] == "robot"
    assert "path with spaces/agent.py" in command[-1]
    assert "BatchMode=yes" in command and "-T" in command
    with pytest.raises(ValueError):
        SSHConfig("-oProxyCommand=something").command()


def test_ros2_standard_message_adaptation_preserves_time_covariance_and_inf():
    pytest.importorskip("rclpy", exc_type=ImportError)
    from sensor_msgs.msg import LaserScan, Imu
    from rsim.remote_ros2 import fill_ros2
    header = {"seq": 10, "stamp": {"secs": 123, "nsecs": 456}, "frame_id": "laser"}
    scan = fill_ros2(LaserScan(), {"header": header, "ranges": np.array([1., np.inf], dtype="f4")})
    assert scan.header.stamp.sec == 123 and scan.header.stamp.nanosec == 456
    assert np.isposinf(scan.ranges[1])
    imu = fill_ros2(Imu(), {"orientation_covariance": np.arange(9, dtype="f8")})
    np.testing.assert_equal(imu.orientation_covariance, np.arange(9))


def test_ros2_mirror_runtime_opens_and_processes_topics(endpoint):
    pytest.importorskip("rclpy", exc_type=ImportError)
    from rsim.remote_ros2 import ChassisROS2

    async def run():
        chassis = Chassis(endpoint)
        relay = ChassisROS2(chassis, prefix="/rsim_test_chassis")
        async with Runtime(relay):
            frame = await relay.get(timeout=5)
            assert set(frame.data) == {"imu", "odom", "scan"}
            await relay.get(after=frame.sequence, timeout=5)
            assert relay._failure is None
            assert set(relay.previous) == {"imu", "odom", "scan"}
        assert chassis.bridge.process.returncode == 0
    asyncio.run(run())
