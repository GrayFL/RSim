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

from rsim import Bundle, Chassis, ProcessSensor, Ros1Bridge, Runtime, SSHConfig, SensorError
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
    request = {"topic": "/cmd_vel", "type": "geometry_msgs/Twist", "data": {"linear": {"x": 1.0}},
               "command": {"controller_id": "test", "controller_epoch": "1", "sequence": 1,
                           "deadline_ns": agent_module.monotonic_ns() + 250000000}}
    assert agent.publish(request)["connections"] == 1
    assert messages[-1].linear.x == 1.0
    with pytest.raises(ValueError, match="replayed"):
        agent.publish(request)
    other = agent_module.Agent(io.BytesIO(), rospy)
    competing = dict(request, command=dict(request["command"], controller_id="second"))
    with pytest.raises(ValueError, match="owned by another"):
        other.publish(competing)
    expired = dict(request, command=dict(request["command"], deadline_ns=0, sequence=2))
    with pytest.raises(ValueError, match="expired"):
        agent.publish(expired)
    agent.commands["/cmd_vel"] = (0, Twist)
    agent.stop_expired()
    assert messages[-1].linear.x == messages[-1].angular.z == 0
    assert not agent.commands
    before = len(messages)
    request["data"]["linear"]["x"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        agent.publish(request)
    assert len(messages) == before


def test_ros1_provider_retires_epochs_after_expiry(monkeypatch):
    now = [10000000000]
    monkeypatch.setattr(agent_module, "monotonic_ns", lambda: now[0])
    agent = agent_module.Agent(io.BytesIO(), None)
    command = {"controller_id": "controller", "controller_epoch": "old", "sequence": 3,
               "deadline_ns": now[0] + 100000000}
    try:
        agent.validate_command("/rsim_test_epoch", command)
        with pytest.raises(ValueError, match="exclusive"):
            agent.validate_command("/rsim_test_epoch", dict(command, controller_epoch="new", sequence=1))
        now[0] += 200000000
        agent.validate_command("/rsim_test_epoch", dict(command, controller_epoch="new", sequence=1,
                                                       deadline_ns=now[0] + 100000000))
        with pytest.raises(ValueError, match="retired"):
            agent.validate_command("/rsim_test_epoch", dict(command, sequence=100,
                                                           deadline_ns=now[0] + 100000000))
    finally:
        for lease in agent.command_leases.values():
            lease.close()


def test_ros1_deadman_releases_all_leases_even_when_one_publisher_fails():
    class BrokenPublisher:
        def publish(self, _):
            raise RuntimeError("publisher failed")
    agent = agent_module.Agent(io.BytesIO(), None)
    leases = [io.StringIO(), io.StringIO()]
    for topic, lease in zip(("/a", "/b"), leases):
        agent.commands[topic] = (0, object)
        agent.publishers[topic] = ("geometry_msgs/Twist", BrokenPublisher())
        agent.command_leases[topic] = lease
    with pytest.raises(RuntimeError, match="zero command failed"):
        agent.stop_expired()
    assert not agent.commands and not agent.command_leases
    assert all(lease.closed for lease in leases)


FAKE_ENDPOINT = '''
import base64, json, os, select, struct, sys, time
def send(value):
    print(json.dumps(value), flush=True)
send(dict(op='ready', version=2, pid=os.getpid(), python='test', node='/fake', monotonic_ns=time.monotonic_ns()))
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
        else: result={'alive':True, 'monotonic_ns':time.monotonic_ns()}
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


def test_remote_command_rejection_preserves_ssh_session(endpoint):
    from rsim import CommandRejected
    path = Path(endpoint.script)
    path.write_text(path.read_text().replace(
        "elif op=='publish': result={'published':True,'echo':request['data']}",
        """elif op=='publish':
            if request['data'].get('linear',{}).get('x')==3:
                send(dict(op='reply',id=request['id'],error='command expired',
                          error_type='command_rejected',reason='expired'))
                continue
            result={'published':True,'echo':request['data']}"""))
    async def run():
        chassis = Chassis(endpoint)
        async with Runtime(chassis):
            with pytest.raises(CommandRejected, match="expired"):
                await chassis.set_velocity(3)
            result = await chassis.stop()
            assert result["published"]
            await asyncio.sleep(.1)
            assert chassis._failure is None and chassis.bridge._failure is None
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
        def factory():
            chassis = Chassis(endpoint, history=3)
            return Bundle(imu=chassis.imu, odom=chassis.odom, scan=chassis.scan)
        source = ProcessSensor(factory, history=3)
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
            frames = await asyncio.gather(*(getattr(relay, name).get(timeout=5)
                                            for name in ("imu", "odom", "scan")))
            await asyncio.sleep(.05)
            assert not hasattr(relay, "get") and not hasattr(chassis, "get")
            await relay.scan.get(after=frames[-1].sequence, timeout=5)
            assert relay._failure is None
            assert set(relay.previous) == {"imu", "odom", "scan"}
        assert chassis.bridge.process.returncode == 0
    asyncio.run(run())
