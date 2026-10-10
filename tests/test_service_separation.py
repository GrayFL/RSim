"""Applications must not acquire hardware; cross-host control must preserve TTL."""
import asyncio
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml

from rsim.adapters.ros2.stm32 import DriverClock
from rsim.core import ComponentError
from rsim.core.commands import CommandRejected
from rsim.config import load_chassis

ROOT = Path(__file__).resolve().parents[1]


def test_clock_translation_consumes_response_delay_and_expires():
    clock = DriverClock()
    # Two hosts with unrelated monotonic epochs and 10 ms each-way latency.
    sent, received, server = 1_000_000_000, 1_020_000_000, 900_010_000_000
    clock.observe('first', server, sent, received)
    deadline = clock.deadline(received + 200_000_000, received)
    assert deadline == 900_210_000_000
    assert deadline < 900_020_000_000 + 200_000_000
    with pytest.raises(CommandRejected, match='expired'):
        clock.deadline(received + 4_000_000_000, received + 4_000_000_000)
    with pytest.raises(ComponentError, match='restarted'):
        clock.observe('restarted', server, sent, received)


def test_slow_clock_samples_do_not_refresh_the_bound():
    clock = DriverClock()
    with pytest.raises(TimeoutError):
        clock.observe('first', 10**12, 1, 100_000_001)
    clock.observe('first', 10**12, 1, 10_000_001)
    before = clock.received
    clock.observe('first', 10**12 + 100_000_000, 100_000_000, 200_000_000)
    assert clock.received == before


@pytest.mark.parametrize('kind', ['stm32', 'imu', 'scan'])
def test_service_rejects_hardware_start_before_constructing_graph(tmp_path, kind):
    settings = yaml.safe_load((ROOT/'examples/control/chassis_topics.example.yaml').read_text())
    settings['chassis'].setdefault(kind, {})['start_driver'] = True
    path = tmp_path/'service.yaml'
    path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError, match='forbidden'):
        load_chassis(path, hardware=False)


def test_topic_service_graph_only_contains_algorithm_driver(tmp_path):
    settings = yaml.safe_load((ROOT/'examples/control/chassis_topics.example.yaml').read_text())
    settings['chassis']['directory'] = str(tmp_path/'out')
    path = tmp_path/'service.yaml'
    path.write_text(yaml.safe_dump(settings))
    robot = load_chassis(path, hardware=False)
    from rsim.adapters.ros2.driver import Driver
    from rsim.runtime.host import SharedSensor
    pending=[robot.control, robot.chassis]; seen=set(); drivers=[]; relays=[]
    while pending:
        node=pending.pop()
        if id(node) in seen: continue
        seen.add(id(node))
        if isinstance(node, Driver): drivers.append(node.package)
        if isinstance(node, SharedSensor): relays.append(node.source_key)
        pending.extend(node.dependencies)
        pending.extend(signal.producer for signal in node.inputs)
    assert drivers == ['robot_localization']
    assert sorted(relays) == sorted(['ros2-topic:imu:/rsim/chassis/imu/data',
        'ros2-topic:odom:/rsim/chassis/odom', 'ros2-topic:state:/rsim/chassis/diagnostics'])
    assert robot.chassis.remote_clock


def test_mapping_application_rejects_hardware_recipes(tmp_path):
    from rsim.apps.mapping_service import run
    for field in ('start_drivers', 'connection'):
        path=tmp_path/'mapping.yaml'
        path.write_text(yaml.safe_dump({'mapping': {field: True}}))
        with pytest.raises(ValueError, match='start hardware separately'):
            asyncio.run(run(SimpleNamespace(config=path, output=tmp_path/'out')))
        assert not (tmp_path/'out').exists()


def test_default_mapping_graph_has_no_hardware_owner(tmp_path):
    from rsim.config import load_mapper
    settings=yaml.safe_load((ROOT/'examples/mapping/mapping_topics.example.yaml').read_text())
    settings['mapping']['database']=str(tmp_path/'map.db')
    path=tmp_path/'mapping.yaml';path.write_text(yaml.safe_dump(settings))
    graph=load_mapper(path).factory()
    from rsim.adapters.ros2.driver import Driver
    pending=[graph];seen=set();packages=[]
    while pending:
        node=pending.pop()
        if id(node) in seen:continue
        seen.add(id(node))
        if isinstance(node,Driver):packages.append(node.package)
        pending.extend(node.dependencies)
        pending.extend(signal.producer for signal in node.inputs)
    assert sorted(packages)==['rtabmap_slam','super_lio']
    assert graph.ingress.topics['points']=='/iv_points'
    assert graph.mapping.camera_prefix=='/rsim/d435'
