"""Final hardware gate and atomic cross-interpreter mailbox checks (no ROS)."""
import importlib.util
from pathlib import Path
import sys
import pytest
from rsim.adapters.ros1_wire import read_packet, write_packet


def gate_module():
    # The actual standalone Python 2 agent is also testable without importing ROS.
    if sys.platform == 'win32':
        pytest.skip('native POSIX gate is exercised on the Linux robot')
    path = Path(__file__).resolve().parents[1] / 'compat' / 'ros1_local_agent.py'
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location('rsim_local_agent_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_atomic_latest_mailbox(tmp_path):
    write_packet(str(tmp_path), 'command', {'sequence': 1, 'v': 0.})
    assert read_packet(str(tmp_path), 'command')['sequence'] == 1
    write_packet(str(tmp_path), 'command', {'sequence': 2, 'v': 0.})
    assert read_packet(str(tmp_path), 'command')['sequence'] == 2
    assert not list(tmp_path.glob('*.tmp'))
    with pytest.raises(ValueError): write_packet(str(tmp_path), '../escape', {})
    with pytest.raises(ValueError): write_packet(str(tmp_path), 'command', {'v': float('nan')})


def test_native_gate_zero_only_expiry_replay_and_interlocks():
    Gate = gate_module().VelocityGate
    now = 10**12
    packet = dict(session='abc', sequence=1, controller_epoch='epoch', deadline_ns=now + 250000000,
                  linear_x=.1, angular_z=.05, armed=True)
    gate = Gate('abc')
    gate.update(packet, now)
    assert gate.output(now, 'ok', (.279, .3))[0] == (0., 0.)
    gate = Gate('abc', enabled=True)
    gate.update(packet, now)
    assert gate.output(now, 'ok', (.279, .3))[0] == (.1, .05)
    assert gate.output(now + 300000000, 'ok', (.279, .3))[0] == (0., 0.)
    assert gate.output(now, 'estop_or_unknown', (.279, .3))[0] == (0., 0.)
    assert gate.output(now, 'ok', (.05, .3))[0] == (0., 0.)
    gate.update(dict(packet, linear_x=.2), now)
    assert gate.accepted == (.1, .05)
    gate.update(dict(packet, sequence=2, linear_x=float('nan')), now)
    assert gate.output(now, 'ok', (.279, .3))[0] == (0., 0.)
    gate.update(dict(packet, sequence=float('inf')), now)
    gate.update(dict(packet, sequence=float('nan')), now)


def test_event_interlock_never_drops_a_fast_transition():
    from types import SimpleNamespace
    module = gate_module()
    relay = module.Relay.__new__(module.Relay)
    relay.received = {}
    relay.estop = None
    relay.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_nsec=lambda: 0)))
    messages = []
    relay.put = lambda name, data, stamp, **extra: messages.append((name, data))
    # encode is orthogonal to this transition check; real ROS uses __slots__.
    original = module.encode
    module.encode = lambda message: {'data': message.data}
    try:
        spec = {'type': 'std_msgs/Bool', 'hz': 1}
        relay.callback(SimpleNamespace(data=False), ('estop', spec))
        relay.callback(SimpleNamespace(data=True), ('estop', spec))
        assert relay.estop is True
        assert len(messages) == 2
    finally:
        module.encode = original


def test_native_gate_differential_mode_and_hard_cap():
    Gate = gate_module().VelocityGate
    now = 10**12
    gate = Gate('session', enabled=True)
    base = dict(session='session', controller_epoch='epoch', deadline_ns=now+250000000, armed=True)
    for seq, (v, w, mode, accepted) in enumerate([
        (0, .5, 'differential', True), (-1., .1, 'differential', True),
        (1.6, 0, 'differential', True), (1.601, 0, 'differential', False),
        (0, .5, 'towing', False), (-.1, 0, 'towing', False),
        (.3, .1, 'towing', True), (0, 0, 'unknown', False)
    ], 1):
        gate.update(dict(base, sequence=seq, linear_x=v, angular_z=w, drive_mode=mode), now)
        assert (gate.reason == 'accepted') is accepted
        assert gate.output(now, 'ok', (1.6, .8))[0] == ((v, w) if accepted else (0., 0.))


def test_hardware_health_does_not_veto_for_oem_navigation_state():
    from types import SimpleNamespace
    module = gate_module()
    relay = module.Relay.__new__(module.Relay)
    now = module.monotonic()
    relay.estop, relay.bumper, relay.scheduler = False, False, 'TaskState::NAVIGATION'
    relay.by_name = {name: SimpleNamespace(get_num_connections=lambda: 1) for name in ('estop', 'bumper')}
    relay.config = {'required_fresh': ['odom']}
    relay.gate = SimpleNamespace(enabled=True)
    relay.output_publishers = []
    relay.ownership_time = relay.limits_time = relay.pose_time = now
    relay.received = {'odom': now}
    assert relay.health(now) == 'ok'
    relay.output_publishers = ['/twist_mux']
    assert relay.health(now) == 'output_publisher_conflict'
