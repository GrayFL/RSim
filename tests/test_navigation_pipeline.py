"""Regressions for source-rate aliasing, direct manual control and taped body."""
import asyncio
import time
from types import SimpleNamespace

import numpy as np
import pytest
from rsim.components.manual_control import ManualController
from rsim.components.vehicle import drive_model
from rsim.components.navigation_map import laser_points
from rsim.core.metronome import Metronome
from rsim.apps.navigation.settings import defaults
from test_ros1_local import gate_module


def test_recovery_and_notification_publish_in_order():
    from rsim.adapters.ros1_local import LocalROS1Bridge
    import threading
    async def run():
        bridge = LocalROS1Bridge({'topics': {}})
        bridge._closed = False
        began, release = threading.Event(), threading.Event()
        calls = []
        def read(names=None):
            calls.append(names)
            if names is None:
                began.set(); assert release.wait(1.)
            seq = 1 if names is None else 2
            return [('pose', dict(data={'value': seq}, stamp_ns=seq, sequence=seq))]
        bridge.read_changed = read
        recovery = asyncio.create_task(bridge.poll())
        assert await asyncio.to_thread(began.wait, 1.)
        notification = asyncio.create_task(bridge.poll({'pose'}))
        await asyncio.sleep(.02)
        assert calls == [None]
        release.set()
        await asyncio.gather(recovery, notification)
        assert [f.data['value'] for f in bridge.pose.frames] == [1, 2]
    asyncio.run(run())


def test_path_independent_notification_roundtrip():
    import socket
    import uuid
    from rsim.adapters.ros1_wire import notification_address
    address = notification_address(uuid.uuid4().hex, 'updates')
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
        receiver.bind(address); receiver.settimeout(.1)
        sender.sendto(b'pose', address)
        assert receiver.recv(64) == b'pose'


def test_latest_sampler_57ms_source_keeps_15hz_and_flushes_last_sample():
    sampler = gate_module().LatestSampler()
    emitted = []
    for tick in range(2001):
        now = tick / 1000.
        if tick % 57 == 0: sampler.push('scan', tick, {'hz': 15}, now)
        emitted.extend((now, item[1]) for item in sampler.due(now))
    assert 29 <= len(emitted) <= 31  # old minimum-interval filter: 18
    assert max(at - value/1000. for at, value in emitted) < .058
    assert sampler.due(2.2)[0][1] == 1995  # trailing sample needs no next callback


def test_cloud_projection_removes_floor_height_not_near_obstacles():
    spec = dict(min_height=.05, max_height=1.2, angle_min=-np.pi/2,
                angle_max=np.pi/2, angle_increment=np.pi/180, range_min=.1, range_max=2.)
    points = [[.24, 0, .032], [.3, 0, .1], [.4, 0, .2], [.2, 0, .05],
              [-.5, 0, .2], [.5, 0, 1.21], [np.nan, 0, .2]]
    ranges = gate_module().project_ranges(points, spec)
    assert ranges[90] == pytest.approx(.2)
    assert np.count_nonzero(np.isfinite(ranges)) == 1
    assert np.isposinf(gate_module().project_ranges([[.24, 0, .03]], spec)).all()
    assert np.isposinf(gate_module().project_ranges([], spec)).all()
    with pytest.raises(ValueError, match='projection'):
        gate_module().project_ranges(points, dict(spec, min_height=-.1))


def test_depth_cutoff_is_in_camera_frame_and_excludes_far_returns():
    gate = gate_module()
    points, depths = gate.depth_limited_points(
        [[.1, 0, .69], [.1, 0, .7], [.1, 0, .701], [0, 0, 0], [0, 0, -1], [float('nan'), 0, .2]], .7)
    assert len(points) == 2 and depths == pytest.approx([.69, .7])
    # A camera displaced from the body origin must use its own optical depth,
    # not the already projected scan's distance from base_footprint.
    points, depths = gate.depth_limited_points([[.9, 0, .3], [1.16, 0, .3], [.2, 0, .3]],
                                               .7, axis=(1., 0., 0.), offset=-.45)
    assert points == pytest.approx(np.array([[.9, 0, .3]]))
    assert depths == pytest.approx([.45])
    assert gate.depth_limited_points([], .7)[0].shape == (0, 3)
    with pytest.raises(ValueError, match='depth filter'):
        gate.depth_limited_points([], float('nan'))
    with pytest.raises(ValueError, match='depth filter'):
        gate.depth_limited_points([], .7, axis=(0., 0., 0.))


def test_overrun_does_not_wait_for_another_whole_control_slot(monkeypatch):
    async def run():
        now = [100.]
        async def sleep(delay): now[0] += delay
        monkeypatch.setattr(asyncio, 'get_running_loop', lambda: SimpleNamespace(time=lambda: now[0]))
        monkeypatch.setattr(asyncio, 'sleep', sleep)
        clock = Metronome(10)
        await clock.tick()
        for _ in range(20):
            now[0] += .101
            await clock.tick()
        assert now[0] == pytest.approx(102.02)
    asyncio.run(run())


@pytest.mark.parametrize('mode', ['towing', 'differential'])
def test_velocity_joystick_crosses_zero_without_release(mode):
    controller = ManualController(); model = drive_model(mode=mode)
    axes = dict(defaults({})['manual'][mode], y_mode='velocity_target')
    measured = np.array([.3, 0.])
    for _ in range(40):
        controller.input(-.5, 0, measured, 0., source='joystick')
        command = controller.target(measured, 0., model, axes, (1., .8))
        measured = model.advance_velocity(measured, [command.linear_x, command.angular_z], .1)
    assert measured[0] < -.45 and not controller.brake_locked


def test_fresh_reverse_key_ignores_stopped_wheel_noise():
    controller = ManualController()
    controller.input(-1, 0, [.4, 0])
    assert controller.brake_locked
    controller.input(0, 0, [.012, 0])
    controller.input(-1, 0, [.012, 0])
    assert not controller.brake_locked


def test_body_self_filter_uses_extrinsic_and_keeps_outside_obstacle():
    identity = dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1])
    sensor = dict(translation=[-.147, 0, .16], quaternion=[0, 0, 0, 1])
    scan = dict(angle_min=0., angle_increment=0., range_min=.01, range_max=10.,
                ranges=[.24]*8 + [.5])
    cloud = laser_points(scan, identity, body_transform=sensor,
                        body_filter=dict(front=.1, rear=.4, width=.54, padding=.01))
    np.testing.assert_allclose(cloud, [[.5, 0.]])
    with pytest.raises(ValueError, match='sensor-to-body'):
        laser_points(scan, identity, body_filter=dict(front=.1, rear=.4, width=.54))


def test_edge_echo_mask_keeps_other_angles_and_farther_returns():
    transform = dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1])
    filter_ = dict(max_range=.25, angle_masks=[],
                   edge_echo_masks=[dict(lower=2.55, upper=2.58, max_range=.42)])
    scan = dict(angle_min=2.53, angle_increment=.005, range_min=.01, range_max=10., ranges=[.37]*15)
    before = laser_points(scan, transform)
    after = laser_points(scan, transform, filter_)
    angles = np.arctan2(before[:, 1], before[:, 0])
    keep = ~((angles >= 2.55) & (angles <= 2.58))
    np.testing.assert_allclose(after, before[keep])
    scan['ranges'] = [.43]*15
    assert len(laser_points(scan, transform, filter_)) == 15
    filter_['edge_echo_masks'][0]['upper'] = 2.7
    with pytest.raises(ValueError, match='edge echo'):
        laser_points(scan, transform, filter_)


def test_near_field_speckle_keeps_surfaces_and_farther_single_points():
    transform = dict(translation=[-.147, 0, 0], quaternion=[0, 0, 0, 1])
    body = dict(front=.1, rear=.4, width=.54, padding=.01)
    filter_ = dict(max_range=.25, angle_masks=[], near_speckle_range=.6)
    scan = dict(angle_min=1.75, angle_increment=.005, range_min=.01, range_max=10., ranges=[float('nan')]*11)
    scan['ranges'][5] = .31  # isolated return just outside the left shell
    assert len(laser_points(scan, transform, filter_, body_transform=transform, body_filter=body)) == 0
    scan['ranges'][4] = .31
    assert len(laser_points(scan, transform, filter_, body_transform=transform, body_filter=body)) == 0
    scan['ranges'][6] = .31
    assert len(laser_points(scan, transform, filter_, body_transform=transform, body_filter=body)) == 3
    scan['ranges'][4] = scan['ranges'][6] = float('nan'); scan['ranges'][5] = .7
    assert len(laser_points(scan, transform, filter_, body_transform=transform, body_filter=body)) == 1


def test_manual_direct_ignores_missing_map_scan_but_not_hardware_interlock():
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    async def run():
        service = NavigationService(SimulationBridge(), {'drive_mode': 'differential'}, simulated=True)
        service._closed = False
        service.zero = lambda: asyncio.sleep(0)
        hardware = {'health': 'stale_scan', 'hardware_health': 'ok', 'limits': [1.6, .8]}
        def latest(name, *args):
            if name == 'status': return SimpleNamespace(data=hardware)
            if name == 'odom': return SimpleNamespace(data={'twist': {'twist': {'linear': {'x': 0.}, 'angular': {'z': 0.}}}})
            raise ValueError('missing_' + name)
        service.latest = latest
        writes = []
        async def write(command, **kwargs): writes.append(command)
        service.bridge.velocity.set = write
        lease = await service.claim()
        await service.command(dict(op='manual_collision', lease=lease, sequence=1, collision_check=False))
        await service.command(dict(op='arm', lease=lease, sequence=2))
        await service.command(dict(op='manual', lease=lease, sequence=3, throttle=-.4, steering=0, input_source='joystick'))
        await service.tick()
        assert writes[-1].linear_x < 0 and service.bridge.manual_direct
        assert service.reason == 'manual_direct' and service.armed
        hardware['hardware_health'] = 'bumper'
        await service.tick()
        assert not service.armed and service.reason == 'bumper'
        # Navigation always uses full sensor health even when manual bypass is on.
        service.mode = 'navigation'
        assert service.motion_health(hardware) == 'stale_scan'
    asyncio.run(run())


def test_native_direct_gate_still_expires_and_rejects_hardware_fault():
    gate = gate_module().VelocityGate('s', enabled=True)
    now = time.monotonic_ns()
    gate.update(dict(session='s', sequence=1, controller_epoch='e', deadline_ns=now+250000000,
                     armed=True, manual_direct=True, drive_mode='differential', linear_x=-.1, angular_z=0.), now)
    assert gate.output(now, 'ok', (1., .8))[0] == (-.1, 0.)
    assert gate.output(now, 'bumper', (1., .8))[0] == (0., 0.)
    assert gate.output(now+250000001, 'ok', (1., .8))[0] == (0., 0.)
