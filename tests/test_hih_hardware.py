"""Field-specific input decoding and conservative collision-envelope changes."""
import asyncio
import importlib.util
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest


def hardware_module():
    root = Path(__file__).resolve().parents[1] / 'compat'
    sys.path.insert(0, str(root))
    spec = importlib.util.spec_from_file_location('hih_hardware_test', root / 'ros1_hih_hardware.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CONFIG = dict(interface='can1', status_id=0x184, command_id=0x604,
              retracted_payload='0400000000000000', timeout=1.5)


def test_pedal_only_fresh_confirmed_retraction_can_shrink():
    pedal = hardware_module().PedalState(CONFIG)
    feedback = bytes.fromhex(CONFIG['retracted_payload'])
    assert not pedal.snapshot(10)['retracted']
    pedal.receive(0x184, feedback, 10.)
    assert not pedal.snapshot(10.)['retracted']
    pedal.receive(0x184, feedback, 10.5)
    assert pedal.snapshot(10.5)['retracted']
    assert not pedal.snapshot(12.1)['retracted']
    pedal.receive(0x184, feedback, 12.2)
    assert not pedal.snapshot(12.2)['retracted']  # stale gap needs two new frames
    pedal.receive(0x184, feedback, 12.7)
    assert pedal.snapshot(12.7)['retracted']
    for raw in ('0000000000000000', '0100000000000000', '0200000000000000',
                '0500000000000000', '0400000000000001', '04'):
        pedal.receive(0x184, bytes.fromhex(raw), 13.)
        assert not pedal.snapshot(13.)['retracted']


def test_extension_invalidates_old_retraction_until_departure_and_return():
    pedal = hardware_module().PedalState(CONFIG)
    feedback = bytes.fromhex(CONFIG['retracted_payload'])
    for t in (10., 10.5): pedal.receive(0x184, feedback, t)
    pedal.receive(0x604, bytes.fromhex('2b01400001000000'), 10.6)
    assert not pedal.snapshot(10.6)['retracted']
    for t in (11., 11.5, 12.): pedal.receive(0x184, feedback, t)
    assert not pedal.snapshot(12.)['retracted']
    pedal.receive(0x184, bytes.fromhex('0100000000000000'), 12.5)
    pedal.receive(0x604, bytes.fromhex('2b01400002000000'), 12.6)
    for t in (13., 13.5): pedal.receive(0x184, feedback, t)
    assert pedal.snapshot(13.5)['retracted']


def test_unrelated_or_extended_can_identifier_does_not_confirm_pedal():
    pedal = hardware_module().PedalState(CONFIG)
    for identifier in (0x182, 0x584, 0x80000184, 0x40000184):
        for t in (10., 10.5): pedal.receive(identifier, bytes.fromhex(CONFIG['retracted_payload']), t)
    assert not pedal.snapshot(10.5)['retracted']


def test_stop_preserves_only_a_known_retracted_endpoint_and_not_across_feedback_loss():
    pedal = hardware_module().PedalState(CONFIG)
    feedback = bytes.fromhex(CONFIG['retracted_payload'])
    stopped = bytes(8)
    stop_command = bytes.fromhex('2b01400000000000')
    for t in (10., 10.5): pedal.receive(0x184, feedback, t)
    pedal.receive(0x604, stop_command, 10.6)
    pedal.receive(0x184, stopped, 11.)
    assert pedal.snapshot(11.)['retracted']
    assert pedal.snapshot(11.)['retained_endpoint']
    pedal.receive(0x604, bytes.fromhex('2b01400001000000'), 11.1)
    pedal.receive(0x604, stop_command, 11.2)
    pedal.receive(0x184, stopped, 11.5)
    assert not pedal.snapshot(11.5)['retracted']
    # A fresh process or a stale gap cannot infer position from neutral 00.
    for t in (12., 12.5): pedal.receive(0x184, feedback, t)
    pedal.receive(0x604, stop_command, 12.6)
    pedal.receive(0x184, stopped, 14.6)
    assert not pedal.snapshot(14.6)['retracted']


def test_pedal_monitor_reads_clock_after_new_frame_wins_the_lock(monkeypatch):
    module = hardware_module()
    monitor = module.PedalMonitor.__new__(module.PedalMonitor)
    monitor.state = module.PedalState(CONFIG)
    monitor.error = None
    feedback = bytes.fromhex(CONFIG['retracted_payload'])
    for t in (10., 10.3): monitor.state.receive(0x184, feedback, t)
    clock = [10.4]
    class ReceiveBeforeReader:
        def __enter__(self):
            monitor.state.receive(0x184, feedback, 10.5)
            clock[0] = 10.51
        def __exit__(self, *args): pass
    monitor.lock = ReceiveBeforeReader()
    monkeypatch.setattr(module, 'monotonic', lambda: clock[0])
    result = monitor.snapshot()
    assert result['feedback'] and result['retracted']
    assert result['age_sec'] == pytest.approx(.01)


def test_feedback_changes_cancel_motion_and_manual_cannot_shrink():
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    from rsim.apps.navigation.settings import defaults

    async def run():
        service = NavigationService(SimulationBridge(), {'ros': {'pedal_feedback': CONFIG}}, simulated=False)
        service._closed = False
        service.zero = lambda: asyncio.sleep(0)
        state = {'pedal': dict(feedback=True, retracted=True, position='retracted', feedback_ns=time.monotonic_ns())}
        def latest(name, *args):
            if name != 'status': raise ValueError('missing_' + name)
            return SimpleNamespace(data=state)
        service.latest = latest
        await service.refresh_pedal()
        assert service.footprint.rear == .27
        await service.pedal_action('stop')
        assert service.footprint.rear == .27
        service.armed, service.mode, service.goal = True, 'manual', [1, 2, 3]
        service.generation = 3
        state['pedal']['retracted'] = False
        state['pedal']['position'] = 'extended_or_moving'
        await service.refresh_pedal()
        assert service.footprint.rear == .81
        assert not service.armed and service.goal is None and service.generation == 4
        await service.apply_settings(defaults({'footprint': {'rear': .27}}))
        assert service.footprint.rear == .81
        lease = await service.claim()
        with pytest.raises(ValueError, match='hardware feedback'):
            await service.command(dict(op='footprint', lease=lease, sequence=1, extended=False))
        state['pedal'].update(retracted=True, feedback_ns=time.monotonic_ns())
        await service.pedal_action('extend')
        await service.refresh_pedal()
        assert service.footprint.rear == .81  # feedback predates our command
        state.clear()
        await service.refresh_pedal()
        assert service.footprint.rear == .81
    asyncio.run(run())


def test_pedal_uses_can_sample_age_instead_of_status_delivery_age():
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge

    async def run():
        service = NavigationService(SimulationBridge(), {'ros': {'pedal_feedback': CONFIG}}, simulated=False)
        service._closed = False
        service.zero = lambda: asyncio.sleep(0)
        data = {'pedal': dict(feedback=True, retracted=True, position='retracted',
                             feedback_ns=time.monotonic_ns()-700_000_000)}
        def latest(name, max_age=None):
            if name != 'status': raise ValueError('missing_' + name)
            if max_age is not None and max_age < .6: raise ValueError('stale_status')
            return SimpleNamespace(data=data)
        service.latest = latest
        await service.refresh_pedal()
        assert service.footprint.rear == .27
        service.armed = True
        await service.refresh_pedal()
        assert service.armed  # same valid CAN endpoint cannot toggle geometry
        data['pedal']['feedback_ns'] = time.monotonic_ns()-1_600_000_000
        await service.refresh_pedal()
        assert service.footprint.rear == .81 and not service.armed
        assert service.last_disarm['reason'] == 'pedal_footprint_changed'
        await service.stop('manual_released')
        assert service.last_stop['reason'] == 'manual_released'
        assert service.last_disarm['reason'] == 'pedal_footprint_changed'
        data['pedal'].update(feedback_ns=time.monotonic_ns(), retracted=False, position='extending')
        await service.refresh_pedal()
        assert service.footprint.rear == .81
    asyncio.run(run())


def test_self_return_filter_keeps_real_nearby_and_far_obstacles():
    import numpy as np
    from rsim.components.navigation_map import laser_points
    # Two beams of padding cover adjacent bins. A third
    # beam outside, a farther return inside, and other near directions survive.
    message = dict(angle_min=-.06, angle_increment=.01, range_min=.1, range_max=50.,
                   ranges=[.18, .18, .18, .18, .4, .18, .18, .18, .18, .18, .18, .18, .18])
    tf = dict(translation=[1, 2, 0], quaternion=[0, 0, 0, 1])
    filter_ = dict(max_range=.25, padding_beams=2, angle_masks=[[-.02, .02]])
    points = laser_points(message, tf, filter_)
    local = points - [1, 2]
    assert len(local) == 5
    assert sorted(np.linalg.norm(local, axis=1)) == pytest.approx([.18, .18, .18, .18, .4])
    angles = np.arctan2(local[:, 1], local[:, 0])
    assert sorted(angles) == pytest.approx([-.06, -.05, -.02, .05, .06])
    assert len(laser_points(message, tf)) == 13


def test_hih_calibrated_edges_cover_recorded_self_returns():
    import numpy as np
    from rsim.components.navigation_map import laser_points
    # Recorded stationary body echoes span up to 3.19 bins outside OEM masks.
    ranges = [float('inf')] * 1200
    for index, distance in ((110, .185), (417, .172), (1059, .188), (1060, .188)):
        ranges[index] = distance
    ranges[600] = .18  # unmasked front obstacle
    ranges[138] = .4   # farther return inside a mask
    ranges[1058] = .19 # outside the four-bin calibrated padding
    message = dict(angle_min=-3.1415927410125732, angle_increment=.005235987715423107,
                   range_min=.1, range_max=50., ranges=ranges)
    tf = dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1])
    filter_ = dict(max_range=.25, padding_beams=4,
                   angle_masks=[[-2.55, -2.42], [-.945, -.8], [.8, .945], [2.42, 2.55]])
    points = laser_points(message, tf, filter_)
    assert sorted(np.linalg.norm(points, axis=1)) == pytest.approx([.18, .19, .4])


def test_fresh_empty_auxiliary_depth_scan_is_not_a_missing_sensor():
    from rsim.components.navigation_map import laser_points
    tf = dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1])
    # Actual scan_top frame: 178 no-return rays and three invalid zeros.
    scan = dict(angle_min=-.8, angle_increment=.01, range_min=.1, range_max=6.,
                ranges=[float('inf')] * 178 + [0.] * 3)
    assert laser_points(scan, tf, allow_empty=True).shape == (0, 2)
    with pytest.raises(ValueError, match='no usable returns'):
        laser_points(scan, tf)  # required primary lidar remains fail-closed
    scan['ranges'][90] = .18
    assert laser_points(scan, tf, allow_empty=True).shape == (1, 2)
    for invalid in ([float('nan')] * 181, [0.] * 181,
                    [float('inf')] * 180 + [float('-inf')]):
        scan['ranges'] = invalid
        with pytest.raises(ValueError, match='no usable returns'):
            laser_points(scan, tf, allow_empty=True)


def test_measured_chassis_dimensions_preserve_original_extended_shape():
    import asyncio
    from types import SimpleNamespace
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    async def run():
        config = dict(ros={'pedal_feedback':{'interface':'can1'}}, footprint={'margin':.03},
                      pedal_shapes={'retracted':{'front':.25,'rear':.25},'extended':{'front':.27,'rear':.81}})
        service = NavigationService(SimulationBridge(), config)
        service._closed = False
        async def zero(): pass
        service.zero = zero
        data = {'pedal':dict(feedback=True,retracted=True,feedback_ns=time.monotonic_ns())}
        def latest(name, *args):
            if name != 'status': raise ValueError('missing_' + name)
            return SimpleNamespace(data=data)
        service.latest = latest
        await service.refresh_pedal()
        assert service.footprint.front == .25 and service.footprint.rear == .25
        assert service.footprint.width == .54 and service.footprint.margin == .03
        data['pedal'].update(retracted=False,feedback_ns=time.monotonic_ns())
        await service.refresh_pedal()
        assert service.footprint.front == .27 and service.footprint.rear == .81
    asyncio.run(run())


def test_measured_outer_beam_is_masked_but_far_obstacle_is_retained():
    import numpy as np
    from rsim.components.navigation_map import laser_points
    data = dict(ranges=np.full(1200, 2.), angle_min=-3.1415927410125732,
                angle_increment=.005235987715423107, range_min=.05, range_max=10.)
    data['ranges'][1058] = .185  # actual 2.39808 rad return, just outside old padding
    transform = dict(translation=[0,0,0],quaternion=[0,0,0,1])
    before = laser_points(data, transform, dict(max_range=.25,padding_beams=4,angle_masks=[[2.42,2.55]]))
    after = laser_points(data, transform, dict(max_range=.25,padding_beams=5,angle_masks=[[2.42,2.55]]))
    assert len(after) == len(before)-1
    assert np.min(np.linalg.norm(after,axis=1)) > 1.99
    data['ranges'][1058] = .7
    assert len(laser_points(data, transform, dict(max_range=.25,padding_beams=5,angle_masks=[[2.42,2.55]]))) == 1200
