"""Manual intent semantics and durable settings, without physical outputs."""
from copy import deepcopy
from dataclasses import replace
import math
import time
import numpy as np
import pytest
from rsim.components.manual_control import ManualController
from rsim.components.vehicle import drive_model, TowingParameters
from rsim.apps.navigation.settings import SettingsStore, defaults, validate, apply_config


@pytest.mark.parametrize('mode', ['towing', 'differential'])
@pytest.mark.parametrize('axis', ['acceleration', 'velocity_target'])
def test_held_reverse_brakes_then_fresh_press_accelerates(mode, axis):
    model = drive_model(mode=mode)
    control = ManualController()
    axes = dict(defaults({})['manual'][mode], y_mode=axis)
    speed = np.array([.8, 0.])
    control.input(-1, 0, speed, 0.)
    for _ in range(60):
        control.input(-1, 0, speed, 0.)  # repeat packets do not rearm reverse
        cmd = control.target(speed, 0., model, axes, (1.6, .8))
        updated = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
        assert updated[0] >= 0
        assert abs(updated[0]-speed[0]) <= model.parameters.linear_deceleration * .1 + 1e-9
        speed = updated
    assert speed[0] == 0 and control.brake_locked
    control.input(0, 0, speed, 0.)
    control.input(-1, 0, speed, 0.)
    for _ in range(20):
        cmd = control.target(speed, 0., model, axes, (1.6, .8))
        updated = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
        assert abs(updated[0]-speed[0]) <= model.parameters.linear_acceleration * .1 + 1e-9
        speed = updated
    assert speed[0] < -.7


@pytest.mark.parametrize('mode', ['towing', 'differential'])
@pytest.mark.parametrize('direction', [-1, 1])
def test_front_steering_angle_converges_and_reverses_yaw(mode, direction):
    model = drive_model(mode=mode)
    control = ManualController()
    axes = dict(y_mode='velocity_target', x_mode='angle_target', angle_reference='steering')
    speed = np.zeros(2)
    control.input(direction*.4, .7, speed, 0.)
    previous = 0.
    for _ in range(100):
        cmd = control.target(speed, 0., model, axes, (1., .8))
        speed = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
        assert abs(control.steering_angle-previous) <= model.parameters.steering_rate*.1+1e-9
        previous = control.steering_angle
    assert control.steering_angle == pytest.approx(.7*math.radians(35), abs=.002)
    assert speed[0]*direction > 0 and speed[1]*direction > 0


@pytest.mark.parametrize('mode', ['towing', 'differential'])
@pytest.mark.parametrize('x_mode', ['velocity_target', 'acceleration', 'angle_target'])
def test_independent_axis_bindings_accept_all_models(mode, x_mode):
    model = drive_model(mode=mode)
    control = ManualController()
    axes = dict(y_mode='velocity_target', x_mode=x_mode, angle_reference='heading')
    speed = np.zeros(2)
    control.input(.4, .6, speed, .2)
    yaw = .2
    for _ in range(100):
        cmd = control.target(speed, yaw, model, axes, (1., .8))
        speed = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
        yaw += speed[1] * .1
    if x_mode == 'angle_target': assert yaw == pytest.approx(.2+.6*math.pi/2, abs=.05)
    else: assert speed[1] > 0


def test_small_effort_cannot_reverse_due_to_friction():
    control = ManualController(); model = drive_model()
    control.input(.01, 0, [0, 0], 0)
    cmd = control.target([0, 0], 0, model, defaults({})['manual']['towing'], (1., .8))
    assert cmd.linear_x == 0


def test_settings_atomic_persistence_restart_split_and_revision(tmp_path):
    path = tmp_path/'settings.json'
    store = SettingsStore(path, {})
    values = deepcopy(store.values)
    values['keyboard']['forward'] = 'ArrowUp'
    values['profiles']['differential']['linear_deceleration'] = 1.8
    values['permissions']['localization'] = True
    store.save(values, 0)
    assert store.active['keyboard']['forward'] == 'ArrowUp'
    assert not store.active['permissions']['localization']
    assert set(store.view()['restart_required']) == {'profiles', 'permissions'}
    with pytest.raises(ValueError): store.save(values, 0)
    restarted = SettingsStore(path, {})
    assert restarted.active == values and restarted.view()['restart_required'] == []
    bad = deepcopy(values); bad['keyboard']['stop'] = 'ArrowUp'
    with pytest.raises(ValueError): store.save(bad, 1)
    assert SettingsStore(path, {}).values == values


def test_settings_api_stages_permissions_applies_keys_and_preserves_on_reopen(tmp_path):
    from fastapi.testclient import TestClient
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    from rsim.apps.navigation.server import create_app
    service = NavigationService(SimulationBridge(), {'optional_scans': []}, simulated=True)
    with TestClient(create_app(service, None, data_dir=tmp_path)) as client:
        for _ in range(50):
            if service.measured is not None and service.map is not None: break
            time.sleep(.05)
        lease = client.post('/api/claim').json()['lease']
        d = client.get('/api/settings').json()
        d['values']['permissions']['motion'] = True
        d['values']['manual']['differential']['x_mode'] = 'acceleration'
        result = client.post('/api/settings', json=dict(lease=lease, sequence=1,
            deadline_ms=time.monotonic()*1000+1500, revision=d['revision'], values=d['values']))
        assert result.status_code == 200, result.text
        data=result.json()
        assert data['restart_required'] == ['permissions']
        assert service.config['manual']['differential']['x_mode'] == 'acceleration'
        assert not service.config['enable_motion'] and not service.armed
        restart=client.post('/api/service/restart', json=dict(lease=lease, sequence=2, deadline_ms=time.monotonic()*1000+1500))
        assert restart.status_code == 409  # a test server cannot kill its host
    assert SettingsStore(tmp_path/'settings.json', {}).values['permissions']['motion'] is True
