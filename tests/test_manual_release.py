"""Release uses vehicle dynamics; only explicit stops and faults write zero."""
import asyncio
from types import SimpleNamespace

import numpy as np
import pytest

from rsim.apps.navigation.service import NavigationService
from rsim.apps.navigation.simulation import SimulationBridge
from rsim.components.manual_control import ManualController
from rsim.components.vehicle import drive_model


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('mode', ['towing', 'differential'])
def test_acceleration_release_coasts_and_reverse_key_brakes(mode, direction):
    model = drive_model(mode=mode)
    control = ManualController()
    axes = dict(y_mode='acceleration', x_mode='velocity_target')
    speed = np.array([direction * .5, 0.])
    control.input(direction, 0., speed)
    control.input(0., 0., speed, active=False)
    cmd = control.target(speed, 0., model, axes, (1., .8))
    coasting = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
    # At .5 m/s, .05 + .15*v² gives about .0087 m/s loss per 100 ms.
    assert .008 < abs(speed[0]) - abs(coasting[0]) < .009
    control.input(-direction, 0., speed)
    cmd = control.target(speed, 0., model, axes, (1., .8))
    braking = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
    assert abs(speed[0]) - abs(braking[0]) == pytest.approx(.12)
    assert control.brake_locked
    # Fresh neutral clears the latch and dissipates remaining speed to rest.
    control.input(0., 0., coasting, active=False)
    for _ in range(100):
        cmd = control.target(coasting, 0., model, axes, (1., .8))
        updated = model.advance_velocity(coasting, [cmd.linear_x, cmd.angular_z], .1)
        assert 0 <= direction * updated[0] <= direction * coasting[0]
        coasting = updated
    assert np.allclose(coasting, 0.)


def test_angular_acceleration_release_coasts_without_turning_back():
    model = drive_model(mode='differential')
    control = ManualController()
    axes = dict(y_mode='acceleration', x_mode='acceleration')
    speed = np.array([0., .4])
    control.input(0., 0., speed, active=False)
    cmd = control.target(speed, 0., model, axes, (1., .8))
    updated = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
    assert .018 < speed[1] - updated[1] < .020
    # A released relative-heading input must not seek its original yaw.
    control.heading_origin = 0.
    axes.update(x_mode='angle_target', angle_reference='heading')
    assert control.target([0., 0.], .5, model, axes, (1., .8)).angular_z == 0.


def test_wheel_noise_does_not_accelerate_a_released_acceleration_axis():
    model = drive_model(mode='differential')
    control = ManualController()
    axes = dict(y_mode='acceleration', x_mode='acceleration')
    previous = np.array([.15, -.3])
    for noise in (0., .01, -.005, .02, -.01, .01):
        measured = previous + np.array([noise, -noise])
        cmd = control.target(measured, 0., model, axes, (1., .8), previous=previous)
        updated = model.advance_velocity(measured, [cmd.linear_x, cmd.angular_z], .1)
        assert 0 < updated[0] < previous[0]
        assert previous[1] < updated[1] < 0
        previous = updated


@pytest.mark.parametrize('direction', [-1, 1])
def test_joystick_step_and_center_are_acceleration_limited(direction):
    model = drive_model(mode='differential')
    p = model.parameters
    control = ManualController()
    axes = dict(y_mode='velocity_target', x_mode='velocity_target')
    speed = np.zeros(2)
    control.input(direction, direction, speed, source='joystick')
    for _ in range(20):
        cmd = control.target(speed, 0., model, axes, (1., .8))
        updated = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
        assert np.all(direction * (updated - speed) >= -1e-9)
        assert abs(updated[0] - speed[0]) <= p.linear_acceleration * .1 + 1e-9
        assert abs(updated[1] - speed[1]) <= p.angular_acceleration * .1 + 1e-9
        speed = updated
    assert abs(speed[0]) > .8
    control.input(0., 0., speed, source='joystick')
    for i in range(20):
        cmd = control.target(speed, 0., model, axes, (1., .8))
        updated = model.advance_velocity(speed, [cmd.linear_x, cmd.angular_z], .1)
        assert abs(updated[0] - speed[0]) <= p.linear_deceleration * .1 + 1e-9
        assert abs(updated[1] - speed[1]) <= p.angular_deceleration * .1 + 1e-9
        assert np.all(direction * updated >= -1e-9)
        if i == 0: assert abs(updated[0]) > .6
        speed = updated
    assert np.allclose(speed, 0.)


@pytest.mark.parametrize('collision_check', [True, False])
@pytest.mark.parametrize('axis', ['acceleration', 'velocity_target'])
@pytest.mark.parametrize('release_mode', ['coast', 'target_zero'])
def test_neutral_uses_controller_until_rest_but_heartbeats_do_not_extend_held_input(collision_check, axis, release_mode):
    async def run():
        config = dict(drive_mode='differential', manual={'differential':
            dict(y_mode=axis, x_mode='velocity_target', angle_reference='heading', release_mode=release_mode)})
        service = NavigationService(SimulationBridge(), config, simulated=True)
        service._closed = service.bridge._closed = False
        service.manual_collision_check = collision_check
        speed, writes = np.array([.5, 0.]), []
        hardware = dict(health='ok', limits=[1.6, .8])
        def latest(name, *args):
            if name == 'status': return SimpleNamespace(data=hardware)
            if name == 'odom': return SimpleNamespace(data={'twist': {'twist':
                {'linear': {'x': speed[0]}, 'angular': {'z': speed[1]}}}})
            raise ValueError('missing_' + name)
        service.latest = latest
        async def write(command, **kwargs): writes.append(command)
        service.bridge.velocity.set = write
        def compute(goal, manual):
            from pyrobo import Bounds2D
            state, obstacles = np.zeros(3), np.empty((0, 2))
            index = service.navigator.obstacle_index(obstacles, 'map', .02)
            result = service.navigator.step(state, speed.copy(), goal, index, Bounds2D(-5, 5, -5, 5), manual=manual)
            return result, state, speed.copy(), obstacles
        service.compute = compute
        lease = await service.claim()
        sequence = 0
        async def send(op, **data):
            nonlocal sequence
            sequence += 1
            await service.command(dict(op=op, lease=lease, sequence=sequence, **data))
        await send('arm')
        await send('manual', throttle=1.)
        service.measured, service.published = speed.tolist(), speed.tolist()
        writes.clear()
        await send('manual', throttle=0., steering=0., input_active=False)
        assert not writes and service.mode == 'manual' and service.armed
        service.manual_deadline = 0.  # released effort need not be repeated
        await send('heartbeat')
        assert service.manual_active()
        await service.tick()
        assert service.armed and service.status()['manual_control']['releasing']
        assert writes[-1].linear_x == pytest.approx(.4913 if release_mode == 'coast' else .38, abs=.0001)
        speed[:] = 0.
        await service.tick()
        assert service.mode == 'idle' and service.reason == 'manual_released' and service.armed
        assert writes[-1].linear_x == 0.
        # A lost nonzero command is still a deadman fault despite heartbeats.
        await send('manual', throttle=1.)
        service.manual_deadline = 0.
        await send('heartbeat')
        await service.tick()
        assert not service.armed and service.reason == 'manual_deadman'
        # Holding the joystick in its dead zone is still an active input;
        # a heartbeat alone must not extend its 250 ms input deadline.
        await send('arm')
        await send('manual', throttle=0., input_source='joystick', input_active=True)
        assert not service.status()['manual_control']['releasing']
        service.manual_deadline = 0.
        await send('heartbeat')
        await service.tick()
        assert not service.armed and service.reason == 'manual_deadman'
        # A neutral coast may not outlive its owner or an explicit stop either.
        for failure in ('stop', 'disconnect'):
            speed[:] = [.5, 0.]
            await send('arm')
            await send('manual', throttle=0., input_active=False)
            if failure == 'stop': await send('stop')
            else:
                service.controllers[lease]['deadline'] = 0.
                await service.tick()
            assert not service.armed and writes[-1].linear_x == 0.
            assert service.reason == ('stop' if failure == 'stop' else 'controller_disconnected')
    asyncio.run(run())


@pytest.mark.parametrize('binding', ['acceleration', 'velocity_target'])
@pytest.mark.parametrize('release_mode', ['coast', 'target_zero'])
def test_held_joystick_center_and_auto_center_have_distinct_intents(binding, release_mode):
    model = drive_model(mode='differential')
    control = ManualController()
    axes = dict(y_mode=binding, x_mode=binding, release_mode=release_mode)
    measured = np.array([.5, -.4])
    control.input(0., 0., measured, source='joystick', active=True)
    cmd = control.target(measured, 0., model, axes, (1., .8))
    held_center = model.advance_velocity(measured, [cmd.linear_x, cmd.angular_z], .1)
    assert held_center == pytest.approx([.38, -.24])
    control.input(0., 0., measured, source='joystick', active=False)
    cmd = control.target(measured, 0., model, axes, (1., .8))
    released = model.advance_velocity(measured, [cmd.linear_x, cmd.angular_z], .1)
    if release_mode == 'target_zero': assert released == pytest.approx(held_center)
    else:
        assert .49 < released[0] < .5
        assert -.4 < released[1] < -.38
    with pytest.raises(ValueError, match='zero axes'):
        control.input(.2, 0., measured, source='joystick', active=False)


def test_resistance_deceleration_uses_braking_bound_not_small_drive_effort():
    model = drive_model(mode='differential')
    # Small positive effort loses to resistance. Its net slowing is governed
    # by the deceleration bound, not the much smaller positive drive effort.
    scalar = model._axis(1., 1.6, .1, .01, .2, .1, .5)
    vector = model._axis([1., -1.], [1.6, -1.6], .1, .01, .2, .1, .5)
    assert .025 < 1. - scalar < .03
    assert vector == pytest.approx([scalar, -scalar])
    assert model.coast_axis(.5, .1, .2, .1, .03) == pytest.approx(.497)
