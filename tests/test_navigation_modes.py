"""Physical model and arbitration requirements, using no ROS publishers."""
import asyncio
from dataclasses import replace
import math
import time

import numpy as np
import pytest

from rsim.components.vehicle import TowingParameters, TowingDynamics, DifferentialDynamics


def test_drive_resistance_equilibrium_cap_and_software_acceleration():
    p = TowingParameters()
    assert 1.6 < p.terminal_speed < 2.1
    model = DifferentialDynamics(p)
    speed = np.zeros(2)
    early = model.advance_velocity(speed, [1.6, 0], .1)[0]
    assert .05 < early < .06  # .6 drive minus .05 friction, then quadratic drag
    for _ in range(400):
        updated = model.advance_velocity(speed, [2., 2.], .05)
        assert np.all(abs(updated-speed) <= np.array([.6, .8]) * .05 + 1e-10)
        speed = updated
    assert speed == pytest.approx([1.6, .8])
    assert model.advance_velocity([1.5, 0], [1.6, 0], .1)[0] - 1.5 < early
    for _ in range(80): speed = model.advance_velocity(speed, [0, 0], .05)
    assert np.allclose(speed, 0)
    with pytest.raises(ValueError): TowingParameters(max_linear=1.61)


@pytest.mark.parametrize('dt', [.01, .02, .1])
def test_signed_differential_pivot_and_reverse(dt):
    model = DifferentialDynamics()
    speed = np.zeros(2)
    for _ in range(round(4 / dt)): speed = model.advance_velocity(speed, [0, .6], dt)
    assert speed == pytest.approx([0, .6])
    target = model.targets(-1, 0)
    for _ in range(round(6 / dt)): speed = model.advance_velocity(speed, [target.linear_x, 0], dt)
    assert speed == pytest.approx([-1.6, 0], abs=.005)


def test_towing_resistance_braking_and_vectorized_rollouts_agree():
    model = TowingDynamics()
    current = np.array([[.8, .12], [.1, -.1], [.02, .02]])
    for _ in range(100):
        old = current.copy()
        current = model.advance_velocity(current, np.zeros_like(current), .02)
        assert current == pytest.approx(np.stack([model.advance_velocity(v, [0, 0], .02) for v in old]))
        assert np.all(current[:, 0] >= abs(current[:, 1]) * .3 - 1e-9)
        assert np.all(abs(current[:, 1]) * .81 <= .12 + 1e-9)
    assert np.allclose(current, 0)


@pytest.mark.parametrize('rear', [.27, .81])
def test_differential_navigation_pivots_to_goal_with_either_pedal(rear):
    from pyrobo import Bounds2D
    from rsim.components.towing_geometry import BodyFootprint, advance
    from rsim.components.towing_navigation import TowingNavigator
    nav = TowingNavigator(footprint=BodyFootprint(rear=rear), drive_mode='differential')
    obs = nav.obstacle_index(np.empty((0, 2)), 'map', .02)
    bounds = Bounds2D(-5, 5, -5, 5)
    state, speed = np.zeros(3), np.zeros(2)
    for _ in range(160):
        decision = nav.step(state, speed, [0, 0, math.pi / 2], obs, bounds)
        assert decision.safe
        cmd = np.array([decision.command.linear_x, decision.command.angular_z])
        assert cmd[0] == 0
        state = advance(state, *((speed+cmd)/2), .1)
        speed = cmd
        if decision.status == 'arrived': break
    assert decision.status == 'arrived'
    assert abs(state[2] - math.pi / 2) < .2


def test_manual_preempts_navigation_resume_and_late_goals():
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    from rsim.runtime import Runtime

    async def run():
        service = NavigationService(SimulationBridge(), {'optional_scans': []}, simulated=True)
        async with Runtime(service):
            for _ in range(80):
                if service.map is not None: break
                await asyncio.sleep(.02)
            lease = await service.claim()
            async def send(n, op, **data):
                await service.command(dict(lease=lease, sequence=n, op=op, **data))
            await send(1, 'goal', x=1, y=0, yaw=0)
            await send(2, 'manual', throttle=.2, steering=0)
            await send(3, 'goal', x=0, y=1, yaw=1)
            assert service.mode == 'manual'
            await send(4, 'resume')
            assert service.mode == 'manual' and service.manual_active()
            await send(5, 'manual', throttle=0, steering=0)
            assert service.mode == 'paused' and np.allclose(service.goal, [0, 1, 1])
            # Geometry and kinematics are separate and support all combinations.
            for n, rear in [(6, False), (8, True)]:
                await send(n, 'footprint', extended=rear)
                await send(n+1, 'drive_mode', drive_mode='differential')
                assert service.footprint.rear == (.81 if rear else .27)
                assert service.navigator.model.targets(0, 1).angular_z > 0
            await service.stop(cancel=True)

    asyncio.run(run())


def test_cancelled_stop_request_still_finishes_after_inflight_publication():
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    from rsim.runtime import Runtime

    async def run():
        bridge = SimulationBridge()
        service = NavigationService(bridge, {'optional_scans': []}, simulated=True)
        entered, release = asyncio.Event(), asyncio.Event()
        original = bridge.velocity.apply
        async def delayed(envelope):
            await original(envelope)
            if envelope.value.linear_x and not entered.is_set():
                entered.set()
                await release.wait()
        bridge.velocity.apply = delayed
        async with Runtime(service):
            async with asyncio.timeout(5):
                while service.map is None: await asyncio.sleep(.02)
            lease = await service.claim()
            await service.command(dict(lease=lease, sequence=1, op='arm'))
            await service.command(dict(lease=lease, sequence=2, op='manual', throttle=.3, steering=0))
            await asyncio.wait_for(entered.wait(), 2)
            request = asyncio.create_task(service.stop('disconnect'))
            await asyncio.sleep(.01)
            request.cancel()
            await asyncio.gather(request, return_exceptions=True)
            release.set()
            await asyncio.sleep(.15)
            assert not service.armed
            assert service.published == [0., 0.]
            assert np.allclose(bridge.target, 0)
    asyncio.run(run())


def test_old_navigation_error_does_not_disarm_new_manual_generation():
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    from rsim.apps.navigation.planning import PlannerPair
    from rsim.runtime import Runtime

    async def run():
        entered, release = asyncio.Event(), asyncio.Event()
        workers = PlannerPair()
        async def delayed_error(service, manual):
            entered.set()
            await release.wait()
            raise ValueError('planner_input_expired')
        workers.compute = delayed_error
        service = NavigationService(SimulationBridge(), {'optional_scans': []},
                                    simulated=True, workers=workers)
        async with Runtime(service):
            async with asyncio.timeout(5):
                while service.map is None: await asyncio.sleep(.02)
            lease = await service.claim()
            async def send(n, op, **data):
                await service.command(dict(lease=lease, sequence=n, op=op, **data))
            await send(1, 'arm')
            await send(2, 'goal', x=1, y=0, yaw=0)
            await asyncio.wait_for(entered.wait(), 2)
            await send(3, 'manual', throttle=.2, steering=0)
            before = service.state.frames[-1].sequence
            release.set()
            await service.state.get(after=before, timeout=2)
            assert service.armed and service.mode == 'manual'
            assert service.last_stop['reason'] == 'new_controller'
            # An error from the current manual generation must still stop it.
            await service.state.get(after=service.state.frames[-1].sequence, timeout=2)
            assert not service.armed and service.last_stop['reason'] == 'planner_input_expired'
    asyncio.run(run())
