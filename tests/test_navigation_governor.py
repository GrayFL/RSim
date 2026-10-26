"""Braking-limited teleoperation, blocked-route progress and physical tray masks."""
import math
import numpy as np
import pytest
from pyrobo import Bounds2D
from rsim.components.towing_navigation import TowingNavigator
from rsim.components.towing_geometry import BodyFootprint, advance, wrap
from rsim.components.navigation_map import laser_points
from rsim.core.commands import VelocityCommand

BODY = BodyFootprint(front=.1, rear=.4, width=.54, margin=.03)
BOUNDS = Bounds2D(-6, 6, -6, 6)


def world(points=(), mode='differential'):
    nav = TowingNavigator(footprint=BODY, drive_mode=mode)
    nav.local.cruise_limits = (.7, .8)
    return nav, nav.obstacle_index(np.asarray(points).reshape(-1, 2), 'map', .035)


def move(nav, state, speed, command):
    target = np.array([command.linear_x, command.angular_z])
    for _ in range(2):
        new = nav.model.advance_velocity(speed, target, .05)
        state = advance(state, *((speed + new) / 2), .05)
        speed = new
    return state, speed


@pytest.mark.parametrize('sign', [1, -1])
def test_held_demand_approaches_wall_smoothly_instead_of_rejecting_two_second_horizon(sign):
    wall = np.c_[np.full(241, sign * 1.4), np.linspace(-6, 6, 241)]
    nav, obstacles = world(wall)
    state, speed = np.zeros(3), np.zeros(2)
    history = []
    for _ in range(150):
        result = nav.step(state, speed, None, obstacles, BOUNDS, manual=VelocityCommand(sign, 0))
        assert result.safe, (state, speed, result.clearance)
        assert result.command.linear_x * sign >= 0 and result.command.angular_z == 0
        state, speed = move(nav, state, speed, result.command)
        assert obstacles.clearances(state[None], BODY, BOUNDS)[0] > 0
        history.append(speed[0] * sign)
    assert max(history) > .6 and history[0] > 0
    assert max(history[-10:]) < .001
    end = 1.4 - (BODY.front if sign > 0 else BODY.rear) - BODY.margin - .035
    assert end - .02 < state[0] * sign < end
    # Once full braking starts, velocity decreases at a bounded rate.
    fall = np.diff(history)[np.argmax(history):]
    assert fall.min() >= -nav.model.parameters.linear_deceleration * .1 - 1e-8


def test_speed_ceiling_is_near_largest_braking_safe_command_and_never_exceeds_demand():
    nav, obstacles = world([[.175, 0]])
    result = nav.step(np.zeros(3), np.zeros(2), None, obstacles, BOUNDS, manual=VelocityCommand(1, 0))
    assert result.safe and 0 < result.command.linear_x < .055
    candidates = np.c_[np.linspace(0, .055, 1001), np.zeros(1001)]
    _, _, clear = nav.local.evaluate(np.zeros(3), np.zeros(2), candidates, obstacles, BOUNDS, .1)
    maximum = candidates[np.flatnonzero(clear > 0)[-1], 0]
    assert maximum - .0003 <= result.command.linear_x <= maximum + .00006


def test_deceleration_is_available_when_full_acceleration_is_unsafe():
    nav, obstacles = world([[.38, 0]])
    measured = np.array([.6, 0])
    full = nav.model.advance_velocity(measured, [1., 0], .1)
    result = nav.step(np.zeros(3), measured, None, obstacles, BOUNDS, manual=VelocityCommand(1, 0))
    assert result.safe
    assert .6 - nav.model.parameters.linear_deceleration * .1 <= result.command.linear_x < full[0]
    assert obstacles.path_safe(result.trajectory, BODY, BOUNDS)


def test_governor_does_not_invent_a_turn_or_erase_current_collision():
    nav, obstacles = world([[.12, 0]])
    result = nav.step(np.zeros(3), np.zeros(2), None, obstacles, BOUNDS, manual=VelocityCommand(1, 0))
    assert not result.safe and result.command.angular_z == 0
    for sign in (-1, 1):
        nav, obstacles = world()
        demand = VelocityCommand(sign * .1, sign * .2)
        result = nav.step(np.zeros(3), np.zeros(2), None, obstacles, BOUNDS, manual=demand)
        expected = nav.model.step(VelocityCommand(), demand, .1)
        assert result.safe and result.command == expected


def test_pyrobo_detour_waypoints_reach_pose_goal_with_dwa():
    wall = np.c_[np.zeros(61), np.linspace(-1.2, 1.2, 61)]
    nav, obstacles = world(wall)
    state, speed, goal = np.array([-2., 0, 0]), np.zeros(2), np.array([2., 0, 0])
    max_y = 0.
    for tick in range(800):
        result = nav.step(state, speed, goal, obstacles, BOUNDS)
        assert result.safe, (tick, result.status, state)
        assert obstacles.path_safe(result.trajectory, BODY, BOUNDS)
        state, speed = move(nav, state, speed, result.command)
        max_y = max(max_y, abs(state[1]))
        if result.status == 'arrived': break
    assert result.status == 'arrived', (state, speed, nav.planning_info())
    assert np.linalg.norm(state[:2] - goal[:2]) < .18 and abs(wrap(state[2])) < .2
    assert max_y > 1.5
    assert nav.plan_diagnostics['api'] == 'pyrobo.planning.RRTConnect.plan'
    assert len(nav.waypoints) > 2 and nav.plan_diagnostics['status'] == 'success'


def test_unreachable_goal_approaches_barrier_waits_then_resumes_when_it_is_removed():
    wall = np.c_[np.ones(321), np.linspace(-8, 8, 321)]
    nav, obstacles = world(wall)
    state, speed, goal = np.zeros(3), np.zeros(2), np.array([2., 0, 0])
    for _ in range(130):
        result = nav.step(state, speed, goal, obstacles, BOUNDS)
        assert result.safe, (state, result.status)
        assert result.status != 'arrived'
        state, speed = move(nav, state, speed, result.command)
    assert .70 < state[0] < .84 and np.max(abs(speed)) < .01
    assert result.status == 'waiting_obstacle' and nav.partial and nav.needs_replan
    assert nav.goal.tolist() == goal.tolist()
    empty = nav.obstacle_index(np.empty((0, 2)), 'map', .035)
    nav.last_plan = -1e20  # A fresh obstacle snapshot makes the next replan due.
    for _ in range(180):
        result = nav.step(state, speed, goal, empty, BOUNDS)
        assert result.safe
        state, speed = move(nav, state, speed, result.command)
        if result.status == 'arrived': break
    assert result.status == 'arrived' and not nav.partial


def test_new_obstacle_keeps_safe_part_of_existing_route():
    nav, empty = world()
    goal = np.array([3., 0, 0])
    nav.step(np.zeros(3), np.zeros(2), goal, empty, BOUNDS)
    wall = nav.obstacle_index(np.c_[np.ones(121), np.linspace(-3, 3, 121)], 'map', .035)
    result = nav.step(np.zeros(3), np.zeros(2), goal, wall, BOUNDS, allow_planning=False)
    assert result.safe and result.command.linear_x > 0
    assert result.status == 'approaching_blockage' and result.planning['needs_replan']
    assert len(result.path) > 1 and result.path[-1, 0] < 1.


@pytest.mark.parametrize('point,kept', [((-.6, 0), False), ((-.45, .26), False),
    ((-.811, 0), True), ((-.6, .271), True), ((-.39, 0), True), ((.1, 0), True)])
def test_tray_filter_uses_only_physical_rear_region_at_observation_time(point, kept):
    point = np.array(point)
    ranges = [np.linalg.norm(point)] + [float('inf')] * 7
    scan = dict(ranges=ranges, range_min=.01, range_max=5.,
                angle_min=np.arctan2(point[1], point[0]), angle_increment=.1)
    identity = dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1])
    transform = dict(translation=[2, 3, 0], quaternion=[0, 0, math.sin(.3), math.cos(.3)])
    original = laser_points(scan, transform, body_transform=identity)
    filtered = laser_points(scan, transform, body_transform=identity,
                            tray_filter=dict(inner=.4, outer=.81, width=.54))
    assert len(original) == 1 and len(filtered) == int(kept)
    if kept: np.testing.assert_allclose(filtered, original)


def test_tray_mask_follows_fresh_feedback_and_cannot_delete_static_map_obstacles():
    import asyncio
    import time
    from types import SimpleNamespace
    from rsim.apps.navigation.service import NavigationService
    from rsim.apps.navigation.simulation import SimulationBridge
    from rsim.components.towing_geometry import ObstacleUnion

    async def run():
        service = NavigationService(SimulationBridge(), dict(footprint=dict(front=.1, rear=.4, width=.54, margin=.03),
            ros={'pedal_feedback': {'timeout': 1.5}},
            pedal_shapes={'retracted': {'front': .1, 'rear': .4}, 'extended': {'front': .27, 'rear': .81}}))
        async def stop(*args, **kwargs): pass
        service.stop = stop
        hardware = {}
        service.latest = lambda *args: SimpleNamespace(data={'pedal': hardware})
        def feedback(position, **extra):
            hardware.clear();hardware.update(feedback=True, feedback_ns=time.monotonic_ns(), position=position, **extra)
        assert service.tray_filter() is None
        feedback('extended');await service.refresh_pedal()
        assert service.tray_filter() == dict(inner=.4, outer=.81, width=.54)
        await service.pedal_action('stop')
        feedback('stopped');await service.refresh_pedal()
        assert service.tray_filter() is not None
        # Only dynamic returns are masked: a mapped obstacle in the same place
        # remains present in the collision index and blocks motion.
        nav = service.navigator
        static = nav.obstacle_index(np.array([[-.6, 0]]), 'map', .035)
        dynamic = nav.obstacle_index(np.empty((0, 2)), 'map', .035)
        assert ObstacleUnion(static, dynamic).clearances(np.zeros((1, 3)), service.footprint, BOUNDS)[0] < 0
        hardware['feedback_ns'] -= 2_000_000_000
        await service.refresh_pedal();assert service.tray_filter() is None
        feedback('retracted', retracted=True);await service.refresh_pedal()
        assert service.tray_filter() is None and service.footprint.rear == .4
    asyncio.run(run())


def test_pyrobo_can_leave_a_safe_asymmetric_pose_near_dock():
    nav, obstacles = world([[-.35, .39]])
    state, goal = np.zeros(3), np.array([1., 0, 0])
    assert obstacles.clearances(state[None], BODY, BOUNDS)[0] > 0
    assert not obstacles.segment_free(state[:2], state[:2], nav.planner.radius, BOUNDS)
    path = nav.plan(state, goal, obstacles, BOUNDS)
    assert len(path) > 2 and not nav.partial and obstacles.path_safe(path, BODY, BOUNDS)
    assert nav.plan_diagnostics['status'] == 'success'
    assert nav.plan_diagnostics['footprint_query'] == 'oriented_body_edges'


def test_braking_envelope_includes_command_in_flight_before_new_target():
    nav, obstacles = world([[.75, 0]])
    state, measured = np.zeros(3), np.array([.6, 0])
    immediate = nav.step(state, measured, None, obstacles, BOUNDS, manual=VelocityCommand(1, 0))
    nav.local.command_context(.3, [.6, 0])
    guarded = nav.step(state, measured, None, obstacles, BOUNDS, manual=VelocityCommand(1, 0))
    assert guarded.safe and guarded.command.linear_x < immediate.command.linear_x
    assert obstacles.path_safe(guarded.trajectory, BODY, BOUNDS)
    # Execute the old target for the modeled delivery delay, then the selected
    # command for one cycle and maximum braking thereafter.
    for target, steps in [(VelocityCommand(.6, 0), 3), (guarded.command, 4), (VelocityCommand(), 15)]:
        for _ in range(steps):
            state, measured = move(nav, state, measured, target)
            assert obstacles.clearances(state[None], BODY, BOUNDS)[0] > 0
    assert np.max(abs(measured)) < .001
