"""Regression tests for human-towing geometry, physics and navigation."""
import math
import numpy as np
import pytest
from rsim.components.vehicle import TowingDynamics, TowingParameters
from rsim.components.towing_geometry import BodyFootprint, advance, forward_connectors, wrap


def test_forward_connectors_endpoints_and_turn_direction():
    rng = np.random.default_rng(4)
    for _ in range(40):
        start, goal = rng.uniform(-2, 2, (2, 3))
        paths = forward_connectors(start, goal, .4)
        assert paths
        for path in paths:
            assert np.linalg.norm(path[-1, :2] - goal[:2]) < 1e-6
            assert abs(wrap(path[-1, 2] - goal[2])) < 1e-6
            movement = np.diff(path[:, :2], axis=0)
            mid_yaw = (path[1:, 2] + path[:-1, 2]) / 2
            assert np.all(np.sum(movement * np.c_[np.cos(mid_yaw), np.sin(mid_yaw)], axis=1) >= -1e-10)


def test_shared_model_brakes_without_pivot_or_reverse():
    model = TowingDynamics()
    current = np.array([.15, .13])
    for _ in range(150):
        old = current.copy()
        current = model.advance_velocity(current, [0, 0], .02)
        assert abs(current[0] - old[0]) <= model.parameters.linear_deceleration * .02 + 1e-9
        assert abs(current[1] - old[1]) <= model.parameters.angular_deceleration * .02 + 1e-9
        assert current[0] >= abs(current[1]) * .3 - 1e-9
        assert current[0] >= 0
    assert np.allclose(current, 0)
    assert model.targets(-1, 0).linear_x == -model.parameters.max_linear


def obstacle_setup(body, points):
    pytest.importorskip('pyrobo')
    from pyrobo import Bounds2D
    from rsim.components.towing_navigation import TowingNavigator
    # Retracting the pedal changes collision geometry, not the location of the
    # person being towed. Both configurations retain the .81 m comfort bound.
    nav = TowingNavigator(TowingParameters(), body)
    return nav, nav.obstacle_index(np.asarray(points).reshape(-1, 2), 'map', .02), Bounds2D(-6, 6, -6, 6)


def test_pedal_is_behind_rotation_center():
    short, long = BodyFootprint(rear=.27), BodyFootprint(rear=.81)
    _, obstacles, bounds = obstacle_setup(long, [[-.7, 0]])
    states = np.array([[0., 0., 0.]])
    assert obstacles.clearances(states, short, bounds)[0] > 0
    assert obstacles.clearances(states, long, bounds)[0] < 0
    assert long.corners(states[0])[:, 0].min() == -.81
    assert long.corners(states[0])[:, 0].max() == .27


@pytest.mark.parametrize('rear', [.27, .81])
def test_open_space_180_turn_reaches_goal_without_pivot(rear):
    nav, obstacles, bounds = obstacle_setup(BodyFootprint(rear=rear), [])
    state, measured = np.zeros(3), np.zeros(2)
    goal = np.array([-1.5, 0., math.pi])
    for tick in range(1500):
        decision = nav.step(state, measured, goal, obstacles, bounds)
        assert decision.safe, (tick, decision.status)
        cmd = np.array([decision.command.linear_x, decision.command.angular_z])
        assert cmd[0] >= abs(cmd[1]) * .3 - 1e-8
        assert abs(cmd[1]) * rear <= .12 + 1e-8
        assert obstacles.path_safe(decision.trajectory, nav.footprint, bounds)
        new = nav.model.advance_velocity(measured, cmd, .1)
        mid = (measured + new) / 2
        state = advance(state, mid[0], mid[1], .1)
        measured = new
        if decision.status == 'arrived': break
    assert decision.status == 'arrived', (state, measured, decision.status)


def test_blocked_turn_does_not_fall_back_to_pivot():
    wall = np.array([[x, y] for x in np.arange(-1, 1.01, .04) for y in [-.4, .4]])
    nav, obstacles, bounds = obstacle_setup(BodyFootprint(rear=.27), wall)
    result = nav.step(np.zeros(3), np.zeros(2), np.array([-.6, 0, math.pi]), obstacles, bounds)
    assert not result.safe
    assert result.command.linear_x == result.command.angular_z == 0


def test_unknown_map_cells_are_not_free_space():
    pytest.importorskip('pyrobo')
    from rsim.components.navigation_map import OccupancyMap
    grid = np.zeros((40, 40), np.int8)
    grid[10:30, 20:] = -1
    message = dict(header={'frame_id': 'map'}, info=dict(width=40, height=40, resolution=.1,
        origin=dict(position=dict(x=-2, y=-2), orientation=dict(x=0, y=0, z=0, w=1))), data=grid.ravel())
    occupancy = OccupancyMap(message)
    assert not occupancy.known_free([1, 0])
    assert not occupancy.known_free([-5, 0])
    assert occupancy.known_free([-1, 0])
    nav, _, bounds = obstacle_setup(BodyFootprint(rear=.27), [])
    obstacles = nav.obstacle_index(occupancy.obstacles, 'map', occupancy.point_radius, occupancy)
    assert obstacles.clearances(np.array([[1., 0., 0.]]), nav.footprint, bounds)[0] < 0


def test_global_detour_and_forward_final_approach():
    wall = np.c_[np.zeros(61), np.linspace(-1.2, 1.2, 61)]
    nav, obstacles, bounds = obstacle_setup(BodyFootprint(), wall)
    state, measured, travelled = np.array([-2., 0., 0.]), np.zeros(2), 0.
    goal = np.array([2., 0., 0.])
    for tick in range(1800):
        decision = nav.step(state, measured, goal, obstacles, bounds)
        assert decision.safe, (tick, decision.status)
        assert obstacles.path_safe(decision.trajectory, nav.footprint, bounds)
        cmd = np.array([decision.command.linear_x, decision.command.angular_z])
        new = nav.model.advance_velocity(measured, cmd, .1)
        mid = (measured + new) / 2
        updated = advance(state, mid[0], mid[1], .1)
        travelled += np.linalg.norm(updated[:2] - state[:2])
        state, measured = updated, new
        if decision.status == 'arrived': break
    assert decision.status == 'arrived', (state, decision.status)
    assert travelled < 12  # Avoid repeated endpoint loops after a simple detour.
