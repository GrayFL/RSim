"""Behavioral equivalence of the fast manual solver and cached collision index."""
from dataclasses import asdict
import time
import numpy as np
import pytest
from rsim.components.vehicle import TowingDynamics
from rsim.components.towing_geometry import BodyFootprint, ObstacleUnion
from rsim.components.towing_navigation import TowingNavigator
from rsim.components.navigation_map import OccupancyMap
from rsim.apps.navigation.planning import PlanningWorker


@pytest.mark.parametrize('dt', [.001, .02, .1, .27])
def test_scalar_physics_matches_batched_candidate_dynamics(dt):
    rng = np.random.default_rng(175)
    current = np.r_[rng.uniform(-2, 2, 120), 0, 1e-11, -1e-11, .001, -.001]
    target = np.r_[rng.uniform(-2, 2, 120), 0, -1, 1, -1, 1]
    for friction, drag in [(0, 0), (.08, .7), (.4, 1.2)]:
        batch = TowingDynamics._axis(current, target, dt, .6, friction, drag, 1.2)
        single = [TowingDynamics._axis(a, b, dt, .6, friction, drag, 1.2).item() for a, b in zip(current, target)]
        np.testing.assert_allclose(single, batch, atol=1e-12, rtol=1e-12)


def map_message():
    grid = np.zeros((100, 100), np.int8)
    grid[0:10, :] = -1; grid[35:40, 35:40] = 100
    return dict(header={'frame_id': 'map'}, info=dict(width=100, height=100, resolution=.1,
        origin=dict(position=dict(x=-5, y=-5), orientation=dict(x=0, y=0, z=0, w=1))), data=grid.ravel())


@pytest.mark.parametrize('rear', [.27, .81])
def test_static_dynamic_union_matches_full_cloud_including_unknown_and_bounds(rear):
    occupancy = OccupancyMap(map_message()); nav = TowingNavigator(footprint=BodyFootprint(rear=rear))
    rng = np.random.default_rng(13); dynamic = rng.uniform(-4, 4, (100, 2))
    args = (occupancy.frame, occupancy.point_radius)
    combined = nav.obstacle_index(np.concatenate((occupancy.obstacles, dynamic)), *args, occupancy)
    union = ObstacleUnion(nav.obstacle_index(occupancy.obstacles, *args, occupancy), nav.obstacle_index(dynamic, *args))
    states = rng.uniform(-5, 5, (1000, 3))
    np.testing.assert_allclose(union.clearances(states, nav.footprint, occupancy.bounds),
                               combined.clearances(states, nav.footprint, occupancy.bounds), atol=1e-12)
    for path in np.split(states, 100):
        assert union.path_safe(path, nav.footprint, occupancy.bounds) == combined.path_safe(path, nav.footprint, occupancy.bounds)


def test_worker_cached_map_exact_output_and_missing_cache_recovery():
    message = map_message(); occupancy = OccupancyMap(message); nav = TowingNavigator(drive_mode='differential')
    job = dict(id=1, generation=0, map_version=occupancy.version, map=message,
        submitted_ns=time.monotonic_ns(), start_deadline_ns=time.monotonic_ns()+10**9,
        vehicle=asdict(nav.model.parameters), footprint=asdict(nav.footprint), planner='rrt',
        drive_mode='differential', cruise_limits=(1, .8), dynamic=np.empty((0, 2)),
        state=[0, 0, 0], measured=[0, 0], goal=None, manual=[0, .4], path=[])
    worker = PlanningWorker('local'); first = worker.compute_job(job)
    cached = worker.compute_job(dict(job, id=2, map=None))
    assert first['ok'] and cached['ok']
    assert first['command'] == cached['command'] and first['safe'] == cached['safe']
    np.testing.assert_array_equal(first['trajectory'], cached['trajectory'])
    missing = PlanningWorker('local').compute_job(dict(job, map=None))
    assert missing['error'] == 'planner_map_missing' and missing['cached_map_version'] is None
