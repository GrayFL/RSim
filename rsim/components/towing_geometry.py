"""Forward circle/line connectors and asymmetric collision geometry.

Circle tangencies construct CSC and CCC Dubins families geometrically. No
planner claims completeness in clutter. Every resulting swept body is checked.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class BodyFootprint:
    front: float = .27
    rear: float = .81
    width: float = .54
    margin: float = .10

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in (self.front, self.rear, self.width)):
            raise ValueError('body dimensions must be positive')
        if not math.isfinite(self.margin) or self.margin < 0: raise ValueError('invalid body margin')

    @property
    def length(self): return self.front + self.rear

    @property
    def offset(self): return (self.front - self.rear) / 2

    @property
    def radius(self): return math.hypot(max(self.front, self.rear), self.width / 2) + self.margin

    def corners(self, state):
        x, y, yaw = state
        xy = np.array([[self.front, self.width / 2], [-self.rear, self.width / 2],
                       [-self.rear, -self.width / 2], [self.front, -self.width / 2]])
        c, s = math.cos(yaw), math.sin(yaw)
        return xy @ np.array([[c, s], [-s, c]]) + [x, y]


def advance(states, speed, yaw_rate, dt):
    """Exact unicycle arc (x forward, y left, yaw counterclockwise)."""
    result = np.array(states, dtype=float, copy=True)
    angle = np.asarray(yaw_rate) * dt
    distance = np.asarray(speed) * dt * np.sinc(angle / (2 * np.pi))
    result[..., 0] += distance * np.cos(result[..., 2] + angle / 2)
    result[..., 1] += distance * np.sin(result[..., 2] + angle / 2)
    result[..., 2] += angle
    return result


def wrap(value): return (value + np.pi) % (2 * np.pi) - np.pi


def forward_connectors(start, goal, radius, spacing=.03):
    """Return all forward CSC/CCC paths ordered by arc length.

    Endpoint headings are constrained. The straight tangent has a positive
    projection, and every circle arc follows its assigned turn direction.
    """
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    candidates = []
    if radius <= 0 or spacing <= 0: raise ValueError('positive radius and spacing required')
    def center(pose, sign):
        return pose[:2] + sign * radius * np.array([-np.sin(pose[2]), np.cos(pose[2])])
    def arc(a, b, sign):
        value = (sign * (b - a)) % (2 * np.pi)
        return 0. if value > 2 * np.pi - 1e-9 else value * radius
    def append(parts):
        state, states = start.copy(), [start.copy()]
        for distance, sign in parts:
            count = max(1, int(np.ceil(distance / spacing)))
            for _ in range(count):
                state = advance(state, 1., sign / radius, distance / count)
                states.append(state.copy())
        if np.linalg.norm(state[:2] - goal[:2]) < 1e-6 and abs(wrap(state[2] - goal[2])) < 1e-6:
            candidates.append((sum(p[0] for p in parts), np.asarray(states)))
    for first in (-1, 1):
        c1 = center(start, first)
        for last in (-1, 1):
            c3 = center(goal, last)
            delta, d = c3 - c1, np.linalg.norm(c3 - c1)
            if d < 1e-9: continue
            ratio = (last - first) * radius / d
            if abs(ratio) > 1: continue
            heading = np.arctan2(delta[1], delta[0]) - np.arcsin(ratio)
            length = float(delta @ np.array([np.cos(heading), np.sin(heading)]))
            append([(arc(start[2], heading, first), first), (max(0., length), 0),
                    (arc(heading, goal[2], last), last)])
        c3 = center(goal, first)
        delta, d = c3 - c1, np.linalg.norm(c3 - c1)
        if 1e-9 < d <= 4 * radius:
            middle = (c1 + c3) / 2
            normal = np.array([-delta[1], delta[0]]) / d
            for side in (-1, 1):
                c2 = middle + side * np.sqrt(max(0., 4 * radius ** 2 - d ** 2 / 4)) * normal
                p12, p23 = (c1 + c2) / 2, (c2 + c3) / 2
                h1 = np.arctan2(*(p12 - c1)[::-1]) + first * np.pi / 2
                h2 = np.arctan2(*(p23 - c2)[::-1]) - first * np.pi / 2
                append([(arc(start[2], h1, first), first), (arc(h1, h2, -first), -first),
                        (arc(h2, goal[2], first), first)])
    if np.linalg.norm(start[:2] - goal[:2]) < 1e-9 and abs(wrap(start[2] - goal[2])) < 1e-9:
        candidates.append((0., start[None].copy()))
    return [states for _, states in sorted(candidates, key=lambda p: p[0])]


class BodyObstacles:
    """Adapt pyrobo's centered rectangle queries to a rear-extended footprint."""
    def __init__(self, point_obstacles, occupancy=None):
        self.points, self.occupancy = point_obstacles, occupancy
        self.cloud, self.xy, self.tree = point_obstacles.cloud, point_obstacles.xy, point_obstacles.tree
        self.point_radius = point_obstacles.point_radius

    def segment_free(self, a, b, radius, bounds):
        if self.occupancy is not None:
            count = max(2, int(np.linalg.norm(np.asarray(a) - b) / self.occupancy.resolution) + 2)
            if not self.occupancy.known_free(np.linspace(a, b, count)).all(): return False
        return self.points.segment_free(a, b, radius, bounds)

    def clearances(self, states, footprint, bounds):
        shifted = np.array(states, copy=True)
        shifted[..., 0] += footprint.offset * np.cos(shifted[..., 2])
        shifted[..., 1] += footprint.offset * np.sin(shifted[..., 2])
        result = self.points.clearances(shifted, footprint, bounds)
        if self.occupancy is not None:
            result = np.where(self.occupancy.known_free(shifted[..., :2]), result, -np.inf)
        return result

    def path_safe(self, states, footprint, bounds):
        if len(states) == 0: return False
        clearance = self.clearances(states, footprint, bounds)
        if len(states) == 1: return bool(clearance[0] > 0)
        travel = np.linalg.norm(np.diff(states[:, :2], axis=0), axis=1)
        rotation = np.abs(wrap(np.diff(states[:, 2]))) * footprint.radius
        return bool(np.all(np.minimum(clearance[:-1], clearance[1:]) > (travel + rotation) / 2))


class ObstacleUnion(BodyObstacles):
    """Exact union for local collision queries, retaining the static map index.

    Each part checks the full body and bounds. Taking the minimum preserves
    every static/dynamic obstacle and unknown-map check; no distance culling.
    Global planners that inspect a point tree still use a single full index.
    """
    def __init__(self, *parts):
        self.parts = parts

    def clearances(self, states, footprint, bounds):
        return np.minimum.reduce([part.clearances(states, footprint, bounds) for part in self.parts])

    def segment_free(self, a, b, radius, bounds):
        return all(part.segment_free(a, b, radius, bounds) for part in self.parts)


class BodyRouteObstacles:
    """Exact straight-body queries for a tight global start or goal.

    A circumscribed disk can reject a valid asymmetric robot pose near a dock.
    Pyrobo's XY graph has no headings, so each edge may be traversed in either
    orientation. The realized route separately checks every turn and segment
    before any of its waypoints are accepted.
    """
    def __init__(self, obstacles, footprint, start, goal):
        self.obstacles, self.footprint = obstacles, footprint
        self.start, self.goal = np.asarray(start), np.asarray(goal)
        self.cloud, self.xy, self.tree = obstacles.cloud, obstacles.xy, obstacles.tree
        self.point_radius = obstacles.point_radius

    def segment_free(self, a, b, radius, bounds):
        a, b = np.asarray(a), np.asarray(b)
        length = np.linalg.norm(b - a)
        if length < 1e-9:
            pose = self.start if np.linalg.norm(a-self.start[:2]) < np.linalg.norm(a-self.goal[:2]) + 1e-9 else self.goal
            return bool(self.obstacles.clearances(np.r_[a, pose[2]][None], self.footprint, bounds)[0] > 0)
        heading = np.arctan2(b[1]-a[1], b[0]-a[0])
        path = np.column_stack((np.linspace(a, b, max(2, int(np.ceil(length/.04))+1)),
                                np.full(max(2, int(np.ceil(length/.04))+1), heading)))
        if self.obstacles.path_safe(path, self.footprint, bounds): return True
        path[:, 2] += np.pi
        return self.obstacles.path_safe(path, self.footprint, bounds)
