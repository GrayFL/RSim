"""pyrobo global planning and DWA using RSim's shared towing vehicle model."""
from __future__ import annotations
from dataclasses import dataclass
import time
import numpy as np
from pyrobo import DWAConfig, DynamicWindow, PointCloud, RRTConnectConfig, Twist2D
from pyrobo.planning import RRTConnect, VoronoiConfig, VoronoiPlanner
from pyrobo.pointcloud import PointObstacles
from pyrobo.types import pose2d, path_poses
from pyrobo.control.dwa.path import path_lookahead, polyline_distances

from ..core.commands import VelocityCommand
from .vehicle import drive_model as make_drive_model, TowingParameters
from .towing_geometry import BodyFootprint, BodyObstacles, advance, wrap, forward_connectors


@dataclass
class Decision:
    command: VelocityCommand
    status: str
    safe: bool
    path: np.ndarray
    trajectory: np.ndarray
    clearance: float = 0.


class TowingWindow(DynamicWindow):
    """Reuse pyrobo's DWA safety/clearance checks with towing rollouts/candidates.

    Every candidate includes its braking tail and a separate immediate-brake
    test. No unconstrained DWA command is clipped after collision checking.
    """
    def __init__(self, model, footprint):
        p = model.parameters
        super().__init__(DWAConfig(max_speed=p.max_linear, max_yaw_rate=p.max_angular,
            linear_acceleration=p.linear_acceleration, angular_acceleration=p.angular_acceleration,
            prediction_time=2., integration_dt=.1, linear_samples=5, angular_samples=9,
            lookahead=.25, goal_tolerance=.18, yaw_tolerance=.2), footprint)
        self.model = model
        self.cruise_limits = (p.max_linear, p.max_angular)

    def _rollout(self, state, velocity, commands, dt, *, stop_only=False):
        states = np.broadcast_to(state, (len(commands), 3)).copy()
        speeds = np.tile([velocity.linear, velocity.angular], (len(commands), 1))
        history, sweeps = [states.copy()], [np.zeros(len(commands))]
        def phase(duration, targets):
            nonlocal states, speeds
            count = max(1, int(np.ceil(duration / self.config.integration_dt)))
            if duration <= 0: return
            h = duration / count
            for _ in range(count):
                old = speeds
                speeds = self.model.advance_velocity(old, targets, h)
                mid = (old + speeds) / 2
                states = advance(states, mid[:, 0], mid[:, 1], h)
                sweep = (np.maximum(abs(old[:, 0]), abs(speeds[:, 0])) +
                         self.footprint.radius * np.maximum(abs(old[:, 1]), abs(speeds[:, 1]))) * h / 2
                history.append(states.copy())
                sweeps.append(sweep)
        phase(dt, commands)
        if not stop_only: phase(self.config.prediction_time - dt, commands)
        endpoint = len(history) - 1
        p = self.model.parameters
        # Coupled braking can maintain v while yaw decelerates; sum bounds both.
        duration = np.max(abs(speeds[:, 0])) / p.linear_deceleration + np.max(abs(speeds[:, 1])) / p.angular_deceleration + .1
        phase(duration, np.zeros_like(commands))
        return np.stack(history, axis=1), np.stack(sweeps, axis=1), endpoint

    def evaluate(self, state, measured, targets, obstacles, bounds, dt):
        velocity = Twist2D(*measured)
        trajectories, sweep, endpoint = self._rollout(state, velocity, targets, dt)
        clearance = self._clearance(trajectories, sweep, obstacles, bounds)
        stop, stop_sweep, _ = self._rollout(state, velocity, targets, dt, stop_only=True)
        clearance = np.minimum(clearance, self._clearance(stop, stop_sweep, obstacles, bounds))
        return trajectories, endpoint, clearance

    def choose(self, state, measured, path, goal, obstacles, bounds, dt, manual=None):
        p = self.model.parameters
        if manual is not None:
            targets = np.array([[manual.linear_x, manual.angular_z]])
        else:
            cruise_v, cruise_w = self.cruise_limits
            lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
            local_path = path[:max(2, int(np.searchsorted(lengths, 1.)) + 1)]
            target, remaining = path_lookahead(local_path[:, :2], state[:2], max(self.config.lookahead, .6 * abs(measured[0])))
            delta = target - state[:2]
            heading = np.arctan2(delta[1], delta[0])
            error = float(wrap(heading - state[2]))
            distance = np.linalg.norm(state[:2] - goal[:2])
            desired_v = min(p.max_linear, cruise_v, np.sqrt(2 * p.linear_acceleration * max(0., lengths[-1] - .08)))
            desired_v *= max(.15, np.cos(error))
            curvature = 2 * np.sin(error) / max(.1, np.linalg.norm(delta))
            if self.model.mode == 'towing' and abs(curvature) > 1e-6:
                desired_v = min(desired_v, .85 * min(p.max_angular, p.rear_lateral_speed / p.rear_extent) / abs(curvature))
            desired_w = desired_v * curvature
            if self.model.mode == 'differential':
                desired_w = float(np.clip(1.5 * error, -p.max_angular, p.max_angular))
                if abs(error) > .5: desired_v = 0.
                if distance <= .13:
                    desired_v = 0.
                    desired_w = float(np.clip(1.5 * wrap(goal[2] - state[2]), -p.max_angular, p.max_angular))
            if distance <= .13 and abs(wrap(goal[2] - state[2])) <= self.config.yaw_tolerance:
                desired_v = desired_w = 0.
            vlow = max(0., measured[0] - p.linear_acceleration * dt)
            vhigh = min(p.max_linear, measured[0] + p.linear_acceleration * dt)
            wlow = max(-p.max_angular, measured[1] - p.angular_acceleration * dt)
            whigh = min(p.max_angular, measured[1] + p.angular_acceleration * dt)
            vs = np.unique(np.r_[np.linspace(vlow, max(vlow, vhigh), 5), desired_v, 0.])
            ws = np.unique(np.r_[np.linspace(wlow, max(wlow, whigh), 9), desired_w, 0.])
            v, w = np.meshgrid(vs, ws)
            targets = np.column_stack((v.ravel(), w.ravel()))
            targets[:, 0] = np.clip(targets[:, 0], -cruise_v, cruise_v)
            targets[:, 1] = np.clip(targets[:, 1], -cruise_w, cruise_w)
            desired_w = float(np.clip(desired_w, -cruise_w, cruise_w))
            # Constrain before predicting, including all ramp and braking phases.
            limit = self.model.yaw_limit(targets[:, 0])
            targets[:, 1] = np.clip(targets[:, 1], -limit, limit)
            targets = self.model.advance_velocity(np.tile(measured, (len(targets), 1)), targets, dt)
            targets = np.unique(targets.round(8), axis=0)
        traj, endpoint, clearance = self.evaluate(state, measured, targets, obstacles, bounds, dt)
        feasible = clearance > 0
        if feasible.any():
            if manual is not None:
                best = 0
            else:
                final = traj[:, endpoint]
                cross = polyline_distances(final[:, :2], local_path[:, :2])
                projected = self.model.advance_velocity(np.tile(measured, (len(targets), 1)), targets, dt)
                preferred = self.model.advance_velocity(np.asarray(measured), [desired_v, desired_w], dt)
                score = (np.linalg.norm(final[:, :2] - target, axis=1) + 3 * cross +
                         .2 * (1 - np.cos(final[:, 2] - heading)) +
                         2 * ((projected[:, 0] - preferred[0]) / (p.linear_acceleration * dt)) ** 2 +
                         2 * ((projected[:, 1] - preferred[1]) / (p.angular_acceleration * dt)) ** 2 +
                         .015 / (.05 + np.maximum(clearance, 0)))
                score[~feasible] = np.inf
                best = int(np.argmin(score))
            command = self.model.advance_velocity(measured, targets[best], dt)
            return VelocityCommand(*map(float, command)), traj[best], float(clearance[best]), True
        braking = np.zeros((1, 2))
        traj, _, clearance = self.evaluate(state, measured, braking, obstacles, bounds, dt)
        command = self.model.advance_velocity(measured, braking[0], dt)
        return VelocityCommand(*map(float, command)), traj[0], float(clearance[0]), False


class TowingNavigator:
    def __init__(self, parameters=None, footprint=None, planner='rrt', drive_mode='towing'):
        self.model = make_drive_model(parameters, drive_mode)
        self.footprint = footprint or BodyFootprint()
        self.local = TowingWindow(self.model, self.footprint)
        self.planner = (VoronoiPlanner(VoronoiConfig(time_budget=.15, clearance=.02), self.footprint)
                        if planner == 'voronoi' else RRTConnect(RRTConnectConfig(time_budget=.08, clearance=.02), self.footprint))
        self.reset()

    def reset(self):
        self.path = np.empty((0, 3))
        self.goal = None
        self.last_plan = -1e20
        self.plan_status = 'idle'
        self.local.reset()
        self.planner.reset()

    def obstacle_index(self, points, frame, point_radius, occupancy=None):
        return BodyObstacles(PointObstacles(PointCloud(points, frame=frame), point_radius=point_radius), occupancy)

    def connector(self, start, goal, obstacles, bounds):
        if self.model.mode == 'differential':
            # A differential base can rotate at either endpoint. Check the
            # entire swept, offset body, including the independent pedal shape.
            delta = goal[:2] - start[:2]
            distance = np.linalg.norm(delta)
            heading = np.arctan2(delta[1], delta[0]) if distance > 1e-8 else goal[2]
            first = float(wrap(heading - start[2]))
            last = float(wrap(goal[2] - heading))
            turn1 = np.tile(start, (max(2, int(abs(first) * self.footprint.radius / .03) + 2), 1))
            turn1[:, 2] = np.linspace(start[2], start[2] + first, len(turn1))
            line = np.linspace(np.r_[start[:2], heading], np.r_[goal[:2], heading], max(2, int(distance / .03) + 2))
            turn2 = np.tile(goal, (max(2, int(abs(last) * self.footprint.radius / .03) + 2), 1))
            turn2[:, 2] = np.linspace(heading, heading + last, len(turn2))
            route = np.concatenate((turn1, line, turn2))
            return route if obstacles.path_safe(route, self.footprint, bounds) else None
        minimum = self.model.parameters.min_turn_radius
        for radius in sorted({max(.4, minimum), max(.6, minimum), max(.9, minimum)}):
            for path in forward_connectors(start, goal, radius):
                if obstacles.path_safe(path, self.footprint, bounds): return path
        return None

    def plan(self, state, goal, obstacles, bounds):
        if self.model.mode == 'differential':
            direct = self.connector(state, goal, obstacles, bounds)
            if direct is not None:
                self.plan_status = 'differential_connector'
                return direct
            self.planner.reset()
            return self.global_route(state, goal, obstacles, bounds)
        # Reserve a straight final approach so yaw settles before position
        # braking. Ending on a tight arc otherwise invites repeated goal loops.
        approaches = []
        for length in (.8, .5, .3):
            approach = goal.copy()
            approach[:2] -= length * np.array([np.cos(goal[2]), np.sin(goal[2])])
            line = np.linspace(approach, goal, int(length / .03) + 2)
            if obstacles.path_safe(line, self.footprint, bounds):
                approaches.append((approach, line[1:]))
        approaches.append((goal, np.empty((0, 3))))
        for target, suffix in approaches:
            direct = self.connector(state, target, obstacles, bounds)
            if direct is not None:
                self.plan_status = 'forward_connector'
                return np.concatenate((direct, suffix))
        for target, suffix in approaches:
            # Reconnect alternative approaches when a particular XY roadmap
            # cannot be rounded into forward curves around its sharp corners.
            self.planner.reset()
            route = self.global_route(state, target, obstacles, bounds)
            if len(route): return np.concatenate((route, suffix))
        return np.empty((0, 3))

    def global_route(self, state, target, obstacles, bounds):
        frame = obstacles.cloud.frame
        result = self.planner.plan(pose2d(*state, frame=frame), pose2d(*target, frame=frame), obstacles, bounds)
        self.plan_status = result.status
        xy = np.asarray(result.path.position)[:, :2]
        if len(xy) < 2: return np.empty((0, 3))
        # The conservative global route supplies safe waypoints; join them with
        # forward curves, never command the polyline's sharp heading changes.
        points = [state.copy()]
        for i in range(1, len(xy) - 1):
            if np.linalg.norm(xy[i] - points[-1][:2]) < 1.: continue
            delta = xy[min(i + 3, len(xy) - 1)] - xy[max(0, i - 3)]
            points.append(np.r_[xy[i], np.arctan2(delta[1], delta[0])])
        points.append(target.copy())
        route, index = [], 0
        while index < len(points) - 1:
            joined = False
            for target in range(len(points) - 1, index, -1):
                path = self.connector(points[index], points[target], obstacles, bounds)
                if path is not None:
                    route.extend(path if not route else path[1:])
                    index, joined = target, True
                    break
            if not joined:
                self.plan_status = 'no_forward_connection'
                return np.empty((0, 3))
        return np.asarray(route)

    def step(self, state, measured, goal, obstacles, bounds, dt=.1, manual=None, *, allow_planning=True):
        # Shared arrays are immutable mmap views. Clamp only an owned copy.
        state, measured = np.asarray(state, float), np.array(measured, dtype=float, copy=True)
        p = self.model.parameters
        if not np.isfinite(state).all() or not np.isfinite(measured).all(): raise ValueError('invalid robot state')
        minimum = -.02 if self.model.mode == 'towing' else -p.max_linear - .03
        if measured[0] < minimum or measured[0] > p.max_linear + .03 or abs(measured[1]) > p.max_angular + .03:
            raise ValueError('measured velocity outside model limits')
        if self.model.mode == 'towing': measured[0] = max(0., measured[0])
        if manual is not None:
            cmd, traj, clear, safe = self.local.choose(state, measured, np.array([state]), state,
                                                       obstacles, bounds, dt, manual=manual)
            return Decision(cmd, 'manual' if safe else 'blocked', safe, np.empty((0, 3)), traj, clear)
        goal = np.asarray(goal, float)
        if self.goal is None or not np.allclose(goal, self.goal):
            self.reset()
            self.goal = goal.copy()
        arrived = np.linalg.norm(goal[:2] - state[:2]) <= .18 and abs(wrap(goal[2] - state[2])) <= .2
        if arrived:
            cmd, traj, clear, safe = self.local.choose(state, measured, np.array([state]), goal, obstacles, bounds, dt, manual=VelocityCommand())
            status = 'arrived' if np.linalg.norm(measured) < .01 else 'braking'
            return Decision(cmd, status if safe else 'blocked', safe, self.path.copy(), traj, clear)
        if len(self.path):
            distances = np.cumsum(np.r_[0., np.linalg.norm(np.diff(self.path[:, :2], axis=0), axis=1)])
            nearby = np.flatnonzero(distances < 1.2)
            nearest = nearby[np.argmin(np.linalg.norm(self.path[nearby, :2] - state[:2], axis=1))]
            self.path = self.path[nearest:]
            if not obstacles.path_safe(self.path, self.footprint, bounds): self.path = np.empty((0, 3))
            # Position tolerance alone cannot finish a heading-constrained goal.
            # If tracking ran out of forward path, plan a checked forward loop.
            remaining = np.linalg.norm(np.diff(self.path[:, :2], axis=0), axis=1).sum()
            if remaining < .1 and not arrived and (self.model.mode == 'towing' or np.linalg.norm(goal[:2] - state[:2]) > .13):
                self.path = np.empty((0, 3))
                self.last_plan = -1e20
        if allow_planning and not len(self.path) and time.monotonic() - self.last_plan > .5:
            self.path = self.plan(state, goal, obstacles, bounds)
            self.last_plan = time.monotonic()
        if not len(self.path):
            return Decision(VelocityCommand(), ('no_path:' + self.plan_status) if allow_planning else 'waiting_global_path',
                            False, self.path.copy(), state[None])
        cmd, traj, clear, safe = self.local.choose(state, measured, self.path, goal, obstacles, bounds, dt)
        return Decision(cmd, 'navigating' if safe else 'blocked', safe, self.path.copy(), traj, clear)
