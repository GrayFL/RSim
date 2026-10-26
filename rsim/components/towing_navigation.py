"""pyrobo global planning and DWA using RSim's shared towing vehicle model."""
from __future__ import annotations
from dataclasses import dataclass, field
import time
import numpy as np
from pyrobo import DWAConfig, DynamicWindow, PointCloud, RRTConnectConfig, Twist2D
from pyrobo.planning import RRTConnect, VoronoiConfig, VoronoiPlanner
from pyrobo.pointcloud import PointObstacles
from pyrobo.types import pose2d, path_poses
from pyrobo.control.dwa.path import path_lookahead, polyline_distances

from ..core.commands import VelocityCommand
from .vehicle import drive_model as make_drive_model, TowingParameters
from .towing_geometry import BodyFootprint, BodyObstacles, BodyRouteObstacles, advance, wrap, forward_connectors


@dataclass
class Decision:
    command: VelocityCommand
    status: str
    safe: bool
    path: np.ndarray
    trajectory: np.ndarray
    clearance: float = 0.
    planning: dict = field(default_factory=dict)


class TowingWindow(DynamicWindow):
    """Reuse pyrobo's DWA safety/clearance checks with towing rollouts/candidates.

    Admissibility checks queued motion, one control interval, then maximum braking.
    The longer DWA horizon scores progress only; it cannot veto a command
    whose actual braking envelope is clear. Commands are bounded before checks.
    """
    def __init__(self, model, footprint):
        p = model.parameters
        super().__init__(DWAConfig(max_speed=p.max_linear, max_yaw_rate=p.max_angular,
            linear_acceleration=p.linear_acceleration, angular_acceleration=p.angular_acceleration,
            prediction_time=2., integration_dt=.05, linear_samples=5, angular_samples=9,
            lookahead=.25, goal_tolerance=.18, yaw_tolerance=.2), footprint)
        self.model = model
        self.cruise_limits = (p.max_linear, p.max_angular)
        self.partial_path = False
        self.command_latency = 0.
        self.reaction_time = 0.
        self.previous_command = np.zeros(2)
        # Keep a small numerical reserve at the speed-search boundary. Without
        # it, sub-millimetre integration/observation differences cause repeated
        # stop/restart decisions when the maximum admissible speed tends to zero.
        self.stopping_reserve = .005

    def command_context(self, latency, previous, reaction=None):
        previous = np.asarray(previous, float)
        reaction = latency if reaction is None else reaction
        if (not np.isfinite(latency) or not 0 <= latency <= 2 or not np.isfinite(reaction) or not 0 <= reaction <= 2
                or previous.shape != (2,) or not np.isfinite(previous).all()):
            raise ValueError('invalid braking command context')
        self.command_latency, self.previous_command, self.reaction_time = float(latency), previous, float(reaction)

    def _rollout(self, state, velocity, commands, dt, *, stop_only=False, brake=True, latency=0.):
        states = np.broadcast_to(state, (len(commands), 3)).copy()
        speeds = np.tile([velocity.linear, velocity.angular], (len(commands), 1))
        history, sweeps = [states.copy()], [np.zeros(len(commands))]
        def phase(duration, targets):
            nonlocal states, speeds
            count = max(1, int(np.ceil(duration / self.config.integration_dt - 1e-10)))
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
        if latency: phase(latency, self.previous_command)
        phase(dt, commands)
        if not stop_only: phase(self.config.prediction_time - dt, commands)
        endpoint = len(history) - 1
        # The next braking command also needs time to be computed/delivered.
        # Checking only the latency before this command loses recursive braking
        # feasibility: a newly accepted speed would have no room for that delay.
        if stop_only and self.reaction_time: phase(self.reaction_time, commands)
        p = self.model.parameters
        # Coupled braking can maintain v while yaw decelerates; sum bounds both.
        duration = np.max(abs(speeds[:, 0])) / p.linear_deceleration + np.max(abs(speeds[:, 1])) / p.angular_deceleration + .1
        # A fixed grid keeps the next tick's braking tail consistent with the
        # tail already checked here, including the last interval at rest.
        duration = np.ceil(duration / self.config.integration_dt) * self.config.integration_dt
        if brake: phase(duration, np.zeros_like(commands))
        return np.stack(history, axis=1), np.stack(sweeps, axis=1), endpoint

    def evaluate(self, state, measured, commands, obstacles, bounds, dt):
        """Check exactly the published target, then its bounded braking tail."""
        trajectories, sweep, endpoint = self._rollout(state, Twist2D(*measured), commands, dt, stop_only=True)
        clearance = self._clearance(trajectories, sweep, obstacles, bounds)
        if self.command_latency and (np.any(abs(measured) > 1e-8) or np.any(abs(self.previous_command) > 1e-8)):
            # Until the new command reaches the chassis, the previous target
            # is still active. Check both immediate and delayed actuation;
            # otherwise IPC and sensor age consume the assumed stopping room.
            delayed, delayed_sweep, delayed_endpoint = self._rollout(state, Twist2D(*measured), commands, dt,
                stop_only=True, latency=self.command_latency)
            clearance = np.minimum(clearance, self._clearance(delayed, delayed_sweep, obstacles, bounds))
            trajectories, endpoint = delayed, delayed_endpoint
        return trajectories, endpoint, clearance - self.stopping_reserve

    def govern(self, state, measured, target, obstacles, bounds, dt):
        """Largest admissible demand along the operator's requested direction.

        A coarse batch searches the whole scale interval before refining its
        highest sampled safe band, rather than assuming a whole curved
        trajectory is monotonic. The refinement step is 1/4096 of demand. Every returned
        command has been checked; an unavoidable collision returns unsafe.
        """
        target = np.asarray(target, float)
        self.limited = False
        requested = self.model.advance_velocity(measured, target, dt)
        traj, _, clear = self.evaluate(state, measured, requested[None], obstacles, bounds, dt)
        if clear[0] > 0:
            return VelocityCommand(*map(float, requested)), traj[0], float(clear[0]), True
        self.limited = True
        scales = np.linspace(0., 1., 17)
        commands = self.model.advance_velocity(np.broadcast_to(measured, (len(scales), 2)),
                                               scales[:, None] * target, dt)
        trajectories, _, clearance = self.evaluate(state, measured, commands, obstacles, bounds, dt)
        safe = np.flatnonzero(clearance > 0)
        if not len(safe):
            return VelocityCommand(*map(float, commands[0])), trajectories[0], float(clearance[0]), False
        best = int(safe[-1])
        command, trajectory, gap = commands[best], trajectories[best], float(clearance[best])
        lo, hi = scales[best], scales[min(best + 1, len(scales)-1)]
        for _ in range(8):
            mid = (lo + hi) / 2
            candidate = self.model.advance_velocity(measured, mid * target, dt)
            future, _, distance = self.evaluate(state, measured, candidate[None], obstacles, bounds, dt)
            if distance[0] > 0:
                lo, command, trajectory, gap = mid, candidate, future[0], float(distance[0])
            else:
                hi = mid
        return VelocityCommand(*map(float, command)), trajectory, gap, True

    def choose(self, state, measured, path, goal, obstacles, bounds, dt, manual=None):
        p = self.model.parameters
        if manual is not None:
            return self.govern(state, measured, [manual.linear_x, manual.angular_z], obstacles, bounds, dt)
        else:
            cruise_v, cruise_w = self.cruise_limits
            lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
            local_path = path[:max(2, int(np.searchsorted(lengths, 1.)) + 1)]
            target, remaining = path_lookahead(local_path[:, :2], state[:2], max(self.config.lookahead, .6 * abs(measured[0])))
            delta = target - state[:2]
            heading = np.arctan2(delta[1], delta[0])
            error = float(wrap(heading - state[2]))
            distance = np.linalg.norm(state[:2] - goal[:2])
            desired_v = min(p.max_linear, cruise_v, np.sqrt(2 * p.linear_deceleration * max(0., lengths[-1] - (.01 if self.partial_path else .08))))
            desired_v *= max(.15, np.cos(error))
            curvature = 2 * np.sin(error) / max(.1, np.linalg.norm(delta))
            if self.model.mode == 'towing' and abs(curvature) > 1e-6:
                desired_v = min(desired_v, .85 * min(p.max_angular, p.rear_lateral_speed / p.rear_extent) / abs(curvature))
            desired_w = desired_v * curvature
            if self.model.mode == 'differential':
                desired_w = float(np.clip(1.5 * error, -p.max_angular, p.max_angular))
                if abs(error) > .5: desired_v = 0.
                if distance <= .13 and not self.partial_path:
                    desired_v = 0.
                    desired_w = float(np.clip(1.5 * wrap(goal[2] - state[2]), -p.max_angular, p.max_angular))
            if distance <= .13 and not self.partial_path and abs(wrap(goal[2] - state[2])) <= self.config.yaw_tolerance:
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
            governed, _, _, admissible = self.govern(state, measured, [desired_v, desired_w], obstacles, bounds, dt)
            if admissible:
                targets = np.vstack((targets, [governed.linear_x, governed.angular_z]))
            targets = np.unique(targets.round(10), axis=0)
        traj, endpoint, clearance = self.evaluate(state, measured, targets, obstacles, bounds, dt)
        feasible = clearance > 0
        if feasible.any():
            if manual is not None:
                best = 0
            else:
                # Candidates are already reachable in this control interval.
                # The scoring horizon then holds that speed; an exact arc
                # replaces dozens of identical dynamics integration steps.
                final = advance(traj[:, endpoint], targets[:, 0], targets[:, 1], self.config.prediction_time - dt)
                cross = polyline_distances(final[:, :2], local_path[:, :2])
                projected = targets
                preferred = self.model.advance_velocity(np.asarray(measured), [desired_v, desired_w], dt)
                score = (np.linalg.norm(final[:, :2] - target, axis=1) + 3 * cross +
                         .2 * (1 - np.cos(final[:, 2] - heading)) +
                         2 * ((projected[:, 0] - preferred[0]) / (p.linear_acceleration * dt)) ** 2 +
                         2 * ((projected[:, 1] - preferred[1]) / (p.angular_acceleration * dt)) ** 2 +
                         .015 / (.05 + np.maximum(clearance, 0)))
                score[~feasible] = np.inf
                best = int(np.argmin(score))
            command = targets[best]
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
        self.partial = False
        self.needs_replan = False
        self.waypoints = np.empty((0, 3))
        self.plan_diagnostics = {}
        self.stall_ticks = 0
        self.local.reset()
        self.planner.reset()

    def obstacle_index(self, points, frame, point_radius, occupancy=None):
        return BodyObstacles(PointObstacles(PointCloud(points, frame=frame), point_radius=point_radius), occupancy)

    def connector(self, start, goal, obstacles, bounds, *, check=True):
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
            return route if not check or obstacles.path_safe(route, self.footprint, bounds) else None
        minimum = self.model.parameters.min_turn_radius
        for radius in sorted({max(.4, minimum), max(.6, minimum), max(.9, minimum)}):
            for path in forward_connectors(start, goal, radius):
                if not check or obstacles.path_safe(path, self.footprint, bounds): return path
        return None

    def planning_info(self):
        return dict(planner=type(self.planner).__name__, status=self.plan_status,
                    partial=self.partial, needs_replan=self.needs_replan,
                    waypoints=self.waypoints.tolist(), diagnostics=self.plan_diagnostics)

    def safe_prefix(self, path, obstacles, bounds):
        """Retain every continuous safe interval up to the first obstruction."""
        if not len(path): return path
        clear = obstacles.clearances(path, self.footprint, bounds)
        if clear[0] <= 0: return path[:0]
        sweep = (np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) +
                 abs(wrap(np.diff(path[:, 2]))) * self.footprint.radius) / 2
        invalid = np.flatnonzero(np.minimum(clear[:-1], clear[1:]) <= sweep)
        return path[:int(invalid[0])+1] if len(invalid) else path

    def plan(self, state, goal, obstacles, bounds):
        """Ask pyrobo for a global route, then realize its waypoints with the body.

        An unreachable goal retains a checked approach prefix. It is explicitly
        partial, never an arrival, and is replanned while the local governor
        brings the chassis to a stop at the blocking obstacle.
        """
        self.partial = self.needs_replan = False
        self.waypoints = np.empty((0, 3))
        route = self.global_route(state, goal, obstacles, bounds)
        if len(route): return route
        failed = self.plan_status
        # A forward vehicle can need a straight final approach before settling
        # its heading. Each alternative is still planned through pyrobo.
        if self.model.mode == 'towing':
            for length in (.8, .5, .3):
                approach = goal.copy()
                approach[:2] -= length * np.array([np.cos(goal[2]), np.sin(goal[2])])
                suffix = np.linspace(approach, goal, int(length / .03) + 2)
                if not obstacles.path_safe(suffix, self.footprint, bounds): continue
                route = self.global_route(state, approach, obstacles, bounds)
                if len(route):
                    self.waypoints = np.vstack((self.waypoints, goal))
                    return np.concatenate((route, suffix[1:]))
        # The XY planner can reject a tight but exact-body-safe direct curve.
        # This checked local connection also handles a pure orientation goal.
        direct = self.connector(state, goal, obstacles, bounds)
        if direct is not None:
            self.plan_status = 'body_connection:' + failed
            self.waypoints = np.array([state, goal])
            return direct
        self.partial = self.needs_replan = True
        self.plan_status = 'partial:' + failed
        reference = self.connector(state, goal, obstacles, bounds, check=False)
        prefix = self.safe_prefix(reference, obstacles, bounds) if reference is not None else np.empty((0, 3))
        self.waypoints = prefix[[0, -1]] if len(prefix) else np.empty((0, 3))
        return prefix

    def global_route(self, state, target, obstacles, bounds):
        frame = obstacles.cloud.frame
        began = time.monotonic()
        result = self.planner.plan(pose2d(*state, frame=frame), pose2d(*target, frame=frame), obstacles, bounds)
        query = 'circumscribed_disk'
        if self.model.mode == 'differential' and result.status in ('invalid_start', 'invalid_goal'):
            exact = BodyRouteObstacles(obstacles, self.footprint, state, target)
            result = self.planner.plan(pose2d(*state, frame=frame), pose2d(*target, frame=frame), exact, bounds)
            query = 'oriented_body_edges'
        self.plan_status = result.status
        self.plan_diagnostics = dict(api='pyrobo.planning.' + type(self.planner).__name__ + '.plan',
            status=result.status, elapsed_ms=result.elapsed * 1000, extensions=result.extensions, footprint_query=query)
        xy = np.asarray(result.path.position)[:, :2]
        if len(xy) < 2: return np.empty((0, 3))
        # Retain all polyline corners. A fixed one-metre decimation used to
        # remove the very waypoints that routed around narrow obstacles.
        delta = np.diff(xy, axis=0)
        norm = np.linalg.norm(delta, axis=1)
        direction = delta / np.maximum(norm[:, None], 1e-12)
        corners = np.flatnonzero(np.linalg.norm(np.diff(direction, axis=0), axis=1) > 1e-5) + 1
        indices = sorted(set([0, *corners.tolist(), len(xy)-1]))
        points = [state.copy()]
        for i in indices[1:-1]:
            vector = direction[i-1] + direction[min(i, len(direction)-1)]
            points.append(np.r_[xy[i], np.arctan2(vector[1], vector[0])])
        points.append(target.copy())
        route, index, used = [], 0, [state.copy()]
        while index < len(points) - 1:
            joined = False
            for end in range(len(points) - 1, index, -1):
                path = self.connector(points[index], points[end], obstacles, bounds)
                if path is not None:
                    route.extend(path if not route else path[1:])
                    used.append(points[end])
                    index, joined = end, True
                    break
            if not joined:
                self.plan_status = 'no_body_connection'
                return np.empty((0, 3))
        self.waypoints = np.asarray(used)
        self.plan_diagnostics.update(waypoints=len(used), total_ms=(time.monotonic()-began)*1000)
        return np.asarray(route)

    def step(self, state, measured, goal, obstacles, bounds, dt=.1, manual=None, *, allow_planning=True):
        # Shared arrays are immutable mmap views. Clamp only an owned copy.
        state, measured = np.asarray(state, float), np.array(measured, dtype=float, copy=True)
        p = self.model.parameters
        if not np.isfinite(state).all() or not np.isfinite(measured).all(): raise ValueError('invalid robot state')
        minimum = -.02 if self.model.mode == 'towing' and manual is None else -p.max_linear - .03
        if measured[0] < minimum or measured[0] > p.max_linear + .03 or abs(measured[1]) > p.max_angular + .03:
            raise ValueError('measured velocity outside model limits')
        if self.model.mode == 'towing' and manual is None: measured[0] = max(0., measured[0])
        if manual is not None:
            cmd, traj, clear, safe = self.local.choose(state, measured, np.array([state]), state,
                                                       obstacles, bounds, dt, manual=manual)
            status = 'manual_limited' if self.local.limited else 'manual'
            return Decision(cmd, status if safe else 'blocked', safe, np.empty((0, 3)), traj, clear)
        goal = np.asarray(goal, float)
        if self.goal is None or not np.allclose(goal, self.goal):
            self.reset()
            self.goal = goal.copy()
        arrived = np.linalg.norm(goal[:2] - state[:2]) <= .18 and abs(wrap(goal[2] - state[2])) <= .2
        if arrived and not self.partial:
            cmd, traj, clear, safe = self.local.choose(state, measured, np.array([state]), goal, obstacles, bounds, dt, manual=VelocityCommand())
            status = 'arrived' if np.linalg.norm(measured) < .01 else 'braking'
            return Decision(cmd, status if safe else 'blocked', safe, self.path.copy(), traj, clear)
        if len(self.path):
            distances = np.cumsum(np.r_[0., np.linalg.norm(np.diff(self.path[:, :2], axis=0), axis=1)])
            nearby = np.flatnonzero(distances < 1.2)
            nearest = nearby[np.argmin(np.linalg.norm(self.path[nearby, :2] - state[:2], axis=1))]
            self.path = self.path[nearest:]
            # Check only the approaching part for replan triggering. The local
            # braking envelope still checks every obstacle and stopping pose.
            lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.path[:, :2], axis=0), axis=1))]
            horizon = max(2., abs(measured[0]) ** 2 / (2*p.linear_deceleration) + .5)
            count = min(len(self.path), max(2, int(np.searchsorted(lengths, horizon)) + 1))
            prefix = self.safe_prefix(self.path[:count], obstacles, bounds)
            if len(prefix) < count:
                self.path = prefix
                self.partial = self.needs_replan = True
                self.plan_status = 'local_path_obstructed'
            remaining = np.linalg.norm(np.diff(self.path[:, :2], axis=0), axis=1).sum()
            if (not self.partial and remaining < .1 and not arrived and
                    (self.model.mode == 'towing' or np.linalg.norm(goal[:2] - state[:2]) > .13)):
                self.path = np.empty((0, 3))
                self.needs_replan = True
                self.last_plan = -1e20
        if allow_planning and (not len(self.path) or self.needs_replan) and time.monotonic() - self.last_plan > .5:
            old_path = self.path.copy()
            self.path = self.plan(state, goal, obstacles, bounds)
            if self.partial and len(old_path) > 1:
                # Continue the existing approach while pyrobo searches again.
                self.path = old_path
            self.last_plan = time.monotonic()
        if not len(self.path) or (self.partial and np.linalg.norm(np.diff(self.path[:, :2], axis=0), axis=1).sum() < .015):
            cmd, traj, clear, safe = self.local.govern(state, measured, [0., 0.], obstacles, bounds, dt)
            status = 'waiting_obstacle' if self.partial else 'waiting_global_path'
            return Decision(cmd, status if safe else 'blocked', safe, self.path.copy(), traj, clear, self.planning_info())
        self.local.partial_path = self.partial
        cmd, traj, clear, safe = self.local.choose(state, measured, self.path, goal, obstacles, bounds, dt)
        moving = abs(cmd.linear_x) + abs(cmd.angular_z) > .001
        self.stall_ticks = 0 if moving else self.stall_ticks + 1
        if self.stall_ticks * dt >= .5: self.needs_replan = True
        status = ('approaching_blockage' if moving else 'waiting_obstacle') if self.partial else 'navigating'
        return Decision(cmd, status if safe else 'blocked', safe, self.path.copy(), traj, clear, self.planning_info())
