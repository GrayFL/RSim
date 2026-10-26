"""Isolated planner components; their only cross-process API is RSim ports."""
from dataclasses import asdict
import asyncio
import os
import time

import numpy as np

from ...core.component import Component
from ...core.commands import CommandSink, CommandRejected, VelocityCommand
from ...components.navigation_map import OccupancyMap
from ...components.towing_geometry import BodyFootprint, ObstacleUnion
from ...components.towing_navigation import TowingNavigator
from ...components.vehicle import TowingParameters


class PlanningWorker(Component):
    """Latest-job mailbox, with CPU work confined to its placed process.

    Both paths and velocity decisions carry the originating control generation,
    map version and monotonic timestamp. A late result never gains a new TTL.
    Global planning is allowed to finish while the controller handles manual
    braking limits independently. The supervisor owns cleanup if a solver blocks.
    """
    def __init__(self, kind):
        super().__init__()
        if kind not in ('local', 'global'): raise ValueError('invalid planner kind')
        self.kind = kind
        self.pending = None
        self.wake = asyncio.Event()
        self.map = None
        self.map_obstacles = None
        self.navigator = None
        self.navigator_key = None
        self.result = self.signal('result', history=2, clock='host:monotonic')
        self.jobs = CommandSink(self, 'jobs', self.accept, fallback=None, safe=self.clear, max_ttl=.5)

    async def accept(self, envelope):
        self.pending = envelope.value
        self.wake.set()
        return {'accepted': self.pending['id']}

    async def clear(self, _=None):
        self.pending = None

    async def open(self):
        self._tasks.add(asyncio.create_task(self.solve_loop(), name='planner:' + self.kind))

    async def solve_loop(self):
        while True:
            await self.wake.wait()
            self.wake.clear()
            await self.solve()

    async def solve(self):
        job, self.pending = self.pending, None
        if job is None: return
        # Keep this worker's transport/lease loop responsive as well. CPU work
        # is isolated by ProcessPlacement; its thread cannot stall the control
        # process or delay acknowledgement of a queued job.
        output = await asyncio.to_thread(self.compute_job, job)
        await self.result.publish(output, stamp_ns=time.monotonic_ns(), clock='host:monotonic')

    def compute_job(self, job):
        began = time.monotonic_ns()
        output = {k: job[k] for k in ('id', 'generation', 'map_version', 'submitted_ns')}
        output.update(pid=os.getpid(), kind=self.kind, ok=False,
                      queue_ms=(began-job['submitted_ns'])/1e6)
        try:
            if began > job['start_deadline_ns']: raise ValueError('planner_input_expired')
            if self.map is None or self.map.version != job['map_version']:
                if job.get('map') is None: raise ValueError('planner_map_missing')
                self.map = OccupancyMap(job['map'])
                self.map_obstacles = None
                if self.map.version != job['map_version']: raise ValueError('map_version_mismatch')
            key = repr((job['vehicle'], job['footprint'], job['planner'], job['drive_mode']))
            if key != self.navigator_key:
                self.navigator = TowingNavigator(TowingParameters(**job['vehicle']), BodyFootprint(**job['footprint']),
                                                planner=job['planner'], drive_mode=job['drive_mode'])
                self.navigator_key = key
            nav = self.navigator
            nav.local.cruise_limits = job['cruise_limits']
            nav.local.command_context(**job.get('command_context', dict(latency=0., previous=[0., 0.])))
            radius = max(.035, self.map.point_radius)
            if self.kind == 'local':
                if self.map_obstacles is None:
                    self.map_obstacles = nav.obstacle_index(self.map.obstacles, self.map.frame, radius, self.map)
                dynamic = nav.obstacle_index(job['dynamic'], self.map.frame, radius)
                obstacles = ObstacleUnion(self.map_obstacles, dynamic)
            else:
                cloud = np.concatenate((self.map.obstacles, job['dynamic']))
                obstacles = nav.obstacle_index(cloud, self.map.frame, radius, self.map)
            state = np.asarray(job['state'], float)
            goal = None if job['goal'] is None else np.asarray(job['goal'], float)
            if self.kind == 'global':
                path = nav.plan(state, goal, obstacles, self.map.bounds)
                output.update(ok=True, path=path, status=nav.plan_status, planning=nav.planning_info())
            else:
                if goal is not None:
                    if nav.goal is None or not np.allclose(nav.goal, goal): nav.reset()
                    nav.goal, nav.path = goal, np.asarray(job['path'], float).reshape(-1, 3).copy()
                    info = job.get('planning', {})
                    nav.partial = bool(info.get('partial', False))
                    nav.needs_replan = bool(info.get('needs_replan', False))
                    nav.plan_status = info.get('status', 'idle')
                    nav.waypoints = np.asarray(info.get('waypoints', []), float).reshape(-1, 3)
                    nav.plan_diagnostics = info.get('diagnostics', {})
                manual = None if job['manual'] is None else VelocityCommand(*job['manual'])
                decision = nav.step(state, np.asarray(job['measured'], float), goal, obstacles, self.map.bounds,
                                    manual=manual, allow_planning=False)
                output.update(ok=True, command=[decision.command.linear_x, decision.command.angular_z],
                              status=decision.status, safe=decision.safe,
                              path=decision.path.tolist() if decision.path.size < 192 else decision.path,
                              trajectory=decision.trajectory.tolist() if decision.trajectory.size < 192 else decision.trajectory,
                              clearance=decision.clearance, planning=decision.planning)
        except (ValueError, RuntimeError) as error:
            output['error'] = str(error)
        output['cached_map_version'] = self.map.version if self.map else None
        output['finished_ns'] = time.monotonic_ns()
        output['compute_ms'] = (time.monotonic_ns() - began) / 1e6
        return output


class PlannerPair:
    """Controller-side bookkeeping for independent global and local workers."""
    def __init__(self):
        self.local, self.global_ = PlanningWorker('local'), PlanningWorker('global')
        self.sequence = 0
        self.global_pending = None
        self.global_delivery = None
        self.global_cursor = self.local_cursor = 0
        self.global_retry = 0.
        self.generation = None
        self.path = np.empty((0, 3))
        self.metrics = {}
        self.cached_maps = {}
        self.planning = {}

    async def deliver_global(self, service, job):
        try:
            await self.global_.jobs.set(job, ttl=.25, _writer=service)
        except (ValueError, RuntimeError, TimeoutError) as error:
            if self.global_pending == job['id']: self.global_pending = None
            self.global_retry = time.monotonic() + .5
            self.metrics['global_delivery'] = dict(id=job['id'], error=str(error))

    async def close(self):
        if self.global_delivery is not None:
            self.global_delivery.cancel()
            await asyncio.gather(self.global_delivery, return_exceptions=True)
            self.global_delivery = None

    def job(self, service, state, measured, dynamic, manual):
        self.sequence += 1
        now = time.monotonic_ns()
        return dict(id=self.sequence, generation=service.generation, submitted_ns=now,
                    start_deadline_ns=now + 250_000_000,
                    map_version=service.map.version, map=service.map_message,
                    vehicle=asdict(service.navigator.model.parameters), footprint=asdict(service.footprint),
                    drive_mode=service.drive_mode, planner=service.config.get('planner', 'rrt'),
                    cruise_limits=service.navigator.local.cruise_limits,
                    command_context=service.braking_context(),
                    state=state.tolist(), measured=measured.tolist(), dynamic=dynamic,
                    goal=None if service.goal is None else service.goal.tolist(),
                    manual=None if manual is None else [manual.linear_x, manual.angular_z],
                    planning=self.planning, path=self.path.tolist() if self.path.size < 192 else self.path)

    async def compute(self, service, manual):
        from ...components.towing_navigation import Decision
        import asyncio
        began = time.monotonic_ns()
        state, measured, dynamic, _ = service.snapshot(build_obstacles=False)
        snapshot_ms = (time.monotonic_ns()-began)/1e6
        if self.generation != service.generation:
            self.path = np.empty((0, 3))
            self.planning = {}
            self.generation = service.generation
            # An older global job keeps running. Its result is discarded below;
            # submitting a new one must not delay the manual/local control path.
        frames = self.global_.result.frames
        if frames and frames[-1].sequence != self.global_cursor:
            self.global_cursor = frames[-1].sequence
            result = frames[-1].data
            self.cached_maps['global'] = result.get('cached_map_version')
            self.metrics['global'] = {k: result.get(k) for k in ('pid', 'compute_ms', 'queue_ms', 'id', 'status', 'error')}
            if result['id'] == self.global_pending: self.global_pending = None
            if result['generation'] == service.generation and result['map_version'] == service.map.version and result['ok']:
                info = result.get('planning', {})
                route = np.asarray(result['path'], float).reshape(-1, 3)
                # A failed replan must not erase a usable approach prefix.
                if not info.get('partial') or len(self.path) <= 1:
                    self.path = route
                    self.planning = info
            self.global_retry = time.monotonic() + .5
        job = self.job(service, state, measured, dynamic, manual)
        if (manual is None and (not len(self.path) or self.planning.get('needs_replan')) and self.global_pending is None
                and (self.global_delivery is None or self.global_delivery.done())
                and time.monotonic() >= self.global_retry):
            self.global_pending = job['id']
            # Even the global worker's acknowledgement is outside the local
            # control path. It must not consume the local input's fixed TTL.
            self.global_delivery = asyncio.create_task(self.deliver_global(service, dict(job, map=None if self.cached_maps.get('global') == service.map.version else job['map'])))
        if self.cached_maps.get('local') == service.map.version: job['map'] = None
        delivery_began = time.monotonic_ns()
        try:
            await self.local.jobs.set(job, ttl=.25, _writer=service)
        except CommandRejected as error:
            if error.reason == 'expired': raise TimeoutError('local delivery expired') from error
            raise
        delivery_ms = (time.monotonic_ns()-delivery_began)/1e6
        async with asyncio.timeout(service.config.get('max_compute_ms', 220) / 1000):
            while True:
                frame = await self.local.result.get(after=self.local_cursor)
                self.local_cursor = frame.sequence
                result = frame.data
                if result['id'] == job['id']: break
        self.cached_maps['local'] = result.get('cached_map_version')
        self.metrics['local'] = {k: result.get(k) for k in ('pid', 'compute_ms', 'queue_ms', 'id', 'error')}
        self.metrics['local'].update(snapshot_ms=snapshot_ms, delivery_ms=delivery_ms,
            return_ms=(time.monotonic_ns()-result.get('finished_ns', time.monotonic_ns()))/1e6,
            total_ms=(time.monotonic_ns()-began)/1e6)
        if not result['ok']:
            if result['error'] in ('planner_input_expired', 'planner_map_missing'): raise TimeoutError(result['error'])
            raise ValueError(result['error'])
        if manual is None:
            self.path = np.asarray(result['path'], float).reshape(-1, 3)
            self.planning = result.get('planning', {})
        decision = Decision(VelocityCommand(*result['command']), result['status'], result['safe'],
                            np.asarray(result['path'], float).reshape(-1, 3),
                            np.asarray(result['trajectory'], float).reshape(-1, 3), result['clearance'], result.get('planning', {}))
        return decision, state, measured, dynamic
