"""Isolated planner components; their only cross-process API is RSim ports."""
from dataclasses import asdict
import asyncio
import os
import time

import numpy as np

from ...core.component import Component
from ...core.commands import CommandSink, CommandRejected, VelocityCommand
from ...components.navigation_map import OccupancyMap
from ...components.towing_geometry import BodyFootprint
from ...components.towing_navigation import TowingNavigator
from ...components.vehicle import TowingParameters


class PlanningWorker(Component):
    """Latest-job mailbox, with CPU work confined to its placed process.

    Both paths and velocity decisions carry the originating control generation,
    map version and monotonic timestamp. A late result never gains a new TTL.
    Global planning is allowed to finish while manual control uses the separate
    local worker. The supervisor owns cleanup even if a solver blocks.
    """
    def __init__(self, kind):
        super().__init__()
        if kind not in ('local', 'global'): raise ValueError('invalid planner kind')
        self.kind = kind
        self.pending = None
        self.map = None
        self.result = self.signal('result', history=2, clock='host:monotonic')
        self.jobs = CommandSink(self, 'jobs', self.accept, fallback=None, safe=self.clear, max_ttl=.5)

    async def accept(self, envelope):
        self.pending = envelope.value
        return {'accepted': self.pending['id']}

    async def clear(self, _=None):
        self.pending = None

    async def open(self):
        self.task('solve', self.solve, hz=50)

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
                self.map = OccupancyMap(job['map'])
                if self.map.version != job['map_version']: raise ValueError('map_version_mismatch')
            nav = TowingNavigator(TowingParameters(**job['vehicle']), BodyFootprint(**job['footprint']),
                                  planner=job['planner'], drive_mode=job['drive_mode'])
            nav.local.cruise_limits = job['cruise_limits']
            cloud = np.concatenate((self.map.obstacles, job['dynamic']))
            obstacles = nav.obstacle_index(cloud, self.map.frame, max(.035, self.map.point_radius), self.map)
            state = np.asarray(job['state'], float)
            goal = None if job['goal'] is None else np.asarray(job['goal'], float)
            if self.kind == 'global':
                path = nav.plan(state, goal, obstacles, self.map.bounds)
                output.update(ok=True, path=path, status=nav.plan_status)
            else:
                if goal is not None:
                    nav.goal, nav.path = goal, np.asarray(job['path'], float).reshape(-1, 3).copy()
                manual = None if job['manual'] is None else VelocityCommand(*job['manual'])
                decision = nav.step(state, np.asarray(job['measured'], float), goal, obstacles, self.map.bounds,
                                    manual=manual, allow_planning=False)
                output.update(ok=True, command=[decision.command.linear_x, decision.command.angular_z],
                              status=decision.status, safe=decision.safe, path=decision.path,
                              trajectory=decision.trajectory, clearance=decision.clearance)
        except (ValueError, RuntimeError) as error:
            output['error'] = str(error)
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
                    state=state, measured=measured, dynamic=dynamic, goal=service.goal,
                    manual=None if manual is None else [manual.linear_x, manual.angular_z], path=self.path)

    async def compute(self, service, manual):
        from ...components.towing_navigation import Decision
        import asyncio
        state, measured, dynamic, _ = service.snapshot(build_obstacles=False)
        if self.generation != service.generation:
            self.path = np.empty((0, 3))
            self.generation = service.generation
            # An older global job keeps running. Its result is discarded below;
            # submitting a new one must not delay the manual/local control path.
        frames = self.global_.result.frames
        if frames and frames[-1].sequence != self.global_cursor:
            self.global_cursor = frames[-1].sequence
            result = frames[-1].data
            self.metrics['global'] = {k: result.get(k) for k in ('pid', 'compute_ms', 'queue_ms', 'id', 'status', 'error')}
            if result['id'] == self.global_pending: self.global_pending = None
            if result['generation'] == service.generation and result['map_version'] == service.map.version and result['ok']:
                self.path = result['path']
            self.global_retry = time.monotonic() + .5
        job = self.job(service, state, measured, dynamic, manual)
        if (manual is None and not len(self.path) and self.global_pending is None
                and (self.global_delivery is None or self.global_delivery.done())
                and time.monotonic() >= self.global_retry):
            self.global_pending = job['id']
            # Even the global worker's acknowledgement is outside the local
            # control path. It must not consume the local input's fixed TTL.
            self.global_delivery = asyncio.create_task(self.deliver_global(service, job))
        try:
            await self.local.jobs.set(job, ttl=.25, _writer=service)
        except CommandRejected as error:
            if error.reason == 'expired': raise TimeoutError('local delivery expired') from error
            raise
        async with asyncio.timeout(service.config.get('max_compute_ms', 220) / 1000):
            while True:
                frame = await self.local.result.get(after=self.local_cursor)
                self.local_cursor = frame.sequence
                result = frame.data
                if result['id'] == job['id']: break
        self.metrics['local'] = {k: result.get(k) for k in ('pid', 'compute_ms', 'queue_ms', 'id', 'error')}
        if not result['ok']:
            if result['error'] == 'planner_input_expired': raise TimeoutError(result['error'])
            raise ValueError(result['error'])
        if manual is None: self.path = result['path']
        decision = Decision(VelocityCommand(*result['command']), result['status'], result['safe'],
                            result['path'], result['trajectory'], result['clearance'])
        return decision, state, measured, dynamic
