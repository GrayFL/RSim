"""One owner for navigation, teleoperation and RSim's velocity command port."""
from __future__ import annotations
import asyncio
from dataclasses import asdict, replace
import math
import os
import time
import uuid
import numpy as np

from ...core.component import Component
from ...core.commands import VelocityCommand, CommandSink
from ...components.vehicle import TowingParameters
from ...components.manual_control import ManualController
from ...components.towing_geometry import BodyFootprint
from ...components.towing_navigation import TowingNavigator
from ...components.navigation_map import OccupancyMap, laser_points, yaw_of


class NavigationService(Component):
    def __init__(self, bridge, config, *, simulated=False, workers=False):
        from .planning import PlannerPair
        self.workers = PlannerPair() if workers is True else workers if isinstance(workers, PlannerPair) else None
        inputs = tuple(bridge.outputs.values())
        if self.workers: inputs += (self.workers.local.result, self.workers.global_.result)
        super().__init__(bridge, inputs=inputs)
        self.bridge, self.config, self.simulated = bridge, config, simulated
        self.command_targets = (bridge.velocity,)
        if self.workers: self.command_targets += (self.workers.local.jobs, self.workers.global_.jobs)
        self.parameters = TowingParameters(**config.get('vehicle', {}))
        self.drive_mode = config.get('drive_mode', 'towing')
        self.footprint = BodyFootprint(**config.get('footprint', {}))
        self.navigator = self.new_navigator()
        self.map = None
        self.map_message = None
        self.map_sequence = 0
        self.lease = None
        self.lease_deadline = self.manual_deadline = 0.
        self.sequence = 0
        self.generation = 0
        self.mode, self.reason = 'idle', 'starting'
        self.goal = None
        self.axes = (0., 0.)
        self.manual_control = ManualController()
        self.armed = False
        self.speed_limits = dict(navigation=min(1., self.parameters.max_linear), manual=min(1., self.parameters.max_linear),
                                 angular=self.parameters.max_angular, handles=True)
        self.speed_limits.update(config.get('speed_limits', {}))
        for key, maximum in (('navigation', 1.6), ('manual', 1.6), ('angular', .8)):
            if not isinstance(self.speed_limits[key], (int, float)) or not 0 < self.speed_limits[key] <= maximum:
                raise ValueError('invalid configured speed limit: ' + key)
        if type(self.speed_limits['handles']) is not bool: raise ValueError('handles setting must be boolean')
        self.pedal = dict(command=None, position='unknown', feedback=False)
        self.maintenance = None
        self.path = np.empty((0, 3))
        self.trajectory = np.empty((0, 3))
        self.observed_points = np.empty((0, 2))
        self.pose_value = None
        self.measured = [0., 0.]
        self.preview = [0., 0.]
        self.published = [0., 0.]
        self.output_lock = asyncio.Lock()
        self.loop_ms = 0.
        self.last_stop = None
        self.needs_reset = False
        self.state = self.signal('state', history=2, clock='system')
        self.map_data = self.signal('map_data', history=1, clock='ros:system')
        self.visual_data = self.signal('visual_data', history=1, clock='system')
        self.actions = self.signal('actions', history=4, clock='host:monotonic')
        # This is the web RPC lease, not a velocity TTL. Manual input still
        # expires after .25 s and original browser deadlines are revalidated.
        self.requests = CommandSink(self, 'requests', self.request, fallback=None, safe=self.request_expired, max_ttl=1.)
        self.pending_action = None
        self.reserved_goal = None

    def new_navigator(self):
        return TowingNavigator(self.parameters, self.footprint,
                               planner=self.config.get('planner', 'rrt'), drive_mode=self.drive_mode)

    def manual_active(self):
        return self.mode == 'manual' and time.monotonic() < self.manual_deadline

    async def open(self):
        self.task('map', self.refresh_map, hz=1)
        self.task('control', self.tick, hz=10)
        self.task('visual', self.refresh_visual, hz=2)
        self.task('actions', self.perform_action, hz=20)

    async def authorize_request(self, lease, sequence):
        self.authorize(lease, sequence)

    async def request(self, envelope):
        request = envelope.value
        try:
            op = request['method']
            data = request.get('data', {})
            if op == 'claim': return {'lease': await self.claim()}
            if op == 'command': await self.command(data)
            elif op == 'stop': await self.stop(**data)
            elif op == 'authorize': self.authorize(data['lease'], data['sequence'])
            elif op == 'intent': await self.intent(data.pop('lease'), data)
            elif op == 'reserve_goal': return {'ticket': await self.reserve_goal(data)}
            elif op == 'commit_goal': await self.commit_goal(data['ticket'], data['command'])
            elif op == 'maintenance':
                await self.set_maintenance(data.get('operation'))
            elif op == 'pedal':
                await self.pedal_action(data['action'])
            elif op == 'settings':
                await self.apply_settings(data['values'])
            elif op == 'localize':
                return {'id': await self.enqueue_localization(data)}
            else: raise ValueError('unknown navigation request')
            return {'ok': True}
        except (ValueError, RuntimeError, TimeoutError) as error:
            return {'error': str(error)}

    async def request_expired(self, _=None):
        if self.armed or self.mode in ('manual', 'navigation'):
            await self.stop('web_command_channel_expired')

    async def perform_action(self):
        action = self.pending_action
        if action is None: return
        try:
            if action.get('generation') != self.generation: raise ValueError('localization cancelled')
            if action.get('lease') and (action['lease'] != self.lease or time.monotonic() >= self.lease_deadline):
                raise ValueError('localization controller disconnected')
            if time.monotonic()*1000 >= action['deadline_ms']: raise ValueError('localization request expired')
            result = await self.bridge.relocalize(action['x'], action['y'], action['yaw'])
        except (ValueError, RuntimeError, TimeoutError) as error:
            result = {'ok': False, 'error': str(error)}
        await self.actions.publish(dict(result, id=action['id']), stamp_ns=time.monotonic_ns(), clock='host:monotonic')
        self.pending_action = None

    async def intent(self, lease, data):
        # Private Shared RPC for recognized speech. It cannot acquire/refresh a
        # lease, arm motion or race the browser's sequence number.
        if lease != self.lease or time.monotonic() >= self.lease_deadline:
            raise ValueError('voice controller disconnected')
        if data.get('op') not in ('goal', 'pause', 'resume', 'cancel'):
            raise ValueError('unsupported voice control')
        await self.command(data, trusted_intent=True)

    async def reserve_goal(self, request):
        self.authorize(request['lease'], request['sequence'])
        ticket = uuid.uuid4().hex
        self.reserved_goal = (ticket, self.lease, request['deadline_ms'])
        return ticket

    async def commit_goal(self, ticket, command):
        self.take_reservation(ticket)
        if command.get('op') != 'goal': raise ValueError('invalid reserved operation')
        await self.command(command, trusted_intent=True)

    def take_reservation(self, ticket):
        reservation = self.reserved_goal
        if reservation is None or reservation[0] != ticket: raise ValueError('goal was superseded or stopped')
        self.reserved_goal = None
        if reservation[1] != self.lease or time.monotonic() >= self.lease_deadline or time.monotonic()*1000 >= reservation[2]:
            raise ValueError('goal authorization expired')
        return reservation

    async def enqueue_localization(self, data):
        if self.pending_action is not None: raise ValueError('localization already pending')
        ticket = data.pop('ticket', None)
        if ticket:
            _, lease, deadline = self.take_reservation(ticket)
        elif self.simulated:
            lease, deadline = None, time.monotonic()*1000+1500
        else: raise ValueError('localization requires a fresh reserved request')
        await self.stop('relocalizing', cancel=True)
        identifier = uuid.uuid4().hex
        self.pending_action = dict(id=identifier, lease=lease, deadline_ms=deadline, generation=self.generation,
                                   **{k: data[k] for k in ('x', 'y', 'yaw')})
        return identifier

    async def relocalize_reserved(self, ticket, x, y, yaw):
        identifier = await self.enqueue_localization(dict(ticket=ticket, x=x, y=y, yaw=yaw))
        cursor = None
        async with asyncio.timeout(3.5):
            while True:
                frame = await self.actions.get(after=cursor)
                cursor = frame.sequence
                if frame.data['id'] == identifier: return frame.data

    async def set_maintenance(self, operation):
        if operation not in (None, 'mapping', 'pedal', 'mapping_unknown'): raise ValueError('unsupported maintenance mode')
        await self.stop('maintenance_' + str(operation), cancel=True)
        self.maintenance = operation

    async def apply_settings(self, values):
        from .settings import validate, apply_config
        values = validate(values)
        if self.measured is None or np.max(np.abs(self.measured)) > .02:
            await self.stop('settings_waiting_for_stop')
            raise ValueError('stop and wait for the chassis before applying settings')
        await self.stop('settings_changed', cancel=True)
        apply_config(self.config, values)
        self.drive_mode = values['drive_mode']
        self.parameters = TowingParameters(**values['profiles'][self.drive_mode])
        self.footprint = BodyFootprint(**values['footprint'])
        self.speed_limits = dict(values['speed_limits'])
        self.navigator = self.new_navigator()
        self.bridge.drive_mode = self.drive_mode
        if self.simulated:
            from ...components.vehicle import drive_model
            self.bridge.model = drive_model(self.parameters, self.drive_mode)

    async def pedal_action(self, action):
        await self.stop('pedal_action', cancel=True)
        self.footprint = replace(self.footprint, rear=.81)
        self.pedal = dict(command=action, position='unconfirmed', feedback=False)

    async def refresh_visual(self):
        try: dynamic = await asyncio.to_thread(self.scan_points)
        except ValueError: dynamic = np.empty((0, 2))
        self.observed_points = dynamic
        await self.visual_data.publish(dict(path=self.path, trajectory=self.trajectory, scan=dynamic),
                                       stamp_ns=time.time_ns(), clock='system')

    def latest(self, name, max_age=None):
        port = self.bridge.outputs.get(name)
        frames = port.frames if port else ()
        if not frames: raise ValueError('missing_' + name)
        frame = frames[-1]
        age = (time.monotonic_ns() - frame.metadata.get('received_ns', 0)) * 1e-9
        if max_age is not None and not 0 <= age <= max_age: raise ValueError('stale_' + name)
        return frame

    async def refresh_map(self):
        try:
            frame = self.latest('map')
            if frame.sequence == self.map_sequence: return
            updated = await asyncio.to_thread(OccupancyMap, frame.data)
            await asyncio.to_thread(updated.image)  # PNG compression stays off the control loop.
            self.map_sequence = frame.sequence
            if self.map is not None and self.map.version == updated.version: return
            await self.stop('map_changed', cancel=True)
            self.map = updated
            self.map_message = frame.data
            await self.map_data.publish(frame.data, stamp_ns=frame.stamp_ns, clock='ros:system')
            self.needs_reset = True
        except ValueError as error:
            self.reason = str(error)

    async def claim(self):
        now = time.monotonic()
        if self.lease and now < self.lease_deadline: raise ValueError('another browser owns control')
        await self.stop('new_controller')
        self.lease, self.sequence = uuid.uuid4().hex, 0
        self.lease_deadline = now + 1.
        return self.lease

    def authorize(self, lease, sequence):
        now = time.monotonic()
        if lease != self.lease or now >= self.lease_deadline: raise ValueError('control lease expired')
        if type(sequence) is not int or sequence <= self.sequence: raise ValueError('replayed or out-of-order control')
        self.sequence, self.lease_deadline = sequence, now + .8

    async def zero(self):
        async with self.output_lock:
            await self.zero_unlocked()

    async def zero_unlocked(self):
        self.bridge.armed = False
        await self.bridge.velocity.set(VelocityCommand(), ttl=.25, _writer=self)
        self.published = [0., 0.]

    async def stop(self, reason='operator_stop', cancel=False, keep_armed=False):
        self.last_stop = dict(reason=reason, at_ms=round(time.monotonic()*1000, 1),
                              mode=self.mode, armed=self.armed, loop_ms=round(self.loop_ms, 1))
        self.reserved_goal = None
        self.generation += 1
        self.armed = self.armed if keep_armed else False
        self.manual_deadline = 0.
        self.axes = (0., 0.)
        self.manual_control.reset()
        self.reason = reason
        if cancel: self.goal = None
        self.mode = 'paused' if self.goal is not None else 'idle'
        self.needs_reset = True
        # A browser disconnect can cancel its request task. The stop write must
        # still finish, including when a previous publication is in flight.
        if not self.bridge._closed: await asyncio.shield(self.zero())
        # A slow in-flight planner may finish after the stop ACK. Publish the
        # confirmed zero immediately instead of leaving the UI's old velocity.
        await self.state.publish(self.status(), stamp_ns=time.time_ns(), clock='system')

    async def command(self, request, *, trusted_intent=False):
        op = request['op']
        # Preserve the browser's original deadline across Shared command hops.
        if op not in ('stop', 'pause', 'cancel') and 'deadline_ms' in request:
            if time.monotonic() * 1000 >= request['deadline_ms']: raise ValueError('control deadline expired')
        # Stop is always permitted to an authenticated operator, even without
        # the current writer lease. It can only reduce this service's authority.
        if op in ('stop', 'pause', 'cancel'):
            await self.stop(op, cancel=op == 'cancel')
            return
        if not trusted_intent: self.authorize(request.get('lease'), request.get('sequence'))
        allowed = ('heartbeat', 'footprint', 'drive_mode', 'speeds', 'manual', 'arm') if self.maintenance in ('mapping', 'mapping_unknown') else ('heartbeat',)
        if self.maintenance and op not in allowed:
            raise ValueError('auxiliary operation in progress: ' + self.maintenance)
        if op == 'heartbeat': return
        if op == 'arm':
            if not self.bridge.motion_enabled: raise ValueError('deployment is zero-only')
            status = self.latest('status', .5).data
            if status['health'] != 'ok': raise ValueError(status['health'])
            self.armed = True
        elif op == 'manual':
            throttle, steer = float(request.get('throttle', 0)), float(request.get('steering', 0))
            self.manual_control.input(throttle, steer, self.measured or [0., 0.],
                                      self.pose_value[2] if self.pose_value else None)
            if throttle == 0 and steer == 0:
                await self.stop('manual_released', keep_armed=True)
                return
            if self.mode != 'manual' or self.axes != (throttle, steer): self.generation += 1
            self.mode, self.axes = 'manual', (throttle, steer)
            self.manual_deadline = time.monotonic() + .25
        elif op == 'goal':
            values = np.array([request[k] for k in ('x', 'y', 'yaw')], float)
            if not np.isfinite(values).all(): raise ValueError('invalid goal')
            if self.map is None or not self.map.known_free(values[:2]): raise ValueError('goal is outside known free map')
            if request.get('map_version') and request['map_version'] != self.map.version: raise ValueError('goal belongs to an outdated map')
            self.generation += 1
            self.goal, self.needs_reset = values, True
            # A new goal is queued while the deadman is held. No navigation or
            # resume request can displace a fresh manual command.
            if not self.manual_active(): self.mode = 'navigation'
        elif op == 'resume':
            if self.goal is None: raise ValueError('no paused goal')
            self.generation += 1
            self.needs_reset = True
            if not self.manual_active(): self.mode = 'navigation'
        elif op == 'drive_mode':
            mode = request.get('drive_mode')
            if mode not in ('towing', 'differential'): raise ValueError('invalid drive mode')
            if self.measured is None or np.max(np.abs(self.measured)) > .02:
                raise ValueError('stop and wait for the chassis before changing drive mode')
            await self.stop('drive_mode_changed')
            self.drive_mode = mode
            self.parameters = TowingParameters(**self.config.get('profiles', {}).get(mode, asdict(self.parameters)))
            self.navigator = self.new_navigator()
            self.bridge.drive_mode = mode
            if self.simulated:
                from ...components.vehicle import drive_model
                self.bridge.model = drive_model(self.parameters, mode)
        elif op == 'footprint':
            await self.stop('footprint_changed', cancel=True)
            extended = request.get('extended')
            if type(extended) is not bool: raise ValueError('extended must be boolean')
            self.footprint = replace(self.footprint, rear=.81 if extended else .27)
            self.needs_reset = True
        elif op == 'speeds':
            limits = {key: float(request[key]) for key in ('navigation', 'manual', 'angular')}
            if not all(math.isfinite(v) and v > 0 for v in limits.values()): raise ValueError('invalid speed settings')
            if max(limits['navigation'], limits['manual']) > self.parameters.max_linear or limits['angular'] > self.parameters.max_angular:
                raise ValueError('speed settings exceed chassis limits')
            if type(request.get('handles')) is not bool: raise ValueError('handles must be boolean')
            await self.stop('speed_settings_changed')
            self.speed_limits.update(limits, handles=request['handles'])
        else:
            raise ValueError('unknown control operation')

    def snapshot(self, *, build_obstacles=True):
        if self.map is None: raise ValueError('waiting_for_map')
        pose = self.latest('pose', .5)
        if not -.1 <= pose.data.get('ros_age', 1e10) <= .5: raise ValueError('stale_transform')
        if pose.data['frame'].lstrip('/') != self.map.frame: raise ValueError('map_frame_changed')
        state = np.r_[pose.data['translation'][:2], yaw_of(pose.data['quaternion'])]
        odom = self.latest('odom', .5).data['twist']['twist']
        measured = np.array([odom['linear']['x'], odom['angular']['z']], float)
        dynamic = self.scan_points()
        obstacles = None
        if build_obstacles:
            cloud = np.concatenate((self.map.obstacles, dynamic))
            obstacles = self.navigator.obstacle_index(cloud, self.map.frame, max(.035, self.map.point_radius), self.map)
        return state, measured, dynamic, obstacles

    def scan_points(self):
        points = []
        required = self.config.get('required_scans', ['scan'])
        optional = self.config.get('optional_scans', [])
        for name in required + optional:
            try:
                frame = self.latest(name, .5)
                if 'world_transform' not in frame.metadata: raise ValueError('missing_scan_transform:' + name)
                age = (time.time_ns() - frame.stamp_ns) * 1e-9
                if not -.1 <= age <= .6: raise ValueError('stale_scan_timestamp:' + name)
                points.append(laser_points(frame.data, frame.metadata['world_transform']))
            except ValueError:
                # Once an optional sensor has produced data, loss is a fault.
                if name in required or self.bridge.outputs[name].frames: raise
        return np.concatenate(points) if points else np.empty((0, 2))

    def compute(self, goal, manual):
        state, measured, dynamic, obstacles = self.snapshot()
        result = self.navigator.step(state, measured, goal, obstacles, self.map.bounds, manual=manual)
        return result, state, measured, dynamic

    def telemetry(self):
        def plain(value):
            if isinstance(value, dict): return {k: plain(v) for k, v in value.items()}
            if isinstance(value, np.ndarray): return plain(value.tolist())
            if isinstance(value, (tuple, list)): return [plain(v) for v in value]
            if isinstance(value, np.generic): return plain(value.item())
            if isinstance(value, float) and not math.isfinite(value): return None
            return value
        result = {}
        for name in ('battery', 'handles', 'mapping'):
            try:
                frame = self.latest(name)
                age = (time.monotonic_ns() - frame.metadata.get('received_ns', 0)) / 1e9
                result[name] = dict(data=plain(frame.data), age_sec=round(age, 2), fresh=0 <= age <= (5 if name == 'battery' else 1.))
            except ValueError: result[name] = dict(data=None, age_sec=None, fresh=False)
        return result

    def cruise_limits(self):
        speed = self.speed_limits['manual' if self.mode == 'manual' else 'navigation']
        handle = self.telemetry()['handles']
        data = (handle['data'] or {}).get('data', [])
        if self.speed_limits['handles'] and handle['fresh'] and len(data) >= 2:
            if data[0]: speed = min(speed * .3, .3)
            elif data[1] and self.mode == 'navigation': speed = max(speed * 2., speed + .4)
        return min(self.parameters.max_linear, speed), min(self.parameters.max_angular, self.speed_limits['angular'])

    async def tick(self):
        started = time.monotonic()
        try:
            # Keep status fresh even when nobody owns control.
            try:
                p = self.latest('pose', .5).data
                self.pose_value = [*p['translation'][:2], yaw_of(p['quaternion'])]
            except ValueError:
                self.pose_value = None
            try:
                odom = self.latest('odom', .5).data['twist']['twist']
                self.measured = [odom['linear']['x'], odom['angular']['z']]
            except ValueError:
                self.measured = None
            if started >= self.lease_deadline and (self.armed or self.mode in ('manual', 'navigation')):
                await self.stop('controller_disconnected')
            if self.mode == 'manual' and started >= self.manual_deadline:
                await self.stop('manual_deadman')
            if self.mode not in ('manual', 'navigation'): return
            status = self.latest('status', .5).data
            if status['health'] != 'ok': raise ValueError(status['health'])
            linear, angular = status['limits']
            capped = replace(self.parameters, max_linear=min(self.parameters.max_linear, linear),
                             max_angular=min(self.parameters.max_angular, angular))
            if self.needs_reset or capped != self.navigator.model.parameters:
                self.navigator = TowingNavigator(capped, self.footprint,
                                                 planner=self.config.get('planner', 'rrt'), drive_mode=self.drive_mode)
                self.needs_reset = False
            generation = self.generation
            cruise = self.cruise_limits()
            self.navigator.local.cruise_limits = cruise
            from .settings import defaults
            axis_modes = self.config.get('manual', defaults({})['manual'])[self.drive_mode]
            manual = self.manual_control.target(self.measured or [0., 0.],
                       self.pose_value[2] if self.pose_value else 0., self.navigator.model, axis_modes, cruise) if self.mode == 'manual' else None
            try:
                if self.workers:
                    result, state, measured, dynamic = await self.workers.compute(self, manual)
                else:
                    result, state, measured, dynamic = await asyncio.to_thread(self.compute, self.goal, manual)
            except (ValueError, RuntimeError, TimeoutError):
                # Errors belong to their originating job just like results.
                # A late navigation error cannot disarm a newer manual input.
                if generation != self.generation: return
                raise
            if generation != self.generation: return
            self.path, self.trajectory = result.path, result.trajectory
            self.pose_value, self.measured = state.tolist(), measured.tolist()
            self.observed_points = dynamic
            self.preview = [result.command.linear_x, result.command.angular_z]
            self.reason = result.status
            self.loop_ms = (time.monotonic() - started) * 1000
            if self.loop_ms > self.config.get('max_compute_ms', 220):
                # Global planning may exceed one tick. Never execute that old
                # result; keep its path and retry on a fresh sensor snapshot.
                self.reason = 'planning_deadline_missed'
                await self.zero()
                return
            if time.monotonic() >= self.lease_deadline: raise ValueError('controller_disconnected')
            if self.mode == 'manual' and time.monotonic() >= self.manual_deadline: raise ValueError('manual_deadman')
            if result.status == 'waiting_global_path':
                await self.zero()
                return
            if not result.safe: raise ValueError(result.status)
            # Final source freshness and hardware interlocks may change while
            # planning. The independent Python 2 watchdog checks them again.
            if self.latest('status', .5).data['health'] != 'ok': raise ValueError('hardware_interlock')
            async with self.output_lock:
                if generation != self.generation: return
                command = result.command if self.armed and self.bridge.motion_enabled else VelocityCommand()
                self.bridge.armed = self.armed
                await self.bridge.velocity.set(command, ttl=.25, _writer=self)
                if generation != self.generation:
                    await self.zero_unlocked()
                else:
                    self.published = [command.linear_x, command.angular_z]
            if result.status == 'arrived': await self.stop('arrived', cancel=True)
        except TimeoutError:
            self.loop_ms = (time.monotonic() - started) * 1000
            self.reason = 'planning_deadline_missed'
            await self.zero()
        except (ValueError, RuntimeError) as error:
            self.loop_ms = (time.monotonic() - started) * 1000
            await self.stop(str(error))
        finally:
            await self.state.publish(self.status(), stamp_ns=time.time_ns(), clock='system')

    def status(self, compact=False):
        try: hardware = self.latest('status', 1.).data
        except ValueError: hardware = {'health': 'bridge_unavailable', 'estop': None}
        result = dict(mode=self.mode, reason=self.reason, armed=self.armed, simulated=self.simulated,
                      drive_mode=self.drive_mode, manual_active=self.manual_active(),
                      controller_pid=os.getpid(),
                      zero_only=not self.bridge.motion_enabled, pose=self.pose_value, measured=self.measured,
                      preview=self.preview, published=self.published, loop_ms=round(self.loop_ms, 1),
                      last_stop=self.last_stop,
                      goal=None if self.goal is None else self.goal.tolist(),
                      control_owned=bool(self.lease and time.monotonic() < self.lease_deadline),
                      hardware=hardware)
        result.update(speed_limits=self.speed_limits, pedal=self.pedal, maintenance=self.maintenance)
        result['manual_control'] = dict(brake_locked=self.manual_control.brake_locked,
                                      steering_angle=self.manual_control.steering_angle,
                                      axes=self.config.get('manual', {}).get(self.drive_mode))
        if not compact:
            result.update(footprint=asdict(self.footprint), vehicle=asdict(self.parameters),
                          telemetry=self.telemetry(),
                          equilibrium=dict(linear=self.parameters.terminal_speed, angular=self.parameters.terminal_yaw_rate),
                          map=None if self.map is None else self.map.metadata())
        if self.workers: result['workers'] = self.workers.metrics
        return result

    async def close(self):
        self.generation += 1
        self.armed = False
        if self.workers: await self.workers.close()
        await self.zero()
