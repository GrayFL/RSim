"""Web-side view of the controller, carried exclusively by Shared ports."""
import asyncio
import os
import time
from types import SimpleNamespace

import numpy as np

from ...core.component import Component
from ...components.navigation_map import OccupancyMap
from ...components.towing_geometry import BodyFootprint


class NavigationClient(Component):
    def __init__(self, provider, config, *, simulated, motion_enabled):
        super().__init__(provider, inputs=tuple(provider.outputs.values()))
        self.provider, self.config, self.simulated = provider, config, simulated
        self.command_targets = (provider.requests,)
        self.bridge = SimpleNamespace(motion_enabled=motion_enabled, relocalize=self.relocalize)
        self.map = None
        self.map_sequence = 0
        self.observed_points = np.empty((0, 2))

    async def open(self):
        self.task('web-map', self.refresh_map, hz=2)

    async def refresh_map(self):
        frames = self.provider.map_data.frames
        if frames and frames[-1].sequence != self.map_sequence:
            frame = frames[-1]
            updated = await asyncio.to_thread(OccupancyMap, frame.data)
            await asyncio.to_thread(updated.image)
            self.map, self.map_sequence = updated, frame.sequence

    def status(self, compact=False):
        frames = self.provider.state.frames
        if not frames:
            return dict(mode='idle', reason='controller_starting', zero_only=not self.bridge.motion_enabled,
                        armed=False, simulated=self.simulated, pose=None, measured=None, preview=[0, 0], published=[0, 0],
                        loop_ms=0., goal=None, control_owned=False, map=None,
                        hardware={'health': 'bridge_unavailable', 'estop': None}, web_pid=os.getpid())
        result = dict(frames[-1].data, web_pid=os.getpid())
        if time.time_ns() - frames[-1].received_ns > 1_000_000_000:
            result.update(reason='controller_status_stale', armed=False)
        if compact:
            for key in ('footprint', 'vehicle', 'equilibrium', 'map'): result.pop(key, None)
        return result

    @property
    def mode(self): return self.status()['mode']

    @property
    def pose_value(self): return self.status().get('pose')

    @property
    def footprint(self): return BodyFootprint(**self.status().get('footprint', {}))

    def visual(self):
        frames = self.provider.visual_data.frames
        return frames[-1].data if frames else {}

    @property
    def path(self): return np.asarray(self.visual().get('path', [])).reshape(-1, 3)

    @property
    def trajectory(self): return np.asarray(self.visual().get('trajectory', [])).reshape(-1, 3)

    def scan_points(self): return np.asarray(self.visual().get('scan', [])).reshape(-1, 2)

    async def rpc(self, method, **data):
        reply = await self.provider.requests.set({'method': method, 'data': data}, ttl=1., _writer=self)
        if reply.get('error'): raise ValueError(reply['error'])
        return reply

    async def claim(self):
        return (await self.rpc('claim'))['lease']

    async def command(self, request): return await self.rpc('command', **request)

    async def apply_settings(self, values, *, lease=None): return await self.rpc('settings', values=values, lease=lease)

    async def validate_controller(self, lease): return (await self.rpc('controller_valid', lease=lease))['valid']

    async def release_controller(self, lease): return await self.rpc('release', lease=lease)

    async def stop(self, reason='operator_stop', cancel=False):
        return await self.rpc('stop', reason=reason, cancel=cancel)

    async def authorize_request(self, lease, sequence):
        return await self.rpc('authorize', lease=lease, sequence=sequence)

    async def intent(self, lease, data): return await self.rpc('intent', lease=lease, **data)

    async def set_maintenance(self, operation): return await self.rpc('maintenance', operation=operation)

    async def pedal_action(self, action): return await self.rpc('pedal', action=action)

    async def reserve_goal(self, command): return (await self.rpc('reserve_goal', **command))['ticket']

    async def commit_goal(self, ticket, command): return await self.rpc('commit_goal', ticket=ticket, command=command)

    async def relocalize_reserved(self, ticket, x, y, yaw): return await self.relocalize(x, y, yaw, ticket=ticket)

    async def relocalize(self, x, y, yaw, *, ticket=None):
        result = await self.rpc('localize', x=x, y=y, yaw=yaw, ticket=ticket)
        after = None
        async with asyncio.timeout(3.5):
            while True:
                frame = await self.provider.actions.get(after=after)
                after = frame.sequence
                if frame.data['id'] == result['id']: return frame.data
