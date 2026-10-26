"""ROS-free, acceleration-limited planar bridge for UI and integration tests."""
import math
import time
import numpy as np
from ...core.component import Component
from ...core.commands import CommandSink, VelocityCommand
from ...components.vehicle import drive_model, TowingParameters
from ...components.towing_geometry import advance


class SimulationBridge(Component):
    def __init__(self, parameters=None, drive_mode='towing'):
        super().__init__()
        self.drive_mode = drive_mode
        self.model = drive_model(parameters or TowingParameters(), drive_mode)
        self.motion_enabled, self.armed = True, False
        self.position, self.speed = np.zeros(3), np.zeros(2)
        self.target = np.zeros(2)
        for name in ('map', 'scan', 'pose', 'odom', 'status'):
            setattr(self, name, self.signal(name, history=2, clock='ros:system'))
        self.velocity = CommandSink(self, 'velocity', self.apply, fallback=VelocityCommand(), safe=self.stop)

    async def emit(self, name, data, **metadata):
        await self.outputs[name].publish(data, stamp_ns=time.time_ns(), clock='ros:system',
                                        metadata=dict(received_ns=time.monotonic_ns(), **metadata))

    async def open(self):
        grid = np.zeros((100, 100), np.int8)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        await self.emit('map', {'header': {'frame_id': 'map'}, 'info': {'width': 100, 'height': 100,
            'resolution': .1, 'origin': {'position': {'x': -5., 'y': -5., 'z': 0.},
                                       'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}, 'data': grid.ravel()})
        self.task('simulation', self.tick, hz=20)

    async def apply(self, envelope):
        self.target = np.array([envelope.value.linear_x, envelope.value.angular_z]) if self.armed else np.zeros(2)

    async def stop(self, fallback=None): self.target = np.zeros(2)

    async def tick(self):
        if self.drive_mode != self.model.mode:
            self.model = drive_model(self.model.parameters, self.drive_mode)
        old = self.speed.copy()
        self.speed = self.model.advance_velocity(old, self.target, .05)
        self.position = advance(self.position, *((old + self.speed) / 2), .05)
        x, y, yaw = self.position
        q = [0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]
        transform = {'translation': [x, y, 0.], 'quaternion': q}
        await self.emit('pose', dict(transform, frame='map', body='base_footprint', ros_age=0.))
        await self.emit('odom', {'twist': {'twist': {'linear': {'x': self.speed[0]}, 'angular': {'z': self.speed[1]}}}})
        angles = np.linspace(-math.pi, math.pi, 180, endpoint=False)
        dx, dy = np.cos(angles + yaw), np.sin(angles + yaw)
        with np.errstate(divide='ignore', invalid='ignore'):
            rx = (np.where(dx >= 0, 4.9, -4.9) - x) / dx
            ry = (np.where(dy >= 0, 4.9, -4.9) - y) / dy
        ranges = np.minimum(rx, ry)
        await self.emit('scan', {'angle_min': -math.pi, 'angle_increment': 2 * math.pi / 180,
            'range_min': .02, 'range_max': 15., 'ranges': ranges}, world_transform=transform,
            body_transform={'translation': [0., 0., 0.], 'quaternion': [0., 0., 0., 1.]})
        p = self.model.parameters
        await self.emit('status', {'health': 'ok', 'estop': False, 'bumper': False, 'scheduler': 'STOP',
            'reason': 'simulation', 'limits': [p.max_linear, p.max_angular], 'published': self.target.tolist(), 'ages': {}})

    async def relocalize(self, x, y, yaw):
        self.position = np.array([x, y, yaw], float)
        return {'ok': True, 'simulated': True}

    async def close(self): await self.stop()
