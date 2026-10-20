"""MuJoCo physics exposed through ordinary RSim signals and a guarded sink."""
from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
import math
from pathlib import Path
import threading
import time

import numpy as np

from rsim.core.component import Component
from rsim.core.commands import CommandRejected, CommandSink, VelocityCommand
from rsim.core.model import Image, PointCloud
from .config import RobotConfig, SensorConfig


def readonly(value):
    array = np.array(value, copy=True)
    array.setflags(write=False)
    return array


def xyz_cloud(points, frame_id, colors=None):
    dtype = [(axis, '<f4') for axis in 'xyz']
    if colors is not None:
        dtype += [(channel, 'u1') for channel in 'rgb']
    array = np.empty(len(points), dtype=dtype)
    for i, axis in enumerate('xyz'):
        array[axis] = points[:, i]
    if colors is not None:
        for i, channel in enumerate('rgb'):
            array[channel] = colors[:, i]
    array.setflags(write=False)
    return PointCloud(array, frame_id)


class MuJoCoSource(Component):
    """One physics owner, one GL thread, many immutable sensor streams.

    scene is a GLB/GLTF or composition JSON; scene_dir accepts prepared data. No file,
    renderer or physics object is opened in __init__. Simulation stamps use a
    per-Runtime clock; command TTLs always use host monotonic time. RGBD/scan
    render only when their ports are demanded. pose/truth are world ground
    truth; odom is integrated wheel-encoder odometry starting at the origin.
    """
    def __init__(self, scene=None, *, scene_dir=None, cache_root="assets/mujoco/scenes",
                 up_axis="auto", floor_height=None, pitch=.08, rebuild=False,
                 spawn=None, robot=None, sensors=None, hz=50, speedup=1.,
                 lidar_hz=10., camera_hz=5., history=128, image_history=8,
                 motion_enabled=False):
        super().__init__()
        if (scene is None) == (scene_dir is None):
            raise ValueError("supply exactly one of scene or scene_dir")
        if not all(math.isfinite(v) and v > 0 for v in (hz, speedup, lidar_hz, camera_hz)):
            raise ValueError("rates and speedup must be finite and positive")
        self.robot = robot or RobotConfig()
        self.sensors = sensors or SensorConfig()
        steps = round(1 / hz / self.robot.timestep)
        if steps < 1 or not np.isclose(steps*self.robot.timestep, 1/hz):
            raise ValueError("1/hz must be an integer multiple of the physics timestep")
        self.scene, self.scene_dir, self.cache_root = scene, scene_dir, cache_root
        self.scene_options = dict(up_axis=up_axis, floor_height=floor_height, pitch=pitch, force=rebuild)
        self.spawn = spawn
        self.hz, self.speedup = hz, speedup
        self.lidar_hz, self.camera_hz = lidar_hz, camera_hz
        self.motion_enabled = bool(motion_enabled)
        for name in ('pose', 'truth', 'odom', 'imu', 'state', 'scan', 'lidar_points'):
            setattr(self, name, self.signal(name, history=history))
        for name in ('image', 'depth', 'camera_info', 'points'):
            setattr(self, name, self.signal(name, history=image_history))
        self.velocity = CommandSink(self, 'velocity', self._apply,
                                    fallback=VelocityCommand(), safe=self._safe, hz=200)
        self.command = VelocityCommand()
        self._command_state = (self.command, 0)
        self._command_lock = None
        self._executor = None
        self.simulation = None

    async def open(self):
        self._command_lock = threading.Lock()
        # Import optional geometry once, before any controller leases are live.
        from graphmap.pose import Pose
        self.pose_type = Pose
        self.clock = f"mujoco:{self._instance_id}"
        for port in self.outputs.values():
            port.clock = self.clock
        self._access = asyncio.Lock()
        self._safe(VelocityCommand())
        self._offset = self._next_scan = self._next_camera = 0.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='rsim-mujoco')
        try:
            await self._work(self._initialize)
            async with self._access:
                packet = await self._work(self._sample, True)
                await self._publish(packet)
            self.task('physics', self._tick, hz=self.hz*self.speedup)
        except BaseException:
            await self.close()
            raise

    def _initialize(self):
        from .scene import prepare_scene
        from .simulation import Simulation
        directory = (Path(self.scene_dir).resolve() if self.scene_dir is not None else
                     prepare_scene(self.scene, self.cache_root, **self.scene_options))
        self.simulation = Simulation(directory, spawn=self.spawn, robot=self.robot, sensors=self.sensors)
        self.prepared_scene = directory
        self._odom = np.zeros(3)
        self._wheel_indices = [int(self.simulation.model.jnt_qposadr[self.simulation.model.joint(f'{side}_wheel_joint').id])
                               for side in ('left', 'right')]
        self._wheels = self.simulation.data.qpos[self._wheel_indices].copy()

    async def _work(self, function, *args):
        return await asyncio.get_running_loop().run_in_executor(self._executor, function, *args)

    def _apply(self, envelope):
        if not isinstance(envelope.value, VelocityCommand):
            raise CommandRejected('expected VelocityCommand')
        if envelope.value != VelocityCommand() and not self.motion_enabled:
            raise CommandRejected('nonzero movement requires motion_enabled=True')
        with self._command_lock:
            self.command = envelope.value
            self._command_state = (envelope.value, envelope.deadline_ns)

    def _safe(self, value):
        if self._command_lock is None:
            self.command = VelocityCommand()
            self._command_state = (self.command, 0)
            return
        with self._command_lock:
            self.command = VelocityCommand()
            self._command_state = (self.command, 0)

    def _demanded(self, *names):
        demand = self._runtime.demanded_ports
        # SharedProvider adds/removes exporters after startup. Their preparation
        # hooks represent live subscriptions beyond the static Runtime roots.
        return any(self.outputs[name] in demand or self.outputs[name]._prepare_hooks for name in names)

    def _advance(self):
        sim = self.simulation
        with self._command_lock:
            command, deadline = self._command_state
        if time.monotonic_ns() >= deadline:
            command = VelocityCommand()
        sim.set_velocity(command.linear_x, command.angular_z)
        sim.step(1/self.hz)
        wheels = sim.data.qpos[self._wheel_indices].copy()
        distance_left, distance_right = (wheels-self._wheels)*self.robot.wheel_radius
        self._wheels = wheels
        distance = (distance_left+distance_right)/2
        angle = (distance_right-distance_left)/self.robot.wheel_track
        yaw = self._odom[2] + angle/2
        self._odom += [distance*math.cos(yaw), distance*math.sin(yaw), angle]
        return self._sample()

    def _sample(self, force=False):
        sim = self.simulation
        stamp = self._offset + float(sim.data.time)
        stamp_ns = round(stamp*1e9)
        header = {'stamp': {'sec': stamp_ns//1_000_000_000, 'nanosec': stamp_ns%1_000_000_000},
                  'frame_id': 'base_footprint'}
        base = sim.world_from_base
        pose = self.pose_type.from_matrix(base, wrd_frame='world', ego_frame='base_footprint')
        odom_pose = self.pose_type(x=self._odom[0], y=self._odom[1], yaw=self._odom[2], degrees=False,
                                   wrd_frame='odom', ego_frame='base_footprint')
        quat = np.roll(sim.data.xquat[sim.body_id], -1)
        gyro = sim.data.sensor('imu_gyro').data.copy()
        accel = sim.data.sensor('imu_accel').data.copy()
        covariance3 = (np.eye(3)*1e-6).ravel().tolist()
        imu = dict(header=dict(header), orientation=dict(zip('xyzw', map(float, quat))),
                   orientation_covariance=covariance3.copy(),
                   angular_velocity=dict(zip('xyz', map(float, gyro))),
                   angular_velocity_covariance=covariance3.copy(),
                   linear_acceleration=dict(zip('xyz', map(float, accel))),
                   linear_acceleration_covariance=covariance3.copy())
        odom = dict(header=dict(header, frame_id='odom'), child_frame_id='base_footprint',
                    pose={'pose': {'position': dict(zip('xyz', map(float, odom_pose.position))),
                                   'orientation': dict(zip('xyzw', map(float, odom_pose.quat)))},
                          'covariance': np.diag([.0001, .0001, 1e6, 1e6, 1e6, .0001]).ravel().tolist()})
        values = dict(pose=pose, truth=pose.copy(), imu=imu, odom=odom,
                      state=dict(backend='mujoco', motion_enabled=self.motion_enabled,
                                 simulation_time=stamp, physics_time=float(sim.data.time),
                                 contacts=int(sim.data.ncon), wheel_targets=sim.data.ctrl.copy().tolist()))
        metadata = {'pose_kind': 'ground_truth', 'world_from_base': readonly(base)}
        if self._demanded('scan', 'lidar_points') and (force or stamp+1e-9 >= self._next_scan):
            scan = sim.lidar()
            ranges = readonly(scan.ranges.astype(np.float32))
            values['scan'] = dict(header=dict(header, frame_id='lidar'), angle_min=float(scan.angles[0]),
                                  angle_max=float(scan.angles[-1]), angle_increment=2*np.pi/len(ranges),
                                  time_increment=0., scan_time=1/self.lidar_hz,
                                  range_min=self.sensors.lidar_min, range_max=self.sensors.lidar_max,
                                  ranges=ranges, intensities=readonly(np.zeros(len(ranges), np.float32)))
            values['lidar_points'] = xyz_cloud(scan.points(), 'lidar')
            metadata['world_from_lidar'] = readonly(scan.world_from_lidar)
            self._next_scan = stamp + 1/self.lidar_hz
        if self._demanded('image', 'depth', 'camera_info', 'points') and (force or stamp+1e-9 >= self._next_camera):
            frame = sim.rgbd()
            values['image'] = Image(readonly(frame.rgb), 'rgb8', 'camera_optical')
            values['depth'] = Image(readonly(frame.depth), '32FC1', 'camera_optical')
            values['camera_info'] = dict(header=dict(header, frame_id='camera_optical'),
                                        width=self.sensors.width, height=self.sensors.height,
                                        K=readonly(frame.K), depth_unit='m', depth_type='optical_z',
                                        world_from_camera_optical=readonly(frame.world_from_camera_optical))
            if self._demanded('points'):
                points, colors = frame.point_cloud()
                values['points'] = xyz_cloud(points, 'camera_optical', colors)
            metadata.update(K=readonly(frame.K), depth_unit='m',
                            world_from_camera_optical=readonly(frame.world_from_camera_optical))
            self._next_camera = stamp + 1/self.camera_hz
        return stamp_ns, values, metadata

    async def _publish(self, packet):
        stamp_ns, values, metadata = packet
        for name, value in values.items():
            extra = dict(metadata)
            if name == 'odom':
                extra['pose_kind'] = 'wheel_odometry'
            await self.outputs[name].publish(value, stamp_ns=stamp_ns, clock=self.clock, metadata=extra)

    async def _tick(self):
        async with self._access:
            await self._publish(await self._work(self._advance))

    async def inspect(self, callback, *args):
        """Simulation-only tooling, serialized on the physics/GL owner thread.

        Application control uses signals/sinks, never this escape hatch. It is
        intended for viewer sync, recordings, diagnostics and integration tests.
        """
        if self._executor is None:
            raise RuntimeError('MuJoCo source is not running')
        async with self._access:
            return await self._work(callback, self.simulation, *args)

    async def reset(self, spawn=None):
        self._safe(VelocityCommand())
        def reset(sim):
            self._offset += float(sim.data.time) + self.robot.timestep
            sim.reset(self.spawn if spawn is None else spawn)
            self._odom[:] = 0
            self._wheels = sim.data.qpos[self._wheel_indices].copy()
            self._next_scan = self._next_camera = 0
            return self._sample(True)
        async with self._access:
            await self._publish(await self._work(reset, self.simulation))

    async def close(self):
        if self._executor is None:
            return
        self._safe(VelocityCommand())
        def cleanup():
            if self.simulation is not None:
                self.simulation.set_velocity(0, 0)
                self.simulation.close()
        try:
            # FIFO executor also completes/cancels an in-flight render before
            # destroying GL on its owning thread.
            await self._work(cleanup)
        finally:
            self._executor.shutdown(wait=True)
            self._executor = None
