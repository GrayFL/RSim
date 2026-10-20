from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import mujoco
import numpy as np

from .config import RobotConfig, SensorConfig
from .model import build_model_xml


def transform(position, rotation):
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation).reshape(3, 3)
    result[:3, 3] = position
    return result


@dataclass
class LaserScan:
    time: float
    angles: np.ndarray
    ranges: np.ndarray
    world_from_lidar: np.ndarray

    def points(self, world=False):
        valid = np.isfinite(self.ranges)
        points = np.column_stack((np.cos(self.angles[valid])*self.ranges[valid],
                                  np.sin(self.angles[valid])*self.ranges[valid],
                                  np.zeros(valid.sum())))
        if world:
            points = points @ self.world_from_lidar[:3, :3].T + self.world_from_lidar[:3, 3]
        return points


@dataclass
class RGBDFrame:
    time: float
    rgb: np.ndarray
    depth: np.ndarray
    K: np.ndarray
    world_from_camera_optical: np.ndarray

    def point_cloud(self, stride=4, world=False):
        if stride < 1:
            raise ValueError("stride must be >= 1")
        v, u = np.mgrid[0:self.depth.shape[0]:stride, 0:self.depth.shape[1]:stride]
        z = self.depth[::stride, ::stride]
        valid = np.isfinite(z) & (z > 0)
        points = np.column_stack(((u[valid]-self.K[0, 2])*z[valid]/self.K[0, 0],
                                  (v[valid]-self.K[1, 2])*z[valid]/self.K[1, 1], z[valid]))
        if world:
            points = points @ self.world_from_camera_optical[:3, :3].T + self.world_from_camera_optical[:3, 3]
        return points, self.rgb[::stride, ::stride][valid]


class Simulation:
    def __init__(self, scene_dir, *, spawn=None, robot=None, sensors=None):
        self.scene_dir = Path(scene_dir).resolve()
        self.metadata = json.loads((self.scene_dir / "metadata.json").read_text(encoding="utf-8"))
        self.robot = robot or RobotConfig()
        self.sensors = sensors or SensorConfig()
        xml = build_model_xml(self.scene_dir, self.robot, self.sensors, return_text=True)
        self.xml_path = self.scene_dir / "world.xml"
        # MuJoCo's native Windows file APIs cannot reliably open Unicode paths.
        # Python loads the files into the virtual filesystem, preserving Chinese
        # workspace/source paths without chdir, copies, or environment changes.
        assets = {p.name: p.read_bytes() for p in self.scene_dir.iterdir()
                  if p.suffix.lower() in (".obj", ".png")}
        self.model = mujoco.MjModel.from_xml_string(xml, assets=assets)
        self.data = mujoco.MjData(self.model)
        self.body_id = self.model.body("base_footprint").id
        self._base_qpos = int(self.model.jnt_qposadr[self.model.joint("base_free").id])
        self.lidar_id = self.model.site("lidar").id
        self.camera_id = self.model.camera("rgbd").id
        self._renderer = None
        self._render_options = mujoco.MjvOption()
        self._render_options.geomgroup[:] = [1, 1, 1, 0, 0, 0]
        self._render_options.sitegroup[:] = 0
        self._ray_groups = np.array([1, 0, 1, 0, 0, 0], dtype=np.uint8)
        self.angles = np.linspace(-np.pi, np.pi, self.sensors.lidar_beams, endpoint=False)
        self._directions = np.column_stack((np.cos(self.angles), np.sin(self.angles), np.zeros(len(self.angles))))
        self._multi_ray_has_normal = "normal:" in mujoco.mj_multiRay.__doc__
        self.navigation = np.load(self.scene_dir / "navigation.npz")
        self.reset(spawn)

    def reset(self, spawn=None):
        spawn = np.asarray(self.metadata["spawn"] if spawn is None else spawn, float)
        if spawn.shape != (3,) or not np.isfinite(spawn).all():
            raise ValueError("spawn must be finite [world_x, world_y, yaw_radians]")
        if self.clearance_at(spawn[:2]) < self.robot.side / np.sqrt(2) + 0.08:
            raise ValueError("Spawn lacks floor support or clearance for the 0.6 m robot")
        mujoco.mj_resetData(self.model, self.data)
        q = self._base_qpos
        self.data.qpos[q:q+3] = [spawn[0], spawn[1], 0.006]
        self.data.qpos[q+3:q+7] = [np.cos(spawn[2]/2), 0, 0, np.sin(spawn[2]/2)]
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(0.5/self.robot.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0
        mujoco.mj_forward(self.model, self.data)

    def clearance_at(self, xy):
        ij = np.floor((np.asarray(xy)-self.navigation["origin"])/self.navigation["resolution"]).astype(int)
        grid = self.navigation["clearance"]
        return float(grid[tuple(ij)]) if np.all(ij >= 0) and np.all(ij < grid.shape) else 0.0

    def set_velocity(self, linear, angular):
        """Command body velocity [m/s, rad/s] via two wheel velocity actuators."""
        if not np.isfinite([linear, angular]).all():
            raise ValueError("Velocity must be finite")
        speeds = np.array([linear-angular*self.robot.wheel_track/2,
                           linear+angular*self.robot.wheel_track/2]) / self.robot.wheel_radius
        speeds /= max(1.0, np.max(np.abs(speeds))/self.robot.max_wheel_speed)
        self.data.ctrl[:] = speeds
        return speeds.copy()

    def step(self, duration=0.02):
        steps = round(duration/self.robot.timestep)
        if steps < 1 or not np.isclose(steps*self.robot.timestep, duration, atol=1e-10):
            raise ValueError("duration must be a positive integer multiple of the physics timestep")
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        # mj_step leaves kinematic quantities at the pre-integration state.
        mujoco.mj_forward(self.model, self.data)

    @property
    def world_from_base(self):
        return transform(self.data.xpos[self.body_id], self.data.xmat[self.body_id])

    @property
    def pose2d(self):
        t = self.world_from_base
        return np.array([t[0, 3], t[1, 3], np.arctan2(t[1, 0], t[0, 0])])

    def graphmap_pose(self):
        """Optional graphmap integration; lazy import keeps the core portable."""
        from graphmap.pose import Pose
        return Pose.from_matrix(self.world_from_base, wrd_frame="world", ego_frame="base_footprint")

    def lidar(self):
        t = transform(self.data.site_xpos[self.lidar_id], self.data.site_xmat[self.lidar_id])
        directions = np.ascontiguousarray(self._directions @ t[:3, :3].T).ravel()
        ids = np.full(len(self.angles), -1, dtype=np.int32)
        distances = np.full(len(self.angles), -1.0)
        args = [self.model, self.data, t[:3, 3].copy(), directions, self._ray_groups, True, -1, ids, distances]
        if self._multi_ray_has_normal:
            args.append(None)
        mujoco.mj_multiRay(*args, len(self.angles), self.sensors.lidar_max)
        distances[(distances < self.sensors.lidar_min) | (distances > self.sensors.lidar_max)] = np.inf
        return LaserScan(float(self.data.time), self.angles.copy(), distances, t)

    def rgbd(self):
        s = self.sensors
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=s.height, width=s.width)
        renderer = self._renderer
        renderer.disable_depth_rendering()
        renderer.update_scene(self.data, camera="rgbd", scene_option=self._render_options)
        rgb = renderer.render().copy()
        try:
            renderer.enable_depth_rendering()
            depth = renderer.render().copy()
        finally:
            renderer.disable_depth_rendering()
        depth[(depth < s.depth_min) | (depth > s.depth_max)] = np.nan
        f = s.height/(2*np.tan(np.deg2rad(s.vertical_fov)/2))
        K = np.array([[f, 0, (s.width-1)/2], [0, f, (s.height-1)/2], [0, 0, 1]])
        # OpenGL camera: right/up/back -> optical: right/down/forward.
        rotation = self.data.cam_xmat[self.camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
        return RGBDFrame(float(self.data.time), rgb, depth, K,
                         transform(self.data.cam_xpos[self.camera_id], rotation))

    def save_snapshot(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        scan, frame = self.lidar(), self.rgbd()
        from PIL import Image
        Image.fromarray(frame.rgb).save(directory / "rgb.png")
        np.save(directory / "depth_m.npy", frame.depth)
        np.savez_compressed(directory / "sensors.npz", time=scan.time, angles=scan.angles, ranges=scan.ranges,
                            K=frame.K, world_from_base=self.world_from_base,
                            world_from_lidar=scan.world_from_lidar,
                            world_from_camera_optical=frame.world_from_camera_optical)
        info = dict(time=scan.time, depth_unit="m", depth_type="optical_z", invalid_depth="NaN",
                    lidar_unit="m", invalid_lidar="+inf", K=frame.K.tolist(),
                    world_from_source=self.metadata["world_from_source"],
                    world_from_base=self.world_from_base.tolist(),
                    world_from_lidar=scan.world_from_lidar.tolist(),
                    world_from_camera_optical=frame.world_from_camera_optical.tolist())
        (directory / "metadata.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
        return scan, frame

    def overview(self):
        """Render a cutaway view of the robot without changing sensor visibility."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.sensors.height, width=self.sensors.width)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [*self.pose2d[:2], .4]
        camera.distance = 3.2
        camera.azimuth = 135
        camera.elevation = -50
        options = mujoco.MjvOption()
        options.geomgroup[:] = [1, 1, 0, 0, 0, 0]
        options.sitegroup[:] = 0
        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(self.data, camera=camera, scene_option=options)
        return self._renderer.render().copy()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self.navigation.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
