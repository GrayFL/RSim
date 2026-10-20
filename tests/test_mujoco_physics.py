"""Physics and sensor regression tests in an analytic 4 m room."""
import json
from pathlib import Path
import tempfile
import unittest

import pytest
pytest.importorskip('mujoco')
pytest.importorskip('trimesh')
pytest.importorskip('scipy')
pytest.importorskip('PIL')

import numpy as np

from rsim.adapters.mujoco.config import SensorConfig
from rsim.adapters.mujoco.scene import rectangles
from rsim.adapters.mujoco.simulation import Simulation


def make_room(directory):
    directory = Path(directory)
    (directory / "scene.xml").write_text('''<mujoco><asset/><worldbody>
      <geom name="floor" type="box" pos="0 0 -0.1" size="2 2 0.1" group="0" contype="1" conaffinity="2"/>
      <geom name="front" type="box" pos="2 0 1.25" size="0.05 2 1.25" group="0" contype="1" conaffinity="2" rgba="0.9 0.2 0.1 1"/>
      <geom name="back" type="box" pos="-2 0 1.25" size="0.05 2 1.25" group="0" contype="1" conaffinity="2"/>
      <geom name="left" type="box" pos="0 2 1.25" size="2 0.05 1.25" group="0" contype="1" conaffinity="2"/>
      <geom name="right" type="box" pos="0 -2 1.25" size="2 0.05 1.25" group="0" contype="1" conaffinity="2"/>
    </worldbody></mujoco>''', encoding="utf-8")
    (directory / "metadata.json").write_text(json.dumps(dict(spawn=[0, 0, 0], world_from_source=np.eye(4).tolist())), encoding="utf-8")
    grid = np.ones((80, 80), bool)
    from scipy.ndimage import distance_transform_edt
    clearance = distance_transform_edt(np.pad(grid, 1))[1:-1, 1:-1]*.05
    np.savez_compressed(directory / "navigation.npz", floor=grid, obstacles=~grid, clearance=clearance, origin=[-2, -2], resolution=.05)


class SimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        make_room(cls.tmp.name)
        cls.sim = Simulation(cls.tmp.name, sensors=SensorConfig(width=160, height=120))

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()
        cls.tmp.cleanup()

    def setUp(self):
        self.sim.reset([0, 0, 0])

    def test_dimensions_and_extrinsics(self):
        sim = self.sim
        np.testing.assert_allclose(sim.model.geom("chassis").size[:2]*2, [.6, .6])
        t_base_lidar = np.linalg.inv(sim.world_from_base) @ sim.lidar().world_from_lidar
        np.testing.assert_allclose(t_base_lidar[:3, 3], [0, 0, .4], atol=1e-9)
        frame = sim.rgbd()
        t = np.linalg.inv(sim.world_from_base) @ frame.world_from_camera_optical
        np.testing.assert_allclose(t[:3, 3], [0, 0, 1], atol=1e-9)
        np.testing.assert_allclose(t[:3, :3], [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-9)

    def test_lidar_hits_visual_walls_and_excludes_robot(self):
        scan = self.sim.lidar()
        np.testing.assert_allclose(scan.ranges[[0, 90, 180, 270]], 1.95, atol=.005)
        self.assertEqual(len(scan.points()), 360)
        self.assertAlmostEqual(scan.time, self.sim.data.time)

    def test_rgbd_metric_depth_and_unprojection(self):
        frame = self.sim.rgbd()
        self.assertEqual(frame.rgb.shape, (120, 160, 3))
        self.assertEqual(frame.rgb.dtype, np.uint8)
        self.assertAlmostEqual(float(frame.depth[60, 80]), 1.95, delta=.01)
        points, colors = frame.point_cloud(stride=10, world=True)
        # Every valid pixel intersects the frontal wall in this narrow room view.
        self.assertGreater(len(points), 100)
        np.testing.assert_allclose(points[:, 0], 1.95, atol=.015)
        self.assertEqual(points.shape, colors.shape)
        self.assertGreater(float(frame.rgb[:, :, 0].mean()), float(frame.rgb[:, :, 1].mean()))

    def test_forward_driven_by_wheels(self):
        sim = self.sim
        sim.set_velocity(.3, 0)
        sim.step(1.0)
        self.assertGreater(sim.pose2d[0], .20)
        self.assertLess(sim.pose2d[0], .36)
        self.assertLess(abs(sim.pose2d[1]), .02)
        self.assertLess(abs(sim.pose2d[2]), .03)

    def test_positive_yaw_and_command_saturation(self):
        sim = self.sim
        np.testing.assert_allclose(sim.set_velocity(0, .8), [-2.12, 2.12])
        sim.step(1.0)
        self.assertGreater(sim.pose2d[2], .6)
        self.assertLess(np.linalg.norm(sim.pose2d[:2]), .03)
        self.assertLessEqual(np.max(np.abs(sim.set_velocity(8, 5))), 12)

    def test_wall_blocks_robot(self):
        sim = self.sim
        sim.set_velocity(.5, 0)
        sim.step(6.0)
        self.assertGreater(sim.pose2d[0], 1.5)
        self.assertLess(sim.pose2d[0], 1.67)
        self.assertGreater(sim.data.ncon, 0)
        self.assertTrue(np.isfinite(sim.data.qpos).all())
        self.assertLess(abs(sim.world_from_base[2, 3]), .025)

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            self.sim.set_velocity(np.nan, 0)
        with self.assertRaises(ValueError):
            self.sim.step(.001)
        with self.assertRaises(ValueError):
            self.sim.reset([10, 10, 0])

    def test_rectangle_partition_preserves_holes(self):
        grid = np.random.default_rng(7).random((10, 12)) > .4
        counts = np.zeros(grid.shape, int)
        for x0, x1, y0, y1 in rectangles(grid):
            counts[x0:x1, y0:y1] += 1
        np.testing.assert_array_equal(counts, grid.astype(int))


if __name__ == "__main__":
    unittest.main()
