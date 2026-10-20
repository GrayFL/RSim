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
import trimesh

from rsim.adapters.mujoco import Simulation, SensorConfig
from rsim.adapters.mujoco.scene import prepare_scene


class SceneConversionTests(unittest.TestCase):
    def test_glb_floor_triangle_texture_and_unicode_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "中文场景"
            root.mkdir()
            room = trimesh.Scene()
            floor = trimesh.creation.box(extents=[4, 4, .2])
            floor.apply_translation([0, 0, -.1])
            room.add_geometry(floor)
            wall = trimesh.creation.box(extents=[.1, 4, 2.5])
            wall.apply_translation([2, 0, 1.25])
            room.add_geometry(wall)
            # A UV triangle exercises MuJoCo's minimum-four-vertices constraint.
            from PIL import Image
            triangle = trimesh.Trimesh(vertices=[[-1, 1, .5], [-.5, 1, .5], [-1, 1, 1]], faces=[[0, 1, 2]], process=False)
            triangle.visual = trimesh.visual.TextureVisuals(uv=[[0, 0], [1, 0], [0, 1]], image=Image.new("RGB", (4, 4), "red"))
            room.add_geometry(triangle)
            glb = root / "房间.glb"
            glb.write_bytes(room.export(file_type="glb"))
            cache = prepare_scene(glb, root / "缓存", up_axis="z", pitch=.1)
            info = json.loads((cache / "metadata.json").read_text(encoding="utf-8"))
            # Detect the upper floor face, not the underside of the solid slab.
            self.assertAlmostEqual(info["selected_floor"], 0, places=5)
            self.assertEqual(prepare_scene(glb, root / "缓存", up_axis="z", pitch=.1), cache)
            self.assertGreater(len(list(cache.glob("*.png"))), 0)
            with Simulation(cache, spawn=[0, 0, 0], sensors=SensorConfig(width=80, height=60)) as sim:
                self.assertGreaterEqual(sim.model.nmesh, 3)
                self.assertAlmostEqual(sim.lidar().ranges[180], 1.95, delta=.01)
                frame = sim.rgbd()
                self.assertAlmostEqual(float(frame.depth[30, 40]), 1.95, delta=.01)
                sim.set_velocity(.3, 0)
                sim.step(1)
                self.assertGreater(sim.pose2d[0], .2)


if __name__ == "__main__":
    unittest.main()
