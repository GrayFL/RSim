"""Movable assets: unit conversion, reset indexing, contact and sensor tracking."""
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
trimesh = pytest.importorskip("trimesh")
pytest.importorskip("scipy")
pytest.importorskip("PIL")

from rsim.adapters.mujoco import Simulation, SensorConfig
from rsim.adapters.mujoco.scene import prepare_scene


def recipe(tmp_path):
    source = tmp_path / "彩色物体.glb"
    cube = trimesh.creation.box()
    cube.visual.vertex_colors = [200, 80, 25, 255]
    source.write_bytes(trimesh.Scene(cube).export(file_type="glb"))
    objects = []
    for name, position, scale, mass in (("prop", [1.1, 0, .015], 1.2, 3),
                                        ("other", [-1, 1.3, .015], .6, 2)):
        objects.append(dict(name=name, source=source.name, position=position, up_axis="y",
                            scale=scale, mass=mass,
                            colliders=[dict(type="box", pos=[0, 0, .5], size=[.5, .5, .5])]))
    config = dict(format="rsim-scene-v1", room=dict(size=[6, 5, 2.6]),
                  spawn=[-1.5, 0, 0], objects=objects)
    path = tmp_path / "房间.scene.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    return path, config


def test_dynamic_asset_floor_reset_and_rays(tmp_path):
    path, _ = recipe(tmp_path)
    directory = prepare_scene(path, tmp_path / "缓存")
    assert prepare_scene(path, tmp_path / "缓存") == directory
    assert prepare_scene(directory) == directory
    with Simulation(directory, sensors=SensorConfig(width=80, height=60)) as sim:
        assert sim.model.nq == 23  # two props + robot free joint + two wheels
        assert sim._base_qpos == 14  # props precede robot in XML
        assert sim.model.body("prop").mass[0] == pytest.approx(3)
        np.testing.assert_allclose(sim.pose2d, [-1.5, 0, 0], atol=.01)
        initial = sim.data.body("prop").xpos.copy()
        assert abs(initial[2]) < .01
        np.testing.assert_allclose(initial[:2], [1.1, 0], atol=.01)
        assert sim.lidar().ranges[180] == pytest.approx(2., abs=.02)
        assert sim.rgbd().depth[30, 40] == pytest.approx(2., abs=.03)
        # Raise the rigid body: it must fall under gravity and stop on the floor.
        q = sim.model.joint("prop_free").qposadr[0]
        sim.data.qpos[q+2] += .35
        sim.step(.1)
        assert .20 < sim.data.body("prop").xpos[2] < .35
        sim.step(1)
        assert abs(sim.data.body("prop").xpos[2]) < .015
        assert sim.data.ncon > 0
        # External force moves the body, and sensor rays follow its new pose.
        sim.data.xfrc_applied[sim.model.body("prop").id, :3] = [0, 80, 0]
        sim.step(.30)
        sim.data.xfrc_applied[:] = 0
        sim.step(.30)
        assert sim.data.body("prop").xpos[1] > .7
        assert sim.lidar().ranges[180] > 3.5
        sim.reset()
        np.testing.assert_allclose(sim.data.body("prop").xpos, initial, atol=1e-8)
        assert sim.lidar().ranges[180] == pytest.approx(2., abs=.02)
        assert np.isfinite(sim.data.qpos).all()


def test_recipe_changes_invalidate_cache_and_bad_placement_rejected(tmp_path):
    path, config = recipe(tmp_path)
    first = prepare_scene(path, tmp_path / "cache")
    config["objects"][0]["mass"] = 4
    path.write_text(json.dumps(config), encoding="utf-8")
    second = prepare_scene(path, tmp_path / "cache")
    assert first != second
    with Simulation(second, sensors=SensorConfig(width=80, height=60)) as sim:
        assert sim.model.body("prop").mass[0] == pytest.approx(4)
    config["objects"][0]["position"] = [8, 0, 0]
    path.write_text(json.dumps(config), encoding="utf-8")
    with pytest.raises(ValueError, match="outside the floor"):
        prepare_scene(path, tmp_path / "cache")
    config["objects"][0]["position"] = [-1.5, 0, 0]
    path.write_text(json.dumps(config), encoding="utf-8")
    with pytest.raises(ValueError, match="spawn lacks"):
        prepare_scene(path, tmp_path / "cache")


def test_concurrent_sensor_sizes_do_not_share_exported_world_xml(tmp_path, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Barrier
    from rsim.adapters.mujoco import simulation

    path, _ = recipe(tmp_path)
    directory = prepare_scene(path, tmp_path / "cache")
    assemble = simulation.build_model_xml
    barrier = Barrier(2)

    def interleaved(*args, **kwargs):
        result = assemble(*args, **kwargs)
        barrier.wait(timeout=10)  # both exports finish before either compilation
        return result

    monkeypatch.setattr(simulation, "build_model_xml", interleaved)

    def make(width):
        with Simulation(directory, sensors=SensorConfig(width=width, height=60)) as sim:
            return sim.model.vis.global_.offwidth

    with ThreadPoolExecutor(max_workers=2) as executor:
        assert list(executor.map(make, [80, 160])) == [80, 160]
