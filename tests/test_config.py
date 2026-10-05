import asyncio
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("graphmap.pose")
pytest.importorskip("yaml")
from graphmap.pose import Pose
from rsim import Component, Runtime
from rsim.config import load_rig, read_config, Mount, UncalibratedMount


def test_reference_geometry_nested_reuse_and_unknown_mounts():
    class Source(Component):
        def __init__(self, **parameters):
            super().__init__()
            self.imu = self.signal("imu")
            self.opens = 0
        async def open(self):
            self.opens += 1
            await self.imu.publish(np.arange(3), stamp_ns=1, clock="test")

    async def run():
        rig = load_rig(Path(__file__).parents[1]/"configs/sensors.yaml", select="robot",
                       factories=dict.fromkeys(("hipnuc", "d435", "robinw"), Source))
        assert rig["robot"]["perception"]["imu"] is rig["inertial"]["imu"] is rig["imu"]
        rgb = Pose(position=[.12,.02,1.07], rotation=[0,1.9,1.8], wrd_frame="base_footprint", ego_frame="rgb")
        lidar = Pose(position=[.09,-.01,.96], rotation=[0,-.1,1.5], wrd_frame="base_footprint", ego_frame="lidar3d")
        assert rig["rgb"].T_base_sensor.allclose(rgb)
        assert rig.transform("rgb", "lidar3d").allclose((~rgb)*lidar)
        assert rig.transform("base_footprint", "rgb").allclose(rgb)
        with pytest.raises(UncalibratedMount):
            rig.transform("rgb", "imu")
        async with Runtime(rig):
            a = await rig["imu"].imu.get(timeout=1)
            b = await rig["inertial"]["imu"].imu.get(timeout=1)
            assert a is b and rig["imu"].source.opens == 1
    asyncio.run(run())


def test_config_selection_overrides_validation_and_estimates(tmp_path):
    import yaml
    config = {"rsim": {"sensors": {
        "imu": {"driver": "fake", "parameters": {"rate": 100}, "mount": {"status": "unknown"}},
        "offline": {"driver": "unavailable"}}, "assemblies": {"a": ["b"], "b": ["a"]}}}
    path = tmp_path/"sensors.yaml"
    path.write_text(yaml.safe_dump(config))
    called = []
    def factory(**kwargs):
        called.append(kwargs)
        return Component()
    rig = load_rig(path, select="imu", overrides={"imu": {"rate": 50}}, factories={"fake": factory})
    assert list(rig.sensors) == ["imu"] and called == [{"rate": 50}]
    with pytest.raises(ValueError, match="cyclic"):
        load_rig(path, select="a", factories={"fake": factory})
    with pytest.raises(ValueError, match="unknown sensor"):
        load_rig(path, select="missing", factories={"fake": factory})
    path.write_text("rsim: {}\nrsim: {}\n")
    with pytest.raises(ValueError, match="duplicate"):
        read_config(path)
    estimated = Mount("body", "imu", (0,0,1), (0,0,0), "estimated")
    with pytest.raises(UncalibratedMount):
        estimated.pose()
    assert estimated.pose(allow_estimated=True).position[2] == 1
    with pytest.raises(ValueError, match="complete"):
        Mount("body", "imu", (None,0,None), None, "measured")


def test_connection_only_recipe_does_not_load_serial_or_ros(tmp_path):
    import yaml
    path = tmp_path/"sensors.yaml"
    path.write_text(yaml.safe_dump({"rsim": {"sensors": {"imu": {
        "driver": "hipnuc", "parameters": {"port": "/dev/ttyUSB99", "mode": "ros2", "baudrate": 115200},
        "mount": {"status": "unknown"}}}}}))
    rig = load_rig(path, providers=False)
    assert rig["imu"].source.factory is None
    assert rig["imu"].source.source_key == "hipnuc:/dev/ttyUSB99"
