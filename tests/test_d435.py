import asyncio

import numpy as np
import pytest

from rsim import D435, Runtime, Sensor
from rsim.devices import _ImageStream
from rsim.ros import D435 as RosD435, Image


def test_d435_profiles_share_one_driver_and_match_topic_namespace():
    color = RosD435(stream="color", serial="012345", depth_profile="480,270,15")
    depth = RosD435(stream="depth", serial="012345", depth_profile="480X270X15")
    driver = color.children[1]
    assert driver.configuration() == depth.children[1].configuration()
    assert driver.parameters["serial_no"] == "_012345"
    namespace = driver.remappings["__ns"]
    name = driver.parameters["camera_name"]
    assert driver.remappings["__node"] == name
    assert color.topic == f"{namespace}/{name}/color/image_raw"
    assert depth.topic == f"{namespace}/{name}/depth/image_rect_raw"
    assert driver.parameters["depth_module.depth_profile"] == "480x270x15"

    first = D435(depth_profile="480,270,15").source.producer
    equivalent = D435(stream="color", depth_profile="480X270X15").source.producer
    conflict = D435(depth_profile="480x270x6").source.producer
    assert first.source_key == equivalent.source_key == conflict.source_key
    assert first.version == equivalent.version
    assert first.version != conflict.version


@pytest.mark.parametrize("profile", ["640x480", "640x480x0", "-1x480x15", None])
def test_d435_rejects_invalid_profiles_before_starting_a_worker(profile):
    with pytest.raises(ValueError, match="profile"):
        D435(depth_profile=profile)


def test_image_stream_ignores_repeated_bundle_samples_and_resets_on_reopen():
    async def run():
        source = Sensor()
        stream = _ImageStream(source, "depth", hz=200, history=2)
        pixels = np.ones((2, 3), dtype="u2")
        sample = {"data": Image(pixels, "16UC1", "depth"), "stamp_ns": 100,
                  "clock": "camera", "received_ns": 200}
        for _ in range(2):
            async with Runtime(stream):
                await source.publish({"depth": sample}, stamp_ns=1, clock="bundle")
                first = await stream.get(timeout=1)
                assert first.data.pixels is pixels
                assert (first.stamp_ns, first.clock, first.received_ns) == (100, "camera", 200)
                await source.publish({"depth": sample}, stamp_ns=2, clock="bundle")
                with pytest.raises(TimeoutError):
                    await stream.get(after=first.sequence, timeout=0.05)
                fresh = dict(sample, stamp_ns=101, received_ns=201)
                await source.publish({"depth": fresh}, stamp_ns=3, clock="bundle")
                second = await stream.get(after=first.sequence, timeout=1)
                assert second.sequence == first.sequence + 1
                assert second.stamp_ns == 101
                assert await stream.get(timestamp_ns=100, clock="camera") is first
    asyncio.run(run())
