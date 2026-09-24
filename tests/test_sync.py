import asyncio
from fractions import Fraction
import pytest

from rsim import Component, Runtime, ClockDomain, ClockTransform, Synchronizer, HistoryMiss


def test_time_join_explicit_clock_conversion_nearest_and_interpolation():
    async def run():
        source = Component()
        image = source.signal("image", clock=ClockDomain("camera"))
        odom = source.signal("odom", clock="robot")
        cloud = source.signal("cloud", clock="robot")
        sync = Synchronizer(image=image, odom=odom, cloud=cloud, reference="image",
                            clock="robot", tolerance_ns=20,
                            transforms={"image": ClockTransform("camera", "robot", 100)},
                            interpolate={"odom": lambda a, b, fraction: a + (b - a) * fraction})
        async with Runtime(sync.output):
            await odom.publish(0.0, stamp_ns=100, clock="robot")
            await odom.publish(2.0, stamp_ns=120, clock="robot")
            point = await cloud.publish("points", stamp_ns=114, clock="robot")
            original = await image.publish("rgb", stamp_ns=10, clock="camera")
            frame = await sync.get(timeout=1)
            assert frame.stamp_ns == 110 and frame.clock == "robot"
            assert frame.data["odom"].data == 1.0
            assert frame.data["odom"].clock == "robot"
            assert frame.data["image"] is original
            assert frame.data["cloud"] is point
            with pytest.raises(HistoryMiss):
                await sync.join(1000, clock="robot", timeout=.01)
        assert ClockTransform("a", "b", 7, Fraction(1001, 1000)).convert(
            10**18, clock="a") == 1001000000000000007
    asyncio.run(run())


def test_no_implicit_cross_clock_join_and_no_extrapolation():
    async def run():
        source = Component()
        a, b = source.signal("a", clock="one"), source.signal("b", clock="two")
        with pytest.raises(ValueError, match="ClockTransform"):
            Synchronizer(a=a, b=b, tolerance_ns=10)
        dynamic = source.signal("dynamic")
        sync = Synchronizer(a=a, b=dynamic, tolerance_ns=100, wait_timeout=.01,
                            interpolate={"b": lambda a, b, t: a + (b - a) * t})
        async with Runtime(source):
            await a.publish(1, stamp_ns=50, clock="one")
            await dynamic.publish(2, stamp_ns=50, clock="two")
            with pytest.raises(ValueError, match="clock"):
                await sync.join(50, clock="one")
            dynamic._history.clear()
            await dynamic.publish(2, stamp_ns=40, clock="one")
            with pytest.raises(HistoryMiss):
                await sync.join(50, clock="one")
    asyncio.run(run())
