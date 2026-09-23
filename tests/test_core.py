import asyncio

import pytest

from rsim import HistoryMiss, Metronome, Reference, Runtime, Sensor, SensorError


class Counter(Sensor):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.opens = self.closes = self.value = 0

    async def open(self):
        self.opens += 1
        self.task("produce", self.produce, hz=100)

    async def produce(self):
        self.value += 1
        await self.publish(self.value, stamp_ns=self.value * 100, clock="test")

    async def close(self):
        self.closes += 1


def test_shared_graph_history_and_shutdown():
    async def run():
        source = Counter(history=2)
        left, right = Sensor(source), Sensor(source)
        root = Sensor(left, right)
        async with Runtime(root):
            assert left.children[0] is right.children[0]
            assert source.opens == 1
            first = await source.get(timeout=1)
            second = await source.get(after=first.sequence, timeout=1)
            assert second.sequence > first.sequence
            assert await source.get(timestamp_ns=second.stamp_ns, clock="test") is second
            with pytest.raises(HistoryMiss):
                await source.get(timestamp_ns=second.stamp_ns, clock="unix")
            third = await source.get(after=second.sequence, timeout=1)
            with pytest.raises(HistoryMiss):
                await source.get(timestamp_ns=first.stamp_ns, clock="test")
            assert third.clock == "test"
        assert source.closes == 1
        assert all(not sensor._tasks for sensor in (root, left, right, source))
        with pytest.raises(SensorError):
            await source.get()
    asyncio.run(run())


def test_key_dedup_and_conflicts():
    async def run():
        first, second = Counter(key="same"), Counter(key="same")
        root = Sensor(Sensor(first), Sensor(second))
        async with Runtime(root):
            assert root.children[1].children[0] is first
            assert second.opens == 0
            assert await second.get(timeout=1) is await first.get()
        with pytest.raises(ValueError, match="conflicting"):
            async with Runtime(Counter(key="same"), Counter(key="same", history=1)):
                pass
    asyncio.run(run())


def test_cycles_rejected_before_resources_start():
    async def run():
        a, b = Counter(), Sensor()
        a.children = (b,)
        b.children = (a,)
        with pytest.raises(ValueError, match="cyclic"):
            async with Runtime(a):
                pass
        assert a.opens == 0
    asyncio.run(run())


def test_reference_cycle_and_repeated_runtime():
    async def run():
        source = Counter(key="counter")
        duplicate = Counter(key="counter")
        reference = Reference(source)
        source.children = (reference,)
        parent = Sensor(source, duplicate, duplicate)
        runtime = Runtime(parent)
        for _ in range(2):
            async with runtime:
                assert await reference.get(timeout=1) is await duplicate.get()
            assert parent.children == (source, duplicate, duplicate)
        assert source.opens == source.closes == 2
    asyncio.run(run())


def test_start_failure_rolls_back_and_get_timeout():
    class Broken(Counter):
        async def open(self):
            await super().open()
            raise RuntimeError("startup")

    async def run():
        source, broken = Counter(), Broken()
        with pytest.raises(RuntimeError, match="startup"):
            async with Runtime(source, broken):
                pass
        assert source.closes == broken.closes == 1
        assert not source._tasks and not broken._tasks
        empty = Sensor()
        async with Runtime(empty):
            with pytest.raises(TimeoutError):
                await empty.get(timeout=0.02)
    asyncio.run(run())


def test_watchdog_propagates_failure_to_parent_waiter():
    class Broken(Sensor):
        async def open(self):
            self.task("failure", self.fail, hz=100)

        async def fail(self):
            raise RuntimeError("compute failed")

    async def run():
        child = Broken()
        root = Sensor(child)
        async with Runtime(root):
            with pytest.raises(SensorError):
                await root.get(timeout=1)
            assert isinstance(child._failure, RuntimeError)
    asyncio.run(run())


def test_cancellation_cleans_resources_and_wakes_waiters():
    async def run():
        sensor = Counter()
        entered = asyncio.Event()

        async def owner():
            async with Runtime(sensor):
                entered.set()
                await asyncio.Future()

        task = asyncio.create_task(owner())
        await entered.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert sensor.closes == 1 and not sensor._tasks
        empty = Sensor()
        async with Runtime(empty):
            waiter = asyncio.create_task(empty.get())
            await asyncio.sleep(0)
        with pytest.raises(SensorError):
            await waiter
    asyncio.run(run())


def test_metronome_skips_overruns():
    async def run():
        clock = Metronome(100)
        await clock.tick()
        await asyncio.sleep(0.04)
        await clock.tick()
        assert clock.missed >= 3
        assert clock.deadline >= asyncio.get_running_loop().time() - 0.01
    asyncio.run(run())
    for hz in (0, -1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            Metronome(hz)


def test_metered_services_and_pending_call_shutdown():
    class Service(Sensor):
        async def open(self):
            self.times = []
            self.service("double", self.double, hz=50, capacity=2)

        async def double(self, value):
            self.times.append(asyncio.get_running_loop().time())
            return value * 2

    async def run():
        sensor = Service()
        async with Runtime(sensor):
            values = await asyncio.gather(*(sensor.call("double", v, timeout=1) for v in range(3)))
            assert values == [0, 2, 4]
            assert sensor.times[-1] - sensor.times[0] >= 0.03
            pending = asyncio.create_task(sensor.call("double", 42))
            await asyncio.sleep(0)
        with pytest.raises(SensorError):
            await pending
    asyncio.run(run())


def test_service_backpressure_waiters_all_exit_on_shutdown():
    class Blocked(Sensor):
        async def open(self):
            async def block():
                await asyncio.Future()
            self.service("block", block, hz=100, capacity=1)

    async def run():
        sensor = Blocked()
        async with Runtime(sensor):
            calls = [asyncio.create_task(sensor.call("block")) for _ in range(8)]
            await asyncio.sleep(0.03)
        results = await asyncio.wait_for(asyncio.gather(*calls, return_exceptions=True), 1)
        assert all(isinstance(error, SensorError) for error in results)
    asyncio.run(run())
