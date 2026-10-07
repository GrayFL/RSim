import asyncio
import os
from pathlib import Path
import time

import numpy as np
import pytest

from rsim import Component, Runtime, Map, ProcessPlacement, LocalPlacement
from rsim import CommandSink, VelocityCommand, Connect, CommandEnvelope, CommandRejected
from rsim import PortNotBound


class MultiOutput(Component):
    def __init__(self, source=None):
        super().__init__(inputs=(() if source is None else (source,)))
        self.array = self.signal("array", history=4, clock="test")
        self.mirror = self.signal("mirror", history=4, clock="test")
        self.info = self.signal("info", clock="test")
        self.previous = 0
        self.opens = 0

    async def open(self):
        self.opens += 1
        self.task("compute", self.compute, hz=20)

    async def compute(self):
        if self.inputs:
            frame = await self.inputs[0].get(after=self.previous)
            self.previous = frame.sequence
            array = frame.data
        else:
            array = np.arange(100, dtype="f8")
        stamp = time.monotonic_ns()
        first = await self.array.publish(array, stamp_ns=stamp, clock="test")
        second = await self.mirror.publish(array, stamp_ns=stamp, clock="test")
        await self.info.publish({"pid": os.getpid(), "opens": self.opens,
                                 "source_inode": (os.stat(array.filename).st_ino
                                                  if isinstance(array, np.memmap) else None),
                                 "same_local_array": first.data is second.data},
                                stamp_ns=stamp, clock="test")


def test_demand_is_exact_and_dependency_only_placement_has_no_exporter():
    async def run():
        source = MultiOutput()
        wrapper = Component()
        wrapper.expose('pose', source.info)
        # The wrapper has one public alias; arrays remain private to source.
        async with Runtime(wrapper, placement={wrapper: ProcessPlacement('wrapped')}) as runtime:
            info = await wrapper.pose.get(timeout=10)
            assert info.sample_id.canonical_port_id == 'info'
            assert len(runtime._binding_plan.channels) == 1
            assert not list(runtime._binding_plan.directory.rglob('*.npy'))
            with pytest.raises(PortNotBound):
                await source.array.get(timeout=1)
        lifetime = Component(source)
        async with Runtime(lifetime, placement={source: ProcessPlacement('resource')}) as runtime:
            assert runtime._binding_plan.channels == {}
            with pytest.raises(PortNotBound):
                await source.info.get(timeout=1)
    asyncio.run(run())


def test_same_graph_multioutput_process_and_fanout_share_one_producer():
    async def run():
        source = MultiOutput()
        left, right = Map(source.array, lambda array: array), Map(source.array, lambda array: array)
        runtime = Runtime(left.output, right.output, source.info, source.mirror,
                          placement={source: ProcessPlacement("compute")})
        previous_info = 0
        for _ in range(2):
            async with runtime:
                a, b, info = await asyncio.gather(left.get(timeout=15), right.get(timeout=15),
                                                   source.info.get(timeout=15))
                mirror = await source.mirror.get(timeout=15)
                assert info.data["pid"] != os.getpid() and info.data["opens"] == 1
                assert info.data["same_local_array"]
                assert info.sequence > previous_info
                previous_info = info.sequence
                assert source.opens == 0
                assert a.data is b.data
                # Output streams carry independent cursors; find the matching
                # publication timestamp rather than compare their sequence IDs.
                matched = await source.array.get(timestamp_ns=mirror.stamp_ns, clock="test")
                assert os.stat(matched.data.filename).st_ino == os.stat(mirror.data.filename).st_ino
                directory = runtime._binding_plan.directory
            assert not directory.exists()
            assert source._binding is None
    asyncio.run(run())


def test_internal_component_boundaries_do_not_materialize_arrays():
    class Pipeline(Component):
        def __init__(self):
            from rsim.components.synthetic import CounterArray
            raw = CounterArray()
            transformed = Map(raw.output, lambda array: {"already_shared": isinstance(array, np.memmap),
                                                        "values": array})
            super().__init__(raw, transformed)
            self.result = self.signal("result")
            self.result._target = transformed.output

    async def run():
        pipeline = Pipeline()
        async with Runtime(pipeline.result, placement={pipeline: ProcessPlacement("pipeline")}):
            frame = await pipeline.result.get(timeout=15)
            assert frame.data["already_shared"] is False
            assert isinstance(frame.data["values"], np.memmap)
    asyncio.run(run())


def test_process_local_resource_follows_each_owner_without_duplicating_a_source():
    class Context(Component):
        process_local = True

        async def open(self):
            self.pid = os.getpid()

    class Owner(Component):
        def __init__(self, context):
            super().__init__(context)
            self.info = self.signal("info")

        async def open(self):
            await self.info.publish(self.dependencies[0].pid, stamp_ns=1, clock="test")

    async def run():
        context = Context(key="executor")
        a, b, c = Owner(context), Owner(context), Owner(context)
        async with Runtime(a.info, b.info, c.info, placement={a: ProcessPlacement("a"),
                                                              b: ProcessPlacement("b")}):
            frames = await asyncio.gather(*(item.info.get(timeout=15) for item in (a, b, c)))
            pids = [frame.data for frame in frames]
            assert len(set(pids)) == 3 and pids[-1] == os.getpid()
    asyncio.run(run())


def test_closed_process_adapter_can_move_to_another_placement():
    from rsim import ProcessSensor
    from rsim.components.synthetic import CounterArray
    async def run():
        source = ProcessSensor(lambda: CounterArray())
        async with Runtime(source.output):
            await source.get(timeout=15)
        async with Runtime(source.output, placement={source: ProcessPlacement("nested")}):
            assert isinstance((await source.get(timeout=15)).data, np.memmap)
    asyncio.run(run())


def test_feedback_signals_cross_two_workers():
    class Feedback(Component):
        def __init__(self, seed):
            super().__init__()
            self.output = self.signal("value")
            self.seed, self.previous = seed, 0

        async def open(self):
            if self.seed:
                await self.output.publish(0, stamp_ns=0, clock="simulation")
            self.task("step", self.step, hz=20)

        async def step(self):
            frame = await self.inputs[0].get(after=self.previous)
            self.previous = frame.sequence
            await self.output.publish(frame.data + 1, stamp_ns=frame.stamp_ns + 1, clock=frame.clock)

    async def run():
        a, b = Feedback(True), Feedback(False)
        a.inputs, b.inputs = (b.output,), (a.output,)
        async with Runtime(b.output, placement={a: ProcessPlacement("a"), b: ProcessPlacement("b")}):
            frame = await b.output.get(timeout=15)
            for _ in range(3):
                frame = await b.output.get(after=frame.sequence, timeout=2)
            assert frame.data >= 7
    asyncio.run(run())


class Actuator(Component):
    def __init__(self):
        super().__init__()
        self.feedback = self.signal("feedback", history=32, clock="host:monotonic")
        self.velocity = CommandSink(self, "velocity", self.apply, fallback=VelocityCommand())

    async def open(self):
        await self.feedback.publish({"ready": True, "pid": os.getpid()},
                                    stamp_ns=time.monotonic_ns(), clock="host:monotonic")

    async def apply(self, envelope):
        await self.feedback.publish({"value": envelope.value, "pid": os.getpid()},
                                    stamp_ns=time.monotonic_ns(), clock="host:monotonic")


def test_command_sink_in_worker_validates_and_stops_at_provider():
    async def run():
        device = Actuator()
        async with Runtime(device, device.velocity, placement={device: ProcessPlacement("device")}):
            initial = await device.feedback.get(timeout=15)
            assert initial.data["pid"] != os.getpid()
            await device.velocity.set(VelocityCommand(1), ttl=.15)
            accepted = await device.feedback.get(after=initial.sequence, timeout=1)
            assert accepted.data["value"] == VelocityCommand(1)
            stopped = await device.feedback.get(after=accepted.sequence, timeout=1)
            assert stopped.data["value"] == VelocityCommand()
            expired = CommandEnvelope(VelocityCommand(2), "test", "1", 1, time.monotonic_ns() - 1)
            with pytest.raises(CommandRejected, match="expired"):
                await device.velocity.set(expired)
    asyncio.run(run())


def test_remote_controller_can_connect_to_local_sink_without_changing_graph():
    async def run():
        source = Component()
        commands = source.signal("commands")
        device = Actuator()
        drive = Connect(commands, device.velocity)
        async with Runtime(drive, placement={drive: ProcessPlacement("controller"),
                                              device: LocalPlacement()}):
            initial = await device.feedback.get(timeout=1)
            # A sample can precede worker readiness; its lifetime must include
            # startup. Wait on supervisor readiness before producing commands.
            host = drive._binding.supervisor
            async with asyncio.timeout(10):
                while host.worker_pid is None:
                    await asyncio.sleep(.02)
            await commands.publish(VelocityCommand(1), stamp_ns=1, clock="test")
            accepted = await device.feedback.get(after=initial.sequence, timeout=2)
            assert accepted.data["value"] == VelocityCommand(1)
            assert accepted.data["pid"] == os.getpid()
    asyncio.run(run())


def test_local_input_to_two_process_consumers_materializes_once():
    async def run():
        source = Component()
        array = source.signal("array", clock="test")
        left, right = MultiOutput(array), MultiOutput(array)
        async with Runtime(left.info, right.info, left.array, placement={
                left: ProcessPlacement("left"), right: ProcessPlacement("right"), source: LocalPlacement()}):
            frame = await array.publish(np.arange(100, dtype="f8"), stamp_ns=1, clock="test")
            a, b = await asyncio.gather(left.info.get(timeout=15), right.info.get(timeout=15))
            assert a.data["pid"] != b.data["pid"] != os.getpid()
            assert a.data["source_inode"] == b.data["source_inode"] == os.stat(frame.data.filename).st_ino
            returned = await left.array.get(timeout=15)
            assert os.stat(returned.data.filename).st_ino == os.stat(frame.data.filename).st_ino
            assert not returned.data.flags.writeable
    asyncio.run(run())


def test_time_join_frames_and_arrays_keep_their_types_across_placement():
    from rsim import Synchronizer, Frame
    async def run():
        source = Component()
        a, b = source.signal("a", clock="robot"), source.signal("b", clock="robot")
        joined = Synchronizer(a=a, b=b, tolerance_ns=0)
        async with Runtime(joined.output, placement={joined: ProcessPlacement("join")}):
            values = np.arange(8, dtype="f8")
            await a.publish(values, stamp_ns=100, clock="robot")
            await b.publish(values, stamp_ns=100, clock="robot")
            frame = await joined.get(timeout=15)
            assert frame.stamp_ns == 100 and frame.clock == "robot"
            assert all(isinstance(value, Frame) for value in frame.data.values())
            a, b = frame.data["a"].data, frame.data["b"].data
            np.testing.assert_array_equal(a, values)
            assert os.stat(a.filename).st_ino == os.stat(b.filename).st_ino
    asyncio.run(run())


def test_native_and_ros2_placements_exchange_commands_and_feedback():
    pytest.importorskip("rclpy", exc_type=ImportError)
    async def run():
        source, device = Component(), Actuator()
        commands = source.signal("commands")
        drive = Connect(commands, device.velocity, ttl=.4)
        async with Runtime(drive, device.feedback, placement={
                device: ProcessPlacement("device", "cyclonedds"),
                drive: ProcessPlacement("controller", "ros2")}):
            initial = await device.feedback.get(timeout=15)
            async with asyncio.timeout(10):
                while drive._binding.supervisor.worker_pid is None:
                    await asyncio.sleep(.02)
            # Discovery may consume the first command's TTL. A control stream
            # sends fresh commands; no old command is made valid again.
            async with asyncio.timeout(8):
                while True:
                    await commands.publish(VelocityCommand(1), stamp_ns=1, clock="test")
                    try:
                        accepted = await device.feedback.get(after=initial.sequence, timeout=.05)
                    except TimeoutError:
                        if drive._binding._failure is not None:
                            raise drive._binding._failure
                        continue
                    break
            assert accepted.data["value"] == VelocityCommand(1)
            assert accepted.data["pid"] != os.getpid()
    asyncio.run(run())
