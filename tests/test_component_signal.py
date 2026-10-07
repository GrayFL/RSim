import asyncio
import numpy as np
import pytest

from rsim import Component, Signal, Runtime, Map, Bundle, ComponentError


class Outputs(Component):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.image = Signal(self, "image", history=2)
        self.state = Signal(self, "state")
        self.opens = self.closes = 0

    async def open(self):
        self.opens += 1

    async def close(self):
        self.closes += 1


def test_multioutput_fanout_roots_and_no_copy():
    async def run():
        source = Outputs()
        left = Map(source.image, lambda value: value, hz=100)
        right = Map(source.image, lambda value: value, hz=100)
        root = Bundle(left=left.output, right=right.output)
        assert not hasattr(source, "get")
        assert not hasattr(source.image, "open")
        assert left.dependencies == () and left.inputs == (source.image,)
        async with Runtime(root.output):
            payload = np.arange(12)
            frame = await source.image.publish(payload, stamp_ns=100, clock="camera")
            a, b = await asyncio.gather(left.get(timeout=1), right.get(timeout=1))
            assert a.data is b.data is payload
            assert await source.image.get() is frame
            assert (await root.get(timeout=1)).data["left"]["data"] is payload
            assert not isinstance(payload, np.memmap)
            assert source.opens == 1
        assert source.closes == 1
    asyncio.run(run())


def test_data_feedback_loop_is_independent_of_ownership():
    class Feedback(Component):
        def __init__(self):
            super().__init__()
            self.output = self.signal("value")
            self.previous = 0

        async def open(self):
            if self.seed:
                await self.output.publish(0, stamp_ns=0, clock="simulation")
            self.task("feedback", self.step, hz=100)

        async def step(self):
            frame = await self.inputs[0].get(after=self.previous)
            self.previous = frame.sequence
            await self.output.publish(frame.data + 1, stamp_ns=frame.stamp_ns + 1,
                                      clock=frame.clock)

    async def run():
        a, b = Feedback(), Feedback()
        a.inputs, b.inputs = (b.output,), (a.output,)
        a.seed, b.seed = True, False
        async with Runtime(b.output):
            frame = await b.output.get(timeout=1)
            for _ in range(3):
                frame = await b.output.get(after=frame.sequence, timeout=1)
            assert frame.data >= 7
        a.dependencies, b.dependencies = (b,), (a,)
        with pytest.raises(ValueError, match="ownership"):
            async with Runtime(a.output):
                pass
    asyncio.run(run())


def test_failure_and_close_wake_every_output_waiter():
    class Broken(Outputs):
        async def open(self):
            self.task("fail", self.fail, hz=100)

        async def fail(self):
            raise RuntimeError("failure")

    async def run():
        source = Broken()
        async with Runtime(source.state):
            results = await asyncio.gather(source.image.get(timeout=1), source.state.get(timeout=1),
                                           return_exceptions=True)
            assert all(isinstance(result, ComponentError) for result in results)
        source = Outputs()
        async with Runtime(source.image):
            tasks = [asyncio.create_task(output.get()) for output in source.outputs.values()]
            await asyncio.sleep(0)
        assert all(isinstance(result, ComponentError)
                   for result in await asyncio.gather(*tasks, return_exceptions=True))
    asyncio.run(run())


def test_alias_outputs_and_local_cursors_across_reopen():
    async def run():
        a, b = Outputs(key="one"), Outputs(key="one")
        for _ in range(2):
            async with Runtime(a.state, b.image):
                prior = a.image._sequence
                frame = await a.image.publish(1, stamp_ns=100, clock="robot")
                assert frame.sequence > prior
                assert await b.image.get() is frame
                assert b.opens == 0
        assert a.opens == a.closes == 2
        assert b._canonical is None
    asyncio.run(run())


def test_hard_dependencies_win_over_cyclic_data_startup_preferences():
    async def run():
        opened = []
        class Resource(Component):
            async def open(self):
                opened.append(self)
        resource = Resource()
        owner = Resource(resource)
        output = owner.signal("output")
        resource.inputs = (output,)
        async with Runtime(resource):
            assert opened == [resource, owner]
    asyncio.run(run())


def test_public_alias_preserves_frame_but_reopen_changes_publication_generation():
    async def run():
        source = Outputs()
        wrapper = Component()
        port = wrapper.expose('public', source.image)
        identity = None
        for _ in range(2):
            async with Runtime(port):
                frame = await source.image.publish(np.arange(3), stamp_ns=10, clock='test')
                assert await port.get() is frame
                assert frame.sample_id != identity
                assert frame.sample_id.publication_sequence == 1
                assert frame.sample_id.canonical_port_id == 'image'
                identity = frame.sample_id
                pending = asyncio.create_task(port.get(after=frame.sequence))
                await asyncio.sleep(0)
            with pytest.raises(ComponentError):
                await pending
    asyncio.run(run())
