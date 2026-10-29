import asyncio
from fractions import Fraction
import time

import cloudpickle
import pytest

from rsim import (Component, Runtime, CommandSink, VelocityCommand, SignalPacer,
                  TickSource, PacerClosed, PacerReset, ComponentError)


def test_task_requires_exactly_one_policy_and_releases_subscription():
    class Consumer(Component):
        async def open(self):
            self.source = TickSource(clock='simulation')
            self.subscription = self.source.subscribe()
            self.calls = 0
            with pytest.raises(ValueError, match='exactly one'):
                self.task('bad', self.run, hz=1, pacer=self.subscription)
            with pytest.raises(ValueError, match='exactly one'):
                self.task('missing', self.run)
            self.task('events', self.run, pacer=self.subscription)

        async def run(self):
            self.calls += 1

    async def run():
        consumer = Consumer()
        async with Runtime(consumer):
            await consumer.source.publish(stamp_ns=10, episode=0, step_index=0)
            async with asyncio.timeout(1):
                while consumer.calls != 1:
                    await asyncio.sleep(.001)
            await asyncio.sleep(.03)
            assert consumer.calls == 1
        assert not consumer.source._subscriptions
        assert consumer.subscription._closed
    asyncio.run(run())


def test_latest_has_independent_cursors_and_bounded_pending_ticks():
    async def run():
        source = TickSource(clock='simulation')
        first, second = source.subscribe(), source.subscribe()
        for step in range(10):
            await source.publish(stamp_ns=step * 10, episode=0, step_index=step)
            assert (await first.tick()).step_index == step
        assert (await second.tick()).step_index == 9
        assert first.skipped == 0 and second.skipped == 9
        await source.aclose()
        with pytest.raises(PacerClosed):
            await first.tick()
    asyncio.run(run())


def test_every_subscription_backpressures_all_publications_and_close_wakes_writer():
    async def run():
        source = TickSource(clock='simulation')
        first, second = (source.subscribe(policy='every', capacity=1) for _ in range(2))
        await source.publish(stamp_ns=0, episode=0, step_index=0)
        writer = asyncio.create_task(source.publish(stamp_ns=1, episode=0, step_index=1))
        await asyncio.sleep(.01)
        assert not writer.done()
        assert (await first.tick()).step_index == 0
        await asyncio.sleep(.01)
        assert not writer.done()
        assert (await second.tick()).step_index == 0
        await asyncio.wait_for(writer, 1)
        assert (await first.tick()).step_index == 1
        writer = asyncio.create_task(source.publish(stamp_ns=2, episode=0, step_index=2))
        await second.aclose()
        await asyncio.wait_for(writer, 1)
        assert (await first.tick()).step_index == 2
        await source.aclose()
    asyncio.run(run())


def test_rational_frequency_does_not_accumulate_rounding_and_reset_isolated():
    async def run():
        source = TickSource(clock='simulation')
        subscription = source.subscribe(hz=30)
        stamps = []
        for step in range(5001):  # 500 Hz physics, 30 Hz consumer, ten seconds.
            stamp = step * 2_000_000
            await source.publish(stamp_ns=stamp, episode=0, step_index=step)
            if subscription._pending:
                stamps.append((await subscription.tick()).stamp_ns)
        assert len(stamps) == 301
        for index, stamp in enumerate(stamps):
            exact = Fraction(index * 1_000_000_000, 30)
            assert 0 <= stamp - exact < 2_000_000
        await source.publish(stamp_ns=0, episode=1, step_index=0)
        reset = await subscription.tick()
        assert reset.episode == 1 and reset.stamp_ns == 0
        with pytest.raises(ValueError, match='advance'):
            await source.publish(stamp_ns=0, episode=1, step_index=0)
    asyncio.run(run())


def test_source_failure_wakes_all_waiters_and_tick_sources_cannot_be_deployed():
    async def run():
        source = TickSource(clock='simulation')
        subscriptions = [source.subscribe() for _ in range(3)]
        waits = [asyncio.create_task(subscription.tick()) for subscription in subscriptions]
        with pytest.raises(TypeError, match='worker-local'):
            cloudpickle.dumps(source)
        await source.aclose(RuntimeError('physical source failed'))
        errors = await asyncio.gather(*waits, return_exceptions=True)
        assert all(isinstance(error, PacerClosed) for error in errors)
        assert all(str(error.__cause__) == 'physical source failed' for error in errors)
    asyncio.run(run())


def test_signal_pacer_uses_native_time_and_shutdown_does_not_hang():
    class Consumer(Component):
        def __init__(self, signal):
            super().__init__(inputs=(signal,))
            self.output = self.signal('output', clock='simulation')

        async def open(self):
            self.pacer = SignalPacer(self.inputs[0])
            self.task('copy', self.copy, pacer=self.pacer)

        async def copy(self):
            tick = self.pacer.last_tick
            await self.output.publish(tick.step_index, stamp_ns=tick.stamp_ns, clock=tick.clock)

    async def run():
        source = Component()
        signal = source.signal('sample', clock='simulation')
        consumer = Consumer(signal)
        async with Runtime(consumer):
            await signal.publish(42, stamp_ns=123, clock='simulation', metadata={'episode': 3, 'step_index': 7})
            result = await consumer.output.get(timeout=1)
            assert result.stamp_ns == 123 and result.data == 7
        assert consumer.pacer._closed
    asyncio.run(run())


def test_host_deadman_expires_while_environment_pacer_is_paused():
    class Actuator(Component):
        def __init__(self):
            super().__init__()
            self.applied = None
            self.velocity = CommandSink(self, 'velocity', self.apply, fallback=VelocityCommand())

        async def open(self):
            self.events = TickSource(clock='paused')
            self.task('environment', self.idle, pacer=self.events.subscribe())

        async def idle(self):
            pass

        async def apply(self, envelope):
            self.applied = envelope.value

    async def run():
        actuator = Actuator()
        async with Runtime(actuator, actuator.velocity):
            await actuator.velocity.set(VelocityCommand(.2), ttl=.05)
            assert actuator.applied.linear_x == .2
            await asyncio.sleep(.1)
            assert actuator.applied == VelocityCommand()
            assert actuator.events.last_tick is None
    asyncio.run(run())


def test_reset_bypasses_old_backpressure_and_invalidates_pending_publication():
    async def run():
        source = TickSource(clock='simulation')
        subscription = source.subscribe(policy='every')
        await source.publish(stamp_ns=0, episode=0, step_index=0)
        blocked = asyncio.create_task(source.publish(stamp_ns=1, episode=0, step_index=1))
        await asyncio.sleep(0)
        reset = await asyncio.wait_for(source.publish(stamp_ns=0, episode=1, step_index=0), .2)
        with pytest.raises(PacerReset):
            await blocked
        assert await subscription.tick() is reset
        with pytest.raises(PacerReset, match='retired'):
            await source.publish(stamp_ns=2, episode=0, step_index=2)
        assert source.last_tick is reset
        await source.aclose()
    asyncio.run(run())
