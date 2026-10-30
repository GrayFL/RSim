"""Actual source cadence, event forwarding, and host-rate control boundaries."""
import asyncio
from fractions import Fraction
import time

import cloudpickle
import pytest

from rsim import (Component, Runtime, SignalPacer, RatePacer, FrameTickPacer, TickSource,
                  HistoryMiss, PacerClosed, PacerReset,
                  Connect, CommandSink, CommandEnvelope, VelocityCommand, ProcessPlacement, LocalPlacement)


async def publish(signal, step, *, episode=0, stamp=None, session='world'):
    return await signal.publish(step, stamp_ns=step * 2_000_000 if stamp is None else stamp,
                                clock='source', metadata=dict(session=session, episode=episode,
                                                              step_index=step))


def test_signal_rate_uses_source_nanoseconds_without_fractional_drift():
    async def run():
        owner = Component()
        output = owner.signal('state', clock='source')
        pacer = SignalPacer(output, hz=30, deduplicate=True)
        stamps = []
        async with Runtime(output):
            pending = asyncio.create_task(pacer.tick())
            for step in range(2501):  # Five source seconds at 500 Hz, without wall sleeps.
                await publish(output, step)
                async with asyncio.timeout(1):
                    while pacer.received <= step:
                        await asyncio.sleep(0)
                if pending.done():
                    stamps.append((await pending).stamp_ns)
                    pending = asyncio.create_task(pacer.tick())
            assert len(stamps) == 151
            for index, stamp in enumerate(stamps):
                assert 0 <= stamp - Fraction(index * 1_000_000_000, 30) < 2_000_000
            assert pacer.filtered == 2501 - 151
            assert pacer.skipped == 0
            await pacer.aclose()
            with pytest.raises(PacerClosed):
                await pending
    asyncio.run(run())


def test_source_delta_and_skip_counts_survive_coalescing_duplicates_and_reset():
    async def run():
        owner = Component()
        output = owner.signal('state', clock='source')
        pacer = SignalPacer(output, deduplicate=True)
        async with Runtime(output):
            first = await publish(output, 0)
            await pacer.tick()
            assert pacer.last_frame is first and pacer.elapsed_ns is None
            for step in range(1, 5):
                await publish(output, step)
            await pacer.tick()
            assert pacer.elapsed_ns == 8_000_000 and pacer.skipped == 3
            consumed = pacer.last_frame
            await publish(output, 4)
            pending = asyncio.create_task(pacer.tick())
            async with asyncio.timeout(1):
                while pacer.duplicates != 1:
                    await asyncio.sleep(0)
            assert not pending.done() and pacer.last_frame is consumed
            await publish(output, 9)
            await pending
            assert pacer.elapsed_ns == 10_000_000
            assert pacer.skipped == 3  # The duplicate was observed, not a dropped publication.
            assert pacer.source_skipped == 7
            await publish(output, 0, episode=1)
            await pacer.tick()
            assert pacer.elapsed_ns is None and pacer.resets == 1
            await publish(output, 10, episode=0)
            with pytest.raises(PacerReset, match='retired'):
                await pacer.tick()
            await pacer.aclose()
    asyncio.run(run())


def test_event_pacer_delivers_distinct_events_with_the_same_source_step():
    async def run():
        owner = Component()
        output = owner.signal('commands', clock='source')
        pacer = SignalPacer(output)
        async with Runtime(output):
            first = await publish(output, 3)
            await pacer.tick()
            second = await publish(output, 3)
            assert pacer.last_frame is first  # No callback can silently change its selected snapshot.
            await pacer.tick()
            assert pacer.last_frame is second
            assert pacer.delivered == 2 and pacer.duplicates == 1
            await pacer.aclose()
    asyncio.run(run())


def test_source_pacer_rejects_unannounced_time_or_step_regression():
    async def run():
        owner = Component()
        output = owner.signal('state', clock='source')
        pacer = SignalPacer(output, deduplicate=True)
        async with Runtime(output):
            await publish(output, 3)
            await pacer.tick()
            await publish(output, 4, stamp=1)
            with pytest.raises(ValueError, match='regress'):
                await pacer.tick()
            await pacer.aclose()
    asyncio.run(run())


def test_rate_control_interrupts_waits_and_does_not_catch_up_after_pause():
    async def run():
        pacer = RatePacer(1)
        await pacer.tick()
        pending = asyncio.create_task(pacer.tick())
        await asyncio.sleep(.005)
        await pacer.pause()
        await pacer.set_rate(100)
        await asyncio.sleep(.03)
        assert not pending.done()
        await pacer.resume()
        await asyncio.wait_for(pending, .2)
        assert pacer.received == 2 and pacer.missed == 0
        assert pacer.actual_start_ns >= pacer.scheduled_deadline_ns
        await pacer.pause()
        pending = asyncio.create_task(pacer.tick())
        await asyncio.sleep(0)
        with pytest.raises(RuntimeError, match='independent'):
            await pacer.tick()
        await pacer.aclose()
        with pytest.raises(PacerClosed):
            await pending
        with pytest.raises(PacerClosed):
            await pacer.resume()
    asyncio.run(run())


def test_rate_pacer_starts_paused_and_is_worker_local():
    async def run():
        pacer = RatePacer(50, rate=2, paused=True)
        with pytest.raises(TypeError, match='worker-local'):
            cloudpickle.dumps(pacer)
        for rate in (0, -1, float('nan'), float('inf'), True):
            with pytest.raises(ValueError, match='finite'):
                await pacer.set_rate(rate)
        pending = asyncio.create_task(pacer.tick())
        await asyncio.sleep(.01)
        assert not pending.done()
        await pacer.resume()
        await asyncio.wait_for(pending, .2)
        assert pacer.received == 1
        await pacer.aclose()
    asyncio.run(run())


def test_frame_tick_waits_for_the_exact_upstream_snapshot_without_claiming_ack():
    async def run():
        source = TickSource(clock='source')
        owner = Component()
        output = owner.signal('converted', clock='source', history=4)
        pacer = FrameTickPacer(output, source.subscribe(policy='every', capacity=2))
        async with Runtime(output):
            tick = await source.publish(stamp_ns=0, episode=0, step_index=0, session='world')
            pending = asyncio.create_task(pacer.tick())
            await asyncio.sleep(.01)
            assert not pending.done()  # Tick delivery did not complete the upstream calculation.
            frame = await publish(output, 0)
            assert await pending == tick
            assert pacer.last_frame is frame and pacer.elapsed_ns is None
            await publish(output, 1)
            await publish(output, 2)  # Newer data must not replace the requested step's snapshot.
            await source.publish(stamp_ns=2_000_000, episode=0, step_index=1, session='world')
            assert (await pacer.tick()).step_index == 1
            assert pacer.last_frame.data == 1 and pacer.elapsed_ns == 2_000_000
            await pacer.aclose()
        await source.aclose()
    asyncio.run(run())


def test_frame_tick_reports_every_history_loss_and_latest_coalesces():
    async def run(policy):
        source = TickSource(clock='source')
        owner = Component()
        output = owner.signal('converted', clock='source', history=1)
        pacer = FrameTickPacer(output, source.subscribe(policy=policy, capacity=2))
        async with Runtime(output):
            await source.publish(stamp_ns=0, episode=0, step_index=0, session='world')
            await publish(output, 0)
            await publish(output, 1)
            if policy == 'every':
                with pytest.raises(HistoryMiss, match='history capacity'):
                    await pacer.tick()
            else:
                pending = asyncio.create_task(pacer.tick())
                await asyncio.sleep(.01)
                assert not pending.done() and pacer.skipped == 1
                await source.publish(stamp_ns=2_000_000, episode=0, step_index=1, session='world')
                assert (await pending).step_index == 1
                assert pacer.last_frame.data == 1
            await pacer.aclose()
        await source.aclose()
    asyncio.run(run('every'))
    asyncio.run(run('latest'))


def test_frame_tick_reset_releases_an_old_computation_and_close_cancels_wait():
    async def run():
        source = TickSource(clock='source')
        owner = Component()
        output = owner.signal('converted', clock='source')
        pacer = FrameTickPacer(output, source.subscribe(policy='every'))
        async with Runtime(output):
            await source.publish(stamp_ns=0, episode=0, step_index=0, session='world')
            pending = asyncio.create_task(pacer.tick())
            await asyncio.sleep(.01)
            await source.publish(stamp_ns=0, episode=1, step_index=0, session='world')
            await publish(output, 0, episode=1)
            assert (await asyncio.wait_for(pending, 1)).episode == 1
            assert pacer.skipped == 1 and pacer.elapsed_ns is None
            await source.publish(stamp_ns=2_000_000, episode=1, step_index=1, session='world')
            pending = asyncio.create_task(pacer.tick())
            await asyncio.sleep(.01)
            await pacer.aclose()
            with pytest.raises(PacerClosed):
                await pending
        await source.aclose()
    asyncio.run(run())


def test_frame_tick_source_failure_interrupts_a_missing_upstream_frame():
    async def run():
        source = TickSource(clock='source')
        owner = Component()
        output = owner.signal('converted', clock='source')
        pacer = FrameTickPacer(output, source.subscribe(policy='every'))
        async with Runtime(output):
            await source.publish(stamp_ns=0, episode=0, step_index=0, session='world')
            pending = asyncio.create_task(pacer.tick())
            await asyncio.sleep(.01)
            await source.aclose(RuntimeError('physical owner failed'))
            with pytest.raises(PacerClosed) as failure:
                await asyncio.wait_for(pending, 1)
            assert str(failure.value.__cause__) == 'physical owner failed'
            await pacer.aclose()
    asyncio.run(run())


@pytest.mark.parametrize('placed', [False, True])
def test_event_connect_forwards_actual_command_identity_and_preserves_expiry(placed):
    async def run():
        owner, actuator = Component(), Component()
        output = owner.signal('commands', clock='source')
        accepted = []
        sink = CommandSink(actuator, 'velocity', accepted.append, fallback=VelocityCommand())
        drive = Connect(output, sink, hz=None)
        placement = {drive: ProcessPlacement('event-drive'), actuator: LocalPlacement()} if placed else None
        async def send(sequence, *, epoch='before', expired=False):
            envelope = CommandEnvelope(VelocityCommand(sequence / 10), 'navigation', epoch, sequence,
                                        time.monotonic_ns() + (-1 if expired else 450_000_000))
            await output.publish(envelope, stamp_ns=10, clock='source',
                                 metadata=dict(session='world', episode=0, step_index=5))
            return envelope
        def movements():
            return [item for item in accepted if item.controller_id == 'navigation']
        async def wait_count(count):
            async with asyncio.timeout(3):
                while len(movements()) < count:
                    await asyncio.sleep(.001)
        async with Runtime(drive, placement=placement):
            first = await send(1)
            await wait_count(1)
            second = await send(2)  # Same source metadata, a new valid command.
            await wait_count(2)
            assert movements() == [first, second]
            sink.guard.retire()
            await send(3)
            await asyncio.sleep(.04)
            await send(1, epoch='expired', expired=True)
            await asyncio.sleep(.04)
            fresh = await send(1, epoch='after')
            await wait_count(3)
            assert movements() == [first, second, fresh]
    asyncio.run(run())
