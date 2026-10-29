import asyncio
import json
import time

import pytest

from rsim import Runtime, HistoryMiss, SingleProcessStepper, StepAck, StepContext
from rsim.components.replay import ReplaySource, ReplaySample


def test_replay_pause_rate_seek_preserve_native_time_and_isolate_history(tmp_path):
    recording = tmp_path / 'recording.jsonl'
    recording.write_text('\n'.join(json.dumps(dict(stamp_ns=10_000_000_000 + i * 100_000_000, data=i))
                                     for i in range(3)))
    async def run():
        source = ReplaySource.from_jsonl(recording, clock='recorded', paused=True)
        async with Runtime(source.output):
            with pytest.raises(TimeoutError):
                await source.output.get(timeout=.03)
            await source.set_rate(10)
            await source.resume()
            first = await source.output.get(timeout=1)
            assert first.stamp_ns == 10_000_000_000 and first.data == 0
            async with asyncio.timeout(1):
                while not source.pacer.finished:
                    await asyncio.sleep(.001)
            last = await source.output.get()
            assert last.stamp_ns == 10_200_000_000 and last.data == 2
            assert source._failure is None
            await source.pause()
            await source.seek(0)
            with pytest.raises(HistoryMiss):
                await source.output.get(timestamp_ns=last.stamp_ns, clock='recorded')
            await source.resume()
            fresh = await source.output.get(after=last.sequence, timeout=1)
            assert fresh.metadata['episode'] == 1
            assert fresh.metadata['session'] == first.metadata['session']
            assert fresh.stamp_ns == first.stamp_ns
            assert fresh.sequence > last.sequence
        assert source.pacer.closed
    asyncio.run(run())


def test_pause_interrupts_a_scheduled_future_sample_and_invalid_seek_keeps_history():
    async def run():
        source = ReplaySource([ReplaySample(0, 0), ReplaySample(5_000_000_000, 1)], clock='recorded')
        async with Runtime(source):
            first = await source.output.get(timeout=1)
            await source.pause()
            with pytest.raises(ValueError):
                await source.seek(5)
            assert await source.output.get() is first
            await source.set_rate(100)
            with pytest.raises(TimeoutError):
                await source.output.get(after=first.sequence, timeout=.08)
            await source.resume()
            second = await source.output.get(after=first.sequence, timeout=.2)
            assert second.stamp_ns == 5_000_000_000
    asyncio.run(run())


def test_replay_validates_order_and_owns_snapshot_data():
    value = {'x': [1]}
    source = ReplaySource([ReplaySample(0, value)], clock='recorded')
    value['x'].append(2)
    assert source.records[0].data == {'x': [1]}
    with pytest.raises(ValueError, match='increase'):
        ReplaySource([ReplaySample(1, 1), ReplaySample(1, 2)], clock='recorded')
    with pytest.raises(ValueError, match='rate'):
        ReplaySource([ReplaySample(1, 1)], clock='recorded', rate=0)


def test_replay_updates_source_freshness_but_cannot_renew_recorded_commands():
    from rsim import CommandEnvelope, VelocityCommand
    async def run():
        old_host = time.monotonic_ns() - 5_000_000_000
        command = CommandEnvelope(VelocityCommand(.1), 'old', 'recorded', 1, old_host)
        source = ReplaySource([ReplaySample(123, command, {'host_monotonic_ns': old_host})], clock='recorded')
        started = time.monotonic_ns()
        async with Runtime(source):
            frame = await source.output.get(timeout=1)
            assert frame.stamp_ns == 123
            assert started <= frame.metadata['host_monotonic_ns'] <= time.monotonic_ns()
            assert frame.metadata['recorded_host_monotonic_ns'] == old_host
            assert frame.data.deadline_ns == old_host < started
    asyncio.run(run())


def test_lockstep_requires_matching_acceptance_before_single_owner_advances():
    async def run():
        state, events = [0], []
        async def sample(context):
            events.append(('state', context.step_index, state[0]))
            return state[0]
        async def compute(context, observed):
            assert observed == context.step_index
            events.append(('compute', context.step_index))
            return 1
        async def accept(context, command):
            events.append(('accept', context.step_index))
            return StepAck(context, 'command')
        async def advance(context):
            assert events[-1] == ('accept', context.step_index)
            state[0] += 1
        async def stop():
            events.append(('stop',))
        runner = SingleProcessStepper(sample=sample, compute=compute, accept=accept, advance=advance, stop=stop)
        for _ in range(5):
            await runner.step()
        assert state == [5]
        assert [stage for stage, _ in runner.trace] == ['state', 'compute', 'accepted', 'advance']
        async def stale(context, command):
            return StepAck(StepContext(context.session, context.episode, 0), 'command')
        runner.accept = stale
        with pytest.raises(ValueError, match='ACK'):
            await runner.step()
        assert runner.aborted and state == [5] and events[-1] == ('stop',)
        with pytest.raises(RuntimeError, match='aborted'):
            await runner.step()
        async def reset():
            state[0] = 0
        await runner.reset(reset)
        runner.accept = accept
        await runner.step()
        assert runner.context.episode == 1 and state == [1]
    asyncio.run(run())


@pytest.mark.parametrize('cancel', [False, True])
def test_lockstep_timeout_and_cancellation_abort_and_stop(cancel):
    async def run():
        entered, stopped = asyncio.Event(), asyncio.Event()
        async def sample(context):
            return 0
        async def compute(context, state):
            entered.set()
            await asyncio.Event().wait()
        async def forbidden(*args):
            pytest.fail('physics advanced before command acceptance')
        async def stop():
            stopped.set()
        runner = SingleProcessStepper(sample=sample, compute=compute, accept=forbidden, advance=forbidden,
                                       stop=stop, timeout=.05)
        task = asyncio.create_task(runner.step())
        await entered.wait()
        if cancel:
            task.cancel()
        with pytest.raises(asyncio.CancelledError if cancel else TimeoutError):
            await task
        assert runner.aborted and stopped.is_set() and runner.context.step_index == 0
    asyncio.run(run())


@pytest.mark.parametrize('blocking', [False, True])
def test_lockstep_rechecks_host_deadline_after_uncooperative_compute(blocking):
    async def run():
        stopped = []
        async def sample(context):
            return 0
        async def compute(context, state):
            if blocking:
                time.sleep(.02)
            else:
                try:
                    await asyncio.sleep(.1)
                except asyncio.CancelledError:
                    pass
            return 1
        async def forbidden(*args):
            pytest.fail('expired computation reached command acceptance or physics')
        async def stop():
            stopped.append(True)
        runner = SingleProcessStepper(sample=sample, compute=compute, accept=forbidden, advance=forbidden,
                                       stop=stop, timeout=.005)
        with pytest.raises(TimeoutError):
            await runner.step()
        assert runner.aborted and stopped == [True]
    asyncio.run(run())


@pytest.mark.parametrize('reset', [False, True])
def test_repeated_cancellation_finishes_stop_or_reset_transaction(reset):
    async def run():
        computing, stopping, stopped, reset_done = (asyncio.Event() for _ in range(4))
        async def sample(context):
            return 0
        async def compute(context, state):
            computing.set()
            await asyncio.Event().wait()
        async def forbidden(*args):
            pytest.fail('cancelled computation advanced')
        async def stop():
            stopping.set()
            await asyncio.sleep(.02)
            stopped.set()
        async def reset_source():
            await asyncio.sleep(.01)
            reset_done.set()
        runner = SingleProcessStepper(sample=sample, compute=compute, accept=forbidden, advance=forbidden,
                                       stop=stop, timeout=1)
        task = asyncio.create_task(runner.reset(reset_source) if reset else runner.step())
        if not reset:
            await computing.wait()
            task.cancel()
        await stopping.wait()
        task.cancel()
        await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert stopped.is_set()
        if reset:
            assert reset_done.is_set() and runner.context.episode == 1 and not runner.aborted
        else:
            assert runner.aborted
    asyncio.run(run())
