import asyncio
from dataclasses import replace
import os
import time

import pytest

from rsim import (CommandRejected, Component, ComponentError, LocalPlacement, ProcessPlacement,
                  Runtime, StageAck, StageBarrier, StageEndpoint, StageExecutor, StageFailed,
                  StageRequest, StepContext)


def request(step=0, revision='map:1/mission:1', task='compute', episode=0):
    return StageRequest(StepContext('world', episode, step), task, revision)


def test_concurrent_duplicates_cache_completion_not_delivery_and_evicted_steps_cannot_rerun():
    async def run():
        started, finish = asyncio.Event(), asyncio.Event()
        calls = []
        async def compute(req):
            calls.append(req)
            started.set()
            await finish.wait()
            return {'count': len(calls)}
        executor = StageExecutor('compute', compute, ack_capacity=1)
        one = asyncio.create_task(executor.execute(request()))
        two = asyncio.create_task(executor.execute(request()))
        await started.wait()
        assert not one.done() and not two.done() and executor.phase == 'COMPUTING'
        finish.set()
        a, b = await asyncio.gather(one, two)
        assert a == b and len(calls) == 1
        a.result['count'] = -1
        cached = await executor.execute(request(), deadline_ns=1)
        assert cached.result['count'] == 1  # Caller cannot mutate cached evidence.
        with pytest.raises(StageFailed, match='different input'):
            await executor.execute(request(revision='map:2/mission:1'))
        await executor.execute(request(1))
        with pytest.raises(StageFailed, match='no longer cached'):
            await executor.execute(request())
        assert len(calls) == 2 and executor.describe()['ack_cache_size'] == 1
        await executor.aclose()
    asyncio.run(run())


@pytest.mark.parametrize('failure', ['exception', 'timeout', 'swallow', 'blocking'])
def test_failed_or_overtime_stage_aborts_without_a_success_ack_and_requires_new_generation(failure):
    async def run():
        calls = []
        async def compute(req):
            calls.append(req)
            if req.context.episode:
                return {'recovered': True}
            if failure == 'exception':
                raise ValueError('actual input revision changed')
            if failure == 'blocking':
                time.sleep(.02)
            else:
                try:
                    await asyncio.sleep(.1)
                except asyncio.CancelledError:
                    if failure != 'swallow':
                        raise
            return {'too_late': True}
        executor = StageExecutor('compute', compute, timeout=.005)
        for _ in range(2):
            with pytest.raises(StageFailed):
                await executor.execute(request())
        assert len(calls) == 1 and executor.phase == 'ABORTED'
        assert executor.describe()['last_ack'] is None
        assert (await executor.reset(request().context))['phase'] == 'ABORTED'
        with pytest.raises(StageFailed):
            await executor.execute(request(1))
        await executor.reset(request(episode=1).context)
        ack = await executor.execute(request(episode=1))
        assert ack.result == {'recovered': True} and len(calls) == 2
        with pytest.raises(StageFailed, match='generation'):
            await executor.execute(request())
        await executor.aclose()
    asyncio.run(run())


def test_repeated_cancellation_finishes_one_bounded_stage_and_preserves_cached_ack():
    async def run():
        started, finish = asyncio.Event(), asyncio.Event()
        calls = []
        async def compute(req):
            calls.append(req)
            started.set()
            await finish.wait()
            return {'completed': True}
        executor = StageExecutor('compute', compute)
        task = asyncio.create_task(executor.execute(request()))
        await started.wait()
        task.cancel()
        await asyncio.sleep(0)
        task.cancel()
        assert not task.done()
        finish.set()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert (await executor.execute(request())).result == {'completed': True}
        assert len(calls) == 1
        await executor.aclose()
    asyncio.run(run())


def test_reset_waiting_for_compute_rechecks_host_deadline_without_retiring_current_generation():
    async def run():
        entered, release = asyncio.Event(), asyncio.Event()
        async def compute(req):
            entered.set()
            await release.wait()
        executor = StageExecutor('compute', compute)
        task = asyncio.create_task(executor.execute(request()))
        await entered.wait()
        reset = asyncio.create_task(executor.reset(request(episode=1).context,
                                                    deadline_ns=time.monotonic_ns()+1_000_000))
        await asyncio.sleep(.005)
        release.set()
        await task
        with pytest.raises(StageFailed, match='expired'):
            await reset
        assert executor.context.episode == 0 and executor.phase == 'COMPLETED'
        await executor.aclose()
    asyncio.run(run())


def test_running_stage_coordination_is_not_a_serializable_recipe():
    import cloudpickle
    async def compute(req):
        return None
    for value in (StageExecutor('compute', compute),
                  StageBarrier(['compute'], context=request().context)):
        with pytest.raises(TypeError, match='local'):
            cloudpickle.dumps(value)


class CountStage(StageEndpoint):
    def __init__(self, task_id='compute', *, crash=False):
        super().__init__(task_id, ack_capacity=2)
        self.count, self.crash = 0, crash

    async def compute(self, req):
        if self.crash:
            os._exit(37)
        await asyncio.sleep(.005)
        self.count += 1
        return {'count': self.count, 'pid': os.getpid()}


class LostReply:
    def __init__(self, sink):
        self.sink, self.dropped = sink, False

    async def set(self, data, *, ttl):
        response = await self.sink.set(data, ttl=ttl)
        if not self.dropped:
            self.dropped = True
            raise TimeoutError('injected ACK loss after worker computation')
        return response


class FailingStage(StageEndpoint):
    def __init__(self, failure):
        super().__init__('compute')
        self.failure = failure

    async def compute(self, req):
        if req.context.episode == 0:
            if self.failure == 'retired':
                raise StageFailed('input generation retired during computation', reason='retired')
            asyncio.current_task().cancel()
            await asyncio.sleep(0)
        return {'recovered': True, 'pid': os.getpid()}


@pytest.mark.parametrize('failure', ['retired', 'self_cancel'])
@pytest.mark.parametrize('placed', [False, True])
def test_callback_retirement_or_self_cancellation_keeps_management_alive_for_explicit_reset(failure, placed):
    async def run():
        stage = FailingStage(failure)
        placement = {stage: ProcessPlacement('recoverable-stage')} if placed else None
        async with Runtime(stage.stage_request, placement=placement):
            with pytest.raises(CommandRejected) as failure_reply:
                await stage.run_stage(request())
            await stage.reset_stage(request(episode=1).context)
            ack = StageAck.from_mapping(await stage.run_stage(request(episode=1)))
            assert ack.result['recovered']
            assert failure_reply.value.reason == ('stale_context' if failure == 'retired' else 'cancelled')
            assert (await stage.describe_stage())['phase'] == 'COMPLETED'
    asyncio.run(run())


@pytest.mark.parametrize('placed', [False, True])
def test_barrier_recovers_lost_reply_via_real_endpoint_without_duplicate_computation(placed):
    async def run():
        stage = CountStage()
        placement = {stage: ProcessPlacement('idempotent-stage')} if placed else None
        async with Runtime(stage.stage_request, placement=placement):
            barrier = StageBarrier(['compute'], context=request().context)
            barrier.begin(request().context)
            ack = await barrier.call(LostReply(stage.stage_request), request())
            assert ack.result['count'] == 1
            assert (ack.result['pid'] != os.getpid()) is placed
            assert barrier.complete() == (ack,) and barrier.context.step_index == 1
            assert barrier.timings[0]['attempts'] == 2
            assert (await stage.describe_stage())['high_water'] == 0
            await stage.reset_stage(request(episode=1).context)
            barrier.reset(request(episode=1).context)
            with pytest.raises(CommandRejected):
                await stage.run_stage(request())
            barrier.begin(request(episode=1).context)
            fresh = await barrier.call(stage.stage_request, request(episode=1))
            assert fresh.context.episode == 1 and fresh.result['count'] == 2
            barrier.complete()
    asyncio.run(run())


class SnapshotStage(StageEndpoint):
    def __init__(self, name, incoming):
        super().__init__(name, inputs=(incoming,))
        self.incoming = incoming
        self.output = self.signal('output', clock='source')

    async def compute(self, req):
        cursor = 0
        while True:
            frame = await self.incoming.get(after=cursor, timeout=.4)
            cursor = frame.sequence
            context = StepContext.from_mapping(frame.metadata['context'])
            if context == req.context:
                break
            if context.step_index > req.context.step_index:
                raise ValueError('requested source snapshot was skipped')
        if frame.data['revision'] != req.input_revision:
            raise ValueError('actual input revision differs')
        revision = req.input_revision + '/' + self.task_id
        value = frame.data['value'] + 1
        await self.output.publish({'revision': revision, 'value': value}, stamp_ns=frame.stamp_ns,
                                  clock='source', metadata=frame.metadata)
        return {'revision': revision, 'value': value, 'pid': os.getpid()}


def test_two_real_process_stages_wait_for_exact_input_and_complete_before_next_stage():
    async def run():
        producer = Component()
        source = producer.signal('sample', clock='source')
        left, right = SnapshotStage('left', source), None
        right = SnapshotStage('right', left.output)
        placement = {producer: LocalPlacement(), left: ProcessPlacement('left-stage'),
                     right: ProcessPlacement('right-stage')}
        async with Runtime(left.stage_request, right.stage_request, right.output, placement=placement):
            barrier = StageBarrier(['left', 'right'], context=request().context)
            for step in range(2):
                context, revision = request(step).context, 'revision-' + str(step)
                barrier.begin(context)
                task = asyncio.create_task(barrier.call(left.stage_request, StageRequest(context, 'left', revision)))
                await asyncio.sleep(.02)
                assert not task.done()  # Transport delivery alone cannot ACK.
                await source.publish({'value': step, 'revision': revision}, stamp_ns=step*10,
                                     clock='source', metadata={'context': context.to_dict()})
                first = await task
                second = await barrier.call(right.stage_request,
                    StageRequest(context, 'right', first.result['revision']))
                assert second.result['value'] == step+2
                assert len({first.result['pid'], second.result['pid'], os.getpid()}) == 3
                assert first.finished_ns <= second.started_ns
                assert len(barrier.complete()) == 2
    asyncio.run(run())


@pytest.mark.parametrize('wrong', ['revision', 'context', 'task'])
def test_mismatched_stage_ack_aborts_barrier_and_forbids_subsequent_stage(wrong):
    async def run():
        class WrongReply:
            async def set(self, value, *, ttl):
                req = StageRequest.from_mapping(value)
                changes = {'input_revision': 'wrong'} if wrong == 'revision' else (
                    {'context': replace(req.context, episode=1)} if wrong == 'context' else {'task_id': 'other'})
                req = replace(req, **changes)
                return StageAck(req.context, req.task_id, req.input_revision, 0, 0).to_dict()
        barrier = StageBarrier(['compute', 'execute'], context=request().context)
        barrier.begin(request().context)
        with pytest.raises(StageFailed, match='ACK'):
            await barrier.call(WrongReply(), request())
        assert barrier.phase == 'ABORTED'
        with pytest.raises(StageFailed):
            await barrier.call(WrongReply(), request(task='execute'))
        with pytest.raises(StageFailed):
            barrier.complete()
    asyncio.run(run())


def test_worker_exit_aborts_waiting_barrier_and_does_not_ack_next_stage():
    async def run():
        stage = CountStage(crash=True)
        async with Runtime(stage.stage_request, placement={stage: ProcessPlacement('crashed-stage')}):
            barrier = StageBarrier(['compute', 'never'], context=request().context, timeout=.4)
            barrier.begin(request().context)
            with pytest.raises((CommandRejected, ComponentError, TimeoutError)):
                await barrier.call(stage.stage_request, request())
            assert barrier.phase == 'ABORTED' and not barrier.acks
            with pytest.raises(StageFailed):
                barrier.complete()
    asyncio.run(run())


def test_barrier_rejects_an_ack_that_claims_completion_in_the_future():
    async def run():
        class FutureReply:
            async def set(self, value, *, ttl):
                req = StageRequest.from_mapping(value)
                now = time.monotonic_ns()
                return StageAck(req.context, req.task_id, req.input_revision,
                                now, now+100_000_000_000).to_dict()
        barrier = StageBarrier(['compute'], context=request().context)
        barrier.begin(request().context)
        with pytest.raises(StageFailed, match='completion time'):
            await barrier.call(FutureReply(), request())
        assert barrier.phase == 'ABORTED' and not barrier.acks
    asyncio.run(run())
