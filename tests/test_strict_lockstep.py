"""Owner ACKs are independent evidence of staging and completed integration."""
import asyncio
from dataclasses import replace
import time

import pytest

from rsim import (CommandAccepted, CommandIdentity, PhysicsAdvanced,
                  SingleProcessStepper, StepAck, StepContext)


class Owner:
    def __init__(self):
        self.context = StepContext('world', 2, 0)
        self.substeps = 0
        self.stopped = 0
        self.staged = None
        self.events = []

    async def sample(self, context):
        assert context == self.context
        self.events.append(('sample', context.step_index))
        return self.substeps

    async def compute(self, context, state):
        assert state == self.substeps
        self.events.append(('compute', context.step_index))
        return CommandIdentity('controller', 'epoch', context.step_index + 1)

    async def accept(self, context, command):
        assert context == self.context
        self.events.append(('accept', context.step_index))
        self.staged = command
        return CommandAccepted(context, command, time.monotonic_ns() + 1_000_000_000,
                               (self.substeps + 5) * 2_000_000).to_dict()

    async def advance(self, context, accepted):
        assert context == self.context and accepted.command_id == self.staged
        self.events.append(('advance', context.step_index))
        self.context = replace(context, step_index=context.step_index + 1)
        self.substeps += 5
        return PhysicsAdvanced(context, self.context, accepted.command_id, 5, 5,
                               self.substeps, self.substeps * 2_000_000).to_dict()

    async def stop(self):
        self.stopped += 1
        self.staged = None

    def runner(self):
        return SingleProcessStepper(sample=self.sample, compute=self.compute, accept=self.accept,
                                    advance=self.advance, stop=self.stop, context=self.context,
                                    require_physics_ack=True, requested_substeps=5)


def test_strict_closed_loop_tracks_source_identity_and_actual_substeps():
    async def run():
        owner = Owner()
        runner = owner.runner()
        for step in range(3):
            observed, command = await runner.step()
            assert observed == step * 5 and command.sequence == step + 1
            assert runner.context == owner.context
            assert runner.last_command_ack.context.step_index == step
            assert runner.last_physics_ack.context.step_index == step + 1
            assert runner.last_physics_ack.stamp_ns == (step + 1) * 10_000_000
            assert [item[0] for item in runner.timings] == ['sample', 'compute', 'accept', 'advance']
            assert all(end >= start for _, start, end in runner.timings)
        assert owner.events == [(stage, step) for step in range(3)
                                for stage in ('sample', 'compute', 'accept', 'advance')]
        assert not runner.aborted and not runner.needs_synchronization
    asyncio.run(run())


@pytest.mark.parametrize('failure', ['delivery_only', 'old_episode', 'expired', 'missing_kind'])
def test_invalid_command_ack_never_authorizes_advancement(failure):
    async def run():
        owner = Owner()
        runner = owner.runner()
        async def invalid(context, command):
            ack = await owner.accept(context, command)
            if failure == 'delivery_only':
                return StepAck(context, 'command')
            if failure == 'old_episode':
                ack['context']['episode'] -= 1
            elif failure == 'expired':
                ack['deadline_ns'] = time.monotonic_ns() - 1
            else:
                del ack['kind']
            return ack
        runner.accept = invalid
        with pytest.raises((ValueError, TimeoutError)):
            await runner.step()
        assert owner.substeps == 0 and owner.stopped == 1 and runner.aborted
        assert not runner.needs_synchronization
    asyncio.run(run())


@pytest.mark.parametrize('failure', ['wrong_command', 'old_episode', 'partial', 'wrong_count',
                                   'aborted', 'lost_ack', 'source_bound', 'not_monotonic'])
def test_uncertain_or_mismatched_physics_ack_aborts_until_explicit_reset(failure):
    async def run():
        owner = Owner()
        runner = owner.runner()
        await runner.step()
        async def invalid(context, accepted):
            result = await owner.advance(context, accepted)
            if failure == 'wrong_command':
                result['command_id']['sequence'] += 1
            elif failure == 'old_episode':
                result['context']['episode'] -= 1
            elif failure == 'partial':
                result['actual_substeps'] = 4
            elif failure == 'wrong_count':
                result['actual_substeps'] = result['requested_substeps'] = 4
            elif failure == 'aborted':
                result['kind'], result['phase'] = 'PhysicsAborted', 'ABORTED'
            elif failure == 'lost_ack':
                raise ConnectionError('reply lost after committed publication')
            elif failure == 'source_bound':
                result['stamp_ns'] += 1
            else:
                result['stamp_ns'] = runner.last_physics_ack.stamp_ns
            return result
        runner.advance = invalid
        with pytest.raises((ValueError, ConnectionError)):
            await runner.step()
        assert runner.aborted and runner.needs_synchronization and owner.stopped == 1
        assert runner.context.step_index == 1 and owner.context.step_index == 2
        assert owner.substeps == 10
        with pytest.raises(RuntimeError, match='aborted'):
            await runner.step()
        assert owner.substeps == 10
        async def reset_source():
            owner.context = StepContext('replacement-world', 0, 0)
            owner.substeps = 0
            return owner.context.to_dict()
        await runner.reset(reset_source)
        runner.advance = owner.advance
        await runner.step()
        assert runner.context == owner.context and owner.substeps == 5
        assert not runner.aborted and not runner.needs_synchronization
    asyncio.run(run())


def test_reset_cannot_invent_source_generation_or_revive_retired_ack():
    async def run():
        owner = Owner()
        runner = owner.runner()
        await runner.step()
        async def stale_reset():
            return replace(owner.context, step_index=0)
        with pytest.raises(ValueError, match='generation'):
            await runner.reset(stale_reset)
        assert runner.aborted and owner.substeps == 5
        async def real_reset():
            owner.context = replace(owner.context, episode=3, step_index=0)
            owner.substeps = 0
            return owner.context
        await runner.reset(real_reset)
        assert runner.context.episode == 3 and runner.last_command_ack is None
        assert runner.last_physics_ack is None and not runner.aborted
    asyncio.run(run())


def test_strict_mode_requires_explicit_source_context_and_substep_budget():
    owner = Owner()
    callbacks = dict(sample=owner.sample, compute=owner.compute, accept=owner.accept,
                     advance=owner.advance, stop=owner.stop)
    with pytest.raises(ValueError, match='source context'):
        SingleProcessStepper(**callbacks, require_physics_ack=True, requested_substeps=5)
    with pytest.raises(ValueError, match='requested_substeps'):
        SingleProcessStepper(**callbacks, context=owner.context, require_physics_ack=True)
    with pytest.raises(ValueError):
        CommandIdentity('controller', 'epoch', True)
    with pytest.raises(ValueError):
        StepContext('world', -1, 0)


def test_cancellation_after_commit_marks_unknown_and_finishes_stop():
    async def run():
        owner = Owner()
        runner = owner.runner()
        committed, stopping, stopped = asyncio.Event(), asyncio.Event(), asyncio.Event()
        async def advance(context, accepted):
            await owner.advance(context, accepted)
            committed.set()
            await asyncio.Event().wait()
        async def stop():
            stopping.set()
            await asyncio.sleep(.01)
            await owner.stop()
            stopped.set()
        runner.advance, runner.stop = advance, stop
        task = asyncio.create_task(runner.step())
        await committed.wait()
        task.cancel()
        await stopping.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert stopped.is_set() and runner.aborted and runner.needs_synchronization
        assert owner.substeps == 5 and runner.context.step_index == 0
    asyncio.run(run())
