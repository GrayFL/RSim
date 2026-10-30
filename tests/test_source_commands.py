import asyncio
from dataclasses import replace
import time

import pytest

from rsim import (CommandEnvelope, CommandGuard, CommandInput, CommandMux,
                  CommandRejected, CommandSink, Component, Connect, LocalPlacement,
                  ProcessPlacement, Runtime, SourceCommandBound, StepContext,
                  VelocityCommand)


def test_source_bound_checks_full_interval_and_does_not_replace_host_deadline():
    context = StepContext('source', 2, 10)
    bound = SourceCommandBound(context, 'simulation', 1_200_000_000)
    # A delayed command can cover a later source sample in the same generation.
    bound.check(StepContext('source', 2, 11), clock='simulation', stamp_ns=bound.deadline_ns)
    for actual, clock, stamp, reason in (
        (StepContext('old', 2, 11), 'simulation', 1, 'retired'),
        (StepContext('source', 3, 11), 'simulation', 1, 'retired'),
        (StepContext('source', 2, 9), 'simulation', 1, 'invalid'),
        (context, 'other', 1, 'invalid'),
        (context, 'simulation', bound.deadline_ns + 1, 'expired'),
    ):
        with pytest.raises(CommandRejected) as error:
            bound.check(actual, clock=clock, stamp_ns=stamp)
        assert error.value.reason == reason
    command = CommandEnvelope(VelocityCommand(.1), 'nav', 'epoch', 1, 100, bound)
    with pytest.raises(CommandRejected, match='expired'):
        CommandGuard().accept(command, now_ns=100)
    with pytest.raises(CommandRejected, match='invalid'):
        CommandGuard().accept(replace(command, source_bound={'deadline_ns': 100}), now_ns=99)
    with pytest.raises(ValueError, match='context'):
        SourceCommandBound(StepContext('source', True, 10), 'simulation', 1)
    with pytest.raises(ValueError, match='nanoseconds'):
        SourceCommandBound(context, 'simulation', 1.2)


def test_ros1_rejects_source_bounds_before_any_network_or_motion():
    from rsim.adapters.ros1 import Chassis, Ros1Bridge

    async def run():
        bridge = Ros1Bridge('unused-host')
        chassis = Chassis('unused-host')
        bound = SourceCommandBound(StepContext('world', 0, 1), 'simulation', 100)
        envelope = CommandEnvelope(VelocityCommand(.1), 'nav', 'epoch', 1,
                                   time.monotonic_ns() + 100_000_000, bound)
        result = await bridge._publish('/cmd_vel', 'geometry_msgs/Twist', {}, envelope)
        assert isinstance(result, CommandRejected)
        with pytest.raises(CommandRejected, match='source-time'):
            await chassis._apply_velocity(envelope)
        assert bridge.process is None and chassis._last_command is None
    asyncio.run(run())


def test_default_envelope_keeps_existing_wire_fields(tmp_path):
    from rsim import Frame
    from rsim.transport.shared import SharedStore, decode
    command = CommandEnvelope(VelocityCommand(.1), 'nav', 'epoch', 1, 100)
    store = SharedStore(tmp_path)
    descriptor = store.put(Frame(command, 1, 'simulation'))['data']
    assert set(descriptor['fields']) == {'value', 'controller_id', 'controller_epoch',
                                        'sequence', 'deadline_ns'}
    assert decode(descriptor, tmp_path) == command
    store.close()


def test_rejected_source_bound_cannot_extend_an_existing_motion_lease():
    async def run():
        actuator = Component()
        applied = []
        def apply(command):
            if command.source_bound is not None:
                raise CommandRejected('provider has no matching source clock')
            applied.append(command.value)
        sink = CommandSink(actuator, 'velocity', apply, fallback=VelocityCommand(), hz=200)
        async with Runtime(sink):
            original = CommandEnvelope(VelocityCommand(.2), 'nav', 'epoch', 1,
                                       time.monotonic_ns() + 100_000_000)
            await sink.set(original)
            invalid = replace(original, sequence=2, deadline_ns=time.monotonic_ns() + 450_000_000,
                source_bound=SourceCommandBound(StepContext('other', 0, 0), 'simulation', 10))
            with pytest.raises(CommandRejected, match='source clock'):
                await sink.set(invalid)
            assert sink.guard.owner == ('nav', 'epoch')
            assert sink.guard.deadline_ns == original.deadline_ns
            await asyncio.sleep(.13)
            assert applied == [VelocityCommand(.2), VelocityCommand()]
            assert sink.guard.sequences['nav', 'epoch'] == 2
    asyncio.run(run())


@pytest.mark.parametrize('placed', [False, True])
def test_mux_connect_and_transport_preserve_original_source_bound(placed):
    async def run():
        source, actuator = Component(), Component()
        commands = source.signal('commands', clock='simulation')
        accepted = []
        sink = CommandSink(actuator, 'velocity', accepted.append, fallback=VelocityCommand())
        mux = CommandMux(nav=CommandInput(commands, 10, .45), fallback=VelocityCommand(),
                         hz=100, ttl=.4)
        drive = Connect(mux.output, sink, hz=None)
        placement = ({drive: ProcessPlacement('source-bound'), mux: ProcessPlacement('source-bound'),
                      source: LocalPlacement(), actuator: LocalPlacement()} if placed else None)
        bound = SourceCommandBound(StepContext('world', 3, 7), 'simulation', 140_000_000)
        original = CommandEnvelope(VelocityCommand(.2), 'nav', 'epoch', 9,
                                   time.monotonic_ns() + 450_000_000, bound)
        async with Runtime(drive, placement=placement):
            # Start the lease after real worker startup, never let spawn renew it.
            original = replace(original, deadline_ns=time.monotonic_ns() + 450_000_000)
            await commands.publish(original, stamp_ns=100_000_000, clock='simulation')
            async with asyncio.timeout(3):
                while not any(item.value == original.value for item in accepted):
                    await asyncio.sleep(.002)
            delivered = [item for item in accepted if item.value == original.value]
            assert delivered and all(item.source_bound == bound for item in delivered)
            assert all(item.deadline_ns <= original.deadline_ns for item in delivered)
            # The original observation and interval survive a new mux identity.
            assert all(item.controller_id.startswith('mux:') for item in delivered)
            await asyncio.sleep(.5)
            assert accepted[-1].value == VelocityCommand()
            assert accepted[-1].source_bound is None
    asyncio.run(run())
