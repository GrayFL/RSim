"""A rejected pre-reset session cannot revive; explicit new sessions recover."""
import asyncio
import time

from rsim import Component, CommandSink, CommandEnvelope, VelocityCommand, Connect, Runtime


def test_connect_drops_retired_epoch_and_accepts_explicit_new_epoch_without_renewal():
    async def run():
        source, actuator = Component(), Component()
        commands = source.signal('commands', clock='simulation')
        accepted = []
        sink = CommandSink(actuator, 'velocity', accepted.append, fallback=VelocityCommand())
        drive = Connect(commands, sink)
        async def publish(epoch, sequence, value):
            envelope = CommandEnvelope(VelocityCommand(value), 'navigation', epoch, sequence,
                                        time.monotonic_ns() + 300_000_000)
            await commands.publish(envelope, stamp_ns=sequence, clock='simulation')
            return envelope
        async with Runtime(drive):
            original = await publish('before-reset', 1, .1)
            async with asyncio.timeout(1):
                while not accepted:
                    await asyncio.sleep(.002)
            assert accepted[0] is original
            sink.guard.retire()
            await publish('before-reset', 2, .2)
            async with asyncio.timeout(1):
                while drive.dropped != 1:
                    await asyncio.sleep(.002)
            navigation = lambda: [item for item in accepted if item.controller_id == 'navigation']
            assert navigation() == [original]
            assert all(item.value == VelocityCommand() for item in accepted
                       if item.controller_id != 'navigation')
            fresh = await publish('after-reset', 1, .3)
            async with asyncio.timeout(1):
                while len(navigation()) != 2:
                    await asyncio.sleep(.002)
            assert navigation()[-1] is fresh  # same deadline, controller and sequence
    asyncio.run(run())
