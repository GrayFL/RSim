import asyncio
import time
import pytest

from rsim import (Component, Runtime, CommandSink, CommandEnvelope, CommandGuard,
                  CommandRejected, Connect, CommandMux, CommandInput, VelocityCommand)


class Device(Component):
    def __init__(self):
        super().__init__()
        self.applied = []
        self.velocity = CommandSink(self, "velocity", self.apply, fallback=VelocityCommand(), hz=200)

    async def apply(self, envelope):
        self.applied.append(envelope.value)


def test_guard_rejects_expired_reordered_epochs_and_competing_controllers():
    guard = CommandGuard(max_ttl=.5)
    def command(controller="a", epoch="first", sequence=1, deadline=200):
        return CommandEnvelope(3, controller, epoch, sequence, deadline)
    guard.accept(command(), now_ns=100)
    for invalid in (command(), command(deadline=100), command(controller="b"), command(epoch="new")):
        with pytest.raises(CommandRejected):
            guard.accept(invalid, now_ns=101)
    guard.accept(command(epoch="new", deadline=400), now_ns=201)
    with pytest.raises(CommandRejected, match="retired"):
        guard.accept(command(sequence=10, deadline=600), now_ns=401)
    with pytest.raises(CommandRejected, match="TTL"):
        guard.accept(command(epoch="third", deadline=10**10), now_ns=401)


def test_provider_deadman_direct_write_and_rejection_does_not_kill_service():
    async def run():
        device = Device()
        async with Runtime(device.velocity):
            await device.velocity.set(VelocityCommand(1), ttl=.04)
            invalid = CommandEnvelope(VelocityCommand(2), "bad", "bad", 1, time.monotonic_ns() - 1)
            with pytest.raises(CommandRejected, match="expired"):
                await device.velocity.set(invalid)
            await asyncio.sleep(.07)
            assert device.applied == [VelocityCommand(1), VelocityCommand()]
            assert device._failure is None
            await device.velocity.set(VelocityCommand(3))
        assert device.applied[-1] == VelocityCommand()
    asyncio.run(run())


def test_connect_exclusivity_mux_override_fallback_and_observable_output():
    async def run():
        source, device = Component(), Device()
        nav, manual = source.signal("nav"), source.signal("manual")
        with pytest.raises(ValueError, match="CommandMux"):
            async with Runtime(Connect(nav, device.velocity), Connect(manual, device.velocity)):
                pass
        mux = CommandMux(nav=CommandInput(nav, 10, .3), manual=CommandInput(manual, 50, .05),
                         fallback=VelocityCommand(), hz=200)
        drive = Connect(mux.output, device.velocity, hz=200)
        async with Runtime(drive):
            await nav.publish(VelocityCommand(1), stamp_ns=1, clock="test")
            await manual.publish(VelocityCommand(2), stamp_ns=1, clock="test")
            await asyncio.sleep(.025)
            assert device.applied[-1] == VelocityCommand(2)
            frame = await mux.get(timeout=1)
            assert frame.data.value == VelocityCommand(2)
            with pytest.raises(CommandRejected, match="owned"):
                await device.velocity.set(VelocityCommand())
            await asyncio.sleep(.06)
            assert device.applied[-1] == VelocityCommand(1)
            mux.override("manual")
            await asyncio.sleep(.025)
            assert device.applied[-1] == VelocityCommand()
            mux.override(None)
            await asyncio.sleep(.025)
            assert device.applied[-1] == VelocityCommand(1)
        assert device.applied[-1] == VelocityCommand()
    asyncio.run(run())


def test_late_provider_rejection_drops_command_without_killing_control_loop():
    class DelayedDevice(Device):
        async def apply(self, command):
            if command.value.linear_x == 3:
                raise CommandRejected("expired after transport delay", reason="expired")
            await super().apply(command)

    async def run():
        source, device = Component(), DelayedDevice()
        commands = source.signal("commands")
        drive = Connect(commands, device.velocity)
        async with Runtime(drive):
            await commands.publish(VelocityCommand(3), stamp_ns=1, clock="test")
            async with asyncio.timeout(1):
                while drive.dropped == 0:
                    await asyncio.sleep(.01)
            assert device._failure is None and drive._failure is None
            await commands.publish(VelocityCommand(), stamp_ns=2, clock="test")
            async with asyncio.timeout(1):
                while not device.applied:
                    await asyncio.sleep(.01)
            assert device.applied == [VelocityCommand()]
    asyncio.run(run())


def test_reopen_retires_old_controller_and_accepts_new_session_immediately():
    async def run():
        device = Device()
        old = CommandEnvelope(VelocityCommand(1), "old", "session", 1, time.monotonic_ns() + 400_000_000)
        async with Runtime(device):
            await device.velocity.set(old)
        async with Runtime(device):
            with pytest.raises(CommandRejected, match="retired"):
                await device.velocity.set(old)
            await device.velocity.set(VelocityCommand(2))
            assert device.applied[-1] == VelocityCommand(2)
    asyncio.run(run())
