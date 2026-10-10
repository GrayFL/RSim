import asyncio
import math
import time
from types import SimpleNamespace

import pytest

from rsim.components.vehicle import VehicleDynamics, VehicleParameters
from rsim.adapters.keyboard import PynputKeyboard
from rsim.runtime import Runtime


def advance(model, keys, seconds, dt=0.02):
    for _ in range(round(seconds / dt)):
        model.step(keys, dt)


def test_terminal_speed_friction_reversal_and_pivot():
    model = VehicleDynamics()
    p = model.parameters
    advance(model, "w", 8)
    assert model.linear == pytest.approx(p.terminal_speed, rel=0.001)
    limit = model.steering_limit()
    assert limit < model.steering_limit(0)
    advance(model, "", 4)
    assert model.linear == 0
    advance(model, "s", 8)
    assert model.linear == pytest.approx(-p.terminal_speed, rel=0.001)
    model.reset()
    advance(model, "a", 8)
    assert model.linear == 0 and model.angular == pytest.approx(
        p.terminal_yaw_rate, rel=0.001
    )
    advance(model, "", 4)
    assert model.angular == 0
    advance(model, "wasd", 1)
    assert model.linear == model.angular == 0


def test_simulation_rate_agreement_and_stall_reset():
    first, second = VehicleDynamics(), VehicleDynamics()
    advance(first, "wa", 2, 0.01)
    advance(second, "wa", 2, 0.04)
    assert first.linear == pytest.approx(second.linear, abs=0.0001)
    assert first.angular == pytest.approx(second.angular, abs=0.0001)
    with pytest.raises(ValueError, match="stalled"):
        first.step("w", 0.5)
    assert first.linear == first.angular == 0
    with pytest.raises(ValueError):
        VehicleParameters(drag=0)
    with pytest.raises(ValueError):
        second.step("", math.nan)


class Listener:
    def __init__(self, **callbacks):
        self.callbacks = callbacks

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def stop(self):
        self.alive = False

    def join(self, timeout):
        pass


def test_listener_thread_events_repeats_release_and_close():
    async def run():
        keys = PynputKeyboard(listener_factory=Listener)
        async with Runtime(keys):
            listener = keys.listener
            press, release = (
                listener.callbacks["on_press"],
                listener.callbacks["on_release"],
            )
            await asyncio.to_thread(press, SimpleNamespace(char="W"))
            await asyncio.to_thread(press, SimpleNamespace(char="W"))
            await asyncio.sleep(0.04)
            assert (await keys.get()).data["keys"] == ["w"]
            release(SimpleNamespace(char="w"))
            press(SimpleNamespace(name="space"))
            await asyncio.sleep(0.04)
            assert (await keys.get()).data == dict(keys=[], brake=True, quit=False)
            press(SimpleNamespace(name="esc"))
            await asyncio.sleep(0.04)
            assert (await keys.get()).data["quit"]
        assert not listener.alive

    asyncio.run(run())


def test_listener_failure_stops_manual_output_without_application_cleanup():
    pytest.importorskip("graphmap")
    from rsim.components.simulated_chassis import SimulatedChassis
    from rsim.components.teleoperation import Teleoperation
    from rsim.core import VelocityCommand

    async def run():
        keys = PynputKeyboard(listener_factory=Listener)
        source = SimulatedChassis(noise=False)
        controller = Teleoperation(keys, source.velocity)
        async with Runtime(controller):
            keys.event("w", True)
            await asyncio.sleep(0.12)
            assert source.command.linear_x > 0
            keys.listener.alive = False
            await asyncio.sleep(0.4)
            assert controller._failure is not None
            assert source.command == VelocityCommand()

    asyncio.run(run())


def test_slow_ack_does_not_stall_model_or_queue_old_key_commands():
    from rsim.components.teleoperation import Teleoperation
    from rsim.core.component import Component, PrimaryComponent
    from rsim.core import CommandSink, VelocityCommand

    class Keys(PrimaryComponent):
        async def open(self):
            self.value = dict(keys=["w"], brake=False, quit=False)
            await self.sample()
            self.task("input", self.sample, hz=100)

        async def sample(self):
            await self.publish(dict(self.value), stamp_ns=time.monotonic_ns(), clock="host:monotonic")

    class SlowSink(Component):
        def __init__(self):
            super().__init__()
            self.velocity = CommandSink(self, "velocity", self.apply, fallback=VelocityCommand())
            self.entered, self.release = asyncio.Event(), asyncio.Event()
            self.commands = []

        async def apply(self, envelope):
            self.entered.set()
            await self.release.wait()
            self.commands.append(envelope)

    async def run():
        keys, sink = Keys(), SlowSink()
        control = Teleoperation(keys, sink.velocity,
                                parameters=VehicleParameters(command_ttl=.4))
        async with Runtime(control):
            await asyncio.wait_for(sink.entered.wait(), 1)
            first = control.state.frames[-1].sequence
            # ACK delay exceeds the model's 200ms loop guard. Sampling and
            # braking must continue while that one command awaits its reply.
            await asyncio.sleep(.23)
            keys.value = dict(keys=[], brake=True, quit=False)
            await asyncio.sleep(.05)
            assert control.state.frames[-1].sequence >= first + 5
            assert control.state.frames[-1].data['linear_x'] == 0
            assert not sink.commands
            sink.release.set()
            await asyncio.sleep(.12)
            assert control._failure is None
            assert sink.commands[0].value.linear_x > 0
            assert len(sink.commands) >= 2
            assert all(c.value == VelocityCommand() for c in sink.commands[1:])

    asyncio.run(run())
