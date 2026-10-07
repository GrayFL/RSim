import asyncio
from contextlib import contextmanager
import os
import signal
import sys
import time
import uuid

import pytest

termios = pytest.importorskip("termios")
import pty

from rsim.adapters.terminal_keyboard import TerminalKeyboard
from rsim.apps.keyboard_control import input_backend
from rsim.runtime import Runtime


@contextmanager
def terminal():
    master, slave = pty.openpty()
    try:
        yield master, slave
    finally:
        os.close(master)
        os.close(slave)


def test_auto_uses_terminal_for_ssh_even_with_forwarded_display():
    assert (
        input_backend(
            "auto", environ={"SSH_TTY": "/dev/pts/1", "DISPLAY": "localhost:10.0"}
        )
        == "terminal"
    )
    assert input_backend("auto", environ={"DISPLAY": ":0"}) == "pynput"
    assert input_backend("pynput", environ={"SSH_TTY": "/dev/pts/1"}) == "pynput"


def test_terminal_pulses_brake_escape_and_exact_restoration():
    async def run(master, slave):
        initial = termios.tcgetattr(slave)
        keys = TerminalKeyboard(fd=slave, repeat_timeout=0.1)
        async with Runtime(keys):
            mode = termios.tcgetattr(slave)
            assert not mode[3] & (termios.ECHO | termios.ICANON)
            assert mode[3] & termios.ISIG
            os.write(master, b"wa")
            await asyncio.sleep(0.04)
            assert (await keys.get()).data["keys"] == ["a", "w"]
            os.write(master, b"w")
            await asyncio.sleep(0.08)
            assert (await keys.get()).data["keys"] == ["w"]
            await asyncio.sleep(0.08)
            assert (await keys.get()).data["keys"] == []
            os.write(master, b" s")
            await asyncio.sleep(0.04)
            assert (await keys.get()).data == dict(keys=["s"], brake=False, quit=False)
            os.write(master, b" ")
            await asyncio.sleep(0.04)
            assert (await keys.get()).data == dict(keys=[], brake=True, quit=False)
            os.write(master, b"\x1b")
            await asyncio.sleep(0.08)
            assert (await keys.get()).data["quit"]
        assert termios.tcgetattr(slave) == initial
        assert os.get_blocking(slave)

    with terminal() as (master, slave):
        asyncio.run(run(master, slave))


def test_terminal_ignores_escape_sequences_and_bracketed_paste():
    async def run(master, slave):
        keys = TerminalKeyboard(fd=slave)
        async with Runtime(keys):
            os.write(master, b"\x1b[A\x1b[D\x1b[200~wasdwasd\x1b[201~")
            await asyncio.sleep(0.08)
            assert (await keys.get()).data == dict(keys=[], brake=True, quit=False)
            os.write(master, b"d")
            await asyncio.sleep(0.04)
            assert (await keys.get()).data["keys"] == ["d"]

    with terminal() as (master, slave):
        asyncio.run(run(master, slave))


def test_hangup_stops_model_and_cancellation_restores_terminal():
    from rsim.core import VelocityCommand
    from rsim.components.teleoperation import Teleoperation
    from rsim.components.simulated_chassis import SimulatedChassis

    async def run():
        master, slave = pty.openpty()
        keys = TerminalKeyboard(fd=slave)
        source = SimulatedChassis(noise=False)
        control = Teleoperation(keys, source.velocity)
        try:
            async with Runtime(control):
                os.write(master, b"w")
                await asyncio.sleep(0.08)
                assert source.command.linear_x > 0
                os.close(master)
                master = None
                await asyncio.wait_for(control.finished.wait(), 0.3)
                await asyncio.sleep(0.04)
                assert source.command == VelocityCommand()
        finally:
            if master is not None:
                os.close(master)
            os.close(slave)
        with terminal() as (_, slave):
            before = termios.tcgetattr(slave)
            opened = asyncio.Event()

            async def session():
                async with Runtime(TerminalKeyboard(fd=slave)):
                    opened.set()
                    await asyncio.Event().wait()

            task = asyncio.create_task(session())
            await opened.wait()
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert termios.tcgetattr(slave) == before

    asyncio.run(run())


@pytest.mark.parametrize("exit_mode", ["key", "sigint"])
def test_cli_over_pty_reaches_simulated_dds_service_and_restores_echo(exit_mode):
    pytest.importorskip("cyclonedds")
    from rsim.drivers import Chassis as Provider
    from rsim.transport.descriptor import TransportConfig
    from rsim.core import VelocityCommand

    async def run(master, slave):
        name = "tty_" + uuid.uuid4().hex
        service = Provider(
            name=name,
            simulate=True,
            motion_enabled=True,
            transport=TransportConfig(domain_id=86),
        )
        source = service.controller.velocity.producer
        before = termios.tcgetattr(slave)
        os.set_blocking(master, False)
        async with Runtime(service):
            process = await asyncio.create_subprocess_exec(
                os.environ.get("RSIM_TEST_CLIENT_PYTHON", sys.executable),
                "-m",
                "rsim.apps.keyboard_control",
                "--name",
                name,
                "--domain",
                "86",
                "--config",
                "examples/control/keyboard.example.yaml",
                stdin=slave,
                stdout=slave,
                stderr=slave,
                env=dict(os.environ, SSH_TTY="/dev/pts/test", DISPLAY=":unavailable"),
            )
            output = bytearray()
            try:
                async with asyncio.timeout(15):
                    while b"Connected;" not in output:
                        try:
                            output.extend(os.read(master, 8192))
                        except BlockingIOError:
                            pass
                        if process.returncode is not None:
                            raise AssertionError(output.decode())
                        await asyncio.sleep(0.01)
                assert b"Input=terminal" in output
                assert not termios.tcgetattr(slave)[3] & termios.ECHO
                until = time.monotonic() + 0.4
                while time.monotonic() < until:
                    os.write(master, b"w")
                    await asyncio.sleep(0.04)
                assert source.command.linear_x > 0.02
                os.write(master, b" ")
                async with asyncio.timeout(0.3):
                    while source.command != VelocityCommand():
                        await asyncio.sleep(0.01)
                if exit_mode == "key":
                    os.write(master, b"q")
                else:
                    process.send_signal(signal.SIGINT)
                assert await asyncio.wait_for(process.wait(), 3) == 0
                assert source.command == VelocityCommand()
                assert termios.tcgetattr(slave) == before
            finally:
                if process.returncode is None:
                    process.kill()
                    await process.wait()

    with terminal() as (master, slave):
        asyncio.run(run(master, slave))
