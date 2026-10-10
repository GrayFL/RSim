import asyncio
import os
import shutil
import signal
import subprocess
import sys
import uuid
from contextlib import contextmanager

import pytest

pytest.importorskip("pygame")
from rsim.adapters.pygame_keyboard import PygameKeyboard
from rsim.adapters.pygame_window import WindowInput
from rsim.runtime import Runtime


def test_failure_summary_keeps_ack_reason_above_empty_timeout_cause():
    from rsim.apps.keyboard_control import failure_message
    from rsim.core import CommandRejected
    timeout = TimeoutError()
    timeout.__cause__ = asyncio.CancelledError()
    rejected = CommandRejected("command acknowledgement expired")
    rejected.__cause__ = timeout
    assert failure_message(rejected) == "CommandRejected: command acknowledgement expired"


def test_focus_brake_repeat_and_held_keys_require_release():
    inputs = WindowInput()
    inputs.focus(True)
    inputs.key("w", True)  # no connection yet
    assert not inputs.keys
    inputs.connect(True, held=["w"])
    inputs.key("w", True, repeat=True)
    assert not inputs.keys
    inputs.key("w", False)
    inputs.key("w", True)
    inputs.key("a", True)
    assert inputs.keys == {"w", "a"} and not inputs.brake
    inputs.focus(False)
    assert inputs.brake and not inputs.keys
    inputs.focus(True, held=["w", "a"])
    inputs.key("w", True)
    assert not inputs.keys and inputs.brake
    inputs.key("w", False)
    inputs.key("w", True)
    assert inputs.keys == {"w"}
    inputs.key("space", True)
    inputs.key("d", True)
    assert inputs.brake and not inputs.keys
    inputs.key("space", False)
    inputs.key("w", True)  # held before braking; must release
    assert inputs.brake and not inputs.keys
    inputs.key("w", False)
    inputs.key("w", True)
    assert not inputs.brake and inputs.keys == {"w"}
    inputs.key("esc", True)
    inputs.key("d", True)
    assert inputs.quit and inputs.brake and not inputs.keys


@pytest.fixture
def xdisplay(monkeypatch):
    pytest.importorskip("Xlib")
    if not shutil.which("Xvfb"):
        pytest.skip("Xvfb is needed for isolated window integration")
    read_fd, write_fd = os.pipe()
    process = subprocess.Popen(
        [
            "Xvfb",
            "-displayfd",
            str(write_fd),
            "-screen",
            "0",
            "1024x768x24",
            "-nolisten",
            "tcp",
        ],
        pass_fds=(write_fd,),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    os.close(write_fd)
    try:
        import select

        assert select.select([read_fd], [], [], 5)[0], "Xvfb did not start"
        display = ":" + os.read(read_fd, 32).decode().strip()
        monkeypatch.setenv("DISPLAY", display)
        monkeypatch.setenv("SDL_VIDEODRIVER", "x11")
        monkeypatch.setenv("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        yield display
    finally:
        os.close(read_fd)
        process.terminate()
        process.wait(timeout=5)


@contextmanager
def xconnection():
    from Xlib.display import Display

    display = Display()
    try:
        yield display
    finally:
        display.close()


async def until(predicate, timeout=5):
    async with asyncio.timeout(timeout):
        while not predicate():
            await asyncio.sleep(0.01)


async def find_window(display, title):
    async with asyncio.timeout(10):
        while True:
            for window in display.screen().root.query_tree().children:
                name = window.get_full_property(display.intern_atom("_NET_WM_NAME"), 0)
                if name is not None and name.value.decode("utf-8") == title:
                    return window
            await asyncio.sleep(0.02)


def press(display, name, down=True):
    from Xlib import XK, X
    from Xlib.ext.xtest import fake_input

    fake_input(
        display,
        X.KeyPress if down else X.KeyRelease,
        display.keysym_to_keycode(XK.string_to_keysym(name)),
    )
    display.sync()


def focus(display, window):
    from Xlib import X

    window.set_input_focus(X.RevertToParent, X.CurrentTime)
    display.sync()


def test_x11_plain_upload_preserves_pixels_without_shm(xdisplay):
    import pygame as pg
    from Xlib import X

    from rsim.adapters.pygame_x11 import X11Surface

    pg.display.init()
    surface = X11Surface(pg, "plain-x11-test", (760, 480))
    try:
        surface.surface.fill((17, 101, 203))
        surface.update([(0, 0, 760, 480)])  # larger than a single X11 request
        surface.display.sync()
        pixels = surface.drawable.get_image(0, 0, 1, 1, X.ZPixmap, 0xFFFFFF).data
        order = (
            "little"
            if surface.display.display.info.image_byte_order == X.LSBFirst
            else "big"
        )
        assert int.from_bytes(pixels, order) & 0xFFFFFF == 0x1165CB
    finally:
        surface.close()
        pg.display.quit()


def test_real_window_keys_focus_and_suspended_renderer_stop_commands(xdisplay):
    pytest.importorskip("graphmap")
    from rsim.components.simulated_chassis import SimulatedChassis
    from rsim.components.teleoperation import Teleoperation
    from rsim.core import VelocityCommand

    async def run(display):
        source = SimulatedChassis(noise=False)
        keys = PygameKeyboard(title="rsim-test-" + uuid.uuid4().hex)
        keys.present(connected=True, dry_run=False)
        control = Teleoperation(keys, source.velocity)
        child = None
        try:
            async with Runtime(keys, control):
                child = keys.process
                window = await find_window(display, keys.title)
                focus(display, window)
                await until(lambda: keys.packet["focused"])
                await asyncio.sleep(0.05)
                press(display, "w")
                press(display, "a")
                await until(
                    lambda: (
                        source.command.linear_x > 0.015
                        and source.command.angular_z > 0.015
                    )
                )
                # Focus a separate window. No key-up is delivered to the control
                # window; losing focus must still clear its commands.
                root = display.screen().root
                other = root.create_window(
                    800, 0, 100, 100, 0, display.screen().root_depth
                )
                other.map()
                focus(display, other)
                await until(lambda: source.command == VelocityCommand(), 0.3)
                focus(display, window)
                await asyncio.sleep(0.12)
                assert source.command == VelocityCommand()
                press(display, "w", False)
                press(display, "a", False)
                press(display, "d")
                await until(lambda: source.command.angular_z < -0.015)
                press(display, "d", False)
                # Stop the entire SDL process as if an X round trip wedged it.
                os.kill(child.pid, signal.SIGSTOP)
                await until(lambda: control._failure is not None, 0.6)
                await until(lambda: source.command == VelocityCommand(), 0.5)
                os.kill(child.pid, signal.SIGCONT)
                other.destroy()
        finally:
            if keys.process is not None and keys.process.pid is not None:
                os.kill(keys.process.pid, signal.SIGCONT)
        assert keys.process is None

    with xconnection() as display:
        asyncio.run(run(display))


@pytest.mark.parametrize("exit_mode", ["escape", "close", "sigint", "command_rejected", "renderer_crash"])
def test_cli_window_controls_dds_simulator_and_cleans_up(xdisplay, exit_mode):
    pytest.importorskip("cyclonedds")
    pytest.importorskip("graphmap")
    from Xlib import X, protocol

    from rsim.core import VelocityCommand
    from rsim.drivers import Chassis as Provider
    from rsim.transport.descriptor import TransportConfig

    async def run(display):
        name = "pg_" + uuid.uuid4().hex
        service = Provider(
            name=name,
            simulate=True,
            motion_enabled=True,
            transport=TransportConfig(domain_id=87),
        )
        source = service.controller.velocity.producer
        async with Runtime(service):
            child = await asyncio.create_subprocess_exec(
                os.environ.get("RSIM_TEST_CLIENT_PYTHON", sys.executable),
                "-m",
                "rsim.apps.keyboard_control",
                "--input",
                "pygame",
                "--name",
                name,
                "--domain",
                "87",
                "--config",
                "examples/control/keyboard.example.yaml",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            try:
                async with asyncio.timeout(15):
                    while True:
                        line = await child.stdout.readline()
                        assert line, "GUI CLI exited before connecting"
                        if b"Connected;" in line:
                            break
                window = await find_window(display, "RSim | Chassis control")
                focus(display, window)
                await asyncio.sleep(0.15)
                press(display, "w")
                await until(lambda: source.command.linear_x > 0.025)
                press(display, "space")
                await until(lambda: source.command == VelocityCommand(), 0.3)
                press(display, "space", False)
                press(display, "w", False)
                if exit_mode == "escape":
                    press(display, "Escape")
                    press(display, "Escape", False)
                elif exit_mode == "close":
                    event = protocol.event.ClientMessage(
                        window=window,
                        client_type=display.intern_atom("WM_PROTOCOLS"),
                        data=(
                            32,
                            [
                                display.intern_atom("WM_DELETE_WINDOW"),
                                X.CurrentTime,
                                0,
                                0,
                                0,
                            ],
                        ),
                    )
                    window.send_event(event)
                    display.flush()
                elif exit_mode == "sigint":
                    child.send_signal(signal.SIGINT)
                elif exit_mode == "command_rejected":
                    service.controller.motion_enabled = False
                    # Use a newly pressed key, not the key just released at the
                    # same X timestamp (SDL can classify that as auto-repeat).
                    press(display, "d")
                    await until(lambda: service.owner is None)
                    await asyncio.sleep(.35)
                    assert child.returncode is None  # fault stays visible
                    assert source.command == VelocityCommand()
                    assert window.id in [w.id for w in display.screen().root.query_tree().children]
                    press(display, "Escape")
                else:
                    pid = int(window.get_full_property(display.intern_atom("_NET_WM_PID"), 0).value[0])
                    os.kill(pid, signal.SIGKILL)
                communication = asyncio.create_task(child.communicate())
                try:
                    output, _ = await asyncio.wait_for(asyncio.shield(communication), 5)
                except TimeoutError:
                    child.send_signal(signal.SIGINT)
                    output, _ = await asyncio.wait_for(communication, 5)
                    pytest.fail("CLI did not exit:\n" + output.decode())
                fault = exit_mode == "renderer_crash"
                assert child.returncode == (1 if fault else 0), output.decode()
                reason = {
                    "escape": b"escape key",
                    "close": b"window close event",
                    "sigint": b"SIGINT / Ctrl-C",
                    "command_rejected": b"nonzero movement requires motion_enabled=True",
                    "renderer_crash": b"exitcode=-9",
                }[exit_mode]
                assert reason in output, output.decode()
                assert b"Keyboard client exit reason=" in output, output.decode()
                if fault:
                    assert b"Traceback" in output and b"control session failed" in output
                if exit_mode == "command_rejected":
                    assert b"retrying in" in output
                await until(lambda: service.owner is None, 0.5)
                assert source.command == VelocityCommand()
                assert window.id not in [
                    w.id for w in display.screen().root.query_tree().children
                ]
            finally:
                if child.returncode is None:
                    child.kill()
                    await child.wait()

    with xconnection() as display:
        asyncio.run(run(display))


def test_close_window_while_discovery_waits(xdisplay):
    async def run(display):
        child = await asyncio.create_subprocess_exec(
            sys.executable,
            "-m",
            "rsim.apps.keyboard_control",
            "--input",
            "pygame",
            "--name",
            "absent_" + uuid.uuid4().hex,
            "--domain",
            "87",
            "--config",
            "examples/control/keyboard.example.yaml",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        try:
            window = await find_window(display, "RSim | Chassis control")
            focus(display, window)
            press(display, "Escape")
            press(display, "Escape", False)
            output, _ = await asyncio.wait_for(child.communicate(), 3)
            assert child.returncode == 0, output.decode()
            assert b"Connected;" not in output
            assert b"Keyboard client exit reason=escape key" in output
        finally:
            if child.returncode is None:
                child.kill()
                await child.wait()

    with xconnection() as display:
        asyncio.run(run(display))


@pytest.mark.parametrize("outage", ["restart", "lost_replies", "renderer_stall"])
def test_reconnects_same_window_without_resuming_held_key(xdisplay, outage):
    pytest.importorskip("cyclonedds")
    pytest.importorskip("graphmap")
    from rsim.drivers import Chassis as Provider
    from rsim.core import VelocityCommand
    from rsim.transport.descriptor import TransportConfig

    async def run(display):
        name = "reconnect_" + uuid.uuid4().hex
        def provider():
            return Provider(name=name, simulate=True, motion_enabled=True,
                            transport=TransportConfig(domain_id=87))
        first = provider()
        runtime = Runtime(first)
        await runtime.__aenter__()
        process = await asyncio.create_subprocess_exec(
            sys.executable, "-m", "rsim.apps.keyboard_control", "--input", "pygame",
            "--name", name, "--domain", "87", "--reconnect-delay", ".1",
            "--config", "examples/control/keyboard.example.yaml",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
        )
        lines = []
        async def log():
            while line := await process.stdout.readline():
                lines.append(line.decode())
        reader = asyncio.create_task(log())
        try:
            await until(lambda: any('Connected;' in line for line in lines), 15)
            window = await find_window(display, "RSim | Chassis control")
            focus(display, window)
            await asyncio.sleep(.1)
            press(display, "w")
            source = first.controller.velocity.producer
            await until(lambda: source.command.linear_x > .02)
            # Keep W physically held across either packet loss or a new server.
            if outage == "restart":
                await runtime.aclose()
            elif outage == "renderer_stall":
                renderer = int(window.get_full_property(display.intern_atom("_NET_WM_PID"), 0).value[0])
                os.kill(renderer, signal.SIGSTOP)
            else:
                publish = first.publisher.publish
                first.publisher.publish = lambda _: None
            await until(lambda: any('retrying in' in line for line in lines), 5)
            assert process.returncode is None
            assert window.id in [w.id for w in display.screen().root.query_tree().children]
            if outage == "restart":
                second = provider()
                runtime = Runtime(second)
                await runtime.__aenter__()
            else:
                second = first
                if outage == "renderer_stall":
                    os.kill(renderer, signal.SIGCONT)
                else:
                    first.publisher.publish = publish
            source = second.controller.velocity.producer
            command_start = len(source.commands)
            await until(lambda: sum('Connected;' in line for line in lines) >= 2, 20)
            await asyncio.sleep(.4)
            assert source.command == VelocityCommand()
            assert all(command == VelocityCommand() for _, command in source.commands[command_start:])
            press(display, "w", False)
            press(display, "d")
            await until(lambda: source.command.angular_z < -.015)
            press(display, "d", False)
            press(display, "Escape")
            await asyncio.wait_for(process.wait(), 5)
            await reader
            assert process.returncode == 0, ''.join(lines)
            await until(lambda: second.owner is None)
            assert source.command == VelocityCommand()
        finally:
            if outage == "renderer_stall" and 'renderer' in locals():
                try:
                    os.kill(renderer, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            if process.returncode is None:
                process.kill()
                await process.wait()
            await reader
            await runtime.aclose()

    with xconnection() as display:
        asyncio.run(run(display))
