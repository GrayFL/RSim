"""ROS-free keyboard client: WASD, space brake, Escape exit."""

import argparse
import asyncio
import os
import sys

from rsim.adapters.keyboard import PynputKeyboard
from rsim.components.teleoperation import Teleoperation
from rsim.devices import Chassis
from rsim.runtime import Runtime
from .control import connection_arguments, transport, vehicle_parameters, until_closed


def input_backend(requested, *, environ=None):
    if requested != "auto":
        return requested
    environ = os.environ if environ is None else environ
    if os.name == "posix" and (
        environ.get("SSH_TTY")
        or environ.get("SSH_CONNECTION")
        or not environ.get("DISPLAY")
    ):
        return "terminal"
    return "pynput"


async def run(args):
    parameters = vehicle_parameters(args.config)
    chassis = Chassis(args.name, transport=transport(args))
    backend = input_backend(args.input)
    if backend == "terminal":
        from rsim.adapters.terminal_keyboard import TerminalKeyboard

        keys = TerminalKeyboard(repeat_timeout=args.key_timeout)
    else:
        keys = PynputKeyboard()
    control = Teleoperation(
        keys, chassis.velocity, parameters=parameters, dry_run=args.dry_run
    )
    print(
        f"Input={backend}; W/S: throttle  A/D: yaw  Space: brake  Escape/Ctrl-C: exit; terminal speed={parameters.terminal_speed:.3f} m/s",
        flush=True,
    )
    if backend == "terminal":
        print(
            f"Terminal repeat timeout={args.key_timeout:.2f}s; hold a key to repeat. "
            "Release is inferred; the initial repeat delay may cause a brief pause. Q also exits.",
            flush=True,
        )
    try:
        async with Runtime(control):
            print(
                "Connected; " + ("ZERO OUTPUT" if args.dry_run else "LIVE OUTPUT"),
                flush=True,
            )
            if sys.stdout.isatty():

                async def status():
                    value = (await control.state.get(timeout=1)).data
                    print(
                        f"\r\033[K{backend} keys={''.join(value['keys']) or '-'} "
                        f"v={value['linear_x']:+.3f} m/s yaw={value['angular_z']:+.3f} rad/s "
                        f"{'BRAKE' if value['brake'] else ''} "
                        f"{'ZERO OUTPUT' if args.dry_run else 'LIVE OUTPUT'}",
                        end="",
                        flush=True,
                    )

                control.task("terminal-status", status, hz=5)
            await until_closed(control, control.finished)
    finally:
        if sys.stdout.isatty():
            print(flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/keyboard.yaml")
    parser.add_argument(
        "--input",
        choices=("auto", "terminal", "pynput"),
        default="auto",
        help="auto selects terminal over SSH, pynput on an X desktop",
    )
    parser.add_argument(
        "--key-timeout",
        type=float,
        default=0.18,
        help="terminal key repeat expiry in seconds (0.05 to 0.5)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="simulate input but transmit only zeros"
    )
    connection_arguments(parser)
    try:
        asyncio.run(run(parser.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
