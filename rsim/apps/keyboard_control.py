"""ROS-free keyboard client: WASD, space brake, Escape exit."""

import argparse
import asyncio
import faulthandler
import logging
import math
import os
import sys

from rsim.adapters.keyboard import PynputKeyboard
from rsim.adapters.pygame_keyboard import DEFAULT_FONTS
from rsim.components.teleoperation import Teleoperation
from rsim.devices import Chassis
from rsim.runtime import Runtime

from .control import connection_arguments, transport, until_closed, vehicle_parameters

logger = logging.getLogger(__name__)


def failure_message(error):
    seen, detail = set(), error
    while id(error) not in seen:
        seen.add(id(error))
        if str(error):
            detail = error
        next_error = error.__cause__ or (None if error.__suppress_context__ else error.__context__)
        if next_error is None:
            break
        error = next_error
    return f"{type(detail).__name__}: {detail}"


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
    backend = input_backend(args.input)
    if not math.isfinite(args.reconnect_delay) or not .1 <= args.reconnect_delay <= 5:
        raise ValueError("reconnect delay must be between 0.1 and 5 seconds")
    logger.info("Starting keyboard client input=%s mode=%s name=%s domain=%s config=%s",
                backend, "ZERO OUTPUT" if args.dry_run else "LIVE OUTPUT",
                args.name, args.domain, args.config)
    print(
        f"Input={backend}; W/S: throttle  A/D: yaw  Space: brake  Escape/Ctrl-C: exit; terminal speed={parameters.terminal_speed:.3f} m/s",
        flush=True,
    )
    if backend == "pygame":
        from .pygame_control import run_pygame
        return await run_pygame(args, parameters, failure_message)
    chassis = Chassis(args.name, transport=transport(args))
    if backend == "terminal":
        from rsim.adapters.terminal_keyboard import TerminalKeyboard
        keys = TerminalKeyboard(repeat_timeout=args.key_timeout)
        print(
            f"Terminal repeat timeout={args.key_timeout:.2f}s; hold a key to repeat. "
            "Release is inferred; the initial repeat delay may cause a brief pause. Q also exits.",
            flush=True,
        )
    else:
        keys = PynputKeyboard()
    control = Teleoperation(keys, chassis.velocity, parameters=parameters, dry_run=args.dry_run)
    try:
        async with Runtime(keys, control):
            print("Connected; " + ("ZERO OUTPUT" if args.dry_run else "LIVE OUTPUT"), flush=True)
            if sys.stdout.isatty():
                async def status():
                    value = (await control.state.get(timeout=1)).data
                    print(
                        f"\r\033[K{backend} keys={''.join(value['keys']) or '-'} "
                        f"v={value['linear_x']:+.3f} m/s yaw={value['angular_z']:+.3f} rad/s "
                        f"{'BRAKE' if value['brake'] else ''} "
                        f"{'ZERO OUTPUT' if args.dry_run else 'LIVE OUTPUT'}",
                        end="", flush=True,
                    )
                control.task("terminal-status", status, hz=5)
            await until_closed(control, control.finished)
        return "keyboard requested exit"
    finally:
        if sys.stdout.isatty():
            print(flush=True)


def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s [%(process)d] %(name)s: %(message)s")
    faulthandler.enable()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/keyboard.yaml")
    parser.add_argument(
        "--input",
        choices=("auto", "terminal", "pynput", "pygame"),
        default="auto",
        help="auto selects terminal over SSH, pynput on an X desktop",
    )
    parser.add_argument(
        "--font",
        default=DEFAULT_FONTS,
        help="pygame font families in fallback order, comma separated",
    )
    parser.add_argument(
        "--window-hz",
        type=float,
        default=20,
        help="pygame dashboard refresh rate (1 to 60 Hz)",
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
    parser.add_argument("--reconnect-delay", type=float, default=1.0,
                        help="pygame reconnect backoff starts here (0.1 to 5 seconds; capped at 5)")
    connection_arguments(parser)
    try:
        reason = asyncio.run(run(parser.parse_args()))
    except KeyboardInterrupt:
        logger.info("Keyboard client exit reason=SIGINT / Ctrl-C")
    except Exception:
        logger.exception("Keyboard client exit reason=control session failed")
        raise SystemExit(1)
    else:
        logger.info("Keyboard client exit reason=%s", reason or "session completed")


if __name__ == "__main__":
    main()
