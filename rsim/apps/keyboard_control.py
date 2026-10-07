"""ROS-free keyboard client: WASD, space brake, Escape exit."""

import argparse
import asyncio
import math
import os
import sys

from rsim.adapters.keyboard import PynputKeyboard
from rsim.adapters.pygame_keyboard import DEFAULT_FONTS, PygameKeyboard
from rsim.components.teleoperation import Teleoperation
from rsim.devices import Chassis
from rsim.runtime import Runtime

from .control import connection_arguments, transport, until_closed, vehicle_parameters


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
    elif backend == "pygame":
        keys = PygameKeyboard(
            fonts=args.font,
            render_hz=args.window_hz,
            timeout=parameters.max_loop_gap,
        )
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

    async def session():
        # Start the window before DDS discovery, so connecting is visible and
        # the user can cancel discovery by closing the window.
        async with Runtime(keys, control):
            print(
                "Connected; " + ("ZERO OUTPUT" if args.dry_run else "LIVE OUTPUT"),
                flush=True,
            )
            if backend == "pygame":

                async def window_status():
                    values = control.state.frames
                    pose = chassis.pose.frames
                    coordinates = None
                    if pose:
                        p = pose[-1].data
                        coordinates = [
                            float(p.position[0]),
                            float(p.position[1]),
                            math.degrees(float(p.euler_rad[2])),
                        ]
                    keys.present(
                        **(values[-1].data if values else {"dry_run": args.dry_run}),
                        connected=True,
                        pose=coordinates,
                        endpoint=f"{args.name} / domain {args.domain}",
                    )

                control.task("window-status", window_status, hz=args.window_hz)
            elif sys.stdout.isatty():

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

    try:
        if backend == "pygame":
            keys.present(
                dry_run=args.dry_run, endpoint=f"{args.name} / domain {args.domain}"
            )
            running = asyncio.create_task(session())
            closed = asyncio.create_task(keys.finished.wait())
            try:
                done, _ = await asyncio.wait(
                    [running, closed], return_when=asyncio.FIRST_COMPLETED
                )
                if running in done:
                    await running
                else:
                    # Cancellation also covers a close during DDS discovery.
                    running.cancel()
                    try:
                        await running
                    except asyncio.CancelledError:
                        pass
            finally:
                for task in (running, closed):
                    task.cancel()
                await asyncio.gather(running, closed, return_exceptions=True)
        else:
            await session()
    finally:
        if sys.stdout.isatty():
            print(flush=True)


def main():
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
    connection_arguments(parser)
    try:
        asyncio.run(run(parser.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
