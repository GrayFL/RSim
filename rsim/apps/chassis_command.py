"""ROS-free imperative command client."""

import argparse
import asyncio

from rsim.devices import Chassis
from rsim.runtime import Runtime
from .control import connection_arguments, transport


async def run(args):
    chassis = Chassis(args.name, transport=transport(args))
    async with Runtime(chassis):
        if args.command == "move":
            result = await chassis.move(args.distance, timeout=args.timeout)
        elif args.command == "rotate":
            result = await chassis.rotate(
                yaw_deg=args.deg, yaw_rad=args.rad, timeout=args.timeout
            )
        elif args.command == "stop":
            result = await chassis.stop()
        else:
            result = (await chassis.pose.get(timeout=5)).data
        print(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    connection_arguments(parser)
    parser.add_argument("--timeout", type=float, default=60.0)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("move").add_argument("distance", type=float)
    rotate = commands.add_parser("rotate")
    angles = rotate.add_mutually_exclusive_group(required=True)
    angles.add_argument("--deg", type=float)
    angles.add_argument("--rad", type=float)
    commands.add_parser("status")
    commands.add_parser("stop")
    try:
        asyncio.run(run(parser.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
