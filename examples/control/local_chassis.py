"""Local chassis calibration or zero-only direct smoke test."""
import argparse
import asyncio
from rsim.runtime import Runtime
from rsim.config.local_chassis import load_local_chassis, calibrate_local_chassis

async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config')
    parser.add_argument('--calibrate', action='store_true')
    args = parser.parse_args()
    if args.calibrate:
        print((await calibrate_local_chassis(args.config)).to_dict())
    else:
        robot = load_local_chassis(args.config)
        async with Runtime(robot.control):
            print((await robot.pose.get(timeout=15)).data)
            await robot.control.move(0.)
            await robot.control.rotate(yaw_deg=0.)

if __name__ == '__main__':
    asyncio.run(main())
