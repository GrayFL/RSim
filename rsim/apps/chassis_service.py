"""Run a named chassis provider in the hardware environment."""

import argparse
import asyncio
import json

from rsim.runtime import Runtime
from .control import connection_arguments, transport


async def run(args):
    from threadpoolctl import threadpool_limits
    from rsim.drivers import Chassis

    robot = None
    if not args.simulate:
        from rsim.config import load_chassis

        robot = load_chassis(args.config, motion_enabled=args.enable_motion)
    service = Chassis(
        robot,
        name=args.name,
        simulate=args.simulate,
        motion_enabled=args.enable_motion,
        transport=transport(args),
    )
    # Small EKF matrices should not fan out over every BLAS worker and starve
    # the control loop. Limit only this dedicated application's lifetime.
    with threadpool_limits(limits=args.blas_threads, user_api="blas"):
        async with Runtime(service):
            await service.pose.get(timeout=20)
            print(
                json.dumps(
                    {
                        "ready": args.name,
                        "motion_enabled": args.enable_motion,
                        "simulated": args.simulate,
                        "domain": args.domain,
                    }
                ),
                flush=True,
            )
            await service.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--config", help="native chassis or legacy local-calibration YAML")
    group.add_argument("--simulate", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--blas-threads", type=int, default=1)
    connection_arguments(parser)
    try:
        args = parser.parse_args()
        if args.blas_threads < 1:
            parser.error("--blas-threads must be positive")
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
