"""Run an estimator/controller over existing ROS topics; never start hardware."""

import argparse
import asyncio
import json
import logging
import math

from rsim.runtime import Runtime
from .control import connection_arguments, transport

logger = logging.getLogger(__name__)


async def run(args):
    from threadpoolctl import threadpool_limits
    from rsim.drivers import Chassis

    # Small EKF matrices should not fan out over every BLAS worker and starve
    # the control loop. Limit only this dedicated application's lifetime.
    with threadpool_limits(limits=args.blas_threads, user_api="blas"):
        delay = args.reconnect_delay
        attempt = 0
        while True:
            # Validate configuration before retrying runtime failures. Recreate
            # only subscribers/algorithms; hardware ownership remains external.
            robot = None
            if not args.simulate:
                from rsim.config import load_chassis
                robot = load_chassis(args.config, motion_enabled=args.enable_motion, hardware=False)
            from rsim.components.vehicle import VehicleParameters
            defaults = VehicleParameters()
            service = Chassis(
                robot, name=args.name, simulate=args.simulate,
                motion_enabled=args.enable_motion, transport=transport(args),
                control=dict(max_linear=defaults.terminal_speed,
                             max_angular=defaults.terminal_yaw_rate) if args.simulate else None,
            )
            attempt += 1
            logger.info("Starting chassis connection attempt=%s", attempt)
            try:
                async with Runtime(service):
                    await service.pose.get(timeout=20)
                    delay = args.reconnect_delay
                    print(json.dumps({"ready": args.name,
                        "motion_enabled": args.enable_motion, "simulated": args.simulate,
                        "domain": args.domain, "attempt": attempt}), flush=True)
                    await service.wait()
                return
            except Exception:
                # Cleanup has ended the old control generation. A new service
                # cannot resume its lease, queued commands or relative action.
                if args.no_reconnect:
                    raise
                logger.exception("Chassis connection failed; old session stopped; retrying in %.1fs", delay)
            await asyncio.sleep(delay)
            delay = min(5, delay * 2)


def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s [%(process)d] %(name)s: %(message)s")
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--config", help="topic-only chassis YAML; hardware must already be running")
    group.add_argument("--simulate", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--blas-threads", type=int, default=1)
    parser.add_argument("--reconnect-delay", type=float, default=1,
                        help="retry failed topic connections after this delay (0.1 to 5 seconds)")
    parser.add_argument("--no-reconnect", action="store_true", help="exit after a runtime failure")
    connection_arguments(parser)
    try:
        args = parser.parse_args()
        if args.blas_threads < 1:
            parser.error("--blas-threads must be positive")
        if not math.isfinite(args.reconnect_delay) or not .1 <= args.reconnect_delay <= 5:
            parser.error("--reconnect-delay must be between 0.1 and 5")
        asyncio.run(run(args))
    except KeyboardInterrupt:
        logger.info("Chassis service exit reason=SIGINT / Ctrl-C")


if __name__ == "__main__":
    main()
