"""Provider entrypoints: run these in the environment containing the ROS drivers.

Application environments import device views from rsim instead. Factories below
are serialized only within the provider's own interpreter, never sent to clients.
"""
import asyncio
import json
from pathlib import Path

from ._device_config import d435_config
from .compose import Bundle
from .devices import _d435_view
from .host import SharedSensor


def D435(
        *,
        serial="",
        stream="depth",
        history=8,
        depth_profile="640x480x15",
        color_profile="640x480x15",
        log_path=None,
        transport=None
    ):
    config = d435_config(serial, depth_profile, color_profile)
    log_path = str(
        Path(log_path).resolve()
        ) if log_path is not None else None

    def factory():
        from .ros import D435 as Source
        return Bundle(
            color=Source(
                **config,
                stream="color",
                history=history,
                log_path=log_path
                ),
            depth=Source(
                **config,
                stream="depth",
                history=history,
                log_path=log_path
                ),
            hz=2 * max(
                int(config[k].split("x")[-1])
                for k in ("depth_profile", "color_profile")
                ),
            history=history
            )

    return _d435_view(
        config, stream, history, factory=factory, transport=transport
        )


def RobinW(ip="192.168.199.97", *, history=8, transport=None):

    def factory():
        from .ros import RobinW as Source
        return Source(ip, history=history)

    return SharedSensor(
        factory,
        key=f"robin:{ip}",
        version="robin-v1",
        history=history,
        transport=transport
        )


def Camera(
        device="/dev/video0",
        *,
        width=640,
        height=640,
        fps=15,
        history=8,
        transport=None
    ):
    config = {
        "device": device, "width": width, "height": height, "fps": fps
        }

    def factory():
        from .uvc import UvcCamera
        return UvcCamera(**config, history=history)

    return SharedSensor(
        factory,
        key=f"uvc:{device}",
        version=json.dumps(config, sort_keys=True),
        history=history,
        transport=transport
        )


async def serve(sensor):
    """Hold a provider lease until cancelled; suitable for create_task in notebooks."""
    from .core import Runtime
    async with Runtime(sensor):
        frame = await sensor.get(timeout=30)
        while True:
            # Observe failures as well as cancellation while retaining ownership.
            frame = await sensor.get(after=frame.sequence)


def main():
    import argparse
    import signal
    from .transport import TransportConfig
    parser = argparse.ArgumentParser(
        description="Start a shared ROS hardware provider"
        )
    parser.add_argument("device", choices=("d435", "robin", "camera"))
    parser.add_argument("--serial", default="")
    parser.add_argument("--ip")
    parser.add_argument("--camera-device", default="/dev/video0")
    parser.add_argument("--depth-profile", default="640x480x15")
    parser.add_argument("--color-profile", default="640x480x15")
    parser.add_argument("--history", type=int, default=8)
    parser.add_argument(
        "--backend", choices=("cyclonedds", "ros2"), default=None
        )
    parser.add_argument("--domain", type=int, default=None)
    parser.add_argument("--log-path", type=Path)
    args = parser.parse_args()
    defaults = TransportConfig()
    transport = TransportConfig(
        args.backend or defaults.backend,
        defaults.domain_id if args.domain is None else args.domain
        )
    common = {"history": args.history, "transport": transport}
    if args.device == "d435":
        if args.log_path:
            args.log_path.parent.mkdir(parents=True, exist_ok=True)
        sensor = D435(
            serial=args.serial,
            depth_profile=args.depth_profile,
            color_profile=args.color_profile,
            log_path=args.log_path,
            **common
            )
    elif args.device == "robin":
        if not args.ip:
            parser.error("robin requires --ip")
        sensor = RobinW(args.ip, **common)
    else:
        sensor = Camera(args.camera_device, **common)

    async def run():
        task = asyncio.create_task(serve(sensor))
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, task.cancel)
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(run())


if __name__ == "__main__":
    main()
