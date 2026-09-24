"""Provider entrypoints: run these in the environment containing the ROS drivers.

Application environments import device views from rsim instead. Factories below
are serialized only within the provider's own interpreter, never sent to clients.
"""
import asyncio
import json
from pathlib import Path

from ._driver_config import d435_setup, robin_setup
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
        transport=None,
        parameters=None,
        ros_args=None
    ):
    """Start RGB-D with arbitrary native ROS parameters and argv options.

    Precedence: convenience options < parameters < ros_args assignments.
    Only enabled image streams are collected. Application views must match the
    effective serial/profiles/history, but need no driver-specific parameters.
    """
    setup = d435_setup(
        serial, depth_profile, color_profile, parameters, ros_args
        )
    config = setup["config"]
    if stream not in ("color", "depth"):
        raise ValueError("stream must be color or depth")
    if stream not in setup["enabled"]:
        raise ValueError(
            f"D435 {stream} stream is disabled by ROS parameters"
            )
    log_path = str(
        Path(log_path).resolve()
        ) if log_path is not None else None

    def factory():
        from .ros import D435 as Source
        setup["options"].arguments(
        )  # Verify parameter files before sharing a recipe.
        return Bundle(
            **{
                name:
                    Source(
                        stream=name,
                        history=history,
                        log_path=log_path,
                        _setup=setup
                        )
                for name in setup["enabled"]
                },
            hz=2 * max(
                int(config[k].split("x")[-1])
                for k in ("depth_profile", "color_profile")
                ),
            history=history
            )

    return _d435_view(
        config,
        stream,
        history,
        factory=factory,
        transport=transport,
        provider_version=setup["options"].signature()
        )


def RobinW(
        ip="192.168.199.97",
        *,
        history=8,
        transport=None,
        parameters=None,
        ros_args=None,
        log_path=None
    ):
    """Start Seyond with native parameters/ROS argv; follow frame_topic/remaps."""
    setup = robin_setup(ip, parameters=parameters, ros_args=ros_args)
    log_path = str(
        Path(log_path).resolve()
        ) if log_path is not None else None

    def factory():
        from .ros import RobinW as Source
        setup["options"].arguments()
        return Source(history=history, log_path=log_path, _setup=setup)

    return SharedSensor(
        factory,
        key=f"robin:{setup['ip']}",
        version="robin-v1",
        history=history,
        transport=transport,
        provider_version=setup["options"].signature()
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


def _parse_args(argv=None):
    import argparse
    import sys
    argv = list(sys.argv[1:] if argv is None else argv)
    split = argv.index("--ros-args") if "--ros-args" in argv else len(argv)
    ros_args = argv[split:]
    parser = argparse.ArgumentParser(
        description="Start a shared ROS hardware provider",
        epilog="Append --ros-args -p NAME:=VALUE --params-file FILE -r FROM:=TO "
        "to pass native ROS options to d435/robin."
        )
    parser.add_argument("device", choices=("d435", "robin", "camera"))
    parser.add_argument("--serial", default="")
    parser.add_argument("--ip")
    parser.add_argument("--camera-device", default="/dev/video0")
    parser.add_argument("--depth-profile", default="640x480x15")
    parser.add_argument("--color-profile", default="640x480x15")
    parser.add_argument(
        "--stream", choices=("color", "depth"), default="depth"
        )
    parser.add_argument("--history", type=int, default=8)
    parser.add_argument(
        "--backend", choices=("cyclonedds", "ros2"), default=None
        )
    parser.add_argument("--domain", type=int, default=None)
    parser.add_argument("--log-path", type=Path)
    args = parser.parse_args(argv[:split])
    if args.device == "robin" and not args.ip and not ros_args:
        parser.error("robin requires --ip or a native lidar_ip parameter")
    if args.device == "camera" and ros_args:
        parser.error(
            "camera uses OpenCV/UVC, not a native ROS driver; --ros-args is unsupported"
            )
    args.ros_args = ros_args
    return args


def _sensor_from_args(args):
    from .transport import TransportConfig
    defaults = TransportConfig()
    transport = TransportConfig(
        args.backend or defaults.backend,
        defaults.domain_id if args.domain is None else args.domain
        )
    common = {"history": args.history, "transport": transport}
    if args.log_path:
        args.log_path.parent.mkdir(parents=True, exist_ok=True)
    if args.device == "d435":
        return D435(
            serial=args.serial,
            stream=args.stream,
            depth_profile=args.depth_profile,
            color_profile=args.color_profile,
            log_path=args.log_path,
            ros_args=args.ros_args,
            **common
            )
    elif args.device == "robin":
        return RobinW(
            args.ip or "",
            ros_args=args.ros_args,
            log_path=args.log_path,
            **common
            )
    else:
        return Camera(args.camera_device, **common)


def main(argv=None):
    import signal
    sensor = _sensor_from_args(_parse_args(argv))

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
