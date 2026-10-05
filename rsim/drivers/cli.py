"""Provider CLI and async lease serving."""
import asyncio
from pathlib import Path
from . import D435, RobinW, Camera, Hipnuc

async def serve(sensor):
    """Hold a provider lease until cancelled; suitable for create_task in notebooks."""
    from rsim.runtime.graph import Runtime
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
    parser.add_argument("device", choices=("d435", "robin", "camera", "imu"))
    parser.add_argument("--port", help="IMU serial port; automatic only with one CP210x")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--imu-mode", choices=("serial", "ros2"), default="serial")
    parser.add_argument("--frame-id", default="hipnuc_imu")
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
    from rsim.transport.descriptor import TransportConfig
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
    elif args.device == "imu":
        return Hipnuc(args.port, mode=args.imu_mode, baudrate=args.baudrate,
                      frame_id=args.frame_id, ros_args=args.ros_args, log_path=args.log_path, **common)
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
