"""Read a remote chassis, optionally mirror standard ROS2 topics or test stop."""
import argparse
import asyncio
import json
from pathlib import Path

from rsim import Chassis, Runtime, SSHConfig


async def run(args):
    assets = Path(__file__).resolve().parents[1] / "assets" / "chassis"
    assets.mkdir(parents=True, exist_ok=True)
    connection = SSHConfig(args.host, remote_script=args.remote_script, python=args.python,
                           setup=tuple(args.setup), master_uri=args.master_uri, ros_ip=args.ros_ip)
    chassis = Chassis(connection, imu_topic=args.imu, odom_topic=args.odom,
                      scan_topic=args.scan, cmd_vel_topic=args.cmd_vel, log_path=assets / "agent.log")
    root = chassis
    if args.ros2:
        from rsim.remote_ros2 import ChassisROS2
        root = ChassisROS2(chassis, prefix=args.ros2)
    async with Runtime(root):
        frames = await asyncio.gather(*(sensor.get(timeout=20)
                                       for sensor in (chassis.imu, chassis.odom, chassis.scan)))
        summary = {name: {"stamp_ns": frame.stamp_ns, "clock": frame.clock,
                          "frame_id": frame.data["header"]["frame_id"]}
                   for name, frame in zip(("imu", "odom", "scan"), frames)}
        summary["scan"]["beams"] = len(frames[2].data["ranges"])
        if args.stop_test:
            summary["zero_velocity"] = await chassis.stop()
        print(json.dumps(summary, indent=2), flush=True)
        (assets / "capture.json").write_text(json.dumps(summary, indent=2))
        previous = 0
        for _ in range(args.frames):
            frame = await chassis.get(after=previous, timeout=10)
            previous = frame.sequence
        if args.serve:
            while True:
                frame = await chassis.get(after=previous, timeout=10)
                previous = frame.sequence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--remote-script", default="Projects/RSim/compat/ros1_agent.py")
    parser.add_argument("--python", default="python2")
    parser.add_argument("--setup", action="append", required=True, help="remote ROS setup.bash; repeatable")
    parser.add_argument("--master-uri", default="http://127.0.0.1:11311")
    parser.add_argument("--ros-ip")
    parser.add_argument("--imu", default="/imu_data")
    parser.add_argument("--odom", default="/odom")
    parser.add_argument("--scan", default="/scan")
    parser.add_argument("--cmd-vel", default="/cmd_vel")
    parser.add_argument("--ros2", metavar="PREFIX", help="mirror ROS2 sensor and cmd_vel topics")
    parser.add_argument("--stop-test", action="store_true", help="send one all-zero Twist")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--serve", action="store_true")
    try:
        asyncio.run(run(parser.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
