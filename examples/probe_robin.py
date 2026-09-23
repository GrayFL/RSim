"""Run after conda activate rsim. Starts and reliably stops its own driver."""
import asyncio
import json
import signal
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


async def main():
    output = Path(__file__).resolve().parents[1] / "assets"
    output.mkdir(exist_ok=True)
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("rsim_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    frames = []

    def receive(msg):
        frames.append({
            "stamp_ns":
                msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
            "frame_id":
                msg.header.frame_id,
            "width":
                msg.width,
            "height":
                msg.height,
            "point_step":
                msg.point_step,
            "bytes":
                len(msg.data),
            "fields": [{
                "name": f.name,
                "offset": f.offset,
                "datatype": f.datatype,
                "count": f.count
                } for f in msg.fields],
            })

    node.create_subscription(
        PointCloud2, "/iv_points", receive, qos_profile_sensor_data
        )
    executable = Path(
        get_package_prefix("seyond")
        ) / "lib/seyond/seyond_node"
    process = None
    try:
        with (output / "robin-driver.log").open("w") as log:
            process = await asyncio.create_subprocess_exec(
                str(executable),
                "--ros-args",
                "-p",
                "lidar_ip:=192.168.199.97",
                stdout=log,
                stderr=log
                )
            async with asyncio.timeout(30):
                while len(frames) < 5:
                    if process.returncode is not None:
                        raise RuntimeError(
                            f"driver exited: {process.returncode}"
                            )
                    executor.spin_once(timeout_sec=0)
                    await asyncio.sleep(0.002)
            (output / "robin-frames.json").write_text(
                json.dumps(frames, indent=2)
                )
            print(json.dumps(frames[-1], indent=2))
    finally:
        if process is not None and process.returncode is None:
            process.send_signal(signal.SIGINT)
            try:
                await asyncio.wait_for(process.wait(), 5)
            except TimeoutError:
                process.kill()
                await process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
