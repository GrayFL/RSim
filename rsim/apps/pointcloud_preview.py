"""Publish a bounded preview of an existing ROS PointCloud2 in another domain."""

import argparse
import logging
import math
import time

logger = logging.getLogger(__name__)


def decimate(message, *, max_points=1200, max_bytes=60000):
    """Uniformly sample records, retaining fields, byte order and source stamp.

    This is display decimation, not a mapping cloud or geometric voxel filter.
    Organized input may include row padding; no padding becomes a point.
    """
    from sensor_msgs.msg import PointCloud2
    import numpy as np

    if max_points < 1 or max_bytes < 1:
        raise ValueError("preview limits must be positive")
    step, width, height = message.point_step, message.width, message.height
    if step < 1 or message.row_step < width * step or len(message.data) < message.row_step * height:
        raise ValueError("invalid PointCloud2 layout")
    budget = min(max_points, max_bytes // step)
    if budget < 1:
        raise ValueError("one point exceeds the preview byte budget")
    count = width * height
    stride = max(1, math.ceil(count / budget))
    indices = np.arange(0, count, stride, dtype=np.int64)
    records = np.ndarray((height, width), dtype=f'V{step}', buffer=message.data,
                         strides=(message.row_step, step))
    data = records[indices // max(1, width), indices % max(1, width)].tobytes()
    return PointCloud2(header=message.header, height=1, width=len(indices),
                      fields=message.fields, is_bigendian=message.is_bigendian,
                      point_step=step, row_step=len(data), data=data,
                      is_dense=message.is_dense)


def main():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import PointCloud2

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='/iv_points')
    parser.add_argument('--output', default='/rsim/preview/points')
    parser.add_argument('--input-domain', type=int, default=0)
    parser.add_argument('--output-domain', type=int, default=43)
    parser.add_argument('--hz', type=float, default=3)
    parser.add_argument('--max-points', type=int, default=1200)
    parser.add_argument('--max-bytes', type=int, default=60000)
    args = parser.parse_args()
    if (not math.isfinite(args.hz) or not 0 < args.hz <= 30 or args.max_points < 1 or
            args.max_bytes < 1 or not all(0 <= d <= 232 for d in (args.input_domain, args.output_domain))):
        parser.error('invalid rate, point/byte budget or domain')
    if args.input_domain == args.output_domain:
        parser.error('preview requires a separate output domain')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    contexts, nodes = [], []
    executors = []
    latest = None
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def receive(message):
        nonlocal latest
        latest = message, time.monotonic()

    try:
        for domain, name in ((args.input_domain, 'rsim_preview_input'),
                             (args.output_domain, 'rsim_preview_output')):
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain,
                        signal_handler_options=SignalHandlerOptions.NO)
            contexts.append(context)
            node = rclpy.create_node(name, context=context)
            nodes.append(node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            executors.append(executor)
        nodes[0].create_subscription(PointCloud2, args.input, receive, qos)
        publisher = nodes[1].create_publisher(PointCloud2, args.output, qos)
        logger.info('Preview %s domain=%s -> %s domain=%s; <=%s points, <=%s bytes/frame, %.1f Hz',
                    args.input, args.input_domain, args.output, args.output_domain,
                    args.max_points, args.max_bytes, args.hz)
        due = time.monotonic()
        while True:
            executors[0].spin_once(timeout_sec=.01)
            executors[1].spin_once(timeout_sec=0)
            now = time.monotonic()
            if now < due:
                continue
            due = now + 1 / args.hz
            item, latest = latest, None
            if item is None or now - item[1] > .5 or not publisher.get_subscription_count():
                continue
            try:
                publisher.publish(decimate(item[0], max_points=args.max_points, max_bytes=args.max_bytes))
            except ValueError:
                logger.exception('Skipping malformed cloud')
    except KeyboardInterrupt:
        logger.info('Preview stopped')
    finally:
        for executor in executors:
            executor.shutdown()
        for node in nodes:
            node.destroy_node()
        for context in contexts:
            context.try_shutdown()


if __name__ == '__main__':
    main()
