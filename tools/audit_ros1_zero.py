#!/usr/bin/env python2
"""Bounded read-only ROS velocity audit. Never creates a publisher."""
from __future__ import print_function
import argparse
import json
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', required=True)
    parser.add_argument('--monitor', default='/rsim/cmd_vel')
    parser.add_argument('--hardware', default='/cmd_vel/joy_pub')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120: raise ValueError('duration must be 1..120 seconds')
    import rospy
    import rosgraph
    from geometry_msgs.msg import Twist
    rospy.init_node('rsim_zero_audit', anonymous=True, disable_signals=True)
    lock, stats = threading.Lock(), {}
    def received(message, topic):
        caller = getattr(message, '_connection_header', {}).get('callerid', 'unknown')
        key = topic + ' <- ' + caller
        value = max(abs(message.linear.x), abs(message.linear.y), abs(message.linear.z),
                    abs(message.angular.x), abs(message.angular.y), abs(message.angular.z))
        with lock:
            item = stats.setdefault(key, {'count': 0, 'nonzero': 0, 'max_abs': 0.})
            item['count'] += 1
            item['nonzero'] += int(value > 1e-9)
            item['max_abs'] = max(item['max_abs'], value)
    topics = [args.monitor, args.hardware]
    master = rosgraph.Master(rospy.get_name())
    def publishers():
        return {topic: nodes for topic, nodes in master.getSystemState()[0] if topic in topics}
    before = publishers()
    subscriptions = [rospy.Subscriber(topic, Twist, received, topic, queue_size=100) for topic in topics]
    time.sleep(args.seconds)
    after = publishers()
    for subscription in subscriptions: subscription.unregister()
    with lock: result = dict(seconds=args.seconds, messages=stats.copy(), publishers_before=before, publishers_after=after)
    with open(args.output, 'w') as stream: json.dump(result, stream, indent=2, sort_keys=True)
    print(json.dumps(result, sort_keys=True))
    rospy.signal_shutdown('bounded audit complete')


if __name__ == '__main__': main()
