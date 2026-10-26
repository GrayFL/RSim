#!/usr/bin/env python2
"""Local ROS1 relay, owned by RSim. Python 2.7, ROS and NumPy.

No nonzero ROS messages are published unless --enable-motion was explicitly
passed by the owner AND every independent gate below is satisfied.
"""
from __future__ import print_function
import argparse
import fcntl
import json
import math
import os
import signal
import socket
import select
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rsim', 'adapters'))
from ros1_wire import VERSION, read_packet, write_packet, notification_address
from ros1_agent import encode, monotonic, monotonic_ns
from ros1_hih_hardware import PedalMonitor


class LatestSampler(object):
    """Rate cap with a retained trailing sample, never every-N-frame aliasing."""
    def __init__(self):
        self.pending, self.next = {}, {}

    def push(self, name, message, spec, now):
        self.pending[name] = (message, spec, now)

    def due(self, now):
        result = []
        for name in list(self.pending):
            if now + 1e-9 < self.next.get(name, 0.): continue
            message, spec, received = self.pending.pop(name)
            period = 1. / spec.get('hz', 20)
            self.next[name] = max(now, self.next.get(name, now)) + period
            result.append((name, message, spec, received))
        return result

    def delay(self, now):
        return min([max(0., self.next.get(name, now)-now) for name in self.pending] + [.1])


def depth_limited_points(points, maximum, axis=(0., 0., 1.), offset=0.):
    """Keep valid camera optical depth before losing height in projection.

    axis/offset are the optical-Z row of the input-frame to camera transform.
    A body-frame planar range is not a camera depth measurement.
    """
    import numpy as np
    axis = np.asarray(axis, float)
    if (axis.shape != (3,) or not np.isfinite(axis).all()
            or abs(float(np.linalg.norm(axis))-1.) > 1e-4
            or not finite(offset) or not finite(maximum) or not 0 < maximum <= 10):
        raise ValueError('invalid camera depth filter')
    xyz = np.asarray(points, dtype=float).reshape(-1, 3)
    depth = xyz.dot(axis) + offset
    keep = np.isfinite(xyz).all(axis=1) & (depth > 0) & (depth <= maximum)
    return xyz[keep], depth[keep]


def project_ranges(points, spec):
    """Project base-frame obstacles, retaining height before the 2-D reduction."""
    import numpy as np
    low, high = float(spec['min_height']), float(spec['max_height'])
    amin, amax, step = [float(spec[k]) for k in ('angle_min', 'angle_max', 'angle_increment')]
    rmin, rmax = float(spec['range_min']), float(spec['range_max'])
    if (not all(finite(v) for v in (low, high, amin, amax, step, rmin, rmax))
            or not 0 <= low < high <= 3 or not 0 < rmin < rmax <= 50
            or not 0 < step <= .1 or not 0 < amax-amin <= 2*math.pi):
        raise ValueError('invalid cloud projection')
    count = int(round((amax-amin)/step))+1
    if count > 4000: raise ValueError('oversized cloud projection')
    ranges = np.full(count, np.inf, dtype=np.float32)
    xyz = np.asarray(points, dtype=float).reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= low) & (xyz[:, 2] <= high)]
    if len(xyz):
        radius, angle = np.hypot(xyz[:, 0], xyz[:, 1]), np.arctan2(xyz[:, 1], xyz[:, 0])
        keep = (radius >= rmin) & (radius <= rmax) & (angle >= amin) & (angle <= amax)
        indices = np.rint((angle[keep]-amin)/step).astype(int)
        np.minimum.at(ranges, np.clip(indices, 0, count-1), radius[keep])
    return ranges


class VelocityGate(object):
    """Final boundary: immutable deployment mode, freshness, TTL and anti-replay."""
    def __init__(self, session, enabled=False, max_linear=1.6, max_angular=.8,
                 min_turn_radius=.3, rear_extent=.81, rear_lateral_speed=.12, profiles=None, min_footprint_rear=.27):
        self.session, self.enabled = session, enabled
        self.max_linear, self.max_angular = max_linear, max_angular
        self.sequence, self.epoch = 0, None
        self.accepted, self.deadline = (0., 0.), 0
        self.reason = 'no_command'
        self.min_turn_radius, self.rear_extent, self.rear_lateral_speed = min_turn_radius, rear_extent, rear_lateral_speed
        self.requested_arm = False
        self.manual_direct = False
        if not finite(min_footprint_rear) or not 0 < min_footprint_rear <= .81:
            raise ValueError('invalid minimum collision footprint')
        self.min_footprint_rear = min_footprint_rear
        self.footprint_rear = .81
        self.profiles = profiles or {}

    def update(self, packet, now_ns):
        if not packet or packet.get('session') != self.session:
            return
        sequence = packet.get('sequence', 0)
        if isinstance(sequence, bool) or not isinstance(sequence, (int, float)) or not finite(sequence) or sequence != int(sequence) or sequence <= self.sequence:
            return
        self.sequence = sequence
        try:
            deadline = packet['deadline_ns']
            epoch = packet['controller_epoch']
            v, w = float(packet['linear_x']), float(packet['angular_z'])
            if (not epoch or (self.epoch is not None and epoch != self.epoch)
                    or not all(math.isfinite(x) for x in (v, w))):
                raise ValueError('invalid controller or velocity')
            if not now_ns < deadline <= now_ns + 500000000:
                raise ValueError('expired or excessive TTL')
            mode = packet.get('drive_mode', 'towing')
            if mode not in ('towing', 'differential'): raise ValueError('unknown drive mode')
            profile = self.profiles.get(mode, {})
            if abs(v) > min(1.6, self.max_linear, profile.get('max_linear', self.max_linear)) + 1e-9 or abs(w) > min(self.max_angular, profile.get('max_angular', self.max_angular)) + 1e-9:
                raise ValueError('velocity outside configured limits')
            if mode == 'towing' and (abs(w) * profile.get('min_turn_radius', self.min_turn_radius) > abs(v) + 1e-8 or abs(w) * profile.get('rear_extent', self.rear_extent) > profile.get('rear_lateral_speed', self.rear_lateral_speed) + 1e-8):
                raise ValueError('velocity violates towing envelope')
            self.epoch, self.deadline = epoch, deadline
            self.requested_arm = bool(packet.get('armed'))
            self.manual_direct = packet.get('manual_direct') is True
            self.footprint_rear = float(packet.get('footprint_rear', .81))
            if not finite(self.footprint_rear) or self.footprint_rear < self.min_footprint_rear:
                raise ValueError('invalid collision footprint')
            self.accepted = (v, w) if packet.get('armed') else (0., 0.)
            self.reason = 'accepted' if packet.get('armed') else 'disarmed'
        except (KeyError, ValueError, TypeError):
            self.deadline, self.accepted, self.reason = 0, (0., 0.), 'invalid_command'
            self.requested_arm = False

    def output(self, now_ns, healthy, limits):
        if not self.enabled:
            return (0., 0.), 'zero_only'
        if now_ns >= self.deadline:
            return (0., 0.), 'command_expired'
        if healthy != 'ok':
            return (0., 0.), healthy
        v, w = self.accepted
        # A live cap reduction invalidates a queued command; do not alter a
        # collision-checked curvature into a different trajectory.
        if abs(v) > limits[0] + 1e-9 or abs(w) > limits[1] + 1e-9:
            return (0., 0.), 'live_limit_reduced'
        return (v, w), self.reason


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


# math.isfinite is missing on Python 2.
if not hasattr(math, 'isfinite'):
    math.isfinite = finite


class Relay(object):
    def __init__(self, config, directory, session, enabled, role='combined'):
        import rospy
        import tf
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from roslib.message import get_message_class
        self.rospy, self.tf, self.Twist = rospy, tf, Twist
        self.config, self.directory, self.session = config, directory, session
        self.role = role
        self.commands = role in ('command', 'combined')
        self.poses = role in ('pose', 'combined')
        self.notify = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.notify.setblocking(False)
        self.command_socket = None
        if self.commands:
            self.command_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.command_socket.bind(notification_address(session, 'command'))
            self.command_socket.setblocking(False)
        self.stopped = threading.Event()
        self.wake = threading.Event()
        self.sampler = LatestSampler()
        self.scan_pending, self.scan_wakes = {}, {}
        self.pending, self.lock, self.counters, self.received = {}, threading.Lock(), {}, {}
        self.estop, self.bumper, self.external_until = None, None, 0.
        self.scheduler = None
        self.limits, self.limits_time = (0., 0.), 0.
        self.output_publishers = []
        self.output_activity = {}
        self.ownership_time = 0.
        self.pedal_monitor = PedalMonitor(config['pedal_feedback']) if self.commands and config.get('pedal_feedback') else None
        self.gate = VelocityGate(session, enabled, config['max_linear'], config['max_angular'],
                                 config.get('min_turn_radius', .3), config.get('rear_extent', .81),
                                 config.get('rear_lateral_speed', .12), config.get('profiles'),
                                 config.get('pedal_feedback', {}).get('retracted_rear', .27))
        self.listener = tf.TransformListener() if role != 'command' else None
        self.monitor = rospy.Publisher(config.get('monitor_topic', '/rsim/cmd_vel'), Twist, queue_size=1) if self.commands else None
        self.hardware = None
        self.hardware_lock = None
        self.hardware_active = False
        self.pose_time = 0.
        if enabled and self.commands:
            topic = config['command_topic']
            import hashlib
            lockpath = '/tmp/rsim-command-%s-%s.lock' % (os.getuid(), hashlib.sha256(topic.encode('utf8')).hexdigest())
            self.hardware_lock = open(lockpath, 'a+')
            fcntl.flock(self.hardware_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.hardware = rospy.Publisher(topic, Twist, queue_size=1)
        self.output_observer = rospy.Subscriber(config['command_topic'], Twist, self.observe_output, queue_size=20) if self.commands else None
        self.initialpose = rospy.Publisher(config.get('initialpose_topic', '/initialpose'), PoseWithCovarianceStamped, queue_size=1) if self.commands else None
        self.subscriptions, self.by_name = [], {}
        for name, spec in config['topics'].items():
            owner = ('command' if name in ('estop', 'bumper', 'scheduler', 'raw_odom') else
                     'pose' if spec['type'] in ('nav_msgs/Odometry', 'sensor_msgs/Imu') else 'sensors')
            if role != 'combined' and role != owner: continue
            cls = get_message_class(spec['type'])
            if cls is None:
                raise ValueError('ROS type unavailable: ' + spec['type'])
            if spec['type'] == 'sensor_msgs/LaserScan' or spec.get('projection'):
                self.scan_wakes[name] = threading.Event()
                worker = threading.Thread(target=self.scan_loop, args=(name,))
                worker.daemon = True
                worker.start()
            subscription = rospy.Subscriber(spec['topic'], cls, self.callback, (name, spec), queue_size=1, tcp_nodelay=True)
            self.subscriptions.append(subscription)
            self.by_name[name] = subscription
        self.writer = threading.Thread(target=self.write_loop)
        self.writer.daemon = True
        self.writer.start()
        if self.commands:
            self.params = threading.Thread(target=self.parameter_loop)
            self.params.daemon = True
            self.params.start()
        self.last_action = 0

    def put(self, name, data, stamp_ns=0, **extra):
        with self.lock:
            seq = self.counters.get(name, 0) + 1
            self.counters[name] = seq
            self.pending[name] = dict(version=VERSION, session=self.session, sequence=seq,
                                      received_ns=monotonic_ns(), stamp_ns=stamp_ns, data=data, **extra)
        self.wake.set()

    def transform(self, target, source, stamp):
        tr, q = self.listener.lookupTransform(target, source, stamp)
        return {'translation': tr, 'quaternion': q}

    def callback(self, message, args):
        name, spec = args
        now = monotonic()
        self.received[name] = now
        if name == 'estop': self.estop = bool(message.data)
        if name == 'bumper': self.bumper = bool(message.data)
        if name == 'scheduler': self.scheduler = message.data
        # Change-only interlocks update immediately, even within one rate slot.
        if name not in ('estop', 'bumper', 'scheduler'):
            with self.lock: self.sampler.push(name, message, spec, now)
            self.wake.set()
            return
        self.convert(name, message, spec, now)

    def convert(self, name, message, spec, received):
        stamp = message.header.stamp if hasattr(message, 'header') else self.rospy.Time.now()
        extra = {}
        is_scan = spec['type'] == 'sensor_msgs/LaserScan' or spec.get('projection')
        if spec.get('projection'):
            import numpy as np
            from sensor_msgs import point_cloud2
            from sensor_msgs.msg import LaserScan
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, self.config['base_frame']
            projection = spec['projection']
            for key in ('angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max'):
                setattr(scan, key, projection[key])
            scan.scan_time = 1. / spec.get('hz', 8)
            try:
                points = np.asarray(list(point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True))).reshape(-1, 3)
                if spec.get('depth_filter'):
                    rule = spec['depth_filter']
                    frame, maximum = rule['frame'], float(rule['max_depth'])
                    axis, offset = (0., 0., 1.), 0.
                    if message.header.frame_id.lstrip('/') != frame.lstrip('/'):
                        self.listener.waitForTransform(frame, message.header.frame_id, stamp, self.rospy.Duration(.15))
                        tr = self.transform(frame, message.header.frame_id, stamp)
                        axis = self.tf.transformations.quaternion_matrix(tr['quaternion'])[2, :3]
                        offset = tr['translation'][2]
                    source_count = len(points)
                    points, depths = depth_limited_points(points, maximum, axis, offset)
                    extra['depth_filter'] = dict(frame=frame, max_depth=maximum, source_points=source_count,
                        accepted_points=len(points), max_accepted_depth=float(depths.max()) if len(depths) else None)
                if message.header.frame_id.lstrip('/') != self.config['base_frame'].lstrip('/'):
                    self.listener.waitForTransform(self.config['base_frame'], message.header.frame_id, stamp, self.rospy.Duration(.15))
                    tr = self.transform(self.config['base_frame'], message.header.frame_id, stamp)
                    rotation = self.tf.transformations.quaternion_matrix(tr['quaternion'])[:3, :3]
                    points = points.dot(rotation.T) + tr['translation']
                scan.ranges = project_ranges(points, projection).tolist()
                extra['projection'] = dict(projection, source_topic=spec['topic'], source_points=len(points))
            except Exception as error:
                # An invalid projection must fail scan validation, never look
                # like a valid, obstacle-free frame.
                scan.ranges = [float('nan')]*8
                extra['projection_error'] = str(error)
            message = scan
        if is_scan:
            try:
                # Sensor messages commonly precede the matching odometry TF by
                # a few milliseconds. Wait only in this subscriber thread;
                # the independent command watchdog continues running.
                self.listener.waitForTransform(self.config['map_frame'], message.header.frame_id, stamp,
                                               self.rospy.Duration(.15))
                extra['world_transform'] = self.transform(self.config['map_frame'], message.header.frame_id, stamp)
                extra['body_transform'] = self.transform(self.config['base_frame'], message.header.frame_id, stamp)
            except Exception as error:
                extra['transform_error'] = str(error)
        extra['source_received_ns'] = int(received * 1e9)
        self.put(name, encode(message), stamp.to_nsec(), **extra)

    def parameter_loop(self):
        """Monitor the direct output boundary, independent of OEM task/TEB state."""
        while not self.stopped.is_set():
            try:
                v = min(1.6, float(self.config['max_linear']))
                w = float(self.config['max_angular'])
                if finite(v) and finite(w) and v > 0 and w > 0:
                    self.limits, self.limits_time = (v, w), monotonic()
                state = self.rospy.get_master().getSystemState()
                if state[0] == 1:
                    publishers = dict(state[2][0]).get(self.config['command_topic'], [])
                    self.output_publishers = [n for n in publishers if n != self.rospy.get_name()]
                    self.ownership_time = monotonic()
            except Exception:
                pass
            self.stopped.wait(1.)

    def observe_output(self, message):
        publisher = message._connection_header.get('callerid', 'unknown')
        if publisher == self.rospy.get_name(): return
        with self.lock:
            previous = self.output_activity.get(publisher, {})
            self.output_activity[publisher] = dict(at=monotonic(), count=previous.get('count', 0) + 1,
                velocity=[message.linear.x, message.angular.z])

    def output_conflicts(self):
        # Explicit deployment migration option. A permitted publisher may
        # still overwrite our output; report it, never silently claim exclusive
        # ownership. Unknown publishers remain a wiring error.
        permitted = self.config.get('allowed_output_publishers', [])
        return [name for name in self.output_publishers if name not in permitted]

    def output_observations(self, now):
        with self.lock:
            return {name: dict(count=value['count'], velocity=value['velocity'], age_sec=round(now-value['at'], 3))
                    for name, value in self.output_activity.items() if now-value['at'] < 10.}

    def write_loop(self):
        try:
            while not self.stopped.is_set():
                self.wake.clear()
                with self.lock: due = self.sampler.due(monotonic())
                for args in due:
                    name = args[0]
                    if name in self.scan_wakes:
                        with self.lock: self.scan_pending[name] = args
                        self.scan_wakes[name].set()
                    else: self.convert(*args)
                with self.lock:
                    pending, self.pending = self.pending, {}
                for name, packet in pending.items():
                    packet['written_ns'] = monotonic_ns()
                    write_packet(self.directory, name, packet)
                    try: self.notify.sendto(name.encode('ascii'), notification_address(self.session, 'updates'))
                    except socket.error: pass  # Owner has a bounded recovery scan.
                with self.lock: delay = self.sampler.delay(monotonic())
                self.wake.wait(delay)
        except Exception as error:
            print('mailbox writer failed: ' + str(error), file=sys.stderr)
            self.stopped.set()

    def scan_loop(self, name):
        # Each scan may wait for a different acquisition TF. A depth frame
        # must not block the line lidar or telemetry writer behind that wait.
        wake = self.scan_wakes[name]
        try:
            while not self.stopped.is_set():
                wake.wait(.2)
                wake.clear()
                with self.lock: args = self.scan_pending.pop(name, None)
                if args: self.convert(*args)
        except Exception as error:
            print('scan conversion failed: ' + str(error), file=sys.stderr)
            self.stopped.set()

    def health(self, now, hardware_only=False):
        if self.estop is not False: return 'estop_or_unknown'
        # These OEM publishers are latched, change-only events. Their age is
        # not a heartbeat. Require live connections plus the driver's raw odom.
        if self.by_name['estop'].get_num_connections() == 0: return 'estop_disconnected'
        if self.bumper is True: return 'bumper'
        if self.bumper is None and self.config.get('require_bumper_initial', False): return 'bumper_unknown'
        if self.by_name['bumper'].get_num_connections() == 0: return 'bumper_disconnected'
        # No upstream task/TEB veto. Shared output is allowed only for explicitly
        # configured legacy publishers and is visible as a warning in status.
        if self.gate.enabled:
            if now - self.ownership_time > 3.: return 'output_ownership_unknown'
            if self.output_conflicts(): return 'output_publisher_conflict'
        if now - self.limits_time > 3.: return 'stale_speed_limits'
        if now - self.received.get('raw_odom', now if 'raw_odom' not in self.config.get('topics', {}) else -1e10) > .6: return 'stale_raw_odom'
        if hardware_only: return 'ok'
        if getattr(self, 'role', 'combined') == 'command':
            received = dict(self.received)
            for role in ('pose', 'sensors'):
                peer = read_packet(self.directory, 'health_' + role)
                if not peer or peer.get('session') != self.session or now - peer.get('received_ns', 0)/1e9 > .6:
                    return 'stale_' + role + '_process'
                received.update(peer['data']['received'])
                if role == 'pose': self.pose_time = peer['data']['pose_time']
        else: received = self.received
        if now - self.pose_time > .6: return 'stale_transform'
        for name in self.config.get('required_fresh', ['scan', 'odom']):
            if now - received.get(name, -1e10) > .6: return 'stale_' + name
        return 'ok'

    def action(self, packet, now_ns):
        if not packet or packet.get('session') != self.session: return
        seq = packet.get('sequence', 0)
        if seq <= self.last_action: return
        self.last_action = seq
        result = {'sequence': seq, 'ok': False}
        try:
            if not now_ns < packet['deadline_ns'] <= now_ns + 2000000000:
                raise ValueError('action expired')
            if packet['op'] != 'initialpose': raise ValueError('unsupported action')
            if not self.config.get('allow_localization', False): raise ValueError('localization writes disabled')
            if self.gate.accepted != (0., 0.): raise ValueError('stop before relocalization')
            from geometry_msgs.msg import PoseWithCovarianceStamped
            x, y, yaw = [float(packet[k]) for k in ('x', 'y', 'yaw')]
            if not all(finite(v) for v in (x, y, yaw)): raise ValueError('invalid pose')
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.config['map_frame']
            msg.header.stamp = self.rospy.Time.now()
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.pose.covariance[0] = msg.pose.covariance[7] = .25
            msg.pose.covariance[35] = .07
            self.initialpose.publish(msg)
            result['ok'] = True
        except Exception as error:
            result['error'] = str(error)
        self.put('action_result', result)

    def run(self):
        last_pose = last_status = 0.
        pose_period = 1. / self.config.get('pose_hz', 20)
        next_pose = monotonic()
        self.put('ready' if self.role == 'combined' else 'ready_' + self.role,
                 {'pid': os.getpid(), 'motion_enabled': self.gate.enabled, 'role': self.role})
        try:
            while not self.stopped.is_set() and not self.rospy.is_shutdown():
                now, now_ns = monotonic(), monotonic_ns()
                heartbeat = read_packet(self.directory, 'heartbeat')
                if not heartbeat or heartbeat.get('session') != self.session or not -50000000 <= now_ns - heartbeat.get('stamp_ns', 0) <= 1000000000:
                    break
                if self.commands:
                    self.gate.update(read_packet(self.directory, 'command'), now_ns)
                    health, hardware_health = self.health(now), self.health(now, hardware_only=True)
                    pedal = self.pedal_monitor.snapshot() if self.pedal_monitor else None
                    effective = hardware_health if self.gate.manual_direct else health
                    if effective == 'ok' and pedal and self.gate.requested_arm and self.gate.footprint_rear < .81 and not pedal['retracted']:
                        effective = 'pedal_footprint_mismatch'
                    velocity, reason = self.gate.output(now_ns, effective, self.limits)
                    msg = self.Twist()
                    msg.linear.x, msg.angular.z = velocity
                    self.monitor.publish(msg)
                    owns_hardware = self.gate.requested_arm and now_ns < self.gate.deadline and effective == 'ok'
                    if self.hardware is not None and (owns_hardware or self.hardware_active): self.hardware.publish(msg)
                    self.hardware_active = owns_hardware
                    self.action(read_packet(self.directory, 'action'), now_ns)
                if self.poses and now >= next_pose:
                    next_pose = max(next_pose + pose_period, now)
                    try:
                        stamp = self.listener.getLatestCommonTime(self.config['map_frame'], self.config['base_frame'])
                        pose = self.transform(self.config['map_frame'], self.config['base_frame'], self.rospy.Time(0))
                        pose['frame'], pose['body'] = self.config['map_frame'], self.config['base_frame']
                        pose['ros_age'] = (self.rospy.Time.now() - stamp).to_sec()
                        if -.1 <= pose['ros_age'] <= .5: self.pose_time = now
                        self.put('pose', pose, stamp.to_nsec())
                    except Exception:
                        pass
                if now - last_status >= .1:
                    last_status = now
                    if not self.commands:
                        self.put('health_' + self.role, dict(received=dict(self.received), pose_time=self.pose_time, pid=os.getpid()))
                    else:
                        self.publish_status(health, hardware_health, reason, pedal, velocity, now)
                if self.command_socket is not None:
                    if select.select([self.command_socket], [], [], .02)[0]:
                        try:
                            while self.command_socket.recv(64): pass
                        except socket.error: pass
                else:
                    self.stopped.wait(max(.001, min(next_pose, last_status+.1)-monotonic()) if self.poses else .05)
        finally:
            self.stopped.set()
            self.wake.set()
            if self.commands:
                for _ in range(3):
                    self.monitor.publish(self.Twist())
                    if self.hardware is not None and self.hardware_active: self.hardware.publish(self.Twist())
                    time.sleep(.02)
            self.rospy.signal_shutdown('RSim owner stopped')
            if self.pedal_monitor: self.pedal_monitor.close()
            if self.hardware_lock is not None: self.hardware_lock.close()
            if self.command_socket is not None: self.command_socket.close()
            self.notify.close()

    def publish_status(self, health, hardware_health, reason, pedal, velocity, now):
        self.put('status', {'health': health, 'reason': reason, 'estop': self.estop,
            'hardware_health': hardware_health, 'manual_direct': self.gate.manual_direct,
            'native_pid': os.getpid(), 'native_role': self.role,
            'bumper': self.bumper, 'scheduler': self.scheduler,
            'motion_enabled': self.gate.enabled, 'published': list(velocity),
            'limits': list(self.limits), 'command_sequence': self.gate.sequence,
            'command_topic': self.config['command_topic'], 'output_publishers': self.output_publishers,
            'pedal': pedal,
            'output_warning': 'shared_output' if self.output_publishers and not self.output_conflicts() else None,
            'output_activity': self.output_observations(now),
            'exclusive_output': not self.output_publishers,
            'ownership_ready': not self.output_conflicts() and now - self.ownership_time <= 3.,
            'ages': {k: now - v for k, v in self.received.items()}})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--lockfile', required=True)
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--role', choices=('combined', 'command', 'pose', 'sensors'), default='combined')
    args = parser.parse_args()
    with open(args.config) as stream: config = json.load(stream)
    lease = open(args.lockfile, 'a+')
    fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    import rospy
    rospy.init_node('rsim_local_' + args.role, anonymous=True, disable_signals=True)
    relay = Relay(config, args.directory, args.session, args.enable_motion, args.role)
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: relay.stopped.set())
    relay.run()


if __name__ == '__main__':
    main()
