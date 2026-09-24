#!/usr/bin/env python
"""ROS1 endpoint for an SSH stdio session. Python 2.7 / 3, ROS stdlib only.

Deploy this standalone file on the ROS1 computer; do not install the Python 3
RSim package there. stdout is a versioned JSON-lines wire, stderr is diagnostics.
"""
from __future__ import print_function

import array
import base64
from collections import deque
import ctypes
import fcntl
import hashlib
import json
import math
import os
import select
import signal
import struct
import sys
import threading
import time
import tempfile

try:
    integer_types = (int, long)
except NameError:
    integer_types = (int,)

MAX_MESSAGE = 8 * 1024 * 1024
ARRAY_TYPES = {"bool": "B", "int8": "b", "uint8": "B", "byte": "b", "char": "B",
               "int16": "h", "uint16": "H", "int32": "i", "uint32": "I",
               "int64": "q", "uint64": "Q", "float32": "f", "float64": "d"}


class Timespec(ctypes.Structure):
    _fields_ = [("sec", ctypes.c_long), ("nsec", ctypes.c_long)]


_clock = ctypes.CDLL(None).clock_gettime


def monotonic():
    value = Timespec()
    if _clock(1, ctypes.byref(value)) != 0:
        raise RuntimeError("CLOCK_MONOTONIC unavailable")
    return value.sec + value.nsec * 1e-9


def monotonic_ns():
    value = Timespec()
    if _clock(1, ctypes.byref(value)) != 0:
        raise RuntimeError("CLOCK_MONOTONIC unavailable")
    return value.sec * 1000000000 + value.nsec


class CommandRejected(ValueError):
    def __init__(self, message, reason="invalid"):
        ValueError.__init__(self, message)
        self.reason = reason


def encode(value, field_type=""):
    if "[" in field_type and field_type.split("[")[0] in ARRAY_TYPES:
        code = ARRAY_TYPES[field_type.split("[")[0]]
        if code in ("b", "B") and isinstance(value, bytes):
            raw = value
        elif code in ("q", "Q"):
            # Python 2.7 array.array does not support q/Q on every build.
            raw = struct.pack("<%d%s" % (len(value), code), *value)
        else:
            packed = array.array(code, value)
            if sys.byteorder != "little":
                packed.byteswap()
            raw = packed.tobytes() if hasattr(packed, "tobytes") else packed.tostring()
        return {"__array__": "?" if field_type.split("[")[0] == "bool" else code,
                "data": base64.b64encode(raw).decode("ascii")}
    if hasattr(value, "_slot_types"):
        return {name: encode(getattr(value, name), kind)
                for name, kind in zip(value.__slots__, value._slot_types)}
    if hasattr(value, "secs") and hasattr(value, "nsecs"):
        return {"secs": value.secs, "nsecs": value.nsecs}
    if isinstance(value, (tuple, list)):
        return [encode(item) for item in value]
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return {"__float__": "nan" if math.isnan(value) else ("inf" if value > 0 else "-inf")}
    return value


class Agent(object):
    def __init__(self, wire, rospy):
        self.wire, self.rospy = wire, rospy
        self.subscriptions, self.publishers, self.commands = {}, {}, {}
        self.latest, self.replies = {}, deque()
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.stopped = threading.Event()
        self.last_input = monotonic()
        self.command_lock = threading.Lock()
        self.command_states, self.command_leases = {}, {}

    def validate_command(self, topic, command):
        """Validate at the hardware provider, after any connection/queue delay."""
        now = monotonic_ns()
        if not isinstance(command, dict):
            raise CommandRejected("Twist requires a command envelope")
        identity = (command.get("controller_id"), command.get("controller_epoch"))
        sequence, deadline = command.get("sequence"), command.get("deadline_ns")
        if (not all(identity) or not isinstance(sequence, integer_types) or sequence <= 0
                or not isinstance(deadline, integer_types)):
            raise CommandRejected("invalid command envelope")
        if deadline <= now:
            raise CommandRejected("command expired", reason="expired")
        if deadline > now + 500000000:
            raise CommandRejected("command exceeds provider TTL")
        state = self.command_states.setdefault(topic, {"owner": None, "deadline": 0,
                                                      "sequences": {}, "retired": set()})
        if identity in state["retired"] or sequence <= state["sequences"].get(identity, 0):
            raise CommandRejected("replayed command or retired controller epoch")
        if state["owner"] is not None and state["owner"] != identity:
            if now < state["deadline"]:
                raise CommandRejected("exclusive command owner; use CommandMux")
        # Separate SSH sessions must not race writes to the same hardware topic.
        # This cooperative lock covers all RSim providers under this Unix user.
        if topic not in self.command_leases:
            key = hashlib.sha256(topic.encode("utf8")).hexdigest()
            path = os.path.join(tempfile.gettempdir(), "rsim-command-%s-%s.lock" % (os.getuid(), key))
            lease = open(path, "a+")
            try:
                fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except IOError:
                lease.close()
                raise CommandRejected("hardware command topic owned by another RSim session")
            self.command_leases[topic] = lease
        if state["owner"] is not None and state["owner"] != identity:
            state["retired"].add(state["owner"])
        state["owner"], state["deadline"] = identity, deadline
        state["sequences"][identity] = sequence
        return deadline * 1e-9

    def reply(self, packet):
        with self.lock:
            if len(self.replies) >= 64:
                raise RuntimeError("response queue is full")
            self.replies.append(packet)
        self.wake.set()

    def write_loop(self):
        try:
            while not self.stopped.is_set():
                self.wake.wait(.1)
                with self.lock:
                    packets = list(self.replies) + list(self.latest.values())
                    self.replies.clear()
                    self.latest.clear()
                    self.wake.clear()
                for packet in packets:
                    data = json.dumps(packet, allow_nan=False, separators=(",", ":")).encode("utf8")
                    if len(data) > MAX_MESSAGE:
                        raise ValueError("outgoing message exceeds size limit")
                    self.wire.write(data + b"\n")
        except Exception as error:
            print("wire closed: " + str(error), file=sys.stderr)
            self.stopped.set()

    def subscribe(self, request):
        from roslib.message import get_message_class
        topic, kind = request["topic"], request.get("type")
        if topic in self.subscriptions:
            raise ValueError("topic already subscribed")
        if kind is None:
            kind = dict(self.rospy.get_published_topics()).get(topic)
        cls = get_message_class(kind) if kind else None
        if cls is None:
            raise ValueError("unknown ROS1 message type for " + topic)
        hz = float(request.get("hz", 50))
        if not 0 < hz <= 1000:
            raise ValueError("subscription hz must be in (0, 1000]")
        previous = [0.0]

        def receive(message):
            now = monotonic()
            if now - previous[0] < 1.0 / hz:
                return
            previous[0] = now
            stamp = message.header.stamp if hasattr(message, "header") else self.rospy.Time.now()
            packet = {"op": "sample", "topic": topic, "type": cls._type,
                      "stamp_ns": stamp.to_nsec(), "data": encode(message)}
            with self.lock:
                self.latest[topic] = packet
            self.wake.set()

        self.subscriptions[topic] = self.rospy.Subscriber(topic, cls, receive, queue_size=1)
        return {"type": cls._type}

    def publish(self, request):
        from roslib.message import get_message_class
        from genpy.message import fill_message_args
        topic, kind = request["topic"], request["type"]
        cls = get_message_class(kind)
        if cls is None:
            raise ValueError("unknown ROS1 message type " + kind)
        message = cls()
        fill_message_args(message, [request["data"]])
        # Validate serialization before enqueueing a message in rospy's thread.
        import io
        message.serialize(io.BytesIO())
        if kind == "geometry_msgs/Twist":
            values = [getattr(vector, axis) for vector in (message.linear, message.angular)
                      for axis in ("x", "y", "z")]
            if any(math.isnan(v) or math.isinf(v) for v in values):
                raise ValueError("velocity must be finite")
        self.advertise(request)
        _, publisher = self.publishers[topic]
        deadline = monotonic() + 2.0
        while publisher.get_num_connections() == 0:
            if monotonic() >= deadline or self.stopped.is_set():
                raise RuntimeError("no ROS1 subscriber connected to " + topic)
            time.sleep(.01)
        with self.command_lock:
            command_deadline = None
            if kind == "geometry_msgs/Twist":
                command_deadline = self.validate_command(topic, request.get("command"))
                self.commands[topic] = (command_deadline, cls)
            publisher.publish(message)
        return {"published": True, "connections": publisher.get_num_connections()}

    def advertise(self, request):
        from roslib.message import get_message_class
        topic, kind = request["topic"], request["type"]
        cls = get_message_class(kind)
        if cls is None:
            raise ValueError("unknown ROS1 message type " + kind)
        if topic not in self.publishers:
            self.publishers[topic] = (kind, self.rospy.Publisher(topic, cls, queue_size=1, latch=False))
        if self.publishers[topic][0] != kind:
            raise ValueError("conflicting publisher type")
        return {"advertised": True}

    def stop_expired(self, all_commands=False):
        errors = []
        with self.command_lock:
            for topic, (deadline, cls) in list(self.commands.items()):
                if all_commands or monotonic() >= deadline:
                    try:
                        self.publishers[topic][1].publish(cls())
                    except Exception as error:
                        errors.append(str(error))
                    finally:
                        del self.commands[topic]
                        lease = self.command_leases.pop(topic, None)
                        if lease is not None:
                            lease.close()
        if errors:
            raise RuntimeError("zero command failed: " + "; ".join(errors))

    def stop(self, request):
        topic = request["topic"]
        identity = (request.get("controller_id"), request.get("controller_epoch"))
        with self.command_lock:
            state = self.command_states.get(topic)
            if state is None or state["owner"] != identity:
                raise CommandRejected("stop request does not own this command session")
            command = self.commands.pop(topic, None)
            try:
                if command is not None:
                    self.publishers[topic][1].publish(command[1]())
            finally:
                lease = self.command_leases.pop(topic, None)
                if lease is not None:
                    lease.close()
                state["deadline"] = 0
        return {"stopped": True}

    def watchdog(self):
        try:
            while not self.stopped.wait(.05):
                self.stop_expired()
                if monotonic() - self.last_input > 10:
                    self.stopped.set()
        except Exception as error:
            print("command watchdog failed: " + str(error), file=sys.stderr)
            self.stopped.set()

    def dispatch(self, request):
        op = request["op"]
        if op == "ping":
            return {"alive": True, "monotonic_ns": monotonic_ns()}
        if op == "stop":
            return self.stop(request)
        if op == "topics":
            import rosgraph
            master = rosgraph.Master(self.rospy.get_name())
            publishers, subscribers, _ = master.getSystemState()
            pubs, subs = dict(publishers), dict(subscribers)
            return [{"topic": topic, "type": kind, "publishers": pubs.get(topic, []),
                     "subscribers": subs.get(topic, [])} for topic, kind in master.getTopicTypes()]
        if op == "subscribe":
            return self.subscribe(request)
        if op == "unsubscribe":
            subscription = self.subscriptions.pop(request["topic"], None)
            if subscription is not None:
                subscription.unregister()
            return {}
        if op == "publish":
            return self.publish(request)
        if op == "advertise":
            return self.advertise(request)
        raise ValueError("unknown operation " + op)

    def run(self):
        for function in (self.write_loop, self.watchdog):
            thread = threading.Thread(target=function)
            thread.daemon = True
            thread.start()
        self.reply({"op": "ready", "version": 2, "pid": os.getpid(), "monotonic_ns": monotonic_ns(),
                    "python": sys.version.split()[0], "node": self.rospy.get_name()})
        buffer = b""
        try:
            while not self.stopped.is_set() and not self.rospy.is_shutdown():
                if not select.select([sys.stdin], [], [], .1)[0]:
                    continue
                data = os.read(sys.stdin.fileno(), 65536)
                if not data:
                    break
                buffer += data
                if len(buffer) > MAX_MESSAGE:
                    raise ValueError("incoming message exceeds size limit")
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    request = json.loads(line.decode("utf8"))
                    self.last_input = monotonic()
                    try:
                        result = self.dispatch(request)
                        response = {"op": "reply", "id": request["id"], "result": result}
                    except Exception as error:
                        response = {"op": "reply", "id": request.get("id"), "error": str(error),
                                    "error_type": "command_rejected" if isinstance(error, CommandRejected) else "request_error",
                                    "reason": getattr(error, "reason", "invalid")}
                    self.reply(response)
        finally:
            self.stopped.set()
            try:
                self.stop_expired(all_commands=True)
            finally:
                for lease in self.command_leases.values():
                    lease.close()
                self.command_leases.clear()
                for subscription in self.subscriptions.values():
                    subscription.unregister()
                for _, publisher in self.publishers.values():
                    publisher.unregister()
                self.rospy.signal_shutdown("SSH session closed")


def main():
    wire = os.fdopen(os.dup(1), "wb", 0)
    os.dup2(2, 1)
    initialized = threading.Event()

    def startup_watchdog():
        deadline = monotonic() + 15
        # Before 'ready', the client sends no bytes. EOF/readability means it
        # left (or violated the handshake). rospy.init_node can otherwise wait
        # forever for an unavailable master after the SSH client has gone.
        while not initialized.is_set():
            ready = select.select([sys.stdin], [], [], .1)[0]
            if not initialized.is_set() and (ready or monotonic() > deadline):
                print("ROS1 startup aborted: client closed or master timed out", file=sys.stderr)
                sys.stderr.flush()
                os._exit(1)

    startup = threading.Thread(target=startup_watchdog)
    startup.daemon = True
    startup.start()
    import rospy
    rospy.init_node("rsim_ros1", anonymous=True, disable_signals=True)
    initialized.set()
    agent = Agent(wire, rospy)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: agent.stopped.set())
    agent.run()


if __name__ == "__main__":
    main()
