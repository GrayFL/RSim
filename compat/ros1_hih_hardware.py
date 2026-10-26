"""Explicit ROBOT-HIH hardware integration; Python 2/3 and stdlib only."""
from __future__ import print_function
import binascii
import ctypes
import socket
import struct
import threading
from ros1_agent import monotonic


class PedalState(object):
    """Fail-closed recognition of a calibrated fully retracted CAN state.

    Every other state uses the extended body. An extension command invalidates
    old retracted feedback until departure and a later return are observed.
    """
    def __init__(self, config):
        self.config = config
        self.last = None
        self.raw = None
        self.consecutive = 0
        self.wait_departure = False
        self.held_retracted = False
        self.command = None

    def receive(self, identifier, data, now):
        raw = binascii.hexlify(data).decode('ascii')
        if identifier == self.config.get('command_id', 0x604):
            if len(data) == 8 and raw.startswith('2b014000') and raw[10:] == '000000':
                action = int(raw[8:10], 16)
                if action in (0, 1, 2):
                    # OEM stop clears its completion code to 00 without moving
                    # the pedal. Preserve only an already confirmed endpoint.
                    self.held_retracted = action == 0 and self.snapshot(now)['retracted']
                    self.command = {0: 'stop', 1: 'extend', 2: 'retract'}[action]
                    if not self.held_retracted: self.consecutive = 0
                    if action == 1: self.wait_departure = True
            return
        if identifier != self.config.get('status_id', 0x184): return
        if self.last is None or now - self.last > self.config.get('timeout', 1.5):
            self.consecutive = 0
            self.held_retracted = False
        self.last, self.raw = now, raw
        if raw not in (self.config['retracted_payload'], '0000000000000000'):
            self.held_retracted = False
        retracted = raw == self.config['retracted_payload'] or (self.held_retracted and raw == '0000000000000000')
        if not retracted: self.wait_departure = False
        self.consecutive = self.consecutive + 1 if retracted and not self.wait_departure else 0

    def snapshot(self, now):
        age = None if self.last is None else now - self.last
        fresh = age is not None and 0 <= age <= self.config.get('timeout', 1.5)
        confirmed = fresh and self.consecutive >= 2 and not self.wait_departure
        position = {'0100000000000000': 'extending', '0200000000000000': 'extended',
                    '0300000000000000': 'retracting', '0400000000000000': 'confirming_retracted',
                    '0000000000000000': 'stopped'}.get(self.raw, 'unknown')
        return dict(automatic=True, position='retracted' if confirmed else position if fresh else 'unknown',
                    retracted=bool(confirmed), feedback=bool(fresh), command=self.command,
                    retained_endpoint=bool(confirmed and self.held_retracted),
                    raw=self.raw, age_sec=age, feedback_ns=int(self.last * 1e9) if self.last is not None else 0,
                    source='can:%s/%03X' % (self.config['interface'], self.config.get('status_id', 0x184)))


class PedalMonitor(object):
    """A receive-only SocketCAN listener; never sends or configures the bus."""
    def __init__(self, config):
        self.state = PedalState(config)
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

    def run(self):
        bus = None
        try:
            # This robot's Python 2 was built without the AF_CAN names/address
            # parser. Numeric Linux constants plus libc bind work with both
            # interpreters; recv/setsockopt still use the stdlib socket object.
            bus = socket.socket(29, socket.SOCK_RAW, 1)
            ids = [self.state.config.get('status_id', 0x184), self.state.config.get('command_id', 0x604)]
            # Match standard data frames, excluding RTR/error/extended frames.
            bus.setsockopt(101, 1,
                           b''.join(struct.pack('=II', i, 0xC00007FF) for i in ids))
            libc = ctypes.CDLL(None, use_errno=True)
            libc.if_nametoindex.argtypes = [ctypes.c_char_p]
            libc.if_nametoindex.restype = ctypes.c_uint
            index = libc.if_nametoindex(self.state.config['interface'].encode('ascii'))
            if not index: raise OSError(ctypes.get_errno(), 'CAN interface unavailable')
            address = ctypes.create_string_buffer(struct.pack('=H2xi8x', 29, index))
            libc.bind.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_uint]
            if libc.bind(bus.fileno(), address, 16) != 0:
                raise OSError(ctypes.get_errno(), 'CAN bind failed')
            bus.settimeout(.2)
            while not self.stopped.is_set():
                try: packet = bus.recv(16)
                except socket.timeout: continue
                identifier, length, data = struct.unpack('=IB3x8s', packet)
                with self.lock: self.state.receive(identifier, data[:length], monotonic())
        except Exception as error:
            self.error = str(error)[:180]
        finally:
            if bus is not None: bus.close()

    def snapshot(self):
        # Read the clock after taking the same lock used by CAN reception.
        # A loop timestamp captured earlier can precede a newly received frame
        # and falsely mark fresh feedback as coming from the future.
        with self.lock: result = self.state.snapshot(monotonic())
        if self.error:
            result.update(error=self.error, feedback=False, retracted=False, position='unknown')
        return result

    def close(self):
        self.stopped.set()
        self.thread.join(.5)
