"""HiPNUC serial IMU, independent of ROS; legacy tags and IMUSOL (0x91).

Wire layout and units follow the HiPNUC HI229 manual supplied with the device.
Only checksum-valid, complete packets produce samples; fields are never carried
over from a previous packet. Reading does not reconfigure device firmware.
"""
from __future__ import annotations

from binascii import crc_hqx
import math
from pathlib import Path
import struct
import time

from rsim.core.component import PrimaryComponent, ComponentError
from rsim.runtime.locks import acquire_device


def serial_port(port=None):
    """Resolve aliases before claiming a port; auto only when unambiguous."""
    if port:
        return str(Path(port).expanduser().resolve())
    from serial.tools import list_ports
    matches = [item.device for item in list_ports.comports()
               if item.vid == 0x10C4 and item.pid == 0xEA60]
    if len(matches) != 1:
        raise ValueError("specify port explicitly: expected exactly one CP210x device")
    return str(Path(matches[0]).resolve())


class HipnucDecoder:
    """Bounded incremental CRC-16/XMODEM decoder; values use SI units.

    Legacy 0xD0 is pitch/roll/yaw with unequal scales. IMUSOL Euler angles are
    roll/pitch/yaw; quaternions are WXYZ on the wire and XYZW at this boundary.
    Unknown packet layouts are rejected rather than guessed.
    """
    sizes = {0x90: 1, 0xA0: 6, 0xB0: 6, 0xC0: 6, 0xD0: 6,
             0xD1: 16, 0xD9: 12, 0xF0: 4, 0x91: 75}

    def __init__(self, *, gravity=9.8, max_payload=1024):
        if not math.isfinite(gravity) or gravity <= 0 or not 76 <= max_payload <= 65535:
            raise ValueError("invalid gravity or packet length limit")
        self.gravity, self.max_payload = gravity, max_payload
        self.buffer = bytearray()
        self.frames = self.crc_errors = self.invalid_payloads = self.discarded_bytes = 0

    def feed(self, data):
        result = []
        # Bound retained memory even when a caller provides a very large read.
        for start in range(0, len(data), self.max_payload):
            self.buffer.extend(data[start:start + self.max_payload])
            while len(self.buffer) >= 2:
                sync = self.buffer.find(b"\x5a\xa5")
                if sync < 0:
                    count = len(self.buffer) - (self.buffer[-1] == 0x5A)
                    self.discarded_bytes += count
                    del self.buffer[:count]
                    break
                self.discarded_bytes += sync
                del self.buffer[:sync]
                if len(self.buffer) < 6:
                    break
                size, checksum = struct.unpack_from("<HH", self.buffer, 2)
                if not 1 <= size <= self.max_payload:
                    self.discarded_bytes += 1
                    del self.buffer[0]
                    continue
                if len(self.buffer) < size + 6:
                    break
                raw = bytes(self.buffer[:size + 6])
                if crc_hqx(raw[6:], crc_hqx(raw[:4], 0)) != checksum:
                    self.crc_errors += 1
                    del self.buffer[0]
                    continue
                del self.buffer[:size + 6]
                try:
                    sample = self._decode(raw[6:])
                except (ValueError, struct.error):
                    self.invalid_payloads += 1
                    continue
                self.frames += 1
                result.append(sample)
        return result

    def _decode(self, payload):
        values, tags, cursor = {}, [], 0
        while cursor < len(payload):
            tag = payload[cursor]
            size = self.sizes.get(tag)
            if size is None or cursor + 1 + size > len(payload) or tag in tags:
                raise ValueError("unknown, repeated or truncated data item")
            tags.append(tag)
            part = payload[cursor + 1:cursor + 1 + size]
            cursor += 1 + size
            if tag == 0x90:
                values["device_id"] = part[0]
            elif tag in (0xA0, 0xB0, 0xC0):
                field, scale = {0xA0: ("acceleration", self.gravity / 1000),
                                0xB0: ("angular_velocity", math.pi / 1800),
                                0xC0: ("magnetic_field", 1e-7)}[tag]
                values[field] = [number * scale for number in struct.unpack("<3h", part)]
            elif tag in (0xD0, 0xD9):
                pitch, roll, yaw = struct.unpack("<3h" if tag == 0xD0 else "<3f", part)
                values["euler_deg"] = ([roll / 100, pitch / 100, yaw / 10] if tag == 0xD0
                                       else [roll, pitch, yaw])
            elif tag == 0xD1:
                w, x, y, z = struct.unpack("<4f", part)
                values["orientation"] = [x, y, z, w]
            elif tag == 0xF0:
                values["pressure_pa"] = struct.unpack("<f", part)[0]
            elif tag == 0x91:
                values["device_id"] = part[0]
                values["device_timestamp_ms"] = struct.unpack_from("<I", part, 7)[0]
                floats = struct.unpack_from("<16f", part, 11)
                values["acceleration"] = [v * self.gravity for v in floats[:3]]
                values["angular_velocity"] = [math.radians(v) for v in floats[3:6]]
                values["magnetic_field"] = [v * 1e-6 for v in floats[6:9]]
                values["euler_deg"] = list(floats[9:12])
                w, x, y, z = floats[12:]
                values["orientation"] = [x, y, z, w]
        for value in values.values():
            if not all(math.isfinite(v) for v in (value if isinstance(value, list) else [value])):
                raise ValueError("nonfinite measurement")
        if "orientation" not in values and "euler_deg" in values:
            from graphmap.pose import Pose
            values["orientation"] = Pose(rotation=values["euler_deg"], degrees=True).quat.tolist()
            values["orientation_source"] = "euler"
        elif "orientation" in values:
            norm = math.sqrt(sum(v * v for v in values["orientation"]))
            if not .5 < norm < 1.5:
                raise ValueError("invalid quaternion")
            values["orientation"] = [v / norm for v in values["orientation"]]
            values["orientation_source"] = "quaternion"
        if not any(k in values for k in ("acceleration", "angular_velocity", "orientation")):
            raise ValueError("packet contains no IMU measurements")
        values["tags"] = tags
        return values


def imu_message(sample, frame_id, *, navigation_frame="device_navigation"):
    """ROS-free Imu-shaped dictionary; absent measurements use covariance -1."""
    result = {"header": {"frame_id": frame_id}, "navigation_frame": navigation_frame,
              "metadata": {k: v for k, v in sample.items()
                           if k not in ("orientation", "acceleration", "angular_velocity")}}
    for source, target, axes in (("orientation", "orientation", "xyzw"),
                                 ("angular_velocity", "angular_velocity", "xyz"),
                                 ("acceleration", "linear_acceleration", "xyz")):
        values = sample.get(source, [0.] * len(axes))
        result[target] = dict(zip(axes, values))
        covariance = [0.] * 9
        if source not in sample:
            covariance[0] = -1.
        result[target + "_covariance"] = covariance
    return result


class SerialIMU(PrimaryComponent):
    """Nonblocking pyserial Component; get() and .imu carry the same Signal.

    Frame time is host monotonic *reception*, not device sample time. Optional
    IMUSOL boot milliseconds remain metadata, without inventing clock alignment.
    """
    def __init__(self, port=None, *, baudrate=460800, frame_id="hipnuc_imu",
                 navigation_frame="device_navigation", gravity=9.8, hz=500,
                 timeout=3., history=128):
        self.port = serial_port(port)
        if (type(baudrate) is not int or baudrate <= 0 or not frame_id or
                not navigation_frame or not math.isfinite(hz) or hz <= 0 or
                not math.isfinite(timeout) or timeout <= 0):
            raise ValueError("invalid serial IMU settings")
        super().__init__(key="serial:" + self.port, output_name="imu", history=history,
                         clock="host:monotonic")
        self.imu = self.output
        self.baudrate, self.frame_id, self.navigation_frame = baudrate, frame_id, navigation_frame
        self.gravity, self.hz, self.timeout = gravity, hz, timeout
        self.serial = self._lease = None
        self.decoder = HipnucDecoder(gravity=gravity)

    def configuration(self):
        return (super().configuration(), self.port, self.baudrate, self.frame_id,
                self.navigation_frame, self.gravity, self.hz, self.timeout)

    def __getstate__(self):
        state = super().__getstate__()
        state.update(serial=None, _lease=None)
        return state

    async def open(self):
        import serial
        self.decoder = HipnucDecoder(gravity=self.gravity)
        self._lease = acquire_device("serial:" + self.port)
        self.serial = serial.Serial(self.port, self.baudrate, timeout=0, exclusive=True)
        self.serial.reset_input_buffer()
        self._last_valid = time.monotonic()
        self.task("serial-read", self._read, hz=self.hz)

    async def _read(self):
        data = self.serial.read(min(self.serial.in_waiting, 16384))
        received = time.time_ns()
        for sample in self.decoder.feed(data):
            self._last_valid = time.monotonic()
            await self.imu.publish(imu_message(sample, self.frame_id, navigation_frame=self.navigation_frame),
                                   stamp_ns=time.monotonic_ns(), clock="host:monotonic", received_ns=received)
        if time.monotonic() - self._last_valid > self.timeout:
            raise ComponentError("no valid HiPNUC packet: check port, baudrate and output protocol")

    async def close(self):
        try:
            if self.serial is not None:
                self.serial.close()
                self.serial = None
        finally:
            if self._lease is not None:
                self._lease.close()
                self._lease = None
