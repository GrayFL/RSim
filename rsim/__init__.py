from .core import Component, ComponentError, PrimaryComponent, HistoryMiss, Metronome, Reference, Runtime, Sensor, SensorError
from .signal import Signal
from .clocks import ClockDomain, ClockTransform
from .sync import Synchronizer, TimeJoin
from .commands import (CommandSink, CommandEnvelope, VelocityCommand, VelocityCommandEnvelope,
                       CommandGuard, CommandRejected, CommandInput, CommandMux, Arbiter, Connect)
from .model import Frame, Image, PointCloud
from .compose import Bundle, Map
from .devices import Camera, D435, RobinW, Hipnuc
from .host import SharedSensor
from .process import ProcessSensor
from .shared import allocate
from .transport import TransportConfig
from .remote import Chassis, Ros1Bridge, SSHConfig
from .deployment import LocalPlacement, ProcessPlacement, LocalReference, SharedMemoryChannel, DDSChannel

__all__ = ["Component", "ComponentError", "PrimaryComponent", "Signal", "Frame", "HistoryMiss", "Metronome", "Runtime", "Sensor", "SensorError",
           "Bundle", "Map", "Camera", "D435", "RobinW", "SharedSensor", "ProcessSensor", "allocate",
           "Reference", "Image", "PointCloud", "TransportConfig", "Chassis", "Ros1Bridge", "SSHConfig",
           "ClockDomain", "ClockTransform", "Synchronizer", "TimeJoin", "CommandSink", "CommandEnvelope",
           "VelocityCommand", "VelocityCommandEnvelope", "CommandGuard", "CommandRejected", "CommandInput",
           "CommandMux", "Arbiter", "Connect", "LocalPlacement", "ProcessPlacement", "LocalReference",
           "SharedMemoryChannel", "DDSChannel", "Hipnuc"]


def __getattr__(name):
    # Geometry remains optional for applications using only sensor adapters.
    if name in {"load_rig", "SensorRig", "Mount", "UncalibratedMount"}:
        from . import config
        return getattr(config, name)
    if name == "SerialIMU":
        from .imu import SerialIMU
        return SerialIMU
    if name == "PlanarOdometry":
        from .odometry import PlanarOdometry
        return PlanarOdometry
    if name in {"ChassisController", "MotionError"}:
        from .motion import ChassisController, MotionError
        return {"ChassisController": ChassisController, "MotionError": MotionError}[name]
    raise AttributeError(name)
