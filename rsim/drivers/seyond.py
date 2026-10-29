from pathlib import Path
from dataclasses import dataclass, field, replace
from typing import ClassVar
from rsim.runtime.sharing import PortSpec, validate_component_ports
from .configuration import fingerprint, freeze_fields, plain, native_setup_required, snapshot_setup, restore_setup
from rsim.runtime.host import SharedSensor
from rsim.adapters.ros2 import Driver, RosSensor
from rsim.adapters.ros2.arguments import RosArguments, effective_settings

def robin_setup(ip, topic="/iv_points", parameters=None, ros_args=None):
    options = RosArguments({"lidar_ip": ip, "frame_topic": topic, **(parameters or {})}, ros_args)
    values, topics = effective_settings(options, "seyond", "/", lambda p: {"points": p["frame_topic"]})
    if not isinstance(values["lidar_ip"], str) or not values["lidar_ip"]:
        raise ValueError("lidar_ip must be a nonempty ROS string")
    return {"ip": values["lidar_ip"], "options": options, "topic": topics["points"]}


class RobinWSource(RosSensor):

    def __init__(
            self,
            ip="192.168.199.97",
            *,
            start_driver=True,
            topic="/iv_points",
            history=16,
            log_path=None,
            parameters=None,
            ros_args=None,
            _setup=None
        ):
        setup = _setup or robin_setup(ip, topic, parameters, ros_args)
        ip, topic = setup["ip"], setup["topic"]
        driver = Driver(
            "seyond",
            "seyond_node", setup["options"],
            key=f"driver:robin:{ip}",
            log_path=log_path
            ) if start_driver else None
        super().__init__(
            topic,
            "points",
            clock=f"device:robin:{ip}",
            driver=driver,
            history=history
            )


@dataclass(frozen=True)
class RobinWFactory:
    ip: str = '192.168.199.97'
    history: int = 8
    parameters: object = None
    ros_args: object = None
    log_path: str | None = None
    _prepared: str | None = field(default=None, repr=False)
    FACTORY_VERSION: ClassVar[str] = 'robin-factory-v1'

    def __post_init__(self):
        if isinstance(self.ros_args, (str, bytes)):
            raise TypeError('ros_args must be a sequence of argv strings, not a shell command')
        freeze_fields(self)
        RosArguments(plain(self.parameters))
        self.port_specs(ip=self.ip, history=self.history)

    @staticmethod
    def port_specs(*, ip='192.168.199.97', history=8):
        return {'points': PortSpec(clock=f'device:robin:{ip}', history_capacity=history)}

    def prepare(self):
        setup = robin_setup(self.ip, parameters=plain(self.parameters), ros_args=self.ros_args)
        return replace(self, _prepared=snapshot_setup(setup))

    def _setup(self):
        native_setup_required(self.ros_args, self._prepared)
        return (restore_setup(self._prepared) if self._prepared is not None else
                robin_setup(self.ip, parameters=plain(self.parameters)))

    def fingerprint(self):
        native_setup_required(self.ros_args, self._prepared)
        return fingerprint(self)

    def build(self):
        setup = self._setup()
        setup['options'].arguments()
        specs = self.port_specs(ip=setup['ip'], history=self.history)
        source = RobinWSource(history=specs['points'].history_capacity,
                              log_path=str(Path(self.log_path).resolve()) if self.log_path else None,
                              _setup=setup)
        validate_component_ports(source, specs)
        return source

    def shared(self, *, transport=None):
        setup = self._setup()
        source = SharedSensor(self.build, key=f"robin:{setup['ip']}", version='robin-v1',
                              history=self.history, transport=transport,
                              provider_version=setup['options'].signature(), output_name='points')
        source.points = source.output
        return source

    def device(self, *, transport=None):
        from rsim.devices.seyond import RobinW as RobinDevice
        return RobinDevice(self._setup()['ip'], history=self.history, transport=transport)


def RobinW(ip='192.168.199.97', *, history=8, transport=None, parameters=None,
           ros_args=None, log_path=None):
    """Compatibility wrapper; all provider construction belongs to the recipe."""
    return RobinWFactory(ip, history, parameters, ros_args, log_path).prepare().shared(transport=transport)
