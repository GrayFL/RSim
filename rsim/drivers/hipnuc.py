"""HiPNUC serial/ROS2 recipes; discovery and ROS parsing are explicit preparation."""
from dataclasses import dataclass, field, replace
import json
from pathlib import Path
from typing import ClassVar

from rsim.runtime.host import SharedSensor
from rsim.runtime.sharing import PortSpec, validate_component_ports
from .configuration import fingerprint, freeze_fields, plain, snapshot_setup, restore_setup


@dataclass(frozen=True)
class HipnucFactory:
    port: str | None = None
    mode: str = 'serial'
    baudrate: int = 460800
    frame_id: str = 'hipnuc_imu'
    navigation_frame: str = 'device_navigation'
    history: int = 128
    parameters: object = None
    ros_args: object = None
    log_path: str | None = None
    _prepared: str | None = field(default=None, repr=False)
    FACTORY_VERSION: ClassVar[str] = 'hipnuc-factory-v1'

    def __post_init__(self):
        if self.mode not in ('serial', 'ros2'):
            raise ValueError('IMU mode must be serial or ros2')
        if self.mode == 'serial' and self.ros_args:
            raise ValueError("ros_args requires mode='ros2'")
        if isinstance(self.ros_args, (str, bytes)):
            raise TypeError('ros_args must be a sequence of argv strings, not a shell command')
        freeze_fields(self)
        self.port_specs(history=self.history)

    @staticmethod
    def port_specs(*, clock='host:monotonic', history=128):
        return {'imu': PortSpec(clock=clock, history_capacity=history)}

    def _resolve(self, *, prepare=False):
        from rsim.adapters.hipnuc import serial_port
        from rsim.adapters.ros2.arguments import RosArguments
        values = dict(port=self.port or '', baudrate=self.baudrate, frame_id=self.frame_id,
                      navigation_frame=self.navigation_frame, history=self.history)
        values.update(plain(self.parameters) or {})
        options, topic = None, None
        if self.mode == 'ros2':
            if not prepare:
                raise ValueError('ROS2 IMU configuration requires explicit factory.prepare()')
            options = RosArguments(values, self.ros_args)
            with options.resolve('hipnuc_imu') as node:
                values = {name: node.get_parameter(name).value
                          for name in node.list_parameters([], depth=0).names}
                topic = node.resolve_topic_name('imu/data')
            port = serial_port(values['port'])
            options.parameters['port'] = port
        else:
            if not values['port'] and not prepare:
                raise ValueError('serial discovery requires explicit factory.prepare() or a port')
            port = serial_port(values['port'])
            values['port'] = port
        return dict(port=port, values=values, options=options, topic=topic)

    def prepare(self):
        return replace(self, _prepared=snapshot_setup(self._resolve(prepare=True)))

    def _setup(self):
        return restore_setup(self._prepared) if self._prepared is not None else self._resolve()

    def fingerprint(self):
        if self._prepared is None and (self.mode == 'ros2' or
                not (self.port or (self.parameters or {}).get('port'))):
            raise ValueError('resolve ROS2 configuration/serial discovery with factory.prepare() first')
        return fingerprint(self)

    def build(self):
        setup = self._setup()
        values = setup['values']
        if self.mode == 'serial':
            from rsim.adapters.hipnuc import SerialIMU
            specs = self.port_specs(history=values['history'])
            source = SerialIMU(**{**values, 'history': specs['imu'].history_capacity})
        else:
            from rsim.adapters.ros2 import Driver, RosSensor
            specs = self.port_specs(clock='ros:sim' if values.get('use_sim_time') else 'ros:system',
                                    history=self.history)
            spec = specs['imu']
            driver = Driver('rsim_hipnuc', 'serial_node', setup['options'],
                            key='hipnuc-node:' + setup['port'],
                            log_path=str(Path(self.log_path).resolve()) if self.log_path else None)
            source = RosSensor(setup['topic'], 'imu', clock=spec.clock,
                               driver=driver, history=spec.history_capacity, hz=500)
        validate_component_ports(source, specs)
        return source

    def shared(self, *, transport=None):
        setup = self._setup()
        signature = (setup['options'].signature() if setup['options'] is not None else
                     json.dumps(setup['values'], sort_keys=True))
        source = SharedSensor(self.build, key='hipnuc:' + setup['port'], version='hipnuc-v1',
                              history=self.history, hz=500, transport=transport,
                              provider_version=self.mode + ':' + signature, output_name='imu')
        source.imu = source.output
        return source

    def device(self, *, transport=None):
        from rsim.devices.hipnuc import Hipnuc as HipnucDevice
        return HipnucDevice(self._setup()['port'], history=self.history, transport=transport)


def Hipnuc(port=None, *, mode='serial', baudrate=460800, frame_id='hipnuc_imu',
           navigation_frame='device_navigation', history=128, parameters=None,
           ros_args=None, transport=None, log_path=None):
    """Compatibility wrapper; native discovery is delegated to explicit prepare."""
    return HipnucFactory(port, mode, baudrate, frame_id, navigation_frame, history,
                         parameters, ros_args, log_path).prepare().shared(transport=transport)
