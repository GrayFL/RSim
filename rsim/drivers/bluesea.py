"""BlueSea UART lidar recipes using the vendor ROS 2 executable."""
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import ClassVar

from rsim.runtime.host import SharedSensor
from rsim.runtime.sharing import PortSpec, validate_component_ports
from .configuration import (fingerprint, freeze_fields, plain, native_setup_required,
                            snapshot_setup, restore_setup, prepare_log_directory)


@dataclass(frozen=True)
class BlueSeaFactory:
    port: str
    topic: str = '/rsim/chassis/scan_raw'
    history: int = 8
    parameters: object = None
    ros_args: object = None
    log_path: str | None = None
    _prepared: str | None = field(default=None, repr=False)
    FACTORY_VERSION: ClassVar[str] = 'bluesea-factory-v1'

    def __post_init__(self):
        if isinstance(self.ros_args, (str, bytes)):
            raise TypeError('ros_args must be a sequence of argv strings, not a shell command')
        freeze_fields(self)
        self.port_specs(history=self.history)

    @staticmethod
    def port_specs(*, history=8):
        return {'scan': PortSpec(clock='ros:system', history_capacity=history)}

    def _resolve(self):
        from rsim.adapters.ros2.arguments import RosArguments, effective_settings
        resolved = str(Path(self.port).expanduser().resolve())
        if not Path(resolved).exists():
            raise ValueError('BlueSea serial port does not exist: ' + resolved)
        defaults = {
            'type': 'uart', 'port': resolved, 'baud_rate': 500000,
            'scan_topic': self.topic, 'frame_id': 'laser_frame',
            'raw_bytes': 3, 'output_360': True, 'output_scan': True,
            'output_cloud': False, 'output_cloud2': False,
            'with_angle_filter': False, 'max_dist': 50.0, 'inverted': True,
        }
        options = RosArguments({**defaults, **(plain(self.parameters) or {})}, self.ros_args)
        values, topics = effective_settings(options, 'bluesea_node', '/',
                                            lambda value: {'scan': value['scan_topic']})
        if values['type'] != 'uart' or not values['output_scan']:
            raise ValueError('BlueSea requires UART mode and scan output')
        if str(Path(values['port']).expanduser().resolve()) != resolved:
            raise ValueError('BlueSea port override conflicts with source identity')
        return dict(port=resolved, options=options, topics=topics)

    def prepare(self):
        setup = self._resolve()
        prepare_log_directory(self.log_path)
        return replace(self, _prepared=snapshot_setup(setup))

    def _setup(self):
        native_setup_required(self.ros_args, self._prepared)
        return restore_setup(self._prepared) if self._prepared is not None else self._resolve()

    def fingerprint(self):
        native_setup_required(self.ros_args, self._prepared)
        return fingerprint(self)

    def build(self):
        from rsim.adapters.ros2 import Driver, RosSensor
        setup = self._setup()
        specs = self.port_specs(history=self.history)
        spec = specs['scan']
        driver = Driver('bluesea2', 'bluesea2_node', setup['options'],
                        key='bluesea-port:' + setup['port'],
                        log_path=str(Path(self.log_path).expanduser().resolve()) if self.log_path else None)
        source = RosSensor(setup['topics']['scan'], 'scan', clock=spec.clock,
                           driver=driver, hz=100, history=spec.history_capacity)
        validate_component_ports(source, specs)
        return source

    def shared(self, *, transport=None):
        setup = self._setup()
        source = SharedSensor(self.build, key='bluesea:' + setup['port'], version='bluesea-v1',
                              history=self.history, transport=transport,
                              provider_version=setup['options'].signature(), output_name='scan')
        source.scan = source.output
        return source

    def device(self, *, transport=None):
        from rsim.devices.bluesea import BlueSea as BlueSeaDevice
        return BlueSeaDevice(self.port, history=self.history, transport=transport)


def BlueSea(port, *, topic='/rsim/chassis/scan_raw', history=8, transport=None,
            parameters=None, ros_args=None, log_path=None):
    """Compatibility wrapper preserving explicit UART identity validation."""
    return BlueSeaFactory(port, topic, history, parameters, ros_args, log_path).prepare().shared(transport=transport)
