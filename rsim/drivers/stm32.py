"""Native ROS2 chassis recipes with explicit resource injection."""
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import ClassVar

from rsim.core.commands import VelocityCommand
from rsim.runtime.sharing import PortSpec, validate_component_ports
from .configuration import (fingerprint, freeze_fields, plain, native_setup_required,
                            snapshot_setup, restore_setup, prepare_log_directory)


@dataclass(frozen=True)
class STM32Factory:
    port: str | None = None
    namespace: str = '/rsim/chassis'
    motion_enabled: bool = False
    history: int = 128
    parameters: object = None
    ros_args: object = None
    log_path: str | None = None
    start_driver: bool = True
    remote_clock: bool = False
    _prepared: str | None = field(default=None, repr=False)
    FACTORY_VERSION: ClassVar[str] = 'stm32-factory-v1'

    def __post_init__(self):
        if isinstance(self.ros_args, (str, bytes)):
            raise TypeError('ros_args must be a sequence of argv strings, not a shell command')
        freeze_fields(self)
        self.port_specs(history=self.history)

    @staticmethod
    def port_specs(*, history=128, max_ttl=.5):
        return {
            'odom': PortSpec(clock='ros:system', history_capacity=history),
            'state': PortSpec(clock='ros:system', history_capacity=history),
            'velocity_feedback': PortSpec(clock='host:monotonic', history_capacity=history),
            'velocity': PortSpec(direction='sink', clock='host:monotonic',
                                 max_ttl=max_ttl, fallback=VelocityCommand()),
        }

    def _resolve(self):
        from rsim.adapters.ros2.arguments import RosArguments, effective_settings
        defaults = dict(port=str(self.port or ''), motion_enabled=self.motion_enabled)
        options = RosArguments({**defaults, **(plain(self.parameters) or {})}, self.ros_args,
                               {'__ns': self.namespace, '__node': 'stm32_driver'})
        values, topics = effective_settings(options, 'stm32_driver', '/', lambda _: {
            name: name for name in ('odom', 'diagnostics', 'set_velocity', 'stop', 'clock')})
        if self.start_driver and not values['port']:
            raise ValueError('STM32 requires an explicit serial port')
        return dict(values=values, topics=topics, options=options,
                    port=str(Path(values['port']).expanduser().resolve()))

    def prepare(self):
        """Resolve native arguments and prepare the optional log directory."""
        setup = self._resolve()
        prepare_log_directory(self.log_path)
        return replace(self, _prepared=snapshot_setup(setup))

    def _setup(self):
        native_setup_required(self.ros_args, self._prepared)
        return restore_setup(self._prepared) if self._prepared is not None else self._resolve()

    def fingerprint(self):
        native_setup_required(self.ros_args, self._prepared)
        return fingerprint(self)

    def build(self, *, ros=None, feedback_sources=None):
        from rsim.adapters.ros2 import Driver
        from rsim.adapters.ros2.stm32 import Stm32Chassis
        setup = self._setup()
        specs = self.port_specs(history=self.history, max_ttl=setup['values'].get('max_command_ttl', .5))
        driver = Driver('rsim_stm32', 'stm32_node', setup['options'],
                        key='stm32-port:' + setup['port'],
                        log_path=str(Path(self.log_path).expanduser().resolve()) if self.log_path else None
                        ) if self.start_driver else None
        source = Stm32Chassis(topics=setup['topics'], driver=driver,
                              history=specs['odom'].history_capacity,
                              max_ttl=specs['velocity'].max_ttl, ros=ros,
                              remote_clock=self.remote_clock, feedback_sources=feedback_sources)
        validate_component_ports(source, specs)
        return source


def STM32(port=None, *, namespace='/rsim/chassis', motion_enabled=False,
          history=128, parameters=None, ros_args=None, log_path=None, start_driver=True, ros=None,
          remote_clock=False, feedback_sources=None):
    """Compatibility wrapper preserving native settings and resource injection."""
    return STM32Factory(port, namespace, motion_enabled, history, parameters, ros_args,
                        log_path, start_driver, remote_clock).prepare().build(
                            ros=ros, feedback_sources=feedback_sources)
