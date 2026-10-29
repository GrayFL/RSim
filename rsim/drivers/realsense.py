from pathlib import Path
from dataclasses import dataclass, field, replace
from typing import ClassVar
from rsim.runtime.sharing import PortSpec, validate_component_ports
from .configuration import fingerprint, freeze_fields, plain, native_setup_required, snapshot_setup, restore_setup
from rsim.adapters.ros2 import Driver, RosSensor
from rsim.adapters.ros2.arguments import RosArguments, effective_settings

from rsim.devices.realsense import d435_config, d435_view
from rsim.core.compose import Bundle

def d435_setup(serial, depth_profile, color_profile, parameters=None, ros_args=None):
    config = d435_config(serial, depth_profile, color_profile)
    node_name = "d435" + ("_" + serial if serial else "")
    params = {
        "camera_name": node_name, "device_type": "d435(?!i)",
        "enable_depth": True, "enable_color": True,
        "enable_infra1": False, "enable_infra2": False,
        "depth_module.depth_profile": config["depth_profile"],
        "rgb_camera.color_profile": config["color_profile"],
        "wait_for_device_timeout": 15.0,
    }
    if serial:
        params["serial_no"] = "_" + serial
    params.update(parameters or {})
    options = RosArguments(params, ros_args, {"__ns": "/rsim", "__node": node_name})
    values, topics = effective_settings(options, "camera", "/camera", lambda _: {
        "depth": "~/depth/image_rect_raw", "color": "~/color/image_raw"})
    serial = values.get("serial_no", "")
    if not isinstance(serial, str):
        raise ValueError("serial_no must be a ROS string (quote numeric serials)")
    config = d435_config(serial.removeprefix("_"), values["depth_module.depth_profile"],
                         values["rgb_camera.color_profile"])
    enabled = []
    for stream in ("color", "depth"):
        value = values["enable_" + stream]
        if type(value) is not bool:
            raise ValueError(f"enable_{stream} must be a ROS boolean")
        if value:
            enabled.append(stream)
    if not enabled:
        raise ValueError("D435 requires at least one enabled color/depth stream")
    return {"config": config, "options": options, "topics": topics, "enabled": enabled}


class D435Source(RosSensor):

    def __init__(
            self,
            *,
            stream="depth",
            serial="",
            start_driver=True,
            history=16,
            log_path=None,
            depth_profile="640x480x15",
            color_profile="640x480x15",
            parameters=None,
            ros_args=None,
            _setup=None
        ):
        if stream not in ("depth", "color"):
            raise ValueError("stream must be depth or color")
        setup = _setup or d435_setup(serial, depth_profile, color_profile, parameters, ros_args)
        serial = setup["config"]["serial"]
        if stream not in setup["enabled"]:
            raise ValueError(f"D435 {stream} stream is disabled by ROS parameters")
        driver = Driver(
            "realsense2_camera",
            "realsense2_camera_node",
            setup["options"],
            key=f"driver:d435:{serial}",
            log_path=log_path
            ) if start_driver else None
        topic = setup["topics"][stream]
        super().__init__(
            topic,
            "image",
            clock=f"device:d435:{serial}",
            driver=driver,
            history=history
            )


@dataclass(frozen=True)
class D435Factory:
    serial: str = ''
    stream: str = 'depth'
    history: int = 8
    depth_profile: str = '640x480x15'
    color_profile: str = '640x480x15'
    log_path: str | None = None
    parameters: object = None
    ros_args: object = None
    _prepared: str | None = field(default=None, repr=False)
    FACTORY_VERSION: ClassVar[str] = 'd435-factory-v1'

    def __post_init__(self):
        if self.stream not in ('color', 'depth'):
            raise ValueError('stream must be color or depth')
        config = d435_config(self.serial, self.depth_profile, self.color_profile)
        for name, value in config.items():
            object.__setattr__(self, name, value)
        if isinstance(self.ros_args, (str, bytes)):
            raise TypeError('ros_args must be a sequence of argv strings, not a shell command')
        freeze_fields(self)
        RosArguments(plain(self.parameters))
        self.port_specs(history=self.history)

    @staticmethod
    def port_specs(*, history=8):
        # The legacy shared protocol exports one Bundle. Its entries retain
        # their individual image sample clocks; it is not a multi-port source.
        return {'output': PortSpec(history_capacity=history)}

    def prepare(self):
        """Explicitly resolve native ROS args into a serializable data snapshot."""
        setup = d435_setup(self.serial, self.depth_profile, self.color_profile,
                           plain(self.parameters), self.ros_args)
        if self.stream not in setup['enabled']:
            raise ValueError(f'D435 {self.stream} stream is disabled by ROS parameters')
        return replace(self, _prepared=snapshot_setup(setup))

    def _setup(self):
        native_setup_required(self.ros_args, self._prepared)
        setup = (restore_setup(self._prepared) if self._prepared is not None else
                 d435_setup(self.serial, self.depth_profile, self.color_profile,
                            plain(self.parameters)))
        if self.stream not in setup['enabled']:
            raise ValueError(f'D435 {self.stream} stream is disabled by ROS parameters')
        return setup

    def fingerprint(self):
        native_setup_required(self.ros_args, self._prepared)
        return fingerprint(self)

    def build(self):
        setup = self._setup()
        setup['options'].arguments()
        config = setup['config']
        specs = self.port_specs(history=self.history)
        history = specs['output'].history_capacity
        source = Bundle(**{
            name: D435Source(stream=name, history=history,
                             log_path=str(Path(self.log_path).resolve()) if self.log_path else None,
                             _setup=setup)
            for name in setup['enabled']},
            hz=2 * max(int(config[key].split('x')[-1]) for key in ('depth_profile', 'color_profile')),
            history=history)
        validate_component_ports(source, specs)
        return source

    def shared(self, *, transport=None):
        setup = self._setup()
        return d435_view(setup['config'], self.stream, self.history,
                         factory=self.build, transport=transport,
                         provider_version=setup['options'].signature())

    def device(self, *, transport=None):
        return d435_view(self._setup()['config'], self.stream, self.history, transport=transport)


def D435(*, serial='', stream='depth', history=8, depth_profile='640x480x15',
         color_profile='640x480x15', log_path=None, transport=None,
         parameters=None, ros_args=None):
    """Compatibility constructor retaining explicit native-argument preparation."""
    return D435Factory(serial, stream, history, depth_profile, color_profile,
                        log_path, parameters, ros_args).prepare().shared(transport=transport)
