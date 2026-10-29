"""UVC recipes; capture resources open with the Component lifecycle."""
from dataclasses import dataclass
import json
import math
from typing import ClassVar

from rsim.runtime.host import SharedSensor
from rsim.runtime.sharing import PortSpec, validate_component_ports
from .configuration import fingerprint


@dataclass(frozen=True)
class CameraFactory:
    device_path: str = '/dev/video0'
    width: int = 640
    height: int = 640
    fps: float = 15
    history: int = 8
    FACTORY_VERSION: ClassVar[str] = 'uvc-factory-v1'

    def __post_init__(self):
        if not isinstance(self.device_path, str) or not self.device_path:
            raise ValueError('device must be a nonempty UVC path')
        if any(type(value) is not int or value <= 0 for value in (self.width, self.height, self.history)):
            raise ValueError('camera dimensions and history must be positive integers')
        if type(self.fps) not in (float, int) or not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError('camera fps must be finite and positive')

    @staticmethod
    def port_specs(*, history=8):
        return {'image': PortSpec(clock='host:unix', history_capacity=history)}

    def fingerprint(self):
        return fingerprint(self)

    def build(self):
        from rsim.adapters.uvc import UvcCamera
        specs = self.port_specs(history=self.history)
        source = UvcCamera(self.device_path, width=self.width, height=self.height,
                           fps=self.fps, history=specs['image'].history_capacity)
        validate_component_ports(source, specs)
        return source

    def shared(self, *, transport=None):
        config = dict(device=self.device_path, width=self.width, height=self.height, fps=self.fps)
        source = SharedSensor(self.build, key=f'uvc:{self.device_path}',
                              version=json.dumps(config, sort_keys=True),
                              history=self.history, transport=transport, output_name='image')
        source.image = source.output
        return source

    def device(self, *, transport=None):
        from rsim.devices.camera import Camera as CameraDevice
        return CameraDevice(self.device_path, width=self.width, height=self.height,
                            fps=self.fps, history=self.history, transport=transport)


def Camera(device='/dev/video0', *, width=640, height=640, fps=15, history=8, transport=None):
    """Compatibility provider constructor; use CameraFactory for a local build."""
    return CameraFactory(device, width, height, fps, history).shared(transport=transport)
