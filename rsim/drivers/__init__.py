"""Provider factories, organized by device; clients use rsim.devices."""
from .realsense import D435
from .seyond import RobinW
from .camera import Camera
from .hipnuc import Hipnuc
from .mapping import Mapper
from .stm32 import STM32
from .chassis import Chassis

async def serve(sensor):
    """Hold a provider lease until cancelled."""
    from .cli import serve as run
    await run(sensor)

__all__ = ['D435', 'RobinW', 'Camera', 'Hipnuc', 'Mapper', 'STM32', 'Chassis', 'serve']

from rsim.runtime.sharing import SharedProvider, serve_shared
__all__ += ["SharedProvider", "serve_shared"]
