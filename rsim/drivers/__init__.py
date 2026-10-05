"""Provider factories, organized by device; clients use rsim.devices."""
from .realsense import D435
from .seyond import RobinW
from .camera import Camera
from .hipnuc import Hipnuc

async def serve(sensor):
    """Hold a provider lease until cancelled."""
    from .cli import serve as run
    await run(sensor)

__all__ = ['D435', 'RobinW', 'Camera', 'Hipnuc', 'serve']
