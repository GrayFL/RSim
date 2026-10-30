"""Provider factories, organized by device; clients use rsim.devices."""
from .realsense import D435, D435Factory
from .seyond import RobinW, RobinWFactory
from .camera import Camera, CameraFactory
from .hipnuc import Hipnuc, HipnucFactory
from .mapping import Mapper
from .stm32 import STM32, STM32Factory
from .chassis import Chassis, ChassisFactory
from .ros_topics import ROS2Topic, ROS2TopicFactory
from .bluesea import BlueSea, BlueSeaFactory
from .odometry import Odometry, OdometryFactory
from .scan_odometry import ScanOdometryFactory
from .native_chassis import NativeChassis, TopicChassis, NativeChassisFactory, TopicChassisFactory
from .mujoco import MuJoCo

async def serve(sensor):
    """Hold a provider lease until cancelled."""
    from .cli import serve as run
    await run(sensor)

__all__ = ['D435', 'RobinW', 'Camera', 'Hipnuc', 'Mapper', 'STM32', 'Chassis', 'ROS2Topic', 'BlueSea', 'serve']

from rsim.runtime.sharing import SharedProvider, serve_shared
__all__ += ["SharedProvider", "serve_shared"]
__all__ += ['Odometry', 'NativeChassis', 'TopicChassis']
__all__ += ['MuJoCo']

__all__ += [
    "CameraFactory", "ROS2TopicFactory", "D435Factory", "RobinWFactory",
    "HipnucFactory", "STM32Factory", "BlueSeaFactory", "ChassisFactory",
    "OdometryFactory", "ScanOdometryFactory",
    "NativeChassisFactory", "TopicChassisFactory",
]
