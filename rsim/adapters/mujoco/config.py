from dataclasses import dataclass

@dataclass(frozen=True)
class RobotConfig:
    # Overall XY footprint, including wheels, is 0.6 x 0.6 m.
    side: float = 0.6
    wheel_radius: float = 0.10
    wheel_width: float = 0.07
    wheel_track: float = 0.53
    max_wheel_speed: float = 12.0
    timestep: float = 0.002


@dataclass(frozen=True)
class SensorConfig:
    lidar_height: float = 0.4
    lidar_beams: int = 360
    lidar_min: float = 0.05
    lidar_max: float = 12.0
    camera_height: float = 1.0
    width: int = 640
    height: int = 480
    vertical_fov: float = 60.0
    depth_min: float = 0.05
    depth_max: float = 10.0

    def __post_init__(self):
        if self.lidar_beams < 4 or self.width < 1 or self.height < 1:
            raise ValueError("Invalid sensor resolution")
        if not 0 < self.vertical_fov < 180:
            raise ValueError("Camera vertical FOV must be in (0, 180) degrees")
        if not 0 < self.lidar_min < self.lidar_max or not 0 < self.depth_min < self.depth_max:
            raise ValueError("Sensor ranges must satisfy 0 < min < max")

