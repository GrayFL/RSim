"""Retired ideal-kinematics demo; production simulation belongs to pyrobo."""


def __getattr__(name):
    if name in {'SimulatedChassis', 'odom_message', 'imu_message'}:
        raise ImportError(
            'rsim.components.simulated_chassis was retired in RSim 0.4; '
            'use pyrobo.rsim.drivers.MemoryChassisFactory. The new finite-force '
            'model is not numerically equivalent to the old ideal/noisy demo. '
            'See docs/migration-v4.md.')
    raise AttributeError(name)
