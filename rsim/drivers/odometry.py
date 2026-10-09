"""Local ROS odometry recipes, usable without mapping or actuator ownership."""


def Odometry(*, imu_mount, directory, wheel=None, imu=None,
             wheel_topic='/rsim/chassis/odom', imu_topic='/rsim/chassis/imu/data',
             scan_topic=None, scan_mount=None, scan=None, ros=None,
             prefix='/rsim/odometry', world_frame='odom_fused',
             imu_options=None, scan_parameters=None, parameters=None, **options):
    """Fuse wheel twist and mounted IMU; optionally add RTAB 2D ICP pose.

    This graph creates native ROS subscriptions and processes, no SharedSensor
    or ROS2Topic provider. Supply Signals to reuse sources already in a graph.
    If ICP is enabled, wheel_topic must carry the same wheel data as wheel.
    """
    from rsim.adapters.ros2 import RosContext, RosSensor
    from rsim.adapters.ros2.odometry import InertialOdometry
    from rsim.adapters.ros2.scan_odometry import ScanOdometry
    from rsim.components.inertial import BodyIMU

    ros = ros if ros is not None else RosContext()
    wheel = wheel if wheel is not None else RosSensor(
        wheel_topic, 'odom', ros=ros, clock='ros:system', history=512, hz=500).odom
    imu = imu if imu is not None else RosSensor(
        imu_topic, 'imu', ros=ros, clock='ros:system', history=512, hz=500).imu
    body_imu = BodyIMU(imu, imu_mount, **(imu_options or {}))
    if scan is not None and scan_topic is not None:
        raise ValueError('pass a scan estimator or a scan topic')
    if scan_topic is not None:
        if scan_mount is None:
            raise ValueError('scan odometry requires a measured scan mount')
        scan = ScanOdometry(ros, prefix=prefix, mount=scan_mount, directory=directory,
            wheel_topic=wheel_topic, scan_topic=scan_topic, odom_frame=world_frame,
            parameters=scan_parameters)
    result = InertialOdometry(wheel, body_imu.imu, ros=ros, scan=scan,
        prefix=prefix, world_frame=world_frame, body_frame=body_imu.mount.wrd_frame,
        directory=directory, parameters=parameters, **options)
    result.body_imu = body_imu
    return result
