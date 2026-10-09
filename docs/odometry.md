# 可组合底盘里程计

底盘控制需要两个端口：`Signal[graphmap.pose.Pose]` 和接收 `VelocityCommand` 的 `CommandSink`。估计器不持有电机控制权。`ChassisController(pose=..., velocity=...)` 提供异步 `move(distance_m)`、`rotate(yaw_deg=... / yaw_rad=...)`；正距离向前，正角度左转。`drivers.Chassis` 可将相同端口挂到已有 DDS 服务，客户端仍使用 `devices.Chassis`。

```python
from rsim.drivers import Odometry, STM32, Chassis
from rsim.runtime import Runtime

motor = STM32(start_driver=False)  # native driver already running on this host
odometry = Odometry(wheel=motor.odom, imu_topic=imu_topic,
                    imu_mount=body_imu_pose, directory=assets / 'odometry')
service = Chassis(pose=odometry.pose, velocity=motor.velocity,
                  state=motor.state, name='chassis', motion_enabled=False)
async with Runtime(service):
    frame = await service.pose.get(timeout=20)
    await service.wait()
```

`imu_mount` 使用 graphmap 的 `T_body_imu`：`wrd_frame=base_footprint`，`ego_frame` 必须与消息一致；四元数顺序 xyzw。`directory` 指向项目根 `assets/` 下的目录，保存原生参数与日志。

## 两种估计器

`drivers.Odometry` 返回有 `pose`、`odometry`、`status` 的 Component。`pose` 为 graphmap Pose；`odometry` 为标准 Odometry 的 Python 字典，包含完整协方差。输出使用 `ros:system` 时钟，可按 Signal 历史接口访问。构建和导入不启动 ROS 进程，Runtime 管理启动、失败传播和回收。

- 轮速＋IMU：轮速只融合 body `vx`、非完整约束 `vy=0` 和 `wz`，避免同时融合由相同编码器积分的位姿。IMU 融合相对初始 yaw、yaw 角速度和前向加速度。
- 加入 2D ICP：提供 `scan_topic`、`scan_mount`；RTAB `icp_odometry` 使用同一 `wheel_topic` 的运动先验与逐束去畸变，其 XY/yaw 位姿加入同一个 EKF。私有 TF 以当前会话首个轮式位姿为原点，不继承 MCU 的历史累计坐标。必须保持启动阶段静止，使轮式、IMU 与扫描建立相同原点。

融合由原生 `robot_localization/ekf_node` 执行，默认 50 Hz、平面运动、3 秒迟到观测历史。扫描通常比轮速/IMU 晚到，滤波器回溯更新后继续发布当前采样时刻的估计，控制反馈不必等待下一次扫描。不会发布全局 TF，也不会使用 SLAM 回环造成跳变的 map 位姿来闭环控制。

`BodyIMU` 在 Python 中用完整三维姿态和安装旋转将信号变换到车体，并从加速度中减去重力；这允许有轻微侧倾的平面底盘正确处理加速度。输入姿态须是 ENU（Z 向上），角速度 rad/s、加速度 m/s²。NED 姿态必须先显式转换；不能仅改变 frame 名称。IMU 位置偏置不用于杆臂加速度补偿；偏离旋转轴明显时应先补偿，或 `use_acceleration=False`。`imu_options.remove_gravity=False` 用于已去重力的输入，不能重复减去重力。

零协方差按未知值处理，使用可配置噪声下限；不可用的测量（协方差首元素为 -1）不会作为零观测。若配置要求该测量，组件报错。默认 yaw/角速度/加速度标准差为 0.07 rad、0.02 rad/s、0.3 m/s²，是原型权重，需要实验标定。横向加速度默认不融合，可用 `lateral_acceleration=True` 显式开启。滤波状态为平面，不能据此获得经过融合的完整六自由度姿态。

轮速或 IMU 源时间超过 `input_timeout` 时停止发布新 pose，原生滤波器的预测不会刷新反馈年龄；控制器随后按 `pose_timeout` 停止。启用 ICP 时会等待首个有效扫描观测；扫描短时缺失可由轮速/IMU维持，前端持续无输出会失败。`status` 报告输入计数、无效观测和各源采样年龄。算法保留噪声下限，但扫描与轮速先验存在相关性，输出协方差不是定位真值误差。

## 配置与原生驱动

[配置模板](../examples/control/native_chassis.example.yaml) 通过 `rsim.config.load_chassis(path)` 组装 `NativeChassis`，选择 `wheel_imu` 或 `scan_imu`。它提供 `.pose`、`.velocity`、`.odometry`、`.control` 和 `.chassis.state`。

```bash
python -m rsim.apps.chassis_service --config configs/chassis_ros2.yaml
```

默认只允许零速。需要运动时增加 `--enable-motion`；若连接已运行的 STM32 节点，该节点也需显式启用运动。`start_driver: false` 直接接入本机原生 ROS 话题；`true` 根据串口、原生 `parameters` 与 `ros_args` 启动驱动。IMU 使用 `rsim_hipnuc`，2D 雷达使用 `bluesea2`。变更 remap 后应保持配置中的 `topic` 与原生输出一致。轮式融合需要 ROS `robot_localization`，二维前端另需 `rtabmap_odom`。

内部读取通过 `RosSensor`，没有 ROS2Topic/SharedSensor 中继或大数据 DDS 解耦。只有最终 chassis 服务供独立 Python 客户端连接，部署与键盘操作沿用 [远程控制](remote-control.md)。旧的外置 IMU 平面标定配置仍由 `load_chassis` 分派给旧加载器；它与这里的完整姿态底盘 IMU 配置分开维护。

建图链路启用 `fusion_parameters` 时也组装同一个 `drivers.Odometry`。本机 ROS 底盘在 `Mapper` 中省略 `connection`，通过 `topics` 指定 IMU、轮速和扫描话题，使用零 chassis 时钟偏移；旧 ROS1 输入仍可显式配置 `connection`。底盘已融合位姿的重复融合不在此输入契约内。

## 2D 雷达的车体遮挡

安装在车体内部的雷达可能看到立柱和线缆。安装 Pose 决定坐标变换；车体回波过滤是另一个步骤，不能通过移动外参来消除真实的车体结构。`scan.topic` 保留原始扫描，可通过显式 `scan.filter` 生成供 ICP、建图和监测使用的扫描：

```yaml
scan:
  topic: /rsim/chassis/scan_raw
  start_driver: false
  filter:
    topic: /rsim/chassis/scan
    # Four-post chassis example only; measure the actual occluded sectors.
    angle_masks: [[-2.34, -1.95], [-1.1, -0.75], [0.75, 1.1], [1.95, 2.34]]
```

角区单位为弧度，在 LaserScan 坐标系中从 +X 朝 +Y 为正。`AngularScanMask` 将遮挡束标为 NaN，并清零对应 intensity；保留束数、索引、角度间隔、逐束时间和首束采样时刻。NaN 表示不可观测，不能当作没有障碍的自由空间。区域是车体实际遮挡造成的盲区，需由其它传感器或现场条件覆盖；库没有内置通用车体角区。

`NativeChassis` 暴露 `.scan`，配置过滤后为处理结果；`.scan_filter` 持有原生 ROS 输出适配器。使用 `drivers.Chassis(robot)` 时服务管理这个扫描资源；直接 `Runtime(robot.scan)` 也可独立运行过滤。ICP 自动接处理后的话题；另行启动 Mapper 时将 `topics.scan` 指向相同话题。原始扫描始终可用于对照和重新配置遮挡。

BlueSea 原生驱动也提供 `with_angle_filter`、`min_angle/max_angle`、`mask1...` 以及重采样参数，具体含义依驱动版本。这里在原始输出后处理，以保留可对照的原始数据。应用层的阴影去噪、地图区域过滤、电梯门过滤与车体遮挡不同，不因使用本组件而自动启用。

融合输入选择参考 [robot_localization 官方配置说明](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/configuring_robot_localization.rst)。
