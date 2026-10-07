# 本机串口底盘

`rsim.drivers.STM32` 将电机板的串口协议适配为 ROS2，再暴露 RSim 的 `odom`、`state` 和 `velocity` 端口。轮式里程计来自 MCU；IMU 是独立设备，需要单独组装。该驱动适用于 Robint delivery 协议（144 字节上行、72 字节下行），不是通用 STM32 固件驱动。

## 构建与启动

将仓库的 `ros2/rsim_stm32` 链接或复制到 ROS2 工作区的 `src/`，在已加载 ROS2 环境的工作区中运行：

```bash
colcon build --packages-select rsim_stm32
source install/setup.bash
```

默认禁止非零速度。串口地址由使用者提供，推荐稳定的设备身份路径。

```python
from rsim import Runtime, VelocityCommand
from rsim.drivers import STM32

chassis = STM32(port=serial_port, parameters={"baudrate": 921600})
async with Runtime(chassis):
    odom = await chassis.odom.get(timeout=10)
    ack = await chassis.velocity.set(VelocityCommand(), ttl=.2)
    state = await chassis.state.get(timeout=5)
```

原生参数、参数文件和 ROS remap 均可通过 `parameters` / `ros_args` 传入：

```bash
python -m rsim.drivers stm32 --port "$CHASSIS_PORT" \
  --ros-args -p baudrate:=921600 -p command_hz:=100.0 -r odom:=wheel_odom
```

CLI 保持驱动运行；函数形式可将 provider 纳入现有异步 Runtime。默认命名空间为 `/rsim/chassis`，ROS2 发布 `odom`、`battery`、`estop`、`is_normal`、`diagnostics`，提供 `set_velocity` 与 `stop` 服务。RSim 自动跟随相关 remap。不要并行启动旧驱动争用同一个串口。

## 时效和生命周期

命令包含 controller ID、session epoch、递增序号和同机 `CLOCK_MONOTONIC` 截止时间。只接受期限内的命令；重复、过期、越界和冲突所有者会被拒绝。**截止时间不能原样跨主机使用**。原生确认表示写入主机串口完成，并非电机执行确认；实际运动应检查 odom/IMU。

C++ 串口线程默认 100 Hz，独立于 Python 和 ROS executor 检查命令有效期及反馈时效。命令到期、急停、碰撞信号、电机错误、协议不符、串口异常或反馈陈旧均归零；恢复反馈不会自动恢复旧速度。默认反馈超时为 0.2 秒，单次命令 TTL 最长 0.5 秒。正常关闭会重复写入零速，不复位里程计、不自动释放急停或清除电机故障。

这不是硬实时保证。操作系统停顿、USB 断线、进程被强制杀死时，主机代码无法保证送达零速；还需 MCU 自身的通信超时保护。IMU、里程计的接收时间也不能冒称为硬件同步采样时间。串口参数和限值在启动时生效。

## 本机位姿和运动

配置模板见 [local_chassis.example.yaml](../examples/control/local_chassis.example.yaml)，可执行组装见 [local_chassis.py](../examples/control/local_chassis.py)，交互示例见 [Notebook](../examples/control/local_chassis.ipynb)。配置文件保存设备身份、原生参数、标定文件位置、滤波和控制选项。

```python
from rsim.config.local_chassis import load_local_chassis as build

robot = build(config_file, motion_enabled=True)
async with Runtime(robot.control):
    pose = (await robot.pose.get(timeout=15)).data  # graphmap.Pose
    await robot.control.rotate(yaw_deg=10.)
    await robot.control.rotate(yaw_rad=-0.1745329252)
    await robot.control.move(distance_m=0.1)
```

本机轮式里程计和外置 IMU 经 `PlanarOdometry` EKF 输出带坐标系标签的 Pose，再供 `ChassisController` 控制；正常运行不连接 ROS1。默认示例为零速。非零运动需显式启用，并在上层集成场地与障碍监测；基础控制器不负责避障规划。

## 外置 IMU 的初步校正

`calibrate_planar_imu` 接收同时静态采集的两组 ROS 无关 IMU 字典，以及参考 IMU 到底盘的已知 graphmap Pose。校正要求机器人静止、接近水平，估计外置陀螺零偏及其坐标系内的竖直轴。`rsim.config.local_chassis.calibrate_local_chassis()` 在采集期间检查轮速、输入新鲜度并保持零速，将结果保存到配置指定的 JSON。

`PlanarIMU` 去零偏后向竖直轴投影，输出虚拟平面 IMU：仅 Z 轴角速率是测量值，X/Y 为高方差占位，姿态与加速度明确标记不可用。源时间与接收时间保持不变。静止重力无法确定安装 yaw、平移、动态时差、加速度偏置或量程误差；`tilt_pose()` 仅供检查最小倾斜旋转，不能充当完整安装外参，也不能用于三维 LIO。动态旋转应再与参考 IMU 和轮式角度交叉验证。

## 验证

```bash
colcon test --packages-select rsim_stm32
python -m pytest tests/test_stm32.py tests/test_imu_calibration.py tests/test_motion.py
```

原生协议测试检查校验、碎片恢复、帧布局和命令仲裁；Python 集成测试使用虚拟串口，验证事件循环阻塞、反馈中断、急停和恢复时的停车行为，不打开真实硬件串口。
