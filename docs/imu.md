# HiPNUC 串口 IMU

同一台串口 IMU 可以直接通过 Python 读取，或经 ROS2 的标准 `sensor_msgs/Imu` 话题接入。两种模式共用 RSim 的协议解析器；打开同一串口的消费者应复用共享 provider，不应同时启动两个独占串口驱动。

## 依赖与 ROS2 包

先按 graphmap 项目的说明准备 Pose v3，再准备串口与配置依赖，最后安装本库。graphmap 是单独维护的源码包，不能假定公开包索引上的同名包满足本项目版本要求。每次安装前先检查 uv 的计划，确认不会改变 NumPy、Torch 等已锁定版本：

```bash
uv pip install --dry-run --python "$(command -v python)" pyserial PyYAML
uv pip install --python "$(command -v python)" pyserial PyYAML
uv pip install --dry-run --python "$(command -v python)" --no-deps -e '.[imu,config]'
uv pip install --python "$(command -v python)" --no-deps -e '.[imu,config]'
```

串口读取需要 pyserial，配置加载需要 PyYAML。Linux 通常已提供 `cp210x` 内核驱动；先检查 USB 枚举、`/dev/serial/by-id/`、设备权限及实际绑定的驱动，不应直接用旧版厂商驱动覆盖当前内核模块。没有指定 port 时，只在恰好发现一台 CP210x 时自动选择，多台设备必须明确指定。

ROS2 入口源码位于 `ros2/rsim_hipnuc`，复用 Python 模块 `rsim.adapters.ros2.imu`。在已加载 ROS2 的开发环境中，将该目录放入 ROS 工作空间并只构建此包：

```bash
# ROS_WORKSPACE 由调用者设置为自己的工作空间。
ln -s "$PWD/ros2/rsim_hipnuc" "$ROS_WORKSPACE/src/rsim_hipnuc"
cd "$ROS_WORKSPACE"
colcon build --packages-select rsim_hipnuc
source install/setup.bash
```

该包运行时需要 `rclpy`、`sensor_msgs` 和已安装的 RSim IMU 依赖；可执行入口使用当前 PATH 中的 `python3`，须与 ROS Python ABI 匹配。无需重编译其他已安装的设备包。

## 库式调用

纯 Python、当前事件循环内直接读取：

```python
from rsim import Runtime
from rsim.adapters.hipnuc import SerialIMU

imu = SerialIMU(port=serial_device, baudrate=460800)
async with Runtime(imu):
    frame = await imu.imu.get(timeout=5)
    acceleration = frame.data["linear_acceleration"]   # x/y/z，m/s²
    gyro = frame.data["angular_velocity"]              # x/y/z，rad/s
```

需要共享源或把采集放到独立进程时：

```python
from rsim.drivers import Hipnuc

imu = Hipnuc(serial_device, mode="serial")    # Python 串口 provider
# 或：imu = Hipnuc(serial_device, mode="ros2") # 启动 ROS2 节点并订阅
async with Runtime(imu):
    frame = await imu.imu.get(timeout=15)
    same = await imu.imu.get(timestamp_ns=frame.stamp_ns, clock=frame.clock)
```

另一个应用可通过 `rsim.Hipnuc(serial_device, history=...)` 连接已有 provider；history 须一致，应用无需 ROS。设备路径先解析符号链接，使 by-id 和 tty 路径不会重复占用同一串口。合作设备锁与 pyserial exclusive 锁控制占用，Runtime 退出释放；断流、CRC 校验失败持续到 timeout 或串口断开会触发组件故障。不会自动修改设备波特率、输出协议或闪存设置。

## 命令行与原生 ROS 参数

```bash
python -m rsim.drivers imu --imu-mode serial --port /dev/ttyUSB0
python -m rsim.drivers imu --imu-mode ros2 --port /dev/ttyUSB0 \
  --ros-args -p baudrate:=460800 -p frame_id:=imu_link -r imu/data:=/sensors/imu

ros2 run rsim_hipnuc serial_node --ros-args \
  -p port:=/dev/ttyUSB0 -p baudrate:=460800 -p frame_id:=imu_link
ros2 launch rsim_hipnuc serial.launch.py port:=/dev/ttyUSB0 baudrate:=460800
```

函数的 `parameters={...}`、`ros_args=[...]` 支持原生参数文件、节点名/namespace/topic remap；订阅器会解析最终话题。主要启动参数为 `port`、`baudrate`、`frame_id`、`navigation_frame`、`gravity`、`hz`、`timeout`、`history`。`mode="serial"` 的 `parameters` 对应 SerialIMU 构造参数，不能传 ROS argv。

ROS2 标准话题为 `imu/data`，采用 sensor-data QoS。ROS 接口仅转发标准 Imu 字段；串口原始标签、Euler 来源、磁场、气压和可选设备毫秒计数保留在 Python 直读的 `metadata` 中。全零磁场/气压可能是设备占位输出，不能据此断言有对应物理传感器。

## 协议、单位与坐标

支持 `5A A5 + uint16 length + uint16 CRC + payload` 帧，CRC-16/XMODEM 覆盖前四字节和 payload。支持旧分项标签 `90/A0/B0/C0/D0/D1/D9/F0` 及 `91` IMUSOL；不支持网关多节点、CAN 或其他新协议。流式解析处理碎片、粘包、坏 CRC 和重新同步，未知或截断布局整包拒绝，不把上一包字段补入本包。

- 加速度转换为 m/s²，保留重力；默认协议换算因子 `gravity=9.8`。
- 角速度转换为 rad/s，四元数统一 xyzw。分项 D0 的 wire 顺序是 pitch/roll/yaw，缩放分别为 0.01/0.01/0.1 度；缺四元数而有 Euler 时用 graphmap Pose 生成四元数。
- 未提供的测量在 covariance 首元素标为 `-1`；提供但没有协方差的测量使用全零，表示未知，并不表示零噪声。
- 本地旧版手册定义载体 FLU、导航 NWU，但固件的轴旋转/导航配置可能已改动。本适配器不猜测实际安装朝向，也不把 IMU 的导航系冒充 `base_footprint` 或 odom。`frame_id` 是传感器局部坐标标签；`navigation_frame` 是记录用标签，不执行坐标变换。
- Python 输出时间域为 `host:monotonic`，ROS2 输出为 `ros:system`（启用仿真时为 `ros:sim`），均为接收/发布时刻。旧分项协议没有采样时间；IMUSOL 的设备开机毫秒数只作为 metadata 保存。两路模式不是同步采集，不应直接混合时间戳做融合。

本实现基于设备随附的 HiPNUC HI229 协议说明和旧版示例。厂商当前 SDK 及 ROS 驱动见 [官方软件仓库](https://github.com/hipnuc/products)；协议与型号支持范围应按具体版本核对，不能仅凭 USB 串口芯片判断 IMU 型号。
