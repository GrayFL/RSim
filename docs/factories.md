# 内置设备配方与兼容状态

`rsim.drivers` 中的类工厂保存不可变配置。`build()` 创建尚未运行的本地组件；`shared()`（支持该方法的工厂）创建共享启动描述；`device()` 只创建连接视图。实际资源由 `Runtime` 的组件生命周期打开和关闭。

```python
from rsim.drivers import CameraFactory, ROS2TopicFactory, ChassisFactory

camera = CameraFactory(device_path="/dev/video0", width=640, height=480)
local_camera = camera.build()
provider = camera.shared()
client = camera.device()
assert client.factory is None

imu = ROS2TopicFactory("/robot/imu", "imu", clock="ros:system")
imu_provider = imu.shared()

# 活动资源作为 build 参数传入，不存放在 Factory 中。
service_recipe = ChassisFactory(name="robot", motion_enabled=False)
service = service_recipe.build(pose=localizer.pose, velocity=chassis.velocity,
                               state=chassis.state)
local_telemetry = service_recipe.port_specs(pose=localizer.pose)
rpc_client = service_recipe.device()
```

这些代码只装配资源图。对一个物理资源只选择一次提供端装配；同时创建 `local_camera` 和 `provider` 是展示两种用法，不应同时运行它们去打开同一设备。

所有新增工厂通过 `fingerprint()` 返回 canonical JSON 的 SHA-256；包含类型、构造版本、运动权限、TTL 和语义配置。日志路径不影响指纹。嵌套字典和列表会独立冻结，调用者随后修改原对象不会改变配方。配置仅接受有限的普通数据；运行中的 ROS context、Signal、Component 等必须显式注入 `build()`。

## 协议边界

| 工厂 | 本地完整端口声明 | 发布/连接方式 |
|---|---|---|
| `CameraFactory` | `image`，host Unix 时间 | 既有单输出 `SharedSensor` / `rsim.devices.Camera` |
| `D435Factory` | `output`：保留各图像源时间的 Bundle | 既有单输出共享 RGB-D Bundle + `image` 流视图 |
| `RobinWFactory` | `points`，设备时钟 | 既有单输出 `SharedSensor` / `rsim.devices.RobinW` |
| `HipnucFactory` | `imu`，串口接收为 host monotonic；ROS 模式为显式 ROS 时钟 | 既有单输出 `SharedSensor` / `rsim.devices.Hipnuc` |
| `BlueSeaFactory` | `scan`，ROS system 时间 | 既有单输出 `SharedSensor` / `rsim.devices.BlueSea` |
| `ROS2TopicFactory` | 所选 `imu/scan/odom/state`，显式源时钟 | 既有单输出 `SharedSensor` / `rsim.devices.ROS2Topic` |
| `STM32Factory` | `odom/state/velocity_feedback/velocity`，ROS 数据时钟与 host 命令期限分开 | 本地 ROS 组件；可显式注入 `ros` 与 `feedback_sources` |
| `ChassisFactory` | 本地 `pose/state` 遥测 | 既有 MotionService RPC / `rsim.devices.Chassis` |

`port_specs(...)` 是各类所在设备模块内唯一完整声明。构建型传感器工厂从该声明取得 history/clock/TTL，并核对构造完成后的真实端口；适配器仍负责采样和命令执行。可调用 `Factory.port_specs(history=...)` 查看配置对应的声明，`RobinWFactory` 还接受 `ip`，`ROS2TopicFactory` 接受 `kind/clock`，`HipnucFactory` 接受 `clock`，`STM32Factory` 接受 `max_ttl`。

RPC 服务的 pose 来自调用方注入的资源，`ChassisFactory.port_specs(pose=...)` 因而要求显式给出同一个 pose 端口，仅在该次调用中读取它的 clock/history，配方不保存活动资源。state 遥测固定为 host monotonic；build 会检查两个本地端口。它没有可脱离输入预设的通用 pose 时钟，也不是多端口 Shared Ports 的启动配方。

这次保持原有单输出共享与 RPC 的线协议、key、version 和客户端调用方式；它们没有变成 `ChassisPortsDevice` 所用的多端口 Shared Ports。特别是 RPC 的运动操作不是 `ChassisFactory.port_specs()` 中的 CommandSink，不应把两类客户端连接到错误的服务。

## 原生参数和设备发现

Native ROS 的参数文件、node-scoped 参数和 remap 规则必须由 ROS 自己解析。新 Factory 的构造不会为此创建 ROS 节点；需要原生 `ros_args` 时先显式 `prepare()`：

```python
from rsim.drivers import D435Factory

recipe = D435Factory(ros_args=["-p", "enable_color:=false"])
prepared = recipe.prepare()  # 在 ROS 提供端执行：临时参数解析节点随后关闭
provider = prepared.shared()
```

`prepare()` 返回新的冻结配方，保存解析后的配置、端口路由与参数文件摘要，不保存 ROS 句柄。`build/shared/device` 不会隐式重复解析原生参数；未准备的原生参数会得到明确错误。参数文件在准备后变化时，构建失败并要求重新准备，不能静默复用旧配置。

`HipnucFactory` 的自动串口发现以及 ROS2 模式也需要显式 `prepare()`；给定串口的纯 Python 模式可以直接 `build()`。`BlueSeaFactory.prepare()` 验证 UART 路径、源标识与 scan 输出；`BlueSeaFactory` 和 `STM32Factory` 的准备阶段创建所需日志目录。准备不会启动设备驱动或发送激光/电机命令。

旧 `Camera/D435/RobinW/Hipnuc/BlueSea/ROS2Topic/STM32/Chassis` 函数只委托相应类。需要原生解析或自动发现的旧函数在委托时调用 `prepare()`，保留之前的便捷行为；新代码可以把准备作为独立、可检查步骤。

## 底盘仿真兼容变化

`Chassis(simulate=True)` 与 `ChassisFactory(simulate=True)` 延迟调用可选 `pyrobo.rsim.drivers.MemoryChassisFactory`，使用它的 `pose/velocity/state` 端口；构造本身不运行物理。此路径需要 pyrobo，纯 RSim、硬件服务和连接客户端无需安装 pyrobo。

该提供端使用 pyrobo 的有限驱动力模型和显式真值定位；旧 RSim 理想运动学、噪声及 IMU bias 不再被当作等价产品实现。控制器不额外运行一份 EKF。运动仍默认关闭；非零运动必须显式 `motion_enabled=True`。这是仿真测试场景，不代表定位或真实硬件验收。

## 全量工厂盘点与未完成项

| 原正式入口 | 本轮状态 | 剩余事项 |
|---|---|---|
| Camera | 已迁 `CameraFactory`；旧函数委托 | UVC/ROS2 真设备采集未验收 |
| D435 | 已迁 `D435Factory`；旧函数委托 | 原生参数准备已 mock 验证，真实 ROS2/相机未验收 |
| RobinW | 已迁 `RobinWFactory`；旧函数委托 | 真实 ROS2/雷达未验收 |
| Hipnuc | 已迁 `HipnucFactory`；旧函数委托 | Python 串口用 PTY 验证，ROS2/实物 IMU 未验收 |
| BlueSea | 已迁 `BlueSeaFactory`；旧函数委托 | 真实 ROS2/UART 雷达未验收 |
| ROS2Topic | 已迁 `ROS2TopicFactory`；旧函数委托 | 真实 ROS2 话题中继未验收 |
| STM32 | 已迁 `STM32Factory`；旧函数委托 | 构造/声明/安全配置已验，真实 ROS2/固件未验收 |
| Chassis | 已迁 `ChassisFactory`；旧函数委托 | 兼容仿真已转 pyrobo；实机控制不在本轮测试范围 |
| Mapper | 已迁 `MappingFactory`，接口 v2 | 构图无文件副作用；真实 ROS2 SLAM 未验收 |
| Odometry / ScanOdometry | 已迁对应 Factory，完整声明显式注入适配器 | 真实 ROS2 EKF/ICP 未验收 |
| NativeChassis | 已迁 `NativeChassisFactory`，返回 Component 组合图 | 实机驱动与运动未验收 |
| TopicChassis | 已迁 `TopicChassisFactory`，只连接已有设备 | 原生话题/硬件激活未验收 |

复杂 Factory 的配置格式、真实重建测试、资源清理及 breaking 变化见[复杂 Factory 生命周期](complex-factories.md)。`local_chassis.build/sources/calibrate` 是应用装配和标定辅助函数，不作为设备工厂登记；MuJoCo 入口由独立的兼容迁移记录管理。没有新增中央注册表。

## 验证范围

`tests/test_builtin_factories.py` 验证完整端口、轻量构造、递归配置隔离、稳定/敏感指纹、连接不启动、旧函数委托、显式准备与参数文件变更，以及模块级配方的真实 spawn 构造。`test_imu.py` 的 serial PTY 共享用例还运行新的绑定方法配方，检查共享实例、串口锁和退出清理；ROS2 模式单独排除。

同时保留 D435 流视图、原生参数纯数据校验、RPC 租约/重放/断联/取消等回归。默认 `task(hz=...)`、命令校验和传输热路径没有在本项改动；本项不据此声称经过性能验收。实际命令、环境、计数和日志记录在项目本地 `PROJECT.md`，跨库导航与时间性能另行验收。

回滚应停止依赖新配方的 Provider，再成套回滚工厂提交及其调用方；不要让新旧进程继续持有相同物理源的 key。返回旧理想仿真需要同步回滚对应兼容迁移，不能只还原导入路径。
