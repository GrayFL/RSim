# 复杂 Factory：配置、依赖与资源生命周期

`rsim.drivers` 导出 `MappingFactory`、`OdometryFactory`、`ScanOdometryFactory`、`NativeChassisFactory`、`TopicChassisFactory`。配方保存冻结的普通数据，`build()` 返回未运行的 Component DAG；`shared()` 保存可以在 Worker 无参调用的绑定方法；`device()` 只连接已有 Provider。活动 Signal、Component、RosContext 和历史缓存只通过 `build(...)` 注入。

```python
from rsim.drivers import OdometryFactory

recipe = OdometryFactory.from_config("odometry.yaml")
local = recipe.build(wheel=wheel_signal, imu=imu_signal, ros=ros_context)
provider = recipe.shared()  # 另一种装配方式：Worker 从配置重建输入
client = recipe.device()
```

上例展示不同部署方式；对同一物理资源只运行一个提供端。`shared()` 不继承前一次 `build()` 的注入对象。需要复用已经运行的提供端时，把其 key 和接口写入 `sources`，或者对本地图显式传入 Signal。

配置文件支持 safe YAML 和 TOML，拒绝重复键、未知字段、不支持的 schema、非有限数值和错误类型。根节点可以直接保存工厂字段，或只包含 `schema_version` 与对应的 `mapping`、`odometry`、`scan_odometry`、`chassis` 节。`overrides` 在构造前递归合并；相对数据库、日志、几何文件、串口及 `--params-file` 路径相对于配置文件解析。

```yaml
schema_version: 1
odometry:
  directory: ./logs
  imu_mount:
    wrd_frame: base_footprint
    ego_frame: imu
    position: [0, 0, 0]  # 替换为实测安装位置
    rotation: [0, 0, 0, 1]
  wheel_topic: /rsim/chassis/odom
  imu_topic: /rsim/chassis/imu/data
  parameters: {frequency: 50.0}
  history: 512
```

安装位姿规范化为带坐标标签的 SE(3) 快照。指纹包含 Factory/接口/schema 版本、规范化配置和输入文件内容摘要；日志目录与随机临时目录不影响指纹。已加载的外部文件改变后，构图或准备失败，需重新加载配方。配置文件本身是读取时的快照，修改原字典或文件不会静默修改旧配方。

## 输入引用

每种工厂只解析自己的显式输入名：Odometry 为 `wheel/imu/scan`（此处 scan 是估计器输出）；ScanOdometry 为原始 `wheel/scan`；Native/Topic 为 `wheel/state/imu/scan`；Mapping 为 `lidar/imu/wheel/scan/camera/camera_info`。Native 的 wheel/state 必须一起配置。

引用有三种协议，不会启动提供端：

```yaml
sources:
  imu:
    kind: shared_device
    key: imu:front
    interface_version: imu-ports-v1
    port: imu
    schema: rsim.value.v1
    clock: ros:system
    history: 512
    transport: {backend: cyclonedds, domain_id: 0}
```

- `ros_topic`：`topic` 指定原生话题，适用于 IMU、轮速、扫描。工厂本地图共用一个 RosContext。
- `shared_device`：连接多端口 SharedProvider；必须明确 `key/interface_version`，可声明 `port/schema/clock/history/transport/revision`。
- `shared_sensor`：连接既有单输出 SharedSensor；用 `version` 代替 `interface_version`，支持 `key/port/clock/history/transport/revision`。D435 Bundle 可额外用 `bundle_stream: color` 选择图像；配置中的 version 必须与已有提供端一致。

`revision` 是调用方提供的资源版本标签。描述中的 `clock` 必须与实际端口一致，不能用改标签代替时钟校准。EKF/ICP 的输入要求 `ros:system`。Mapping 的设备点云保留逐点时间，通过 `timing.lidar` 对齐；共享设备相机必须同时提供 image 和 camera_info，并对设备时钟明确给出 `timing.camera.offset_s`。RGB 和标定帧必须使用同一时钟，标定/安装坐标需能建立到图像 optical frame 的变换。

## Native 与 Topic 的控制所有权

```python
from rsim import Runtime
from rsim.drivers import NativeChassisFactory

recipe = NativeChassisFactory.from_config("robot.yaml")
robot = recipe.build()
# Runtime(robot) 暴露 pose/odom/state/velocity，供外部命令写者使用。
# Runtime(robot.control) 激活本地 move/rotate 控制器，并由它独占 velocity。
provider = recipe.shared()
service = recipe.service(source=provider)  # 服务复用这个提供端，不再构建一份硬件
```

NativeRobot 是 Component，完整共享端口为 `pose/odom/state/velocity`；`.odometry` 仍指估计器组件，`.control` 是独立的本地控制根。底层 STM32 sink 由组合图独占，公开速度 sink 转发原始命令身份、序号和 host deadline，不生成新的有效期。运动默认关闭，必须明确 `motion_enabled=True`。原生硬件没有仿真 source context，因此拒绝携带 `source_bound` 的命令。

TopicChassisFactory 只连接已有话题或提供端，禁止硬件 `port/parameters/ros_args/start_driver=True`，不会自动创建 ROS2Topic relay。它仍可启动明确的 EKF/ICP 算法进程。需要跨进程中继时，先独立启动提供端，再声明输入引用。新的共享接口为 `native-chassis-ports-v1`；原生诊断 state 不冒充内存/MuJoCo 的通用 measured-state 协议。

Native 配置包含原生 `ros_args` 时，先在提供端环境调用 `prepare()`；临时 ROS 参数解析节点会被销毁，配方只保留解析结果和摘要。运动权限、设备身份与有效原生参数冲突时拒绝构图。

## Mapping 与持久化资产

MappingFactory 独占 `pose/odometry/map/rgb_map/status` 的完整声明，`history` 应用于真实输出和共享视图。`mapping-ports-v2` 使用实际封闭编解码器的 `rsim.value.v1` schema；旧 v1 的专用 schema 字符串与真实端口不一致，已弃用。

```python
from rsim.drivers import MappingFactory

recipe = MappingFactory.from_config("mapping.yaml")
local = recipe.build(input_source=recorded_inputs)  # 或 build(ros=ros, sources={...})
client = recipe.device()
```

只有协议信息的轻量客户端可使用：

```python
from rsim.devices import Mapper
from rsim.drivers import MappingFactory

client = Mapper("mapping", ports=MappingFactory.port_specs(history=3),
                interface_version=MappingFactory.INTERFACE_VERSION)
```

设备层不导入 Factory。也可以直接把提供端发给调用方的 ports/interface_version 传入设备入口；`load_mapper(..., providers=False)` 继续提供配置式连接入口。

`database_policy: new` 是目前唯一支持的策略。构图不创建数据库目录；Runtime 首先取得数据库和相邻 `.laser` 档案的独占所有权，再检查已有内容。非空数据库或档案会拒绝新会话，不会自动恢复或覆盖。启动失败、正常关闭和重复关闭都释放锁，保留用户数据库、地图档案、外部 YAML 和日志。恢复既有 RTAB 会话尚未实现。

## 生成参数与迁移

EKF、ICP、MappingFusion 和 RTAB 参数在构造时保持为 `GeneratedParameters` 内容快照。Driver.open 原子写入当前会话的私有 YAML，验证摘要后启动原生进程；启动失败、取消或关闭先停止子进程，再清理自有临时目录。原始 `--params-file` 只读取和校验，始终由用户持有。

| 旧行为/入口 | 当前行为 |
| --- | --- |
| 构造后在输出目录读取 `*-parameters.yaml` | 通过 `driver.generated_parameters.parameters` 检查配置；临时文件仅运行时存在 |
| `Odometry(...)` | 委托 OdometryFactory；活动输入、ROS context、历史缓存仅在 build 注入 |
| 直接构造 InertialOdometry/ScanOdometry/MappingOutput | 必须显式传 Factory 的完整 `ports`；优先使用 Factory.build |
| NativeRobot 普通容器 | 变为 Component；外部速度写入与本地 controller 使用不同 Runtime 根 |
| `TopicChassis(relay_transport=...)` | 删除自动 relay 启动；使用原生话题或已有提供端引用 |
| `devices.Mapper(name, history=...)` | 使用 Factory.device，或明确传入 ports/interface_version |
| `devices.mapping.mapping_ports` | 已删除，完整声明只在 MappingFactory.port_specs |
| `mapping-ports-v1` | 更新提供端和客户端到 v2，不能混用旧实例 |

`tests/test_complex_factories.py` 覆盖真实 spawn 构图、两种共享输入协议、单 ROS owner、原始命令期限、持久资源竞争与失败回滚。共享 NativeRobot 测试以测试组件替代物理电机/原生估计器；它验证组合和传输，不代表实机验收。`tests/test_generated_parameters.py` 启动真实可执行子进程检查运行期参数和清理。真实 ROS2 EKF/ICP/SLAM 与硬件激活单独标记为未运行；CPU、MuJoCo 和跨库导航验收见对应阶段记录。
