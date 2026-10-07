# ROS 驱动与 DDS 应用分离

RSim 将驱动启动和应用连接分为两个入口。`rsim.drivers` 在具备设备驱动的环境中启动共享源；`rsim` 的设备工厂只连接已有源。两侧仍使用同一套 `Component`、`Signal.get()`、`Runtime` 和组合接口。

ROS 2 原生话题跨机器传输到 RViz 时，Fast DDS 的 UDP 包长、收发缓冲与 SHM 配置见 [ROS 2 网络部署](ros2-network.md)。该配置与下述原生 Cyclone DDS 描述信息后端分别生效。

底盘命令与 Pose 使用原生 Cyclone DDS 的小型 JSON 服务，可直接跨机器通信，双端网卡与静态发现配置见 [远程控制](remote-control.md)。下述共享内存传感器通道仍以同机为边界。

## 多输出组件

新多输出接口使用 `SharedProvider` / `SharedComponent`，`serve_shared()` 仅保持源租约，按 Runtime 的具体 Signal 需求创建 exporter；详细协议和历史语义见 [多端口共享](shared-components.md)。建图已迁移到该协议，见 [建图接口](mapping.md)。下方 D435 等单输出例子仍使用兼容适配器。

## 运行两个独立环境

驱动环境需要与 ROS 兼容的解释器和设备驱动。以 D435 为例，在终端运行：

```bash
python -m rsim.drivers d435 --backend ros2
```

应用环境安装 `rsim[dds]`，无需安装或加载 ROS。配置相同的 `ROS_DOMAIN_ID`，未设置时默认为 0，然后运行：

```python
from rsim import D435, Runtime

color = D435(stream="color")
depth = D435(stream="depth")
async with Runtime(color, depth):
    rgb = await color.get(timeout=30)
    distance = await depth.get(timeout=30)
    same = await depth.get(timestamp_ns=distance.stamp_ns, clock=distance.clock)
```

可以在 Jupyter 中直接运行上述异步代码。普通脚本则放入异步函数，由 `asyncio.run()` 启动。可执行示例见 [DDS 客户端 Notebook](../examples/camera/dds_client.ipynb)。

驱动也可以在 Notebook 中运行，无需 ROS 命令行启动 Python 节点：

```python
import asyncio
from rsim.drivers import D435, serve

provider = D435()
provider_task = asyncio.create_task(serve(provider))
```

该任务保持驱动租约并传播启动/运行错误；调试时查看 `provider_task.done()` 和完成后的 `provider_task.exception()`。结束时取消并等待任务，使 Runtime 完成清理：

```python
provider_task.cancel()
try:
    await provider_task
except asyncio.CancelledError:
    pass
```

共享源在其他使用者持有租约时继续运行，最后一个使用者退出后关闭硬件。源还未启动时，应用侧明确报错，不会尝试在应用解释器中加载 ROS 或执行驱动工厂。

## 配置与迁移

从 0.2 开始，原来自动启动硬件的 `rsim.D435()`、`rsim.RobinW()`、`rsim.Camera()` 改为只连接。单环境程序若希望保留自动启动行为，将这些工厂的导入改为 `rsim.drivers`；其他 `Runtime`、`Map`、`ProcessSensor` 等仍从 `rsim` 导入。驱动日志参数 `log_path` 归驱动端配置。

设备标识、profile 和 history 应在两端保持一致。D435 的两路默认均为 `640x480x15`，history=8；CLI 可通过 `--serial`、`--depth-profile`、`--color-profile`、`--history` 和 `--log-path` 配置。相同物理相机不要混用空序列号和显式序列号。冲突配置被拒绝，不会悄悄复用不同设备设置。

驱动专属配置通过 `parameters={...}` 或 `ros_args=[...]` 传入；CLI 在 `--ros-args` 后接受原生 `-p`、`--params-file`、`-r` 等选项，见 [驱动参数说明](drivers.md)。这些参数仅归 provider 所有，无 ROS 客户端无需复制它们。若覆盖设备标识或 profile，客户端需使用覆盖后的有效值。`SharedSensor` 的 `provider_version` 支持这种分离：provider 之间比较驱动配置，连接型客户端仍只检查公共 version/history。

也可以显式配置后端与 domain：

```python
from rsim import TransportConfig, D435

transport = TransportConfig(backend="cyclonedds", domain_id=0)
camera = D435(transport=transport)
```

`ProcessSensor`、`SharedSensor` 以及设备工厂均接受 `transport`；新建图可用 `ProcessPlacement(name, transport=...)` 选择部署后端，见 [架构文档](architecture.md)。未传时，后端取 `RSIM_TRANSPORT`（默认 `cyclonedds`），domain 取 `ROS_DOMAIN_ID`（默认 0）。新建 worker 会继承所选 domain 和后端；显式指定 domain 时，worker 内的 ROS 驱动也使用该 domain。已有源的客户端可以选不同后端，但必须位于同一 domain。

通用共享数据源同样支持生产与连接分离：

```python
from rsim import SharedSensor
from rsim.components.synthetic import CounterArray

provider = SharedSensor(lambda: CounterArray(), key="counter", version="1")
client = SharedSensor(key="counter", version="1")
```

工厂只在创建源的环境内序列化并启动，不发送给连接者。不同 Python 版本之间传输的是描述信息和数组，不是 pickle 对象。应用内的嵌套 `ProcessSensor` 使用应用自身的解释器；共享源仍留在驱动环境。

## 同一 DDS 总线

`DescriptorTransport` 统一创建发布者、订阅者和轮询任务。`cyclonedds` 后端只依赖原生 Cyclone DDS；`ros2` 后端按需加载 rclpy。`ProcessSensor`、`SharedSensor` 和 worker 不再直接操作 ROS 消息或执行器。

两个后端使用同一个 DDS wire contract：

| 项目 | 约定 |
| --- | --- |
| domain | 同一个 `ROS_DOMAIN_ID`，或显式 `TransportConfig.domain_id` |
| ROS topic | `/rsim/frames/p<源实例标识>` |
| 原生 DDS topic | `rt/rsim/frames/p<源实例标识>` |
| DDS type | `std_msgs::msg::dds_::String_`，一个 `data` 字符串字段 |
| 编码 | XCDR1，字符串内容为 JSON 帧描述信息 |
| QoS | Reliable、TransientLocal、KeepLast(16) |

topic 前缀遵循 [ROS 2 到 DDS 的命名映射](https://design.ros2.org/articles/topic_and_service_names.html)。原生端定义等价 IDL 类型，不导入 `std_msgs`。ROS 与原生 DDS 端点直接发现并互通，没有额外 domain、数据转发桥或第二套数组缓冲。

新部署的 Signal 通道使用 `/rsim/channels/p<端口实例标识>`，数据 QoS 与上表一致。CommandSink 使用 `/rsim/commands/p<端口实例标识>/request` 和 `/reply`，可靠、volatile，不保留历史命令。命令重试保留原 deadline，并在 provider 去重和校验。

`Frame`、`Image`、`PointCloud` 位于独立的 `rsim.core.model`；公共包也重导出这些类型。历史查询、时间域和接收时刻语义不变。

## 共享内存与所有权

DDS 只传描述信息，完整数组通过 Linux tmpfs 的只读 mmap 共享。图像/点云首次进入共享存储可能复制；已有完整共享数组跨层转发复用 inode，新计算输出可使用 `allocate()`。这不是 ROS 驱动全链路零拷贝，也不是原生 DDS loan。

同机源发现和生命周期仍依赖同 Unix 用户的文件锁、Unix socket lease、独立监督进程与 WatchDog。跨 domain 的物理设备锁仍阻止重复启动同一驱动。保留这些机制，是为了处理历史淘汰、进程崩溃、多个消费者共享以及递归子进程退出；DDS 发现本身不能替代它们。

历史淘汰可以删除文件路径，但已经打开的映射继续有效。较慢读者可能跳过已淘汰的描述信息，`get()` 是最新帧接口，不保证无损录像。本机注册表与 mmap 路径不构成跨机器传输方案。

## 依赖与验证

原生后端要求 Cyclone DDS Python 绑定能在目标解释器中实际导入并创建端点。Python 3.14 的延迟注解与已发布 Cyclone DDS 11.0.1 的旧 IDL 实现不兼容；需要包含上游 Python 3.14 修复的版本，不能只看包是否已安装。已验证的上游源码基线为 [68b2acfd](https://github.com/eclipse-cyclonedds/cyclonedds-python/tree/68b2acfd5ac56bd0f095e51914a9f6c494675327)。从源码安装前，按 [官方安装说明](https://github.com/eclipse-cyclonedds/cyclonedds-python) 准备匹配的 Cyclone DDS C 库并设置 `CYCLONEDDS_HOME`。

一般回归不需要相机；双环境测试通过环境变量选择客户端解释器，不在测试中固定本机路径：

```bash
python -m pytest -q
RSIM_TEST_CLIENT_PYTHON=/path/to/client/python python -m pytest -q tests/test_cross_environment.py tests/test_shared_component.py
```

测试覆盖原生/ROS 双向互通、晚加入订阅、domain 隔离、连接失败、不同解释器下的数据模型、相同 inode、多层进程、历史查询和源租约。双环境客户端及其子进程通过 import guard 禁止加载 ROS；硬件 Notebook 另外验证真实图像。
