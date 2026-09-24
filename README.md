# RSim 传感器原型

基于 DDS 的 Python 传感器接口原型，ROS 2 作为可选驱动兼容层。应用环境无需 ROS，使用 `Sensor.get()` 读取数据，以 `Bundle`、`Map`、`ProcessSensor` 组合传感器；驱动可以运行在另一个 Python 环境中。

## 依赖与安装

Python 版本要求见 [pyproject.toml](pyproject.toml)。应用侧跨进程传输默认使用原生 Cyclone DDS，驱动侧的 ROS 适配器才需要与 ROS 安装匹配的解释器及设备驱动。共享内存与进程监督实现依赖 Linux。

应用环境从项目根目录安装：

```bash
python -m pip install -e '.[dds]'
```

上述命令不安装 ROS 2 或硬件驱动。驱动环境另行准备 ROS 工作空间；UVC 适配额外需要 OpenCV。测试需要 pytest，Notebook 需要 Jupyter kernel 和相应绘图依赖。Python 3.14 的 Cyclone DDS 兼容说明、双环境运行方式见 [DDS 文档](docs/dds.md)，绘图和资产约定见 [开发说明](docs/development.md)。

## 直接在 Notebook 中使用

选择具备项目依赖的 kernel。下面的示例使用合成数据源，不需要连接硬件，直接使用 Notebook 已有的事件循环：

```python
from rsim import Runtime
from rsim.synthetic import CounterArray

sensor = CounterArray(size=1024)
async with Runtime(sensor):
    frame = await sensor.get(timeout=5)
    values = frame.data
    same = await sensor.get(timestamp_ns=frame.stamp_ns, clock=frame.clock)
```

`async with` 退出、异常或取消都会释放本调用者的资源。共享硬件在最后一个使用者退出后停止。返回数组保有自己的只读内存映射，因此已经取出的帧在运行时退出后仍可读；不再使用时删除 Python 引用即可释放其页映射。

附带 [Notebook 实验示例](examples/prototype.ipynb)，包含嵌套进程和硬件组合。运行前应按使用环境调整 kernel、设备配置及绘图依赖。普通脚本可以在程序入口使用 `asyncio.run()`；Notebook 直接 `await` 即可。

[D435 Notebook](examples/d435.ipynb) 演示共享彩色与深度采集、等待新帧、时间戳回查、图像显示和数据保存。

[DDS 客户端 Notebook](examples/dds_client.ipynb) 在无 ROS 的应用环境中连接另一环境已启动的 D435，直接通过原生 DDS 和 mmap 取帧。

## 使用硬件

`rsim.drivers` 中的设备工厂负责启动硬件；`rsim` 中的同名工厂只连接已有源。例如，在 ROS 环境中运行 `python -m rsim.drivers d435 --backend ros2`，在另一应用环境中使用 `rsim.D435()`。两端复用同一 DDS domain，默认取 `ROS_DOMAIN_ID`，无需数据桥。源未启动时，应用端报错，不会自动加载 ROS。

`Camera` 提供 UVC 图像，`RobinW` 提供点云，`D435` 提供 RealSense 图像流。设备路径、雷达地址及相机序列号由调用方配置提供。单环境中希望由程序启动驱动时，例如配置好 `camera_device` 和 `lidar_ip` 后：

```python
from rsim import Bundle, Runtime
from rsim.drivers import Camera, RobinW

camera = Camera(device=camera_device)
lidar = RobinW(ip=lidar_ip)
rig = Bundle(camera=camera, lidar=lidar)

async with Runtime(rig):
    image = (await camera.get(timeout=20)).data.pixels
    points = (await lidar.get(timeout=20)).data.points
    samples = (await rig.get(timeout=20)).data
```

接口返回图像数组或带有 `x/y/z/intensity` 等字段的结构化点数组，不要求上层依赖 ROS message 类型。UVC 接口只提供其视频通道；深度流需要相应设备适配与驱动支持。

`D435(stream="color")` 与 `D435(stream="depth")` 复用同一个 RGB-D 采集进程；只有驱动环境需要 `realsense2_camera`。两端均接受 `serial`、`depth_profile`、`color_profile`、`history` 和 `transport`，驱动工厂另接受日志路径 `log_path`。两路 profile 格式为 `宽x高x帧率`，默认 `640x480x15`，应根据设备及 USB 连接能力选择。同一相机的调用者须使用一致的序列号写法、profile 和 history，冲突配置会被拒绝。返回的是未做像素配准、未承诺曝光同步的原始彩色与深度流；`get(after=...)` 不会将重复的组合快照当作新图像。

驱动端 `D435`、`RobinW` 还支持任意原生节点参数，例如 `D435(parameters={"publish_tf": False}, ros_args=["--log-level", "warn"])`。CLI 对应 `python -m rsim.drivers d435 --ros-args -p publish_tf:=false --log-level warn`，也支持 `--params-file` 和话题 remap。覆盖顺序、单流模式和两种调用方式见 [驱动参数说明](docs/drivers.md)。

ROS1 底盘可通过 `Chassis(SSHConfig(...), ...)` 跨机器读取 IMU、里程计、2D 雷达并发送速度；远端兼容 Python 2.7，主机无需 ROS1。可选 `ChassisROS2` 提供标准 ROS2 / DDS 双向话题。部署、生命周期和零速测试见 [ROS1 跨机器通信](docs/ros1.md)，可运行示例为 `python -m examples.chassis --help`。

## 像积木一样组合

```python
import numpy as np
from rsim import Map, ProcessSensor, Runtime, allocate
from rsim.synthetic import CounterArray

def squared(values):
    output = allocate(values.shape, values.dtype)
    np.square(values, out=output)
    return output

def graph():
    return Map(CounterArray(), squared, hz=10)

sensor = ProcessSensor(graph)
async with Runtime(sensor):
    values = (await sensor.get(timeout=30)).data
```

工厂在子进程内创建图，子节点默认共用该进程的 asyncio 事件循环。工厂内部可以再次创建 `ProcessSensor`。`cloudpickle` 支持 Notebook 中定义的工厂；工厂只用于启动时传递代码配置，帧数组不通过 pickle 传输。不要捕获正在运行的 Sensor 或设备句柄。

`RobinW()`、`Camera()`、`D435()` 是公共设备连接工厂。同一用户、同一 DDS domain、同一设备配置的多个调用者复用一个采集实例。源通过 `rsim.drivers` 显式启动；低层 `rsim.ros` 提供同进程 ROS 适配器，用于自定义驱动组合。物理设备排他锁继续避免不同 domain 或绕过公共工厂的协作进程重复占用设备。

通用共享计算/数据源使用 `SharedSensor(factory, key="唯一源标识", version="配置版本")`。相同 key 必须有相同 version 和 history；工厂变更时更新 version。不同调用者的函数不逐字比较，只运行第一个使用者提交的工厂。

应用侧省略 factory，使用 `SharedSensor(key="唯一源标识", version="配置版本")` 只连接。工厂不会跨应用解释器传输；数据模型 `Frame`、`Image`、`PointCloud` 位于独立的 `rsim.model`。`TransportConfig` 可选择 `cyclonedds` 或 `ros2` 后端，它们直接使用同一 DDS topic 和类型，配置细节见 [DDS 文档](docs/dds.md)。

普通 `children` 构成拥有关系，重复对象或相同 source key 只启动一次。`Reference(target)` 是不拥有目标的引用边，可以反向引用祖先；目标必须由同一个 Runtime 中其他位置拥有。真正的拥有关系环会在启动资源前拒绝。反馈数据循环必须自行提供初始样本/延迟，无法自动解决互相等待第一帧的问题。

## 时间、任务、服务

- `get()` 返回最新帧；`get(after=frame.sequence)` 等待本句柄的下一帧。
- 历史按帧数有界保存；时间戳查询必须指定 `clock`，默认精确匹配，也可设置 `tolerance_ns` 做最近邻查询；窗口外抛出 `HistoryMiss`。
- `received_ns` 是接收侧 Unix 时间。不同设备的源时间戳可能属于不同时间域；未经确认或转换，不能直接用它们做跨设备对齐。
- `Bundle` 提供各路最新样本并保留各自时间元数据，不做标定或时间同步融合。
- 自定义 Sensor 在 `open()` 中调用 `self.task(name, async_callback, hz=...)`。`Metronome` 基于单调时钟，超时时跳过错过的节拍，避免补偿式突发运行。
- `self.service(name, async_handler, hz=..., capacity=...)` 注册有限队列的限速服务；同进程调用者 `await sensor.call(name, ..., timeout=...)`。该本地服务接口目前不自动导出为跨进程 RPC；跨进程统一的数据读取接口是 `get()`。
- 每个传感器均有 WatchDog；任务异常会传播到父节点及等待中的 `get()`。协程 WatchDog 无法打断同一进程中的阻塞计算，应将这种图放进 ProcessSensor。

## 数据共享的精确边界

原型采用 **DDS 帧描述信息 + Linux tmpfs 不可变数组映射**，不宣称 rclpy 原生 loan 或整条驱动链路零拷贝。

1. ROS Image / PointCloud2 到 Python 消息的中间件路径可能复制。适配器在消息 buffer 上构造 NumPy 视图，不再复制像素/点记录。
2. 普通 NumPy 计算结果首次发布进入共享存储会复制一次；子进程历史从此保存共享数组，避免另留一份原始 payload。后续读者映射同一 inode；跨层转发完整的只读 memmap 通过硬链接复用同一物理数据。
3. 若计算需要输出即共享，使用 `out = rsim.allocate(shape, dtype)`，再调用支持 `out=` 的 NumPy 算法直接写入。子进程提交该输出不复制数组 payload；主进程以只读 mmap 获取同一 inode。发布代表移交所有权，生产者不可继续修改或保留可写别名。
4. 每帧新建不可变文件，历史淘汰只 unlink，不覆盖读者持有的数据。较慢读者尚未映射时遇到已淘汰帧会丢弃该帧；已映射帧不会失效。内存还取决于调用者长期持有的帧数量。
5. 描述信息使用真实 DDS String topic，原生 DDS 和可选 rclpy 后端共享 wire format；数组不进入 DDS 序列化。此通道是本机共享通道，不能直接用于远程机器，也不是 Fast DDS 的原生 Data Sharing/loan 实现。

## 测试与示例

测试覆盖任务调度、历史查询、共享数组、进程生命周期、源复用和 DDS 后端互通。在具备测试依赖及原生 DDS 的环境中，从项目根目录运行（缺少 ROS 时跳过 ROS 互通测试）：

```bash
mkdir -p assets
python -m pytest -q --junitxml=assets/tests.xml
```

硬件示例需另行检查设备连接、地址配置及驱动，再按需运行：

```bash
python -m examples.library_robin     # 低层库式雷达读取和历史查询
python -m examples.process_robin     # 两层进程 + 体素计算 + 主循环心跳
python -m examples.camera            # UVC 相机经 ROS 图像话题读取
python -m examples.stack             # 联合运行，相机 + 共享雷达 + 子进程计算
```

生成的采集结果、图像、日志和测试报告统一放在项目根目录的 `assets/`，不随 Git 提交。设计与限制见 [架构文档](docs/architecture.md)，绘图、资产和文档维护约定见 [开发说明](docs/development.md)。
