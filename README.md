# RSim：异步组件与时间序列接口

RSim 是机器人底层适配与组合计算原型。`Component` 管生命周期和计算，`Signal[T]` 管数据、历史和时间戳；`CommandSink[T]` 管写入。应用不需要 ROS message 类型，ROS1 / ROS2 / DDS / mmap 留在适配与传输层。

```python
from rsim import Runtime, Map, ProcessPlacement
from rsim.components.synthetic import CounterArray

source = CounterArray(size=1024)
squared = Map(source.output, lambda values: values ** 2)

# Notebook 直接 await；脚本在入口使用 asyncio.run()。
async with Runtime(squared.output, placement={squared: ProcessPlacement("compute")}):
    frame = await squared.output.get(timeout=15)
    same = await squared.output.get(timestamp_ns=frame.stamp_ns, clock=frame.clock)
    assert same is frame
```

去掉 `placement` 后，同一张图运行在当前进程。多个消费者读取同一个 Signal，不会重复启动 producer。同进程保留 Frame / payload 引用；跨进程边界才使用共享数组与 DDS 描述信息。

## 安装

Python 版本要求见 [pyproject.toml](pyproject.toml)。本地应用安装：

```bash
python -m pip install -e '.[dds]'
```

跨进程实现需要 Linux 和可用的 Cyclone DDS；同进程组合不加载 DDS。只有 ROS 适配器需要匹配 ROS 的解释器及设备驱动，UVC 另需 OpenCV。不同 Python 环境之间通过共享 provider 连接，见 [DDS 文档](docs/dds.md)。测试额外需要 pytest；Notebook 需要 Jupyter 和绘图依赖。

## 定义组件与多输出

```python
from rsim import Component

class Analysis(Component):
    def __init__(self, image):
        super().__init__(inputs=(image,))
        self.mean = self.signal("mean", history=32)
        self.shape = self.signal("shape", history=8)
        self.previous = 0

    async def open(self):
        self.task("analyse", self.analyse, hz=15)

    async def analyse(self):
        frame = await self.inputs[0].get(after=self.previous)
        self.previous = frame.sequence
        await self.mean.publish(float(frame.data.pixels.mean()),
                                stamp_ns=frame.stamp_ns, clock=frame.clock)
        await self.shape.publish(frame.data.pixels.shape,
                                 stamp_ns=frame.stamp_ns, clock=frame.clock)
```

`Runtime(analysis.mean)` 自动找到 producer、输入及运行依赖。多输出组件没有含义模糊的 `get()`，使用 `analysis.mean.get()` / `analysis.shape.get()`。`Map.output`、`Bundle.output` 和相机等明确指定 primary output 的组件仍支持 `component.get()` 快捷调用。旧 `Sensor` 是单输出兼容基类，新组件优先使用 `Component` 或 `PrimaryComponent`。

- `dependencies` 是资源所有权 DAG；`inputs` 是 Signal 数据边，允许带初值的反馈环。
- `get()` 取最新值；`get(after=frame.sequence)` 等待本 Signal 的新值。sequence 是本地观察游标，不是跨进程采样 ID。
- 时间戳查询必须指定 `clock`；历史有界，找不到容差内样本时抛出 `HistoryMiss`。
- `Bundle` 取各路 latest snapshot；`Synchronizer` 围绕参考时间 join，跨时钟必须配置 `ClockTransform`，插值需显式提供函数。
- task / service 由 Metronome 控速，每个 Component 有 WatchDog；同一循环的阻塞计算应放入独立 ProcessPlacement。

完整语义、迁移与限制见 [架构文档](docs/architecture.md)。源码职责、依赖方向、新增设备与内部导入迁移见 [包结构与扩展](docs/modules.md)。

## 硬件入口

驱动环境的 `rsim.drivers` 启动共享硬件源；应用侧 `rsim` 中的同名工厂只连接已有源。设备配置由调用方提供。

```python
from rsim import Bundle, Runtime
from rsim.drivers import D435, RobinW

camera = D435(stream="color", serial=camera_serial)
lidar = RobinW(ip=lidar_ip)
snapshot = Bundle(image=camera.image, points=lidar.points)
async with Runtime(snapshot.output):
    rgb = (await camera.image.get(timeout=30)).data.pixels
    cloud = (await lidar.points.get(timeout=30)).data.points
```

`Camera.image` 提供 UVC 图像，`D435.image` 提供选定的彩色或深度流，`RobinW.points` 提供结构化点云。D435 的彩色和深度视图复用一个采集进程，默认 profile 为 `640x480x15`；调用者须使用一致的设备标识、profile 和 history。

驱动参数没有白名单：`D435(parameters={"publish_tf": False}, ros_args=["--log-level", "warn"])`；CLI 对应 `python -m rsim.drivers d435 --ros-args -p publish_tf:=false --log-level warn`。参数文件、remap、覆盖优先级见 [驱动参数说明](docs/drivers.md)。

`Chassis` 将 ROS1 设备映射为 `imu / odom / scan / state` Signals 和 `velocity` CommandSink。远端兼容 Python 2.7，主机无需 ROS1；SSH 网络段传输消息内容。部署和零速测试见 [ROS1 文档](docs/ros1.md)。

HiPNUC IMU 支持 Python 串口直读与 ROS2 节点两种模式，均通过 `.imu.get()` 返回数据，见 [IMU 接入](docs/imu.md)。`load_rig()` 可读取 YAML 初始化设备、复用嵌套组合中的同一个传感器，并用 graphmap Pose 表示安装外参，见 [配置与组装](docs/configuration.md)。

`PlanarOdometry` 融合轮式 odom 与 IMU 角速度，输出 `graphmap.pose.Pose`；`ChassisController` 提供 `await move(distance_m)`、`await rotate(yaw_deg=... / yaw_rad=...)` 和 `await stop()`。运动默认关闭，详见 [位姿融合与底盘控制](docs/motion.md)。

电机板直连本机时，`rsim.drivers.STM32` 提供原生 ROS2 串口驱动和相同的 RSim 端口，可与外置 IMU 组装为独立的本机控制链路。见 [本机底盘与初步 IMU 校正](docs/local-chassis.md) 和 [Notebook](examples/control/local_chassis.ipynb)。

`load_mapper()` 组装 Super-LIO 与 RTAB-Map：3D 雷达和底盘 IMU 构建激光几何，D435 RGB 投影上色，独立 EKF 可融合轮速、底盘陀螺和二维扫描里程计，RTAB-Map 通过视觉/激光配准及回环优化关键帧位姿。`await mapper.pose.get()` 读取 graphmap 位姿，`.rgb_map.get()` 读取彩色点云，`.map.get()` 提供带稳定来源编号的地图。`GraphMap` 将它转换为 InfoPoints / IndexDB，并在回环后重新关联体素和来源特征。接口、配置与同步边界见 [多传感器建图](docs/mapping.md)。

## 控制与仲裁

```python
from rsim import Connect, CommandMux, CommandInput, VelocityCommand

mux = CommandMux(
    navigation=CommandInput(navigation.velocity_command, priority=10, timeout=.25),
    manual=CommandInput(joystick.output, priority=50, timeout=.15),
    fallback=VelocityCommand(),
)
drive = Connect(mux.output, chassis.velocity)
async with Runtime(drive):
    command = await mux.output.get(timeout=5)  # 可观察的控制输出
```

直接写入使用 `await chassis.velocity.set(VelocityCommand())`。一个 sink 的多个写入者必须通过 CommandMux；Connect 占有的 sink 不接受旁路直接写入。支持优先级、手动选择、超时 fallback。命令携带 controller id、epoch、sequence 和 deadline，provider 校验过期、重放与独占关系。ROS1 底盘兼容端在硬件侧独立执行截止时间和零速保护。

## 进程、共享与退出

`ProcessPlacement("name")` 将对应组件放入命名 worker；同名组件共用事件循环，多输出无需启动多个 worker。未显式指定位置的 ownership 依赖跟随拥有者，Signal 输入保持自己的位置。跨位置只导出声明的 Signal / CommandSink，普通对象属性和本地 service 不变成远程调用。

`SharedProvider(component, key=...)` 显式启动多端口提供者；`SharedComponent(key=..., ports=...)` 只连接。`Runtime(mapper.pose)` 只请求 pose，Component 根展开全部公开输出，dependencies 和 placement 不增加输出需求；未请求的远端端口抛 `PortNotBound`。源租约、端口订阅与命令控制权分开管理，见 [多端口共享](docs/shared-components.md)。

`SharedSensor(factory, key=..., version=...)` 保留单输出兼容适配器和租约；省略 factory 时仅连接。相同 key 的创建者须匹配配置，其他客户端离开不会停止仍有租约的硬件。旧 `ProcessSensor(factory)` 保留为单输出兼容入口，工厂内仍可嵌套进程。

同机通道使用 DDS 描述信息 + tmpfs 只读 mmap，普通数组首次跨边界需要一次 materialization，后续消费者共享 inode。完整的只读 memmap 转发可复用 inode；它不是 ROS 驱动全链路零拷贝，也不是中间件原生 loan。发布后不得修改 payload；已读取的映射在历史淘汰或 Runtime 退出后仍可读。

`async with Runtime(...)` 管理正常关闭、取消和异常回收；独立 lease supervisor 在父进程被杀死时停止 worker，回收部署存储。CPU 阻塞与控制 provider 应隔离，协程 WatchDog 无法抢占同一事件循环内的阻塞代码。

## 示例与验证

- [RGB 建图 Notebook](examples/mapping/mapping.ipynb)：激光几何与 RGB 上色、位姿、来源特征和 graphmap 导出。
- [IMU 与配置组装 Notebook](examples/imu/imu_rig.ipynb)：Python/ROS2 两种采集方式、固定外参、共享的嵌套组合。
- [底盘控制 Notebook](examples/control/chassis_motion.ipynb)：EKF 位姿、异步前进/后退/旋转；默认内存模拟，实机部分只发零速。
- [Component / Signal Notebook](examples/components/components.ipynb)：多输出、fan-out、进程部署、模拟控制与手动覆盖，无需硬件。
- [D435 Notebook](examples/camera/d435.ipynb)：彩色与深度、历史回查、图像和采集资产。
- [无 ROS 客户端 Notebook](examples/camera/dds_client.ipynb)：连接另一解释器已启动的相机。
- [原型实验 Notebook](examples/prototype/prototype.ipynb)：保留已有嵌套进程和硬件实验。

```bash
python -m examples.components.components             # 子进程计算，内存模拟执行器
python -m examples.components.components --local     # 同一张图在当前循环运行
python -m examples.ros1.chassis --help         # 远端配置、ROS2 镜像、零速测试
mkdir -p assets
python -m pytest -q --junitxml=assets/tests.xml
```

绘图使用 `scipykit.mtp_initializer`，生成图像、报告和日志放在根目录 `assets/`。本地环境及开发记录仅写入不提交 Git 的 `PROJECT.md`；通用约定见 [开发说明](docs/development.md)。

键盘及跨环境指令控制见 [解耦控制接口](docs/control.md)，跨机器 DDS 与 WSL 配置见 [远程控制](docs/remote-control.md)，示例按主题整理在 [examples 索引](examples/README.md)。服务由 `rsim.drivers.Chassis` 提供，无 ROS 客户端使用 `rsim.devices.Chassis`。
