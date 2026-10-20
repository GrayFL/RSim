# MuJoCo 仿真后端

`rsim.drivers.MuJoCo` 将纹理室内场景、差速底盘、雷达和 RGBD 接入 RSim 的 Signal / CommandSink。控制应用继续使用 `rsim.devices.Chassis`，不需要增加 MuJoCo 分支。仿真源和真实驱动的差异留在部署组装中。

## 安装与运行

从包含本适配器的 RSim 源码安装：

```sh
python -m pip install -e ".[mujoco]"
python -m rsim.apps.mujoco --scene "data/scene.glb" --headless --mode demo --seconds 12
python -m rsim.apps.mujoco_motion --scene "data/scene.glb"
```

场景需自行准备，以米为单位，包含近水平地面。`--scene` 必填；GLB/GLTF 的朝上轴、选层和出生点参数见 `python -m rsim.apps.mujoco --help`。默认缓存为 `assets/mujoco/scenes`，demo 输出为 `assets/mujoco/latest`，运动示例为 `assets/mujoco/motion`，相对于工作目录。

GUI 模式省略 `--headless`，可用 W/S/A/D、空格、R/P/Q 控制运动、停止、重置、快照和退出。额外 `--preview` 显示传感器窗口，需安装带 GUI 的 OpenCV。RGBD 即使在 headless 下也需要 OpenGL。macOS 被动 viewer 需要 MuJoCo 的 `mjpython` 启动方式；当前实现的实测平台见本地开发记录。

默认机器人占地 0.6 m × 0.6 m；雷达在底盘地面中心上方 0.4 m，前向 RGBD 在上方 1 m。双轮执行器驱动真实接触动力学。地面/障碍碰撞为单楼层平面和体素表面近似，不支持楼梯/坡道或完整多层通行。`demo` 使用真值净空和雷达避障，不是 SLAM 或全局导航。

## 可移动物体与组合场景

`--scene` 和 `MuJoCo(scene=...)` 也接受 `rsim-scene-v1` JSON 或已有转换目录。JSON 相对路径以其自身目录为基准。顶层指定 `room: {"size": [6, 5, 2.6]}` 或 `base_scene: "../hm3d/scene.glb"`，两者选一。`base_scene` 当前要求自包含 GLB；`resolution`、`up_axis`、`floor_height`、`spawn: [x,y,yaw]` 可配置。转换目录需要 `scene.xml`、`navigation.npz`、`metadata.json`。

`objects` 列表每项包含 `name`、`source`、`position: [x,y,z]`、`mass`、`colliders`；可选 `up_axis: "y"/"z"`、`yaw`、`scale`、`target_height`、`max_faces`（每个 mesh 默认 30000）、`friction`（默认 `[0.65,0.005,0.0001]`）。对象坐标先转 Z-up，XY 外包框居中、底部 Z=0，再统一缩放。`position` 是该原点在世界中的米制位置。碰撞配置在缩放前的对象坐标内：

```json
{"type": "box", "pos": [0,0,0.5], "size": [0.3,0.3,0.5]}
```

`box.size` 是半尺寸；`sphere.size=[r]`；`cylinder.size=[r,half_height]`；`capsule.size=[r]` 配 `fromto=[x1,y1,z1,x2,y2,z2]`。非 capsule 可配弧度 `euler`。系统按各碰撞部件体积分配总质量并计算惯量，每个物体有独立 freejoint，重置恢复 XML 初态。显示网格 group 0、碰撞代理 group 3；新增物体 contype/conaffinity=2/3，同时接触环境、机器人和其他物体。

`objects.py` 使用 fast-simplification 减面，将源颜色采样到减面三角形后保存为贴图，保留外观而不把显示网格当整体凸碰撞体。`composition.py` 验证初始占地、地面支持和出生点。导航图记录初始占地，物体推动后不自动更新；实际接触与传感器跟随刚体。场景默认支持平地，扫描碰撞的高度范围仍是 0.08–1.16 m。

JSON 组合的转换参数来自文件，CLI `--up-axis/--floor-height/--resolution` 只作用于直接输入的 GLB/GLTF。文件内容、对象文件及基础扫描的路径/大小/修改时间参与缓存签名；并发构建仍需使用不同缓存目录。可复用的桌椅场景模板及构建/检查脚本位于 MuJoCo 部署工程的 `assets/templates/` 和 `examples/`。

## 与真机共用设备 API

下面 `task()` 只依赖应用层设备接口，可原样用于已启动的真机服务：

```python
import asyncio
from rsim import Runtime
from rsim.devices import Chassis
from rsim.drivers import MuJoCo, Chassis as Provider
from rsim.transport.descriptor import TransportConfig

async def task(chassis):
    await chassis.move(.2, timeout=10)
    await chassis.rotate(yaw_deg=30, timeout=10)
    await chassis.stop()

async def main():
    source = MuJoCo("data/scene.glb", motion_enabled=True)
    transport = TransportConfig("inprocess")
    provider = Provider(name="demo", pose=source.pose, velocity=source.velocity,
                        state=source.state, transport=transport, motion_enabled=True)
    chassis = Chassis("demo", transport=transport)
    async with Runtime(provider, chassis, source.scan, source.image, source.depth):
        await task(chassis)
        rgb = (await source.image.get()).data.pixels
        depth = (await source.depth.get()).data.pixels
        scan = (await source.scan.get()).data
        print(rgb.shape, depth.shape, scan["ranges"].shape)

if __name__ == "__main__":
    asyncio.run(main())
```

真机组装时，驱动环境提供已有的底盘服务；应用构造匹配服务名称和传输配置的 `Chassis`，在 `Runtime(chassis)` 中调用同一 `task()`。部署方式见 [运动控制](motion.md) 与 [远程控制](remote-control.md)。本示例不自动启动或连接硬件。

`inprocess` 是显式选择的本地描述信息传输，按 domain 和 asyncio 事件循环隔离，保留有界队列与 polling 语义。它不跨进程、不连接另一脚本、不与 DDS/ROS2 通信。本地 inprocess 仿真无需 DDS 或 ROS。Windows 也支持原有 ProcessSensor、ProcessPlacement、SharedProvider 和 SharedComponent 的跨进程路径，选择 Cyclone DDS；详细部署及跨解释器示例见 [Windows 部署](windows.md)。

仿真源和控制器均默认禁止非零运动，需分别 `motion_enabled=True`。短期速度写入为 `await chassis.drive(VelocityCommand(.2, 0), ttl=.25)`；持续运动需刷新，类型从 `rsim.core` 导入。控制器保留独占、取消、反馈过期、超时及退出停止；源在每次物理推进前再校验命令截止时间。

## 分进程与跨解释器运行

先在提供者环境启动物理 worker 与控制服务：

```sh
python -m rsim.apps.mujoco_service --scene "data/scene.glb" --name mujoco --domain 0 --enable-motion
```

另一个终端或兼容 Python 环境中运行相同的设备运动流程：

```sh
python -m rsim.apps.mujoco_motion --connect mujoco --domain 0 --output assets/mujoco/client
```

提供者通过 SharedProvider 把物理隔离到独立进程，控制服务通过 DDS 读位姿、写速度；客户端只导入设备/共享端口，不加载 MuJoCo。RGBD、雷达按订阅动态采集，图像通过只读 mmap 传递。`--enable-motion` 省略时只允许零速。服务可用 Ctrl+C 关闭；自动验证可加 `--seconds N` 与 `--ready-file PATH`。name/domain 必须一致，图像共享要求同主机同用户。

客户端保存 RGB、深度、内外参、各流时间戳和 motion_summary，后者包含客户端/物理 PID 及共享映射标记。RGB 与深度按时间戳匹配；雷达独立采样并记录其自己的时间戳。

## 输出与时钟

所有输出为 RSim `Frame`。数组发布后只读，字典/Pose 同样不应修改；处理时需要写入请先复制。

| 端口 | payload / 坐标与频率 |
|---|---|
| `pose`、`truth` | graphmap Pose，world ← base_footprint 真值，默认 50 Hz |
| `odom` | ROS 风格普通字典，实际轮转角积分的 odom ← base_footprint，零起点，50 Hz |
| `imu` | 普通字典，刚体姿态、MuJoCo gyro/accelerometer、协方差，50 Hz |
| `state` | backend、motion_enabled、simulation_time、physics_time、contacts、wheel_targets，50 Hz |
| `scan` | 普通字典，ranges/角度/量程/header 等，10 Hz |
| `lidar_points` | `PointCloud`，结构化 x/y/z，雷达局部坐标，10 Hz |
| `image` | `Image(rgb8)`，uint8 RGB，5 Hz |
| `depth` | `Image(32FC1)`，float32 米制光学 Z，5 Hz |
| `camera_info` | K、分辨率、深度约定、世界外参，5 Hz |
| `points` | `PointCloud`，结构化 x/y/z/r/g/b，光学坐标、像素 stride=4，5 Hz |
| `velocity` | 写端口 `CommandSink[VelocityCommand]` |

机器人为 X 前/Y 左/Z 上；光学相机为 X 右/Y 下/Z 前。雷达默认 360 束、`[-π,π)`、0.05–12 m，无回波 `+inf`。深度默认 640×480、0.05–10 m、无效 `NaN`；垂直视场 60°。Frame metadata 的 `world_from_base`、采样时的 `world_from_lidar` 或 `world_from_camera_optical` 可用于转换。

`pose` 是真值，`odom` 是轮式估计；不要混用。要验证原有轮速/IMU 融合，使用 `ChassisController(source, motion_enabled=True)`；它自动读取 odom/imu。设备服务也可直接组装独立的估计器，再把其输出传给 Provider：

```python
from rsim.components.odometry import PlanarOdometry
estimator = PlanarOdometry(source.odom, source.imu)
provider = Provider(name="demo", pose=estimator.pose, velocity=source.velocity,
                    state=source.state, transport=transport, motion_enabled=True)
```

同一速度写端口只交给一个控制器管理，避免在此 Provider 之外再激活占有 source.velocity 的控制器。

默认物理步长 0.002 s，每 50 Hz tick 推进 0.02 s。参数 `hz` 的倒数必须为物理步长整数倍；`speedup` 改变调度频率而不改变步长，实际速度受计算和渲染能力限制。TTL 始终按主机单调时钟判断。

每个源使用独立 `mujoco:<instance>` 时钟。`await source.reset()` 将物理时间和轮里程计归零，但增加时钟偏移，使流时间戳保持单调；不要直接与墙钟或另一个源比较。重置前先停止控制，估计器有自身内部状态，重复定位实验应重新建立 Runtime/估计器。

Runtime 按请求端口决定是否采集雷达/渲染 RGBD；`Runtime(source)` 请求所有端口。`image_history=8`，其他流默认 `history=128`。仅使用 odom/imu/state 时无需创建 Renderer。

## 仿真专用入口与维护

`MuJoCo(scene, cache_root=..., up_axis=..., floor_height=..., pitch=.08, spawn=..., sensors=...)` 接收源文件；已预处理场景使用 `MuJoCo(scene_dir=...)`，两者只能选一个。配置类型为 `rsim.adapters.mujoco.RobotConfig/SensorConfig`。

低层 `Simulation`、`prepare_scene` 位于 `rsim.adapters.mujoco`，兼容原来的同步 API。低层调用者自行 step、采样和 close；它不提供 RSim 命令 TTL。正在运行的源通过 `await source.inspect(callback)` 串行化快照/渲染调试，不应从外部线程读取修改 MjData 或自己推进物理。

`source.py` 管生命周期与端口，`scene.py` 管预处理，`model.py` 管 MJCF，`simulation.py` 管物理和传感器，`apps/mujoco*.py` 管演示。全部物理和离屏 GL 在同一工作线程上创建、使用和释放。可选依赖不在公共驱动/设备导入时加载。

场景缓存版本、碰撞高度范围、中文路径 VFS 加载和传感器坐标必须随实现维护。缓存命中尚不检查资产完整性，GLTF 外部资源变化或缓存损坏后需 `--rebuild`；并发转换使用独立缓存目录。RSim 中不保存机器专用数据路径。

相关测试使用生成房间而非 HM3D，需要 OpenGL：

```sh
python -m pytest tests/test_mujoco_physics.py tests/test_mujoco_scene.py tests/test_mujoco_source.py tests/test_inprocess_transport.py tests/test_architecture.py -q
```

覆盖物理/坐标、Unicode 场景转换、设备 move/rotate/取消/独占、停止/TTL、传感器历史、重置、轮速/IMU 融合和导入边界。实际数据演示输出和本机结果记录在 `assets/` 及本地 `PROJECT.md`。
