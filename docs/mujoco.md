# MujocoSim 提供端

MuJoCo World、线程所有权、传感器快照、reset 和命令期限检查由独立 MujocoSim 的 `mujocosim.rsim` 维护。RSim 只提供通用 Component、Shared Ports、RPC 与传输，不再维护 MuJoCo 专用运行实现。

```python
from mujocosim.rsim.drivers import MujocoChassisFactory
from rsim import Runtime
from rsim.devices import ChassisPortsDevice

factory = MujocoChassisFactory(scene="room.scene.json", motion_enabled=False)
source = factory.build()
device = ChassisPortsDevice.from_component(source)
async with Runtime(device.pose):
    frame = await device.pose.get(timeout=30)
```

工厂构造和 build 不启动引擎，Runtime open 才创建一个 World；所有 step/sample/reset/inspect 在同一 owner 串行执行。底盘、IMU、相机和雷达共用这个 World，读取 Signal 不推进物理。引擎与渲染依赖只安装在提供端。

跨进程提供端显式调用 `factory.shared(key=...)` 并持有 Runtime；消费端仅使用 `ChassisPortsDevice(key=..., clock=...)`，可以不安装 MujocoSim 和 MuJoCo。如使用 RPC，则通过 `factory.service(source=shared)` 包装同一源，再用 `rsim.devices.Chassis` 连接；仅启动 Shared Ports 不等于启动 RPC。

接口族、状态/速度单位和导航接入见 [公共底盘端口](chassis-ports.md)。仿真专用 reset 使用 `mujocosim.rsim.devices.SimulationDevice` 的请求和确认；通用 Device 不增加仿真管理方法。

Factory 默认运动关闭。实际应用命令前检查原 host monotonic deadline，仿真暂停/加速不延长 TTL。reset 清除旧命令，原生时间归零并增加 episode；端口 clock 启动前固定。旧连续偏移模式只能显式配置。

[单关节教程](../examples/mujoco_adapter.py) 通过 MujocoSim 的 Source/CommandSpec/Observation 展示自定义后端；运行 `python -m examples.mujoco_adapter` 需要该项目的 engine 与 rsim 依赖。旧 RSim 入口只保留过渡委托，详见 [迁移说明](migration-v4.md)。具体场景、实际引擎、离屏图像与设备测试在 MujocoSim 仓库执行。
