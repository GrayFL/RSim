# MuJoCo 协议适配器

RSim 只对接外部仿真后端，不拥有机器人、场景、碰撞或相机实现。完整仿真平台在独立的 MujocoSim 项目；原 `rsim.apps.mujoco*` 已迁到其 `mujocosim.apps`，原物理与场景模块不再作为 RSim API。

## 最小接入示例

```sh
python -m pip install -e ".[mujoco]"
python -m examples.mujoco_adapter
```

[单关节教程](../examples/mujoco_adapter.py) 实现一个小型 MuJoCo 后端，用 torque 命令和 angle 信号说明协议。它无需外部数据、场景缓存、graphmap 或 MujocoSim。MuJoCo extra 只安装该引擎；adapter 本身没有引擎或几何库导入依赖。

```python
from rsim.drivers import MuJoCo
from rsim.adapters.mujoco import CommandSpec

source = MuJoCo(
    MyBackend,                         # 无参数工厂，在所有者线程中构造
    outputs={"angle": 8},              # 名称 -> 历史容量
    commands={"torque": CommandSpec(0., lambda x: x == 0.)},
    hz=50, speedup=1, motion_enabled=True,
)
```

工厂及配置应可序列化，创建 Component 时不打开物理/GL 资源。不要在 RSim 内导入上层项目作为默认实现。

## 后端契约

`SimulationBackend` 是结构性 Protocol，不要求继承：

| 方法 | 责任 |
|---|---|
| `open()` | 构造模型、物理状态与所需资源 |
| `step(duration, commands)` | 按后端语义应用命令并推进物理；自行验证步长 |
| `sample(demand, force=False)` | 返回 Observation；自行决定传感器频率与按需采样 |
| `reset(*args, **kwargs)` | 恢复项目定义的初态，参数由项目决定 |
| `stop()` | 使物理控制进入安全状态 |
| `close()` | 释放资源，包括部分初始化失败的情况 |

以上方法和工厂都在同一个工作线程运行。MuJoCo renderer 的创建/使用/销毁应留在该线程。adapter 把事件循环上的不可变 DemandSnapshot 传入后端，不读取 RSim 私有需求 hook。

Observation 包含 `time`（物理秒）、`values`（输出名到数据）、公共 metadata 和按端口覆盖的 port_metadata。数据是拥有独立存储的发布快照，后端不能在发布后改写数组；坐标、单位、Pose、图像编码等语义由项目明确并验证。RSim 不假设存在 image、pose、velocity 或轮关节。

CommandSpec 明确 fallback、is_safe 与可选 validate。motion_enabled=False 只接受安全值。每个写端口有独立租约；工作线程在 step 前再次检查主机 monotonic 截止时间，过期值替换为 fallback。仿真 speedup 不延长 TTL。底层后端仍需把 fallback 变为正确执行器状态。

reset 清空命令、更新 episode，并对发布时钟增加偏移，保证 Frame.stamp_ns 不回退。Observation 内 payload 的时间字段属于项目数据，adapter 不修改。`inspect(callback)` 在工作线程串行访问后端，仅用于本地诊断，不替代 Signal/CommandSink，也不是跨进程 RPC。

## MujocoSim 的具体组装

差速构型、HM3D、桌椅、雷达/RGBD、轮编码器和仿真应用已迁到 MujocoSim。该项目的 `integrations/rsim.py` 提供当前 demo 的 create_source；其内部注入 DifferentialDriveBackend 并声明端口。`devices.Chassis` 等通用设备 API 保持原样。

原 `MuJoCo(scene=...)` / `MuJoCo(scene_dir=...)` 调用应迁到 MujocoSim 的 create_source，或者自行实现上述后端。RSim 不通过兼容转发反向依赖 MujocoSim。项目级安装和运行请以 MujocoSim README 为准。

## 验证范围

```sh
python -m pytest tests/test_mujoco_adapter.py -q
```

只覆盖本 adapter 的注入、需求转交、端口发布、线程归属、命令期限、复位时钟、停止和失败释放。测试无需 MuJoCo，实际引擎桥接另运行单关节教程。具体机器人物理与场景图像测试归 MujocoSim；不由此入口运行 RSim 其他套件。
