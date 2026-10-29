# 通用底盘共享端口

`rsim.devices.ChassisPortsDevice` 连接已运行的 Shared Ports 提供端。现有 `rsim.devices.Chassis` 仍是 RPC 客户端，需要对应的控制服务。两种连接不能互相替代。

```python
from rsim import Runtime
from rsim.devices import ChassisPortsDevice

device = ChassisPortsDevice(key="robot", clock="simulation", max_ttl=.5)
async with Runtime(device.pose, device.state, device.velocity):
    pose = await device.pose.get(timeout=10)
    # 非零运动需提供端显式开启，并由应用持续提供新鲜命令。
```

本地源可用 `ChassisPortsDevice.from_component(source)` 得到 `ChassisPorts`；构造视图不会启动任何资源。Runtime 的 Component 根只展开输出，写端口需显式作为根或由 Connect 请求。

接口族为 `chassis-ports-v1`。公共最小能力是：

| 端口 | 语义 |
|---|---|
| pose | graphmap Pose，米，右手世界系到车体系的位姿；具体 frame 名由应用配置 |
| state | `linear_x` m/s、`angular_z` rad/s、`acceleration` m/s²、`yaw_acceleration` rad/s²，以及 `frame/episode/step_index/pose_kind` |
| velocity | `VelocityCommand(linear_x, angular_z)`；独占控制和主机单调 deadline；默认最大 TTL .5 s，安全回退为零速度 |

pose/state 必须具有同一源 clock/stamp；metadata 包含 episode、step_index、pose_kind。ground truth 必须标明，不代表定位或 SLAM 已完成。StateCodec 和差速原生命令转换由 pyrobo 的导航边界负责，不能把任意调试字典直接传给算法。

每个具体 Factory 本地拥有完整 `port_specs`，包括可选传感器。通用客户端只约束公共子集，可以连接具有额外端口的同族提供端。Provider 的完整集合、schema、方向、clock、history、TTL、fallback 在构造后及 Runtime open 后校验；错误会包含端口、期望和实际值。消费者可选择自己的 history 容量，不能改变 Provider 的物理行为。

`schema='rsim.value.v1'` 表示现有编码器，不是设备语义登记。fallback 在提供端可信配方中校验，未修改线 manifest；客户端仍需通过对应设备的安全回归证明实际停止行为。

工厂是配置和配方，`build()` 返回未运行 Component，`shared()` 返回启动描述，`device()` 只连接。轻量 `ComponentFactory` Protocol 与 `validate_component_ports` 从 `rsim.runtime` 导出，无需继承公共基类或登记具体设备。云端任意输入不属于可信配方接口。

新的仿真/回放源可在 metadata 中给出 episode（及可选 session）。episode/session 改变时，Signal 发布和共享导入边界隔离旧历史；`await signal.clear_history()` 用于受控 reset/seek 边界。查询时间戳只能找到当前阶段的帧，读者 sequence 和 sample cursor 不回退，已有 Frame 引用继续可读。`received_ns` 仍表示 wall time；主机安全 deadline 不随环境时间回退或暂停。
