# 多端口共享与精确订阅

同机 Linux 的多个 Python 环境可以共享同一组件。生产者环境显式启动 `SharedProvider`，客户端用 `SharedComponent` 声明已知端口；两侧通过命名 Signal / CommandSink 通信。客户端不接收工厂或驱动代码。实现限于同一用户、同一主机及共享注册表和文件存储命名空间，不能将 mmap 描述信息直接用于跨机器图像或点云传输。

## 精确需求

| 写法 | 激活与绑定 |
| --- | --- |
| `Runtime(mapper.pose)` | 启动所需组件，只请求 pose |
| `Runtime(mapper.pose, mapper.map)` | 请求两个输出 |
| `Runtime(mapper)` | 请求该根组件的全部公开输出，不请求写端口 |
| `inputs=(mapper.pose,)` | 在消费者位置请求 pose |
| `Component(resource)` | 仅激活资源，不请求其输出 |
| `placement={mapper: ProcessPlacement("mapping")}` | 指定位置，不增加端口需求 |
| `Runtime(device.velocity)` / `Connect(source, device.velocity)` | 显式请求命令端口 |

根、inputs、command targets 在 Runtime 启动时确定。读取未请求的远端端口抛出 `PortNotBound`；增加根或输入声明，再重新打开 Runtime。同进程已启动组件的其他本地输出仍可读取。裁剪的是传输，不承诺裁剪生产者内部计算。

用 `parent.expose("pose", child.pose)` 暴露子组件端口。别名自动建立生命周期依赖，沿用实际生产者的 Frame、历史和 SampleId，不添加轮询任务；关闭或失败的 parent 也会使这个公开端口不可用。

## 提供者和客户端

提供者环境中的 `mapping` 是已经组装好的多输出 Component：

```python
from rsim import SharedProvider, ProcessPlacement, serve_shared

provider = SharedProvider(
    mapping, key="example:mapping", interface_version="example-ports-v1",
    provider_version=config_fingerprint,
    placement={mapping: ProcessPlacement("mapping")},
)
await serve_shared(provider)
```

`serve_shared()` 只保持源组件的租约，不订阅其输出。普通脚本在 `asyncio.run()` 中运行；Notebook 可直接 await，或用 `asyncio.create_task()` 保留任务，退出时 cancel 并 await。需要延迟组装时，可以在提供者环境传 `factory=build_mapping, ports=known_ports`；工厂只能在该环境启动的受监督进程中执行。

客户端的声明与提供者保持相同的接口版本、schema、时钟和 sink TTL：

```python
from rsim import SharedComponent, PortSpec, Runtime

mapper = SharedComponent(
    key="example:mapping", interface_version="example-ports-v1",
    ports={
        "pose": PortSpec(clock="robot", history_capacity=32),
        "map": PortSpec(clock="robot", history_capacity=2),
    },
)
async with Runtime(mapper.pose):
    frame = await mapper.pose.get(timeout=10)
```

具体建图接口使用 `rsim.devices.Mapper()`，无需手工声明端口。`rsim.drivers.Mapper()` 和 `load_mapper(..., providers=True)` 返回明确的 launcher；`load_mapper(..., providers=False)` 只连接。运行示例见 [共享端口 Notebook](../examples/runtime/shared_ports.ipynb)。

已知端口在构造时就存在。动态接口先 `await describe_shared(key)` 获取 manifest，再构造代理；没有运行的 provider 会明确报错，不自动启动。源身份 `component_key`、运行实例 `instance_id`、接口 `interface_version/schema` 与配置竞争检查 `provider_version` 分开。只读客户端无需知道驱动配置摘要；同 key 的第二个 launcher 必须匹配配置。

## 协议与数据路径

注册表使用同用户 Unix socket 和生命周期文件锁，协议版本为 2，只处理 `describe / subscribe / unsubscribe`。请求带 ID，同一连接中的订阅重试不会重复增加引用。连接断开释放该会话所有订阅；重连必须显式重新打开 Runtime，不沿用已结束实例的样本或控制权。

Manifest 包含 key、instance、接口/配置版本、host boot ID、DDS domain，以及各端口的方向、schema、clock、history_capacity、max_ttl。声明地图元数据不会创建地图 exporter 或共享数组。

订阅返回 `subscription_id`、`binding_id`、`channel_generation`、topic、storage_descriptor 与 effective_history。一个端口的多个客户端复用 exporter；最后一次退订释放该端点，再次订阅获得新通道代次。生产者历史、DDS 缓存与客户端历史独立：新订阅只保证获得仍保留的 latest，或等待下一帧，不回放生产者完整历史。客户端 history_capacity 只限制自己的缓冲。

`runtime.port_binding` 的 exporter/importer、共享存储和命令包装供 placement 与共享服务共用。Provider 内的 ProcessPlacement 接收动态订阅，在实际生产者进程创建端点；数组不绕经 launcher。相同 Runtime 内兼容的代理声明合并，多个视图不会重复启动生产者。 如果共享提供者再次包装 SharedComponent，显式订阅和退订会沿代理逐层传递；纯转发保留原 SampleId，并通过完整 mmap 的硬链接复用数组。

普通 NumPy 数组第一次跨边界会复制到共享文件。不可变同一对象的扇出复用已有分配；仍有原路径的完整只读 mmap 可以硬链接转发，切片或已 unlink 的映射可能需要复制。这不是按内容全局去重，也不表示原生 ROS 驱动端到端无复制。动态导出旧 Frame 不修改原 Frame、不向源重复 publish。prepare hook 按绑定持有，关闭一个绑定不会清掉其他绑定的 hook。

源租约、端口引用和控制权分别管理。Launcher 退出后，只要其他客户端还持有源租约，provider 就继续运行。独立 supervisor 监视连接，即使生产者事件循环阻塞，也会在最后租约退出后终止进程并回收存储。ProcessPlacement 则仍严格跟随所属 Runtime。物理设备锁是额外保护，不会自动合并两个不同算法图。

## 身份、时钟和控制

`Frame.sample_id` 是可选的 `SampleId(producer_instance_id, canonical_port_id, publication_sequence)`，新传输自动携带。别名、导入和纯转发保留身份；实际计算产生新身份。重新打开 producer 会创建新实例，旧接口手工构造的 Frame 仍兼容。`Frame.sequence` 保留为当前读端的本地游标，不能跨读端比较。退订再订阅同一保留样本时，通道代次改变，SampleId 不变。

`stamp_ns / clock / received_ns` 原样通过端口传输。跨时钟仍需显式 ClockTransform；同机 monotonic deadline 只能在相同 host boot ID 的范围内比较。SampleId 不是地图观测来源编号，也不替代采样时间。

共享 sink 仍由最终 provider 的 CommandGuard 校验独占会话、epoch、sequence、deadline 和最大 TTL。通道额外绑定 provider instance 与当前订阅 token；断连撤销 token，并对该会话拥有的命令执行 fallback。发现、重试和转发均不延长 deadline。Component 根不会自动取得命令绑定。实际控制器仍须与阻塞计算隔离；通用异步 deadman 不具备抢占同进程阻塞代码的能力。

单个端口的 codec/export 错误通过该端口的错误状态报告；provider 运行故障会结束实例并通知全部客户端。既有底盘小消息 RPC 和远程控制协议保持独立。

## 兼容入口

`SharedSensor` / `ProcessSensor` 保留原有单 primary 输出的 legacy 适配器及工厂启动路径；它们不作为 v2 多端口协议的父类。已有相机、雷达单输出客户端可继续使用，新的多输出算法优先使用 SharedProvider。旧 mapping 单 snapshot 协议需要客户端和 provider 一起迁移；`MappingView` 现在只为命名输出建立别名，不再拆包重发布。
