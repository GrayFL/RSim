# RSim 0.4 跨库迁移

四库继续独立维护。RSim 提供通用 runtime/transport/client；pyrobo 提供纯算法和可选运行组件；MujocoSim 提供纯引擎模型和可选运行组件；graphmap 保持纯几何/地图能力。具体设备完整声明在所属 Factory，未建立集中 contracts 或插件注册表。

| 原入口 | 当前入口与兼容策略 |
|---|---|
| `rsim.devices.Chassis` | 仍是 RPC，不改变语义；Shared Ports 使用新增 `ChassisPortsDevice` |
| 内置 driver 函数 | 已迁类的旧函数委托对应 `*Factory`，详见 [全部工厂状态](factories.md) |
| `pyrobo.integrations.rsim` | `pyrobo.rsim`，旧路径委托同一实现 |
| `mujocosim.integrations.rsim` | `mujocosim.rsim.drivers/components/devices`，旧路径仅兼容入口 |
| `rsim.adapters.mujoco` | 已撤出实现；过渡类型别名指向 `mujocosim.rsim.components`，需本地 MujocoSim，计划 0.5 删除 |
| `rsim.drivers.MuJoCo` | 弃用构造包装，调用新 owner，显式保留 `legacy_continuous` reset 时间 |
| `rsim.components.simulated_chassis` | 0.4 退役并提供迁移错误；改用 `MemoryChassisFactory`，不保留重复产品模拟器 |
| `Chassis(simulate=True)` | 延迟构造 pyrobo Memory provider，RPC 客户端接口保持；定位显式使用真值，动力学为有限力模型 |

旧 SimulatedChassis 的理想瞬时运动、噪声/bias、无界命令记录不属于新内存模型的数值兼容承诺。控制器回归中的最小理想反馈在 tests-only fixture 维护；真实跨库导航测试使用 pyrobo 模型。停止命令仍立即进入安全路径，但物理速度按有限制动力衰减。

新的 MuJoCo Factory 在构造时声明 clock，reset 保留物理原生时间并增加 episode，旧命令 epoch 失效。Signal 在 episode/session 改变时隔离旧历史。需要旧连续偏移时间的本地构造应明确选择 `reset_policy="legacy_continuous"`；不能把两种时间策略混用。

新 pyrobo/MujocoSim 扩展要求 RSim >=0.4。`rsim[mujoco]` 是过渡 extra，指向 `mujocosim[engine,rsim]`；`rsim[simulation]` 提供 pyrobo 仿真。提供端安装相应引擎依赖，通用消费者只需 RSim、pyrobo 和声明的纯依赖；不要用同名公共包替代这四个项目的选定源码。

源码开发可继续采用明确的项目符号链接或 `.pth`，不要求构建 wheel。不同解释器的依赖必须独立声明，不将全量开发环境路径注入轻量消费者。`tools/refactor_environments.py` 与 `tools/check_navigation_environments.py` 提供显式源链接环境及同主机跨环境导航的验收工具；脚本的选定依赖清单与结果记录不替代发布安装验证。

运行模式边界：同一消费者支持 Memory/MuJoCo、本地/线程/进程、同主机不同解释器；锁步与实时模式分别验收。跨主机 deadline 转换、跨进程锁步屏障、完整加速导航及实体硬件均不由本轮软件测试保证。

回滚先停止消费者及 Provider，等待安全回退并保留日志；按四库匹配版本成套恢复源码，再创建新 Provider。不反序列化旧活动资源或跨版本 pickle，不单独回滚一库后复用旧 worker。用户 notebook 输出与场景资源不因迁移重写。
