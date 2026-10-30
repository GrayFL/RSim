# 任务节奏、回放与显式步进

默认 `Component.task(name, callback, hz=...)` 保留原 Metronome 路径。可选 `pacer=` 控制任务何时唤醒，两者必须且只能提供一个。CommandSink deadman、服务、watchdog、进程租约仍使用主机单调时间。

```python
from rsim import Component, SignalPacer

class Consumer(Component):
    def __init__(self, state):
        super().__init__(inputs=(state,))

    async def open(self):
        self.pacer = SignalPacer(self.inputs[0], hz=30, deduplicate=True)
        self.task("control", self.control, pacer=self.pacer)

    async def control(self):
        frame = self.pacer.last_frame
        # 使用 frame 的源时间与 episode；不要把旧输入换成新的安全租期。
```

Pacer 提供 async `tick()` 与 `aclose()`。每个任务在所属 worker 的 `open()` 绑定自己的订阅，取消时 Runtime 关闭它；不得将 asyncio 条件变量或运行中的 TickSource 序列化给 worker。SignalPacer 通过已有 Signal 传输处理最新状态，不查询远端时钟。

`SignalPacer(signal, hz=None, deduplicate=False)` 默认逐次处理观察到的 publication。同一个物理 step 可以有多个独立命令/规划请求，默认不会按 step 丢掉它们。状态消费者可设 `deduplicate=True`，拒绝同一阶段内的时间/步序倒退，并忽略重复 step/stamp；`hz` 使用源时间的整数/Fraction锚点筛选，不是主机频率。回调应读取 `last_frame`，不要另取可能已经改变的 latest。`elapsed_ns` 是两次实际交付的源时间差，首帧和新 session/episode 为 None；应用需明确首帧策略，不能把跳帧时差伪装成名义周期。

诊断计数中，`received` 是实际观察的 publication 数，`delivered` 是回调可处理的交付数，`filtered` 是源频率筛选数，`duplicates` 是重复 step/stamp 数；`skipped` 用 SampleId 的 publication_sequence 记录未观察的发布缺口，跨进程不依赖本地读者sequence。`source_skipped` 统计相邻交付step缺口，包括主动降频，不能全部解释为过载。`resets` 记录观察到的session/episode切换。SignalPacer不提供every背压。

`Connect(commands, sink, hz=None)` 显式使用源事件转发，避免加速算法仍被固定host频率限速；省略hz仍为原100Hz。事件模式保留命令身份、原deadline、多写者claim与过期/退役拒绝。它不会把每次转发变成新的有效决策。

`RatePacer(hz, rate=1., paused=False)` 是可选物理源的主机节奏：支持async `pause/resume/set_rate/aclose`，暂停或改速会唤醒已有等待，恢复不补跑暂停期间的步。它只控制wall节奏，环境stamp仍由源以固定物理dt推进；owner必须独立退役命令/旧结果并管理episode。`missed`、`received`、`scheduled_deadline_ns`、`actual_start_ns` 可供诊断。默认 `task(hz=...)` 不走此实现，host安全任务也不继承源的pause/rate。

`TickSource(clock=...)` 由物理源显式调用 `publish(stamp_ns=..., episode=..., step_index=..., session=...)`。它不生成物理时间。`subscribe(hz=..., policy="latest")` 为每个消费者维护独立游标，只保留最新待处理 tick 并记录 skipped；`policy="every", capacity=N` 使用有界队列，满时反压源。频率用源时间锚点与有理数计算，例如 500 Hz 源与 30 Hz 任务不会累积周期取整误差。关闭/失败唤醒读写双方。

新 episode/session 是显式控制边界，可以越过旧队列的背压并清空旧 tick；被替代的待发布调用收到 `PacerReset`，源应丢弃该旧阶段工作。旧 episode 和已退役 session 不能再次进入队列。

本地流水线可使用 `FrameTickPacer(signal, subscription)`，从独立TickSubscription领取tick后等待上游真正发布对应session/episode/step/stamp/clock的Frame，回调仍读取 `last_frame` 与 `elapsed_ns`。它解决订阅者唤醒顺序不同的问题，不是计算或命令ACK。every要求上游频率与Signal有界history容量足够保留目标Frame；已丢失时明确HistoryMiss。latest可合并失去匹配的旧tick并计skipped。reset和源失败也会中断等待旧上游Frame；关闭同时释放订阅和等待任务。应用必须验证订阅与源在同一worker，不得将本地TickSource序列化后当作跨进程背压。

`every` 确认的是 tick 交付，不能据此宣称算法和命令已完成。默认实时导航、加速物理源与严格锁步是不同模式；现有 pyrobo 导航仍按其显式执行配置运行，完整加速导航还需同步接入环境 Pacer 和到期策略。

`rsim.components.replay.ReplaySource` 消费 `ReplaySample(stamp_ns, data, metadata)` 快照，也可显式 `from_jsonl(path, clock=...)` 加载 JSON 记录。`pause/resume/set_rate/seek` 均为 async；seek 清空历史、增加 episode，保留原始 stamp 和读者 sequence。EOF 等待 seek/close。暂停和 EOF 不会为已发送命令续租，消费者还须检查数据新鲜度。

回放的 `host_monotonic_ns` 记录当前发布时刻，原记录中的该值转存 `recorded_host_monotonic_ns`，避免旧录制时间使新回放输入永久陈旧。payload 内已有命令的原 deadline 保持不变，过期命令不能因回放重新生效。

`SingleProcessStepper` 是显式顺序执行工具：sample(k) → compute(k) → accept(command(k)) → advance(k+1)。accept 必须在真正接纳后返回匹配当前 session/episode/step 的 `StepAck(context, "command")`；超时、失败、取消先调用 stop 并中止会话，恢复须显式 reset。它不改变资源 DAG，不替调用方推进模型。确定性仍要求固定 seed、逻辑预算和稳定结果选择；本轮未实现跨进程 ACK 屏障，也未宣称 MuJoCo 导航已支持严格锁步。

阶段间显式复核主机 deadline，避免阻塞回调或吞掉取消后继续接受命令。停止/reset 是有预算的独立事务，重复取消也先等待事务结束再向调用方传播。它无法抢占同步阻塞或保证硬实时；实际执行器仍需自己的 host deadline 检查。

回归入口：`tests/test_pacing.py`、`tests/test_source_pacing.py`、`tests/test_replay_lockstep.py`、`tests/test_episode_history.py`、`tests/test_epoch_forwarding.py`。软件性能可用 `python -m tools.benchmark_pacing --output assets/pacing.json` 测量；结果包含多频率、组件数、数据大小、进程模式和负载。跨进程的观察采用 latest，parent CPU 不冒充整机/所有 worker 的 CPU。
