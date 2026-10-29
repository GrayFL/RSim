# 任务节奏、回放与显式步进

默认 `Component.task(name, callback, hz=...)` 保留原 Metronome 路径。可选 `pacer=` 控制任务何时唤醒，两者必须且只能提供一个。CommandSink deadman、服务、watchdog、进程租约仍使用主机单调时间。

```python
from rsim import Component, SignalPacer

class Consumer(Component):
    def __init__(self, state):
        super().__init__(inputs=(state,))

    async def open(self):
        self.pacer = SignalPacer(self.inputs[0])
        self.task("control", self.control, pacer=self.pacer)

    async def control(self):
        frame = self.pacer.last_frame
        # 使用 frame 的源时间与 episode；不要把旧输入换成新的安全租期。
```

Pacer 提供 async `tick()` 与 `aclose()`。每个任务在所属 worker 的 `open()` 绑定自己的订阅，取消时 Runtime 关闭它；不得将 asyncio 条件变量或运行中的 TickSource 序列化给 worker。SignalPacer 通过已有 Signal 传输处理最新状态，不查询远端时钟。

`TickSource(clock=...)` 由物理源显式调用 `publish(stamp_ns=..., episode=..., step_index=..., session=...)`。它不生成物理时间。`subscribe(hz=..., policy="latest")` 为每个消费者维护独立游标，只保留最新待处理 tick 并记录 skipped；`policy="every", capacity=N` 使用有界队列，满时反压源。频率用源时间锚点与有理数计算，例如 500 Hz 源与 30 Hz 任务不会累积周期取整误差。关闭/失败唤醒读写双方。

新 episode/session 是显式控制边界，可以越过旧队列的背压并清空旧 tick；被替代的待发布调用收到 `PacerReset`，源应丢弃该旧阶段工作。旧 episode 和已退役 session 不能再次进入队列。

`every` 确认的是 tick 交付，不能据此宣称算法和命令已完成。默认实时导航、加速物理源与严格锁步是不同模式；现有 pyrobo 导航仍按其显式执行配置运行，完整加速导航还需同步接入环境 Pacer 和到期策略。

`rsim.components.replay.ReplaySource` 消费 `ReplaySample(stamp_ns, data, metadata)` 快照，也可显式 `from_jsonl(path, clock=...)` 加载 JSON 记录。`pause/resume/set_rate/seek` 均为 async；seek 清空历史、增加 episode，保留原始 stamp 和读者 sequence。EOF 等待 seek/close。暂停和 EOF 不会为已发送命令续租，消费者还须检查数据新鲜度。

回放的 `host_monotonic_ns` 记录当前发布时刻，原记录中的该值转存 `recorded_host_monotonic_ns`，避免旧录制时间使新回放输入永久陈旧。payload 内已有命令的原 deadline 保持不变，过期命令不能因回放重新生效。

`SingleProcessStepper` 是显式顺序执行工具：sample(k) → compute(k) → accept(command(k)) → advance(k+1)。accept 必须在真正接纳后返回匹配当前 session/episode/step 的 `StepAck(context, "command")`；超时、失败、取消先调用 stop 并中止会话，恢复须显式 reset。它不改变资源 DAG，不替调用方推进模型。确定性仍要求固定 seed、逻辑预算和稳定结果选择；本轮未实现跨进程 ACK 屏障，也未宣称 MuJoCo 导航已支持严格锁步。

阶段间显式复核主机 deadline，避免阻塞回调或吞掉取消后继续接受命令。停止/reset 是有预算的独立事务，重复取消也先等待事务结束再向调用方传播。它无法抢占同步阻塞或保证硬实时；实际执行器仍需自己的 host deadline 检查。

回归入口：`tests/test_pacing.py`、`tests/test_replay_lockstep.py`、`tests/test_episode_history.py`、`tests/test_epoch_forwarding.py`。软件性能可用 `python -m tools.benchmark_pacing --output assets/pacing.json` 测量；结果包含多频率、组件数、数据大小、进程模式和负载。跨进程的观察采用 latest，parent CPU 不冒充整机/所有 worker 的 CPU。
