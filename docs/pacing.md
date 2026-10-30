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

`every` 确认的是 tick 交付，不能据此宣称算法和命令已完成。默认实时导航、加速物理源与严格锁步是不同模式；应用需按其模式显式装配计算、命令接纳和物理推进。

`rsim.components.replay.ReplaySource` 消费 `ReplaySample(stamp_ns, data, metadata)` 快照，也可显式 `from_jsonl(path, clock=...)` 加载 JSON 记录。`pause/resume/set_rate/seek` 均为 async；seek 清空历史、增加 episode，保留原始 stamp 和读者 sequence。EOF 等待 seek/close。暂停和 EOF 不会为已发送命令续租，消费者还须检查数据新鲜度。

回放的 `host_monotonic_ns` 记录当前发布时刻，原记录中的该值转存 `recorded_host_monotonic_ns`，避免旧录制时间使新回放输入永久陈旧。payload 内已有命令的原 deadline 保持不变，过期命令不能因回放重新生效。

`SingleProcessStepper` 是显式顺序执行工具：sample(k) → compute(k) → accept(command(k)) → advance(k+1)。默认保留 `StepAck(context, "command")` 的旧回调模式。需要证明真实闭环时，使用提供端的真实 `StepContext` 并显式启用严格模式：

```python
from rsim import SingleProcessStepper, StepContext

runner = SingleProcessStepper(
    sample=sample, compute=compute, accept=accept, advance=advance, stop=stop,
    context=StepContext.from_mapping(source_description['context']),
    require_physics_ack=True, requested_substeps=10,
)
await runner.step()
```

严格模式的 `accept(context, command)` 必须在最终 owner 验证并保存命令后返回 `CommandAccepted(context, command_id, deadline_ns, source_deadline_ns=None, port=None)`。`command_id` 是 `CommandIdentity(controller_id, controller_epoch, sequence)`。`advance(context, accepted)` 使用这一身份请求固定数量子步，并在积分及发布完成后返回 `PhysicsAdvanced(request_context, context, command_id, requested_substeps, actual_substeps, physical_substeps, stamp_ns, phase='READY', state=None)`。提供端也可返回这些对象的 `to_dict()` JSON 形式；协调器使用 `from_mapping()` 验证，`state` 对 RSim 是不透明摘要。命令接受回调负责将实际发送的命令与 ACK 关联，协调器核对物理 ACK 使用完全相同的身份。

只有同 session/episode 的下一控制边界、完整子步数、相同命令身份、递增源时间且不越过固定 source deadline 才能完成严格一步。`last_command_ack`、`last_physics_ack`、`trace` 与 `timings`（各阶段主机开始/结束 ns）可用于审计。两种 ACK 含义不同；收到命令、tick 或不完整推进均不能构造成功的物理 ACK。构造 `StepContext` 现在拒绝空 session、布尔或负数 step/episode。

超时、失败、取消会调用 `stop()` 并中止会话；若已发起推进却没有验证到匹配结果，`needs_synchronization=True`，不能据此认为物理没有推进。应查询提供端已提交状态或 reset；只有显式 `runner.reset(reset_source)` 才恢复此协调器。严格模式的 reset 回调必须返回源实际新 session/episode 的 step zero context，不能由协调器推测。reset 同时淘汰协调器保留的 ACK。它不改变资源 DAG，不替调用方推进模型；确定性仍要求固定 seed、逻辑预算和稳定结果选择。此工具本身不提供跨进程计算屏障。

阶段间显式复核主机 deadline，避免阻塞回调或吞掉取消后继续接受命令。停止/reset 是有预算的独立事务，重复取消也先等待事务结束再向调用方传播。它无法抢占同步阻塞或保证硬实时；实际执行器仍需自己的 host deadline 检查。

## 跨进程计算阶段

`StageRequest(context, task_id, input_revision)` 和 `StageAck(context, task_id, input_revision, started_ns, finished_ns, result=None)` 提供 JSON `to_dict/from_mapping`。revision 是应用定义的非空字符串，可编码 map/mission revision 和上游发布身份；RSim 不解释其领域含义。ACK 的开始/结束时间使用 host monotonic ns，result 是小型 JSON 输出摘要。它确认该计算阶段完成，仍不等于命令接纳或物理推进。

继承 `rsim.StageEndpoint`，声明 dependencies/inputs 并实现 async `compute(request)`。回调必须等待指定的真实输入快照、检查完整 context/revision，完成计算并发布输出后再返回摘要。不同 DDS 端口的到达顺序不保证一致，因此不能因为管理请求先到就读取任意 latest。`stage_request` 是普通 CommandSink，支持 ProcessPlacement；`run_stage`、`describe_stage`、`reset_stage` 都通过这个声明端口，普通 Python 方法没有被变成隐式 RPC。

Endpoint 在 open 创建 worker-local `StageExecutor(task_id, callback, timeout=5., ack_capacity=32)`。相同 context/task/revision 的重试返回原完成结果，计算不会重做；同一步换 revision、跳步、淘汰后重发旧步都拒绝。缓存有界，高水位仍阻止旧步再次执行。失败或超时使此 generation ABORTED，查询不会把失败当成成功 ACK。显式 `reset_stage(new_context)` 淘汰缓存；new_context 必须来自源的新 session/episode 且 step zero。同 generation 的 reset 重试只返回已有状态，不会清除已完成计算或恢复 ABORTED。算法自身的 RNG/cache reset 仍由领域组件负责。

协调器显式指定阶段次序，并可在严格 Stepper 的 compute 回调内执行：

```python
from rsim import StageBarrier, StageRequest

barrier = StageBarrier(('plan', 'execute'), context=source_context)
barrier.begin(source_context)
planned = await barrier.call(planner.stage_request,
    StageRequest(source_context, 'plan', planning_revision))
executed = await barrier.call(executor.stage_request,
    StageRequest(source_context, 'execute', execution_revision))
acks = barrier.complete()
```

每个 call 验证 ACK 的 context/task/revision 完全吻合。`timeout` 是每个阶段的主机预算，`retries` 限定丢响应后的相同请求重试次数，`request_ttl` 限定管理命令租期。重试只更换传输 envelope，不更换逻辑请求或延长已经开始的计算期限。全部阶段完成后，调用方才接受命令并请求固定物理子步；资源 DAG 不决定这个顺序。屏障超时/取消/worker 故障后 ABORTED，须停止 owner，再 reset 源和各 endpoint，最后 `barrier.reset(new_context)`。不能将 worker 的 DDS 收包当成计算完成。

`StageFailed.reason='stale_context'` 表示逻辑 source context 过旧，与命令通道 controller epoch 的 `retired` 不同。管理端口拒绝一个旧上下文后，仍允许同一客户端提交新上下文；真正已退役的运动控制器继续被拒绝。`StageExecutor` 与 `StageBarrier` 不可序列化，必须在所属 worker/coordinator 内创建。诊断提供有限 ACK 缓存、high_water、实际阶段耗时，以及 barrier 的调用耗时/尝试次数。原始重放、单进程 ACK 与 Stage 协议测试分别验证各自边界；通用阶段测试不能替代应用的跨进程导航验收。

回归入口：`tests/test_pacing.py`、`tests/test_source_pacing.py`、`tests/test_replay_lockstep.py`、`tests/test_strict_lockstep.py`、`tests/test_stages.py`、`tests/test_episode_history.py`、`tests/test_epoch_forwarding.py`。软件性能可用 `python -m tools.benchmark_pacing --output assets/pacing.json` 测量；结果包含多频率、组件数、数据大小、进程模式和负载。跨进程的观察采用 latest，parent CPU 不冒充整机/所有 worker 的 CPU。
# Source-time command intervals

`CommandEnvelope` accepts an optional `source_bound=SourceCommandBound(context,
clock, deadline_ns)`. This is an additional immutable source-time limit; its
original host-monotonic `deadline_ns` still applies independently. The bound's
`StepContext(session, episode, step_index)` identifies the observation used to
prove the command interval. A delayed consumer must not move that interval's end
to the provider's current time at acceptance.

The source owner calls `bound.check(current_context, clock=source_clock,
stamp_ns=proposed_interval_end_ns)` before accepting and before each physical
interval. Old generations, future observations, wrong clocks and intervals
crossing the source deadline are rejected. Equality at the interval end is
allowed. Applying the host deadline, model constraints, permissions and fallback
remains the provider's responsibility. A provider without the specified source
context must reject the bounded command. This is validity metadata, not a
command-acceptance or physical-advance acknowledgement.

An owner that queues command application behind a reset must also compare its
generation before and after acquiring the owner lock, including commands without
a source bound. `CommandGuard.retire()` records a lifecycle boundary. A rejected
application cannot roll back a concurrent reset to the older controller lease;
the new retirement and disarmed state take precedence.

`Connect` and the closed-schema shared transport preserve the bound. `CommandMux`
may choose a new controller identity but keeps the selected source bound and
never extends the original host deadline. Adapters changing payloads should use
`dataclasses.replace(envelope, value=converted_value)` so they retain both limits.
The legacy five-argument envelope remains valid with `source_bound=None`; its
wire representation retains the five existing fields. A nonempty bound requires
both peers and the final provider to support this extension.
