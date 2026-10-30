# 性能测量与复现

`tools/benchmark_matrix.py` 是可选开发工具。运行完整矩阵需要可导入的 pyrobo、graphmap、MujocoSim、MuJoCo、CycloneDDS、psutil 和 PyYAML；不属于 RSim 核心依赖。所有提供端和导航 API 使用正常安装/链接路径，不在产品中调整导入路径。

```bash
python -m tools.benchmark_matrix run --output /absolute/path/results \
  --baseline /absolute/path/refactor-v4-checkout --repeat 3 \
  --cpus 4-7 --seconds 5 --warmup 1
```

输出目录必须不存在。`--cases core,pacer,ipc,factory,memory-realtime` 可选择子集；导航还包括 `memory-environment`、`memory-accelerated`、`memory-lockstep-local`、`memory-lockstep-process`，以及对应的 `mujoco-*`。加速运行分别测量 1、2、10 倍配置。core 按 AB/BA 顺序运行基线与候选，其余场景在相邻重复中倒序运行。不要同时运行其他性能基准或重测试。

每次子进程保留四库实际导入位置、HEAD/工作区 diff 摘要、工具摘要、环境版本、CPU affinity、完整命令、配置/场景指纹、逐样本 JSON 和日志。`manifest.json` 记录全部运行及退出码。失败结果仍保留，不能只选择成功运行计算结论。

指标口径：

- 默认 task 的唤醒误差为实际 callback 开始减它所用 Metronome 的计划 deadline。RatePacer 使用其公开 deadline；源事件没有主机计划 deadline，jitter 为 null。测量包装仅安装于基准进程，会产生额外开销；基线和候选必须使用相同包装。
- callback duration 是墙钟时间。`service:*` 包含等待请求时间，Connect 包含等待输入时间，不能当作纯计算时间。导航诊断的 compute 时间、StageAck 的实际阶段起止和调用端 RTT 分别保留。
- 外部采样器每约 20ms 观察协调进程及所有后代，包括 worker、supervisor、store guard。CPU 是各 PID/create-time 身份在窗口内首次与末次样本之差的总和；进程消失后未采到的尾部 CPU 不在此数中。单样本的进程 CPU 为 null。RSS 是同时采样的总和，共享页可能重复计数；峰值为采样峰值，不能等同系统精确 high-water mark。
- IPC bytes 是成功交给 Cyclone Endpoint 的实际 JSON UTF-8 字节数，排除 CDR、网络头及 DDS 重传。数组 mmap materialization 与 queue/history counters 保留在 Runtime 原始诊断中，不能把文件逻辑大小当作磁盘写入量。
- GPU 和 native allocation 无可靠专用统计时明确为 null 并附原因。RSS 已含原生驻留内存，不能从中反推 allocator 分配量。

core 先预热指定秒数，再测指定秒数。连续导航先完整跑一轮预热，再以新提供端跑第二轮；观测窗从第一条到最后一条状态样本。完整 Runtime RTF 使用从初始 stamp=0 到显式 pause 的实际物理增量，除以 open 到 close 的全部墙钟时间。锁步先跑一轮预热，再跑两轮 reset 导航，控制窗口及完整 Runtime 分别报告。导航轨迹本身不是平稳负载，报告应称“预热后的导航窗口”，不能据此推断无限长稳态性能。

实时/环境 1 倍配对用相同 .1s/.02s 配置；加速 1/2/10 用相同 .2s/.04s 配置；锁步 local/process 用相同 .1s 逻辑预算配置。跨配置家族的差异不能全部归因于调度模式。所有导航样本以到达、停车、无碰撞、固定物理 dt 和匹配 ACK 为有效前提。配置倍率不是实测 RTF。

Factory 层测量五种真实配方的解析、未启动图构造、shared 描述和 spawn 重建，另测生成参数文件生命周期及真实 Native 组合的 DDS 往返。最后一项明确以测试电机和估计器替代硬件；不代表 ROS2 EKF/ICP/SLAM 或硬件运行性能。

原有 `tools/compare_pacing.py paired`、`extensions`、`memory` 继续保留。配对前先做 baseline 自比较；旧 interval error 是相邻观测源 stamp 的周期误差，不能冒充绝对 wake jitter。不同层应使用各自预算，保留每轮结果和离散程度。Python/asyncio/DDS 的这些测试不构成硬实时保证。
