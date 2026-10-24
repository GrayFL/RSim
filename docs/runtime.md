# 跨平台运行时

应用仍以 `Component → Signal / CommandSink` 组成图。平台原语集中在
`rsim.platforms`，只依赖标准库；`runtime` 决定需求、放置、租约与关闭顺序，
`transport` 决定数据格式、DDS 描述信息和共享数组。Linux 与 Windows 使用同一套
运行时策略，不同之处由原语实现。其他系统明确报能力不足，不默认套用 Linux。

## 参数与路径

```python
from pathlib import Path
from rsim import Runtime, ProcessPlacement
from rsim.runtime import RuntimeOptions, RuntimePaths, ShutdownPolicy

options = RuntimeOptions(
    paths=RuntimePaths(storage_root=Path('assets/payload'),
                       registry_root=Path('assets/registry'),
                       log_root=Path('assets/logs')),
    shutdown=ShutdownPolicy(grace_s=5, kill_wait_s=5),
    startup_timeout_s=30,
)
# source 是已构造的普通 Component。
async with Runtime(source.output, placement={source: ProcessPlacement('compute')},
                   options=options) as runtime:
    frame = await source.output.get(timeout=5)
    report = runtime.diagnostics()
```

路径优先级为显式 `RuntimePaths`、`RSIM_REGISTRY_ROOT / RSIM_STORAGE_ROOT /
RSIM_LOG_ROOT`、平台默认值。相对路径在启动前固定为绝对路径；仅创建参数对象不
创建目录。`SharedProvider`、`SharedComponent`、legacy `ProcessSensor/SharedSensor`
可显式传 `options=`，否则继承所属 Runtime。连接者的关闭策略不修改已有 provider。
legacy 的显式 `log_path=` 优先于日志根目录。

Linux 默认使用当前用户的临时 registry、`/dev/shm` 数据及 XDG state 日志。
Windows 使用 LOCALAPPDATA 下的 registry/日志和用户临时数据目录。每次实例创建
私有子目录；日志和 `.report.json` 保留在独立日志目录。它们需要调用者自行管理
保留周期，不应放进短命数据目录。已有 registry/日志目录必须通过权限检查；
运行时拒绝过宽权限、符号链接和 junction，不自动修复来历不明的目录。

registry v3 namespace 包含平台、启动标识、用户身份与存储根域，再区分 DDS domain
和组件 key。Windows 使用 token SID，Unix 使用 UID。Windows IPC 是仅 loopback 的
随机令牌认证连接，记录受私有 ACL 保护；Unix stream 校验双方 UID，长 registry
路径通过持有目录 fd 绑定原目录中的 socket。端点身份匹配后才允许清理。

此共享域只适用于同一主机、启动实例、用户和本地存储域。网络盘不用于共享数组；
网络 DDS 可达不代表另一台主机可读取 mmap 路径。跨主机仍应使用显式复制/中继协议。

## 生命周期和所有权

监督状态为 `NEW → STARTING → RUNNING → STOPPING → CLOSED`，失败保留为 `FAILED`。
`RUNNING` 要求 worker 完成初始化并原子发布 ready，启动 PID 本身不构成就绪。
父循环、监督者和计算 worker 分离，阻塞的计算循环不会阻止监督者回收它。

| 所有权 | 生命周期 |
|---|---|
| 本地组件 | Runtime 按依赖逆序关闭；Python 无法安全强杀一个本地线程 |
| ProcessPlacement / ProcessSensor | 私有 Runtime 管道租约；父进程退出即触发关闭 |
| SharedProvider / SharedSensor | 独立的源连接租约；launcher 关闭后其余客户端可继续使用 |
| 端口订阅 | 仅决定输出的物化与读取，不等于源租约或速度命令控制权 |

启动锁负责同 key 的启动与最后客户端退出竞争；监督者另持实例所有权锁，直到旧
worker 和数据清理完成。连接者只声明端口和订阅，不执行 provider 工厂。ready 前
失败、取消启动和 schema 冲突均回滚本次资源，不接管或清理其他运行实例。

关闭时先拒绝新公共命令，再让已有控制器发送零速 fallback，停止任务和服务，
逆序关闭组件，最后释放绑定、进程和数据文件。宽限期后强制终止受管进程；所有等待
有界。取消不会取消独立清理任务，也不会被当成成功吞掉。清理失败会有错误、告警
或持久化报告；不能把未完成的本地 native 任务标成已关闭，可在资源释放后重试
`aclose()`。需要可强杀的计算应放到 ProcessPlacement。

Windows worker 挂起创建、加入 kill-on-close Job、恢复执行，只继承明确的标准句柄。
Job 句柄不继承；独立 shared supervisor 按租约规则显式脱离父 Job。Linux 依靠受管
租约链、进程组与 exec trampoline 的父死亡信号。后者不承诺拦住主动脱组的任意
第三方守护进程，因此诊断中的 `os_enforced_tree_containment` 为 false。

## 需求与共享存储

`Component.has_demand('image')` 和 `demand_snapshot()` 查询 Runtime 的独立需求表。
静态图和动态订阅使用幂等 token；只要还有一个消费者，输出就有需求。新增昂贵源可
在事件循环查询需求，把不可变 snapshot 传入工作线程。线程只读 snapshot；释放后
最多完成已经开始的那一批采样。`get()` 不会建立隐式订阅。

Signal 的 prepare hook 仍只负责物化。MuJoCo 不再检查 hook 判断渲染需求，无图像
订阅时不创建 Renderer；相机与物理的 native 操作继续在同一所有者线程执行。

数组首次进入共享存储可能复制；完整、不可变且确认 backing identity 的映射转发
可以硬链接。视图、不同布局、已删除来源路径或跨卷会明确复制并记录原因。
`rsim.transport.files.backing_identity(array)` 提供不透明的本机后备标识；不应将
inode 或 Windows 文件 ID 的格式作为业务 API。已取得的只读 Frame/mmap 在历史
淘汰与 unlink 后仍有效；迟到的 descriptor 可能已经无法打开，并计为淘汰丢弃。

诊断区分 history 淘汰、接收队列丢弃、descriptor 淘汰、物化字节、映射次数、硬链接
和复制原因。latest 读取可以跳帧，不是无损消费承诺。

## 诊断与迁移

```sh
python -m rsim.apps.doctor --require process,shared-mapping,dds --deep --json
python -m rsim.apps.doctor --require render --json
```

`doctor` 按请求实际创建 worker、DDS 端点、子进程保留映射或小型渲染器，输出 JSON
及失败的非零退出码。包含解释器、事件循环、路径、用户/boot scope、文件系统和树
回收能力。它不会在普通 import 阶段运行。Runtime 的同步 `diagnostics()` 则描述
当前图、需求、worker/实例路径、持久日志、端口计数及清理错误。

SharedComponent 的 10 Hz 存活检查使用 `describe(diagnostics=False)`，仍返回实例身份、
端口错误和租约计数，但省略 `transport_stats`；跨 placement 查询也传递该选项。
这条路径不递归扫描共享文件目录，避免大量保留帧的诊断 I/O 阻塞命令确认。
需要完整统计时显式调用 `await describe_shared(key, transport=...)`（位于
`rsim.runtime.sharing`），或在已有连接上调用 `await connection.request('describe')`。
显式 describe 默认仍包含完整 transport_stats；不要把该文件统计放入高频控制循环。
定时刷新后的 `SharedComponent.manifest` 不保证包含完整统计，不能将缺失值解释为零。

现有公共构图方式与封闭 dictionary wire codec 保持，端口协议仍为 v2；新增描述
统计字段是可忽略字段。registry 使用新的 v3 namespace，旧实例须先在旧版本中关闭，
再启动新版本；不能通过改目录名复用旧实例。平台原语直接使用 `rsim.platforms`，
旧平台转发模块已移除。进程入口使用 `runtime.supervisor`、`runtime.shared_supervisor`
及对应 worker；Linux 外部程序启动使用 `platforms.linux.exec`。

根入口 `rsim.Chassis` 仍为 ROS1 兼容适配器。应用连接控制服务时使用
`from rsim.devices import Chassis`；组装 provider 时使用
`from rsim.drivers import Chassis`；ROS1 示例应显式使用
`from rsim.adapters.ros1 import Chassis as Ros1Chassis`。

数据字段与时间规则见 [数据契约](data-contracts.md)，开发检查见 [本地测试](validation.md)。
