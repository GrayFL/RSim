# 共享文件的本地基准

`tools/benchmark_runtime.py` 用合成数组、真实 provider 子进程、DDS 描述符和只读
mmap 测量运行时，不需要硬件。直接在已有开发环境运行，需要 NumPy、cloudpickle、
Cyclone DDS 和 psutil；不构建 wheel，也不创建测试环境。

## Windows 对比

```powershell
python tools/benchmark_runtime.py --workloads rgbd --variant file --variant temporary --variant direct --variant direct-temporary --seconds 30 --warmup-seconds 2 --idle-seconds 3 --settle-seconds 8 --repeat 2 --output assets/benchmark/windows.json
python tools/compare_benchmarks.py assets/benchmark/windows.json --output assets/benchmark/windows-summary.json
```

每种模式独立启动、预热、测量、关闭，再观察延迟写回；偶数轮反向执行，减少顺序影响。
默认读取并校验完整数组。多个消费者是同一个客户端进程中的独立订阅；这不是多台机器
或多个客户端进程的吞吐量测试。命令确认、TTL 回退和过期命令检查仍保留，命令只作用于
基准内部的模拟组件。

| 模式 | 数组生成方式 | Windows 文件属性 |
|---|---|---|
| `file`（Linux 默认） | 普通 ndarray，首次进入共享存储时复制 | 普通文件 |
| `temporary`（Windows 默认） | 同上 | `FILE_ATTRIBUTE_TEMPORARY` |
| `direct` | 源直接填写共享映射 | 普通文件 |
| `direct-temporary` | 源直接填写共享映射 | `FILE_ATTRIBUTE_TEMPORARY` |

Windows 共享文件默认使用临时属性，基准默认也选择 `temporary`；显式选择 `file`
或 `direct` 时会关闭此属性，保留普通文件对照。Windows 在内存充足时可根据此提示
减少临时文件的存储写入，但它不是禁止落盘的开关。
[Microsoft 文件属性说明](https://learn.microsoft.com/en-us/windows/win32/fileio/file-attribute-constants)

`direct` 在合成源中拥有一个 `SharedStore`，每帧分配独立映射，填充后转为只读。
这用于观察省去首次复制的影响，不是循环覆盖同一缓冲区，也不表示当前所有适配器都能
直接填充共享数组。减少复制与减少磁盘写入需要分别判断。

## Linux 对比

在 Linux 主机运行相同代码；当前默认共享根目录是 `/dev/shm`。先确认另一个测试目录
实际位于磁盘文件系统，再分别测试。下面的 `/path/to/local-disk-directory` 需替换为
已有本地目录；不要用网络盘。

```sh
findmnt -T /dev/shm
findmnt -T /path/to/local-disk-directory
python tools/benchmark_runtime.py --workloads rgbd --storage-root /dev/shm --seconds 30 --repeat 2 --output assets/benchmark/tmpfs.json
python tools/benchmark_runtime.py --workloads rgbd --storage-root /path/to/local-disk-directory --seconds 30 --repeat 2 --output assets/benchmark/disk.json
python tools/compare_benchmarks.py assets/benchmark/tmpfs.json assets/benchmark/disk.json
```

`tmpfs` 使用内存，但可能进入 swap；同时看报告中的可用内存和 `swap_io_delta`。
磁盘文件也可能在页缓存中完成短寿命数据的读写，所以“磁盘文件路径”不代表每字节
payload 都会写入块设备。[Linux tmpfs 文档](https://docs.kernel.org/filesystems/tmpfs.html)

## 读懂结果

| 报告字段 | 含义与边界 |
|---|---|
| `observed_payload_bytes`、`observed_publication_hz` | 依据消费者观察到的源帧号跨度推算产出，包含期间跳过的帧，不是磁盘写入 |
| `consumer_read_bytes`、`payload_read_ms` | 实际遍历数组的字节数与耗时；多个消费者分别计数 |
| `transport_delta.materialized_bytes` | 运行时额外复制到共享存储的逻辑字节数，不是落盘量 |
| `resources.processes.*.io_delta` | 采样窗口内各进程的 I/O 差值；可能包含非磁盘流量，且 mmap 写回未必计入该进程 |
| `io.steady.disks.<device>.delta.write_bytes` | 稳态窗口中操作系统报告的整台主机该设备写入量，含其他程序和文件系统元数据 |
| `io.idle` | 启动前的背景 I/O；用于判断噪声，不能机械地从运行量扣除 |
| `io.post_close` | 关闭、清理后观察到的整机 I/O；有限等待不能证明所有脏页均已写回 |
| `io.whole_run` | 启动、预热、稳态、控制检查、清理和延迟观察的整段设备 I/O |
| `resources.sampled_peak_unique_file_bytes` | 采样时存活 `.npy` 文件的逻辑长度峰值，硬链接去重；不是累计写入或物理驻留内存 |
| `resources.processes.*.sampled_peak_rss_bytes` | 各进程采样到的 RSS 峰值，共享页可能重复计数，也不包含全部系统缓存 |
| `resources.minimum_host_available_bytes` | 采样到的整机可用内存最低值 |

JSON 使用字节，终端使用 MiB。缺失或复位的计数保留为不可用，不当成零。Windows 上
psutil 不提供 swap 进出流量，因此该字段为 `null`。设备写入量也不是 SSD 内部 NAND
实际磨损量。[psutil 计数器说明](https://psutil.io/api/)

报告保留每个磁盘设备的独立计数。`--disk PhysicalDrive0`（Windows）或 `--disk sda`
（Linux）只筛选终端及汇总显示，不改变原始 JSON。先确认文件所在卷对应的设备；
Linux 的整盘、分区与 device-mapper 可能重叠，不能直接求和。程序不会把整机计数声称
为 RSim 独占写入，也不会自动扣掉背景值。

`compare_benchmarks.py` 接受一个或多个同主机、同 Python 环境的新版报告，按
workload、variant、存储根目录分别汇总各轮的最小值、中位数和最大值。终端显示中位数，
JSON 保留范围、配置与有效计数数量；延迟是各轮 p95 的汇总，不是合并所有帧的 p95。
它不再依赖固定阈值文件，不给出自动通过/失败结论。旧版报告只取描述符、未完整读取
数组，测量口径不同，需重跑，不能混合比较。

## 调整负载

默认 `20 Hz`、历史长度 `4`、稳态 `10` 秒；磁盘结论建议使用更长窗口并重复测量。
`rgbd` 为 `640×480×4` 个 float32（每帧约 4.69 MiB），是等字节量合成负载；
`points` 为 300,000 个 float32，不是 300,000 个三维点。`fanout`、`late`、`slow`
和 `mapping-control` 保留多订阅、晚加入、慢消费等场景。可用逗号选择多种 workload。

- `--history` 改变保留帧数，可观察更长数据寿命对内存和写回的影响。
- `--elements` 覆盖所选 workload 的 float32 元素数，用于匹配实际每帧字节量。
- `--hz` 改变发布频率，结合实际 `observed_publication_hz` 判断是否跟得上。
- `--read-mode metadata` 只取描述符，用于单独分析订阅开销；不能代表真实数组读取。
- `--sample-interval` 控制进程/文件采样频率；较短间隔本身会增加开销，采样峰值可能漏过瞬时峰值。
- `--source` 选择另一份源码，子进程也使用它；该源码须支持当前 RuntimeOptions 等接口。

传输诊断的前后 `describe` 请求与消费者窗口略有时间差，复制量可能多/少一帧；
`mapping_creations` 只统计运行时拥有的存储，不包括 `direct` 的源自有存储。
文件统计只覆盖仍有路径的文件，已 unlink 但被外部数组引用保留的映射不在其中。
本工具会释放自身的引用并清理私有目录，不能代替真实应用中长期持有帧的内存检查。

测试时避免同时运行其他重 I/O 工作。报告写盘在测量窗口外，但写回可能延后；每轮保留
空闲和退出后观察，不清系统缓存、不强制 flush、不改 pagefile、也不制造内存压力。
要评估真实应用，需匹配它的数组大小、保留历史、消费速度和长期引用方式，再观察较长
时段。缓存写回由操作系统控制，仅凭短期磁盘计数不能承诺永不落盘。
[Microsoft 文件缓存说明](https://learn.microsoft.com/en-us/windows/win32/fileio/file-caching)
