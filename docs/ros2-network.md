# ROS 2 跨机器通信配置

本页适用于使用 `rmw_fastrtps_cpp` 的 ROS 2 驱动、建图节点和 RViz。RSim 原生 Cyclone DDS 描述信息总线不读取 Fast DDS XML；本机 mmap 数组存储也不因此获得跨机器访问能力，见 [DDS 与应用分离](dds.md)。

## 加载传输配置

仓库提供可选配置 [fastdds-remote.xml](../ros2/config/fastdds-remote.xml)：

| 参数 | 起始值 | 用途 |
| --- | --- | --- |
| UDP `maxMessageSize` | 1200 字节 | 限制 DDS UDP 数据报大小，减少 IP 分片 |
| UDP `sendBufferSize` / `receiveBufferSize` | 各 4 MiB | 请求 socket 收发缓冲，应与内核上限配合 |
| SHM `segment_size` | 每个 participant 64 MiB | 保留本机共享内存传输；按并发量和可用共享内存调整 |
| SHM `port_queue_capacity` | 8192 | 接收描述符队列容量，不是 ROS topic 的历史深度 |

网络 MTU 与 DDS UDP 数据报大小不相等。IPv4 的普通 UDP 数据报还需要 28 字节的 IP/UDP 头，IPv6 无扩展头时需要 48 字节。该 profile 是保守起点，较小报文会增加包数和 CPU 开销，不是所有部署的最优设置。它不配置网卡 MTU、路由、接口白名单、domain 或发现对端。

发送端和接收端均安装这份 XML，在启动相应进程前设置其绝对路径。从仓库根目录启动的终端可以使用：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$(realpath ros2/config/fastdds-remote.xml)"
```

随后正常运行 Python 程序、驱动入口或 `rviz2`。若机器上没有仓库，将 XML 复制到其部署配置目录，再把环境变量指向该文件。已创建的 DDS participant 不会热加载环境变量，已有 ROS 进程需要正常退出后重启。RSim 已存在的共享 provider 也不会因新客户端设置了环境变量而改变配置；应在其所有使用者正常关闭后，由采用新环境的 provider 再次启动。

Notebook 应在启动 kernel 的环境中设置这些变量，或在任何 ROS/DDS participant 创建之前设置：

```python
import os
from pathlib import Path

profile = Path(profile_path).expanduser().resolve(strict=True)
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
# 此后再创建 ROS context 或启动传感器。
```

该文件只配置 participant 的传输，不需要设置 `RMW_FASTRTPS_USE_QOS_FROM_XML=1`。ROS 参数 `--params-file` 也不是 DDS XML 的加载入口。进程退出后，`unset FASTRTPS_DEFAULT_PROFILES_FILE` 可使后续进程恢复中间件默认传输配置。

## UDP 缓冲的两个层级

Linux/WSL 上先检查：

```bash
sysctl net.core.rmem_max net.core.wmem_max
```

两个值应至少为 `4194304`。若已有更高上限，保持原值。仅在不足时提高：

```bash
sudo sysctl -w net.core.rmem_max=4194304
sudo sysctl -w net.core.wmem_max=4194304
```

需要持久化时，把所需的两个赋值放进部署机器的 `/etc/sysctl.d/90-ros2-udp-buffers.conf` 并用 `sudo sysctl -p /etc/sysctl.d/90-ros2-udp-buffers.conf` 加载；重启后复核。WSL 中 ROS 进程使用的是 WSL 内核的 socket 上限，因此应在 WSL 内检查和配置。

`rmem_max/wmem_max` 只是应用能请求的上限。XML 中的 `receiveBufferSize/sendBufferSize` 才会让 DDS 创建 socket 时请求 4 MiB。无需为此提高所有应用的 `rmem_default/wmem_default`；只改默认值或上限也不会自动扩大已经打开的 socket。

启动后按进程检查实际值：

```bash
ss -uapnm
```

找到目标进程的 socket 及其 `skmem`。Linux 对 `SO_RCVBUF/SO_SNDBUF` 的值加倍用于内核记账，因此请求 4 MiB 常显示为 `rb8388608` / `tb8388608`；接收 socket 和仅发送 socket 不一定同时设置两个方向。`d` 是该 socket 的丢包计数。还应观察重组失败、UDP 错误增量及应用实际接收序号。

## QoS 与时延

XML 不覆盖 topic QoS。静态/缓存地图按发布端使用 Reliable + Transient Local；实时点云根据“最新显示”或“必须保留所有样本”的需要设置可靠性和历史深度。Reliable 不等于应用必定逐帧处理，较小的 KeepLast 队列可能淘汰旧帧。

缓冲和历史深度增加可以吸收突发，也可能增加延迟与内存占用。验收时同时记录发送序号、接收序号、采样时间、接收时间和运行负载；不要把积压后的集中回调频率当作持续实时吞吐。

SHM 传输有助于本机进程间通信，但不能直接等同于 ROS/Python 全链路零拷贝。此 profile 禁用默认传输后显式添加 UDP 和 SHM，因此不会像 UDP-only 诊断配置那样移除 SHM。多个高负载 participant 的共享内存容量需合并评估。

## 建图负载下的配置边界

Fast DDS 2.14 会以所注册传输的最小 `maxMessageSize` 限制 participant 的发送大小。因此，给 UDP 设置较小上限，也可能使同一 participant 的本机大点云分成更多片；保留 SHM 并不能消除这一影响。应在建图、录制、障碍监测和远端显示同时开启时验证输入连续性与服务响应，不能只依据轻量订阅或合成消息测试判断配置可用。

Fast DDS 的 `fastdds.max_message_size` 属性还支持限制单个 DataWriter 的发送大小。按话题配置时，必须同时检查发现流量：应用 DataWriter 的限制不自动覆盖内置发现 writer；为内部点云保留大分片，也意味着远端订阅这些话题时仍可能收到大 UDP 数据报。抓包应区分地图数据、原始点云和 DDS 发现报文，核查实际 IP 分片及应用接收结果。详情见 [发送大小属性](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/property_policies/non_consolidated_qos.html#maximum-message-size) 和 [传输上限的实现](https://github.com/eProsima/Fast-DDS/blob/v2.14.6/src/cpp/rtps/network/NetworkFactory.cpp)。

跨机器包长优化与本机计算调度需要分别测量。大消息反序列化、点云处理和数值库线程竞争可能阻塞控制协程；可通过进程部署隔离接收处理，并按负载限制数值库线程数。验收仍以命令确认延迟、原始反馈新鲜度和远端停止结果为准，不为消除超时报错而放宽运动命令有效期。

参考：[Fast DDS 传输 XML](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/xml_configuration/transports.html)、[配置文件环境变量](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/env_vars/env_vars.html)、[SHM 传输](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/transport/shared_memory/shared_memory.html)、[Linux socket 缓冲](https://man7.org/linux/man-pages/man7/socket.7.html)。
