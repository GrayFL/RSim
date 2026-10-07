# 跨机器 DDS 底盘控制

远端沿用 `rsim.devices.Chassis`、`chassis_command` 和 `keyboard_control`，不需要另一个控制协议。驱动、EKF 和运动闭环留在机器人上；客户端只交换小型 JSON 命令、应答和 Pose。无需在客户端安装 ROS，也不需要共享硬件 YAML 或标定文件。

## 双端 DDS 配置

这些控制 CLI 明确使用原生 Cyclone DDS。`RMW_IMPLEMENTATION`、`FASTRTPS_DEFAULT_PROFILES_FILE` 和 Fast DDS XML 不控制这个连接。两端安装相同版本的 RSim 与项目使用的 graphmap；客户端依赖可通过 `python -m pip install -e '.[teleop]'` 安装。使用尚未提交的工作区功能时，普通 Git clone 不包含这些文件，需先同步相应版本。

从两端各自的项目根目录设置环境变量，再启动 Python：

```bash
# 两端交换本机/对端地址；填写互相可达的 LAN 或 VPN IPv4 地址。
export RSIM_DDS_LOCAL_IP="<本机地址>"
export RSIM_DDS_PEER_IP="<对端地址>"
export RSIM_DDS_DOMAIN=42
export CYCLONEDDS_URI="file://$PWD/examples/control/cyclonedds.remote.xml"
```

模板 [cyclonedds.remote.xml](../examples/control/cyclonedds.remote.xml) 指定本地网卡地址、禁用多播、使用双向静态 peer。这样不用依赖 VPN 或 WSL 的多播发现。DDS UDP payload 上限为 1200 字节，分片为 1025 字节；这是小型控制消息的配置，大图像和点云传输需单独评估。参见 Cyclone DDS 的 [网卡选择](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html)、[静态发现](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-config.html) 和 [XML 配置参考](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)。

`RSIM_DDS_LOCAL_IP` 必须实际出现在本机 `ip -brief address` 中；不能填一个只存在于 NAT 外侧的地址。用 `ip route get <对端地址>` 检查走的网卡和源地址。环境必须在 DDS participant 创建之前设置；修改配置后重启相应程序，Notebook 需要重启 kernel。

上述命令让模板只作用于 domain 42，避免改变其他 domain 的 DDS 配置。`RSIM_DDS_DOMAIN` 只改变 XML 适用范围，**不代替 CLI 的 `--domain`**；两端两处必须一致。不要为此修改硬件 ROS 的 `ROS_DOMAIN_ID`。服务名两端也必须一致，示例均为默认的 `chassis`。

## 联调顺序

先在机器人上启动模拟服务，模拟器不会打开串口：

```bash
python -m rsim.apps.chassis_service --simulate --enable-motion --domain 42
```

在远端运行：

```bash
python -m rsim.apps.chassis_command --domain 42 status
python -m rsim.apps.chassis_command --domain 42 move 0
python -m rsim.apps.keyboard_control --domain 42 \
  --config examples/control/keyboard.example.yaml --input pygame --dry-run
```

`status` 只订阅状态，不取得控制权。键盘 `--dry-run` 会取得控制权但始终发送零速；先退出已有键盘控制进程，避免出现 `another client`。程序内可直接使用：

```python
from rsim.devices import Chassis
from rsim.runtime import Runtime
from rsim.transport.descriptor import TransportConfig

chassis = Chassis("chassis", transport=TransportConfig(
    backend="cyclonedds", domain_id=42,
))
async with Runtime(chassis):
    pose = (await chassis.pose.get(timeout=5)).data
    print(pose)
    await chassis.move(0)
    await chassis.rotate(yaw_deg=0)
```

实机将服务命令替换为 `python -m rsim.apps.chassis_service --config configs/local_chassis.yaml --domain 42`，默认禁止非零运动。需要实际运动时再加 `--enable-motion`，并去掉客户端 `--dry-run`。服务端本地硬件配置见 [本机底盘](local-chassis.md)；控制参数和生命周期见 [控制接口](control.md)。

## WSL 和键盘

WSL2 可采用 mirrored 网络，或让 WSL 自己连接 VPN，关键是 DDS 公布的地址在两端能够直接收发 UDP。`wslinfo --networking-mode` 可检查当前模式。默认 NAT 与单纯 SSH/TCP 转发不能代替这个条件；参见 [WSL 网络说明](https://learn.microsoft.com/en-us/windows/wsl/networking)。SSH 的端口号不填入 DDS peer；控制流量不经过 SSH。

**WSLg 本地窗口 + DDS**：在 Windows Terminal 的 WSL shell 中设置前面的 DDS 环境变量，运行：

```bash
python -m rsim.apps.keyboard_control --domain 42 \
  --config examples/control/keyboard.example.yaml --input pygame --dry-run
```

pygame 会自行打开 Linux 图形窗口，点击窗口后按 WASD，不需要 xterm。输入与渲染在 WSL，命令通过 DDS 到机器人；窗口内有真实松键和组合键事件。

**SSH X 转发窗口 + 同机 DDS**：在带有 X 服务的客户端上建立转发会话：

```bash
ssh -X -C <机器人SSH地址>
# 在机器人上进入项目根目录，使用装有依赖的环境
python -m rsim.apps.keyboard_control --domain 42 \
  --config examples/control/keyboard.example.yaml --input pygame --window-hz 10 --dry-run
```

此时 Python 与 DDS 客户端运行在机器人上，只有窗口画面和键盘事件走 SSH。`--domain` 与正在运行的服务一致；例如服务仍使用默认domain0，这里也用 `--domain 0`。两者同机时无需设置远端 WSL 的 DDS 网卡地址；若使用前面的单播 profile，应沿用机器人一端的环境变量。X转发需要两端的X11/xauth和服务端允许转发，`DISPLAY` 由SSH设置，不手动改为 `:0`。后端在检测到SSH转发DISPLAY时默认使用SDL的x11驱动；可用 `SDL_VIDEODRIVER=x11` 显式指定。

Windows 的 XLaunch / VcXsrv 也可以作为显示端：启动 X Server，再从已配置 X 转发的 SSH 会话运行上面的命令。它提供显示服务，SDL 仍由运行 Python 一端的 pygame 使用。对于 TCP/SSH DISPLAY，后端通过 pygame 的 SDL 窗口接收事件，在 Surface 上绘制，再用普通 XPutImage 上传变化区域；这样避开部分 SDL 版本跨机尝试 MIT-SHM 时退出的问题，无需修改 X Server 的共享内存配置。转发路径需要 `python-xlib`（已列入 Linux teleop 依赖）及常见的24/32位RGB显示；本地WSLg的Unix显示仍使用常规pygame显示路径。

窗口失焦立即制动、重新聚焦须重新按键；Esc/关闭按钮退出。先保持 `--dry-run` 验证输入，准备实际控制时再去掉。字体和刷新率见 [pygame 面板](control.md#pygame-窗口)。这里的窗口刷新在独立进程，SSH绘制卡顿不会阻塞控制协程；输入超时会停止控制。

无图形界面时仍可用 `--input terminal`。普通终端没有松键事件，按键重复和多键限制见 [键盘输入说明](control.md#终端与-pynput)。

保留 `--input pynput` 供已有桌面工作流使用。它监听X桌面的按键，例如在WSLg的Linux X11图形终端内运行：

```bash
python -m rsim.apps.keyboard_control --domain 42 \
  --config examples/control/keyboard.example.yaml --input pynput --dry-run
```

WSLg 使用 XWayland 时，pynput 的捕获范围限于 X 应用；不能把 Windows Terminal、Windows 浏览器或其他原生 Windows 窗口中的按键当作已被监听。单独设置 `DISPLAY` 或 SSH X 转发也不会改变这一点，见 [pynput 平台限制](https://pynput.readthedocs.io/en/latest/limitations.html)。新窗口控制使用 pygame 即可。

## 发现与连接排查

- 两端检查 domain、服务名、网卡地址和 `CYCLONEDDS_URI`；XML 不是 Fast DDS profile。没有共同网卡路由时，ping 成功不代表 DDS 已公布正确地址。
- 模板 participant index 为自动选择 0–15。示例 domain 42 对应 **UDP 17910–17941**；两端主机、Windows 与 WSL Hyper-V 防火墙需要允许来自对端的这些 UDP 包。仅开放 SSH TCP 端口无效，不需要直接关闭防火墙。其他 domain `D` 的范围是 `7410 + 250D` 至 `7441 + 250D`，参见 [Cyclone DDS 端口配置](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html#cyclonedds-domain-discovery-ports)。
- `status` 连接超时：先看 DDS 发现、地址、UDP 防火墙；`no sufficiently fresh clock sample`：检查延迟、丢包和事件循环停顿，握手要求近期存在往返不超过 50 ms 的样本。
- `another client`：已有控制者占用；先正常退出它。`motion disabled`：服务端未开启非零运动。零速键盘模式下，画面中的模型速度可变化，输出仍为零。

网络耗时消耗控制命令的有效期，默认控制会话期限为 0.3 秒；不要为掩盖丢包而直接拉长期限。跨机 Pose 保留 provider 时间域，不能直接把其单调时间戳当作客户端时钟。短时模拟联通不能代替实机、长时间或建图负载下的验收。

这个配置用于可信 LAN/VPN。当前协议没有客户端身份认证，DDS domain 与服务名不是访问控制。控制消息可以跨机；传感器的同机 mmap/零拷贝通道不因此自动变成跨机器传输。
