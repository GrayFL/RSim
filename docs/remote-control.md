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

模板默认只作用于 domain 42，避免改变其他 domain 的 DDS 配置。`RSIM_DDS_DOMAIN` 只改变 XML 适用范围，**不代替 CLI 的 `--domain`**；两端两处必须一致。不要为此修改硬件 ROS 的 `ROS_DOMAIN_ID`。服务名两端也必须一致，示例均为默认的 `chassis`。

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
  --config examples/control/keyboard.example.yaml --input terminal --dry-run
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

在 Windows Terminal 的 WSL shell 中运行客户端时，使用 `--input terminal`。客户端确实在 WSL 上运行，由 DDS 直连机器人；这个模式不需要再 SSH 到机器人。但普通终端仍没有松键事件，按键重复和多键限制见 [键盘输入说明](control.md#键盘与车辆模拟)。

若需要 pynput 的真实按下/松开与组合键，在 WSLg 的 Linux X11 图形终端（例如 xterm）内运行下面命令，并让该 Linux 窗口获得焦点：

```bash
python -m rsim.apps.keyboard_control --domain 42 \
  --config examples/control/keyboard.example.yaml --input pynput --dry-run
```

从已设置上述 DDS 环境的本地 WSL shell 启动 `xterm`，新窗口会继承网络配置，但仍需使用装有项目依赖的 Python 环境。若未安装 xterm，可由发行版包管理器安装。首次保持 `--dry-run`，观察 `keys` 是否随着按下/松开更新。WSLg 使用 XWayland 时，pynput 的捕获范围限于 X 应用；不能把 Windows Terminal、Windows 浏览器或其他原生 Windows 窗口中的按键当作已被监听。单独设置 `DISPLAY` 或 SSH X 转发也不会改变这一点，见 [pynput 平台限制](https://pynput.readthedocs.io/en/latest/limitations.html)。

## 发现与连接排查

- 两端检查 domain、服务名、网卡地址和 `CYCLONEDDS_URI`；XML 不是 Fast DDS profile。没有共同网卡路由时，ping 成功不代表 DDS 已公布正确地址。
- 模板 participant index 为自动选择 0–15。默认 domain 42 对应 **UDP 17910–17941**；两端主机、Windows 与 WSL Hyper-V 防火墙需要允许来自对端的这些 UDP 包。仅开放 SSH TCP 端口无效，不需要直接关闭防火墙。其他 domain `D` 的范围是 `7410 + 250D` 至 `7441 + 250D`，参见 [Cyclone DDS 端口配置](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html#cyclonedds-domain-discovery-ports)。
- `status` 连接超时：先看 DDS 发现、地址、UDP 防火墙；`no sufficiently fresh clock sample`：检查延迟、丢包和事件循环停顿，握手要求近期存在往返不超过 50 ms 的样本。
- `another client`：已有控制者占用；先正常退出它。`motion disabled`：服务端未开启非零运动。零速键盘模式下，画面中的模型速度可变化，输出仍为零。

网络耗时消耗控制命令的有效期，默认控制会话期限为 0.3 秒；不要为掩盖丢包而直接拉长期限。跨机 Pose 保留 provider 时间域，不能直接把其单调时间戳当作客户端时钟。短时模拟联通不能代替实机、长时间或建图负载下的验收。

这个配置用于可信 LAN/VPN。当前协议没有客户端身份认证，DDS domain 与服务名不是访问控制。控制消息可以跨机；传感器的同机 mmap/零拷贝通道不因此自动变成跨机器传输。
