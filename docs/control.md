# 解耦底盘控制

`rsim.drivers.Chassis` 在驱动环境内运行位姿估计、控制器和 DDS 服务；`rsim.devices.Chassis` 只连接命名服务，不启动 ROS 或串口驱动。后者可放在不同的 Python 环境中，公开 `pose`、`state`、`velocity`、`drive()`、`move()`、`rotate()` 和 `stop()`。

## 启动服务

先用内存模拟底盘验证非零动作：

```bash
python -m rsim.apps.chassis_service --simulate --enable-motion
```

实机使用 [本机底盘配置](local-chassis.md)。省略 `--enable-motion` 时，控制器和原生电机驱动都禁止非零运动：

```bash
python -m rsim.apps.chassis_service --config configs/local_chassis.yaml
```

也可以直接组装并纳入已有事件循环：

```python
from rsim.runtime import Runtime
from rsim.config.local_chassis import load_local_chassis
from rsim.drivers import Chassis

robot = load_local_chassis(config_file, motion_enabled=False)
service = Chassis(robot, name="chassis")
async with Runtime(service):
    await service.wait()
```

独立服务 CLI 默认将 BLAS 线程限制为 1，避免小矩阵运算争抢控制循环；可用 `--blas-threads` 调整。库式组装不修改调用方线程设置。

所有端使用同一 `--name` 和 `--domain`；默认 `chassis` / `0`。函数中通过 `TransportConfig(domain_id=...)` 选择 domain。连接服务不需要共享配置文件；硬件参数与标定仅在服务端读取。

跨机器使用同一客户端，网卡选择、静态发现、端口和 WSL 键盘设置见 [远程控制配置](remote-control.md)。

## 指令客户端

```python
from rsim.devices import Chassis
from rsim.runtime import Runtime

chassis = Chassis("chassis")
async with Runtime(chassis):
    frame = await chassis.pose.get(timeout=5)
    historical = await chassis.pose.get(timestamp_ns=frame.stamp_ns, clock=frame.clock)
    pose = await chassis.move(distance_m=0.1)
    await chassis.move(-0.1)
    await chassis.rotate(yaw_deg=20)
    await chassis.rotate(yaw_rad=-0.3490658504)
```

返回值是带坐标系标签的 `graphmap.pose.Pose`；距离沿起始底盘 X 轴计量，正向前，角度正向左。每次旋转恰好指定一个角度单位。闭环和相对目标保留在服务端；客户端取消、超时或断开时结束对应动作并请求停车。可用 `timeout=` 设置动作等待期限。

```bash
python -m rsim.apps.chassis_command status
python -m rsim.apps.chassis_command move 0
python -m rsim.apps.chassis_command rotate --deg 0
```

`stop()` 释放**当前客户端自己的**控制会话；另一个客户端不能通过它抢占或中断已有所有者。命令行每次创建新会话，因此 `stop` 子命令只用于确认当前是否空闲，不是全局急停。现场急停由硬件负责。

## 键盘与车辆模拟

准备本地参数：

```bash
mkdir -p configs
cp examples/control/keyboard.example.yaml configs/keyboard.yaml
python -m rsim.apps.keyboard_control --config configs/keyboard.yaml
# 实机零速联调：模型响应输入，发给服务的速度始终为零
python -m rsim.apps.keyboard_control --config configs/keyboard.yaml --dry-run
```

W/S 施加正/负线加速度，A/D 施加正/负角加速度。松键后速度逐渐衰减；空格立即制动，Esc 或 Ctrl-C 结束。倒车时 A 仍表示左转的正角速度，这是差速机器人的角速度约定。

CLI 默认 `--input auto`：SSH 会话优先使用终端输入，即使设置了转发的 `DISPLAY`；本地 X 桌面使用 pynput。也可显式选择：

```bash
# 在 SSH 交互终端运行，不需要 X 转发
python -m rsim.apps.keyboard_control --input terminal
# 在键盘所在的桌面会话运行
python -m rsim.apps.keyboard_control --input pynput
```

终端模式关闭字符回显和行缓冲，不需回车，状态行显示 `keys`、`v`、`yaw` 和输出模式。退出或异常清理时恢复终端设置；输入断开时制动并退出。还可按 Q 退出。如果 SSH 没有分配交互终端，使用 `ssh -t`。

普通终端只传字符，没有真实的松键事件。终端模式将字符作为短时输入，依靠系统按键重复续期；`--key-timeout` 默认 0.18 秒（范围 0.05–0.5），到期后进入原有摩擦减速过程。首次长按的重复延迟可能造成短暂间断，也不能准确恢复任意多键同时按住；同轴的新方向字符覆盖旧方向，空格清除输入并制动，下一次方向字符恢复控制。pynput 模式保留完整按下/松开状态，相反键同时按下抵消输入。

模型分别积分线速度和角速度：

\[
\dot v=u a-\operatorname{sign}(v)(f+d v^2),\qquad
v_{\max}=\sqrt{(a-f)/d}.
\]

其中 `acceleration` 为最大输入加速度，`friction` 为固定减速度，`drag` 为二次阻力系数；角速度使用对应的 `angular_*` 参数。静止时摩擦不会让车辆自发反向。默认平衡线速度为 0.1 m/s，平衡角速度约 0.15 rad/s。服务端还独立限制最大速度，修改模拟参数时应使推导上限落在服务端范围内。

虚拟转向角上限随速度缩小：`δmax = steering_max / (1 + (v / steering_speed_scale)²)`。角速度限制是 `|v| / wheelbase × tan(δmax)` 加上随线速度快速衰减的原地转向项，再受平衡角速度约束。`pivot_rate` 与 `pivot_transition_speed` 控制原地转向及其过渡。它是可调的运动手感模型，不是轮胎动力学或避障规划器。面板中的 steering 是归一化的虚拟角度，不代表差速底盘有实体转向轮。

`hz` 控制积分/发令频率；`command_ttl` 是单条速度有效期；`max_loop_gap` 限制循环停顿。异常停顿、监听器退出、输入陈旧都会停止发令并清理。pynput 回调只将按键投递到 asyncio 循环，控制计算和通信由协程 task 完成；参见 [pynput 监听文档](https://pynput.readthedocs.io/en/latest/keyboard.html#monitoring-the-keyboard)。

pynput 需要客户端进程可访问的桌面会话，捕获范围是该桌面而非 SSH 字符输入；设置 X 转发不等于监听本地终端的按键，见 [平台限制](https://pynput.readthedocs.io/en/latest/limitations.html)。普通 SSH 使用上面的 terminal 模式。无 ROS 环境仍需安装 `rsim[teleop]` 和项目使用的 graphmap 库。

交互版见 [键盘 Notebook](../examples/control/keyboard.ipynb)，使用相同 pynput 来源，界面异步显示速度、转向限制与零速模式；按键来自 kernel 的桌面，不来自浏览器 DOM。需要 `rsim[notebook]`。指令式 Notebook 见 [commands.ipynb](../examples/control/commands.ipynb)。

## 生命周期与协议边界

服务使用 DDS JSON 请求/响应与位姿遥测，不发送 Python 对象或 pickle。服务端校验启动代次、会话、递增序号、速度限值与截止时间；重复请求 ID 返回缓存结果，避免动作重复执行。同一用户同一主机/domain/name 的文件锁避免重复服务，物理串口另有设备锁。多个客户端可同时订阅，控制权仅属于一个带期限的会话。

客户端以 20 Hz 续租，默认会话期限 0.3 秒；服务端以 100 Hz 检查。手动速度还有独立命令期限，续租不会刷新旧速度。进程被杀或循环停顿时，服务端取消动作；原生 STM32 串口线程继续独立检查命令期限。服务端失败也取消内部动作。正常退出请求零速，硬断线、主机冻结仍依赖 MCU 通信超时保护，见 [原生驱动边界](local-chassis.md)。

握手使用请求往返期间的单调时钟构造保守转换界限，网络耗时消耗 TTL。往返超过 50 ms 的时钟样本不会覆盖近期有效样本；连续 3 秒没有合格样本则停止服务连接。重启服务会使旧客户端失败，需要显式重新连接；旧会话不会自动恢复运动。位姿按真实源帧去重，保留时间域与历史，不用重复遥测伪造新位姿。

控制和小型位姿消息不依赖同机共享内存，可在 DDS 发现可达、同 domain 的其他主机连接；跨机时钟漂移、网络配置和负载条件需按部署环境验收。DDS 控制应置于可信网络；原型没有客户端身份认证。服务最多保留 1024 个会话的序号历史、64 个动作结果，达到会话限额会拒绝新会话。
