# 解耦底盘控制

`rsim.drivers.Chassis` 在驱动环境内运行位姿估计、控制器和 DDS 服务；`rsim.devices.Chassis` 只连接命名服务，不启动 ROS 或串口驱动。后者可放在不同的 Python 环境中，公开 `pose`、`state`、`velocity`、`drive()`、`move()`、`rotate()` 和 `stop()`。

## 启动服务

先用内存模拟底盘验证非零动作：

```bash
python -m rsim.apps.chassis_service --simulate --enable-motion
```

先在硬件主机独立启动驱动，再在服务主机运行[话题接入与里程计](odometry.md)。省略 `--enable-motion` 时服务禁止非零运动；服务不启动电机或传感器，也不修改硬件原有的运动许可：

```bash
python -m rsim.apps.chassis_service --config configs/chassis_topics.yaml
```

也可以直接组装并纳入已有事件循环：

```python
from rsim.runtime import Runtime
from rsim.config import load_chassis
from rsim.drivers import Chassis

robot = load_chassis(config_file, motion_enabled=False, hardware=False)
service = Chassis(robot, name="chassis")
async with Runtime(service):
    await service.wait()
```

独立服务 CLI 默认将 BLAS 线程限制为 1，避免小矩阵运算争抢控制循环；可用 `--blas-threads` 调整。库式组装不修改调用方线程设置。

服务 CLI 在运行时连接失败后默认清理旧会话，并重新订阅已有话题、建立驱动时钟和里程计。重试从1秒退避至最多5秒，`--reconnect-delay` 调整初始间隔，`--no-reconnect` 可选择失败即退出。恢复时使用新的服务代次，旧速度和相对运动不会续跑；客户端须重连。配置解析错误直接报错。此过程不启动或重启硬件，算法重建也不保证位姿历史连续。

所有端使用同一 `--name` 和 `--domain`；默认 `chassis` / `0`。函数中通过 `TransportConfig(domain_id=...)` 选择 domain。连接服务不需要共享配置文件；硬件参数在硬件主机配置，安装标定与控制参数在服务端读取。

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
# 独立控制窗口，适合 WSLg 或 SSH X 转发
python -m rsim.apps.keyboard_control --input pygame --dry-run
```

### pygame 窗口

`--input pygame` 打开一个小窗口，接收真实按下/松开和组合键，不需要 xterm 或全局键盘监听。面板显示 WASD 高亮、连接状态、模型线/角速度、虚拟转向、服务端 Pose，以及 `ZERO OUTPUT` / `LIVE OUTPUT`。速度条是模型指令，不是实测轮速；`--dry-run` 中模型会变化，发给服务的速度始终为零。

窗口获得焦点后按 WASD；空格立即制动，Esc 或关闭窗口退出。失焦、最小化会清空按键并制动，重新聚焦不会恢复此前按住的方向，须松开后重新按键。制动后同样不会因仍按住方向键而自动恢复。窗口使用 [pygame 窗口与键盘事件](https://www.pygame.org/docs/ref/event.html)，无需捕获其他应用中的按键。

字体默认 `Inconsolata,Sarasa Mono SC`，按顺序逐字回退，不使用字体图标；字体需要安装在 **运行 Python 的一端**。可用 `--font "字体一,字体二"` 更改顺序。都不可用时回退 pygame 默认字体，但该字体未必包含中文。`--window-hz` 默认20，范围1–60；X转发可选10以降低绘制频率。只更新变化区域，未初始化音频设备。

GUI 在独立子进程的主线程运行，通过有界非阻塞通道传递按键与显示数据；控制协程和 DDS 留在父进程。输入/模型更新与发送确认分别计时，只保留最新待发送指令，等待消耗其有效期，不积压旧按键。GUI 卡住、退出或输入超过 `max_loop_gap` 没有更新时停止控制，现有命令期限仍生效。GUI 依赖属于 `rsim[teleop]`，也可在已有控制环境内单独安装 `pygame>=2.5`。从函数调用时使用 [PygameKeyboard](../rsim/adapters/pygame_keyboard.py) 与现有 `Teleoperation` 组装，脚本入口需放在 `if __name__ == "__main__":` 下，以支持 spawn。

pygame CLI 默认自动重连：连接超时、指令确认超时或服务重启后，停止续发运动指令，请求零速/释放会话，窗口保留并显示 `RECONNECTING`。临时输入停顿同样停止控制，等待输入恢复后重连；不会用旧输入维持运动。重试间隔从1秒逐步增加到最多5秒；`--reconnect-delay` 可设置初始间隔（0.1–5秒）。每次重建独立 DDS 客户端、时钟与控制会话；取得新位姿并确认零速后才显示已连接。**重连不会恢复旧速度或旧按键，须松开按键后重新按下才会运动。** 网络不可达时，底盘原有指令 TTL/deadman 负责停止，不因重试延长有效期，也不自动启动硬件或服务。窗口始终可用 Escape、关窗或 Ctrl-C 退出；窗口进程死亡仍需重新启动客户端。

客户端日志记录启动模式、显示后端、窗口子进程 PID/退出码、重连次数及最终退出原因，区分 Escape、窗口关闭、Ctrl-C 与异常。每次连接失败保留完整堆栈；窗口自身故障等不可恢复错误以退出码1结束。窗口输入超时记录实际年龄与阈值，指令确认记录耗时和剩余 TTL。通过 `start_remote.sh` 启动时，日志保存在客户端机器的根 `assets/bringup/remote/`。`Connected` 只说明当次连接就绪，排查中断应查看 `Control connection lost` 及其异常链；资源清理不会被视为用户主动关闭。自动重连属于 pygame 应用层，库式 `rsim.devices.Chassis` 仍保留单会话失败语义。

WSLg 本地启动和 SSH X 转发命令见 [远程控制](remote-control.md#wsl-和键盘)。Notebook 原有面板继续使用 pynput；独立 pygame 窗口请使用 CLI。

### 终端与 pynput

终端模式关闭字符回显和行缓冲，不需回车，状态行显示 `keys`、`v`、`yaw` 和输出模式。退出或异常清理时恢复终端设置；输入断开时制动并退出。还可按 Q 退出。如果 SSH 没有分配交互终端，使用 `ssh -t`。

普通终端只传字符，没有真实的松键事件。终端模式将字符作为短时输入，依靠系统按键重复续期；`--key-timeout` 默认 0.18 秒（范围 0.05–0.5），到期后进入原有摩擦减速过程。首次长按的重复延迟可能造成短暂间断，也不能准确恢复任意多键同时按住；同轴的新方向字符覆盖旧方向，空格清除输入并制动，下一次方向字符恢复控制。pynput 模式保留完整按下/松开状态，相反键同时按下抵消输入。

模型分别积分线速度和角速度：

\[
\dot v=u a-\operatorname{sign}(v)(f+d v^2),\qquad
v_{\max}=\sqrt{(a-f)/d}.
\]

其中 `acceleration` 为最大输入加速度，`friction` 为固定减速度，`drag` 为二次阻力系数；角速度使用对应的 `angular_*` 参数。静止时摩擦不会让车辆自发反向。默认平衡线速度为 0.2 m/s，原地角速度上限为 0.4 rad/s，角加速度输入为 1.0 rad/s²（扣除摩擦与阻力后是实际模型加速度）。话题服务模板对应限制为 0.2 / 0.4，硬件环境模板通过原生参数设置 0.25 / 0.5。已有本地配置不会自动更新；必须核对客户端、服务和硬件三层，原生驱动参数需重启对应驱动才生效。

虚拟转向角上限随速度缩小：`δmax = steering_max / (1 + (v / steering_speed_scale)²)`。角速度限制是 `|v| / wheelbase × tan(δmax)` 加上随线速度快速衰减的原地转向项，再受平衡角速度约束。`pivot_rate` 与 `pivot_transition_speed` 控制原地转向及其过渡。它是可调的运动手感模型，不是轮胎动力学或避障规划器。面板中的 steering 是归一化的虚拟角度，不代表差速底盘有实体转向轮。

`hz` 控制积分/发令频率；`command_ttl` 是单条速度有效期；`max_loop_gap` 限制循环停顿。异常停顿、监听器退出、输入陈旧都会停止发令并清理。pynput 回调只将按键投递到 asyncio 循环，控制计算和通信由协程 task 完成；参见 [pynput 监听文档](https://pynput.readthedocs.io/en/latest/keyboard.html#monitoring-the-keyboard)。

pynput 需要客户端进程可访问的桌面会话，捕获范围是该桌面而非 SSH 字符输入；设置 X 转发不等于监听本地终端的按键，见 [平台限制](https://pynput.readthedocs.io/en/latest/limitations.html)。普通 SSH 使用上面的 terminal 模式。无 ROS 环境仍需安装 `rsim[teleop]` 和项目使用的 graphmap 库。

交互版见 [键盘 Notebook](../examples/control/keyboard.ipynb)，使用相同 pynput 来源，界面异步显示速度、转向限制与零速模式；按键来自 kernel 的桌面，不来自浏览器 DOM。需要 `rsim[notebook]`。指令式 Notebook 见 [commands.ipynb](../examples/control/commands.ipynb)。

## 生命周期与协议边界

服务使用 DDS JSON 请求/响应与位姿遥测，不发送 Python 对象或 pickle。服务端校验启动代次、会话、递增序号、速度限值与截止时间；重复请求 ID 返回缓存结果，避免动作重复执行。同一用户同一主机/domain/name 的文件锁避免重复服务，物理串口另有设备锁。多个客户端可同时订阅，控制权仅属于一个带期限的会话。

客户端以 20 Hz 续租，默认会话期限 0.3 秒；服务端以 100 Hz 检查。手动速度还有独立命令期限，续租不会刷新旧速度。进程被杀或循环停顿时，服务端取消动作；原生 STM32 串口线程继续独立检查命令期限。服务端失败也取消内部动作。正常退出请求零速，硬断线、主机冻结仍依赖 MCU 通信超时保护，见 [原生驱动边界](local-chassis.md)。

握手使用请求往返期间的单调时钟构造保守转换界限，网络耗时消耗 TTL。往返超过 50 ms 的时钟样本不会覆盖近期有效样本；连续 3 秒没有合格样本则停止服务连接。重启服务会使旧客户端失败，库式调用方需重建连接；pygame 应用按上文自动重建，旧会话不会恢复运动。位姿按真实源帧去重，保留时间域与历史，不用重复遥测伪造新位姿。

控制和小型位姿消息不依赖同机共享内存，可在 DDS 发现可达、同 domain 的其他主机连接；跨机时钟漂移、网络配置和负载条件需按部署环境验收。DDS 控制应置于可信网络；原型没有客户端身份认证。服务最多保留 1024 个会话的序号历史、64 个动作结果，达到会话限额会拒绝新会话。
