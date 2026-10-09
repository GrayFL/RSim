# 底盘控制示例

完整接口与参数说明见 [控制文档](../../docs/control.md)。

原生 ROS 底盘使用 [native_chassis.example.yaml](native_chassis.example.yaml) 组装轮速＋完整姿态 IMU，可选择加入 2D ICP；[native_chassis.ipynb](native_chassis.ipynb) 提供驱动侧零速示例，详见 [里程计说明](../../docs/odometry.md)。内部直接使用 ROS 信号，最终服务与以下客户端接口一致。

跨机器客户端使用 [DDS 配置模板](cyclonedds.remote.xml)；双端环境变量、WSL 输入与端口配置见 [远程控制说明](../../docs/remote-control.md)。

1. 在驱动环境执行 `python -m rsim.apps.chassis_service --simulate --enable-motion`。
2. 将 [keyboard.example.yaml](keyboard.example.yaml) 复制到根目录 `configs/keyboard.yaml`。
3. 在客户端环境执行 `python -m rsim.apps.keyboard_control`，或打开 [键盘面板](keyboard.ipynb)。
4. [指令客户端](commands.ipynb) 演示跨环境的 `move` / `rotate` 与 Pose 历史访问。

独立小窗口可直接运行 `python -m rsim.apps.keyboard_control --input pygame --config examples/control/keyboard.example.yaml --dry-run`。支持 WSLg 或 SSH X 转发、真实组合键、失焦制动，并显示模型速度、转向和 Pose。字体默认 `Inconsolata,Sarasa Mono SC`；见 [pygame 面板说明](../../docs/control.md#pygame-窗口)。

SSH 中 CLI 自动使用终端输入，也可显式指定 `--input terminal`。不需回车，空格制动，Esc/Q/Ctrl-C 退出。终端没有松键事件，默认按字符重复和 0.18 秒期限推断松开；完整多键按下/松开使用桌面上的 `--input pynput`。Notebook 面板仍使用 kernel 桌面的 pynput。

接实机时，用 `--config configs/local_chassis.yaml` 替换 `--simulate`；省略 `--enable-motion` 并使用客户端 `--dry-run` 可只发零速。

[local_chassis.ipynb](local_chassis.ipynb) 用于驱动侧标定、组装与直接零速检查，配置模板是 [local_chassis.example.yaml](local_chassis.example.yaml)。对应脚本：`python -m examples.control.local_chassis configs/local_chassis.yaml`。

[chassis_motion.ipynb](chassis_motion.ipynb) 保留本地 EKF/控制算法演示；可复用的模拟底盘在 `rsim.components.simulated_chassis`，硬件组装在 `rsim.drivers.local_chassis`，配置入口在 `rsim.config.local_chassis`。
