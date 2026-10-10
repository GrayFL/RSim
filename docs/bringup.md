# 独立启动硬件、服务和客户端

每个脚本只管理**当前机器**上的一个职责。手动登录到目标机器，按需启动；脚本没有 SSH 调度，也不会为了启动应用自动启动硬件。硬件持续发布 ROS2 话题，多个应用可以独立订阅。关闭控制器或建图服务，不关闭硬件。

| 在哪里运行 | 脚本 | 启动内容 |
| --- | --- | --- |
| 底盘硬件主机 | `examples/control/start_hardware.sh` | tmux 的 stm32、imu、bluesea 三个独立窗口，只运行原生 ROS2 驱动 |
| 控制服务主机 | `examples/control/start_chassis.sh` | 已有 ROS 话题的 ROS2Topic 中继、EKF、可选 ICP、controller 与 DDS 控制接口 |
| 键盘客户端主机 | `examples/control/start_remote.sh` | pygame 窗口，连接已有 controller |
| 相机所在主机 | `examples/camera/start_camera.sh` | `python -m rsim.drivers d435` |
| 3D 雷达所在主机 | `examples/lidar/start_lidar.sh` | `python -m rsim.drivers robin` |
| 外挂 IMU 所在主机 | `examples/imu/start_imu.sh` | `python -m rsim.drivers imu` |
| 建图服务主机 | `examples/mapping/start_mapping.sh` | 已有话题的时间对齐、Super-LIO、RTAB-Map 与地图接口 |
| 可视化客户端主机 | `examples/ros2/start_viewers.sh` | 独立 rqt、rviz 窗口，仅订阅 ROS |

`examples/_launch.sh` 只封装本地 tmux、环境激活和日志。每个角色的命令直接写在对应 sh 中；本机配置放 `configs/<角色>.env`，从对应的 `.env.example` 复制填写。可用 `RSIM_ENV_FILE` 指定其它文件。这些是 Bash 文件，可写 `export` 和 argv 数组；只加载自己维护的配置。

所有入口支持 `start`（tmux 后台）、`run`（前台）、`plan`（查看命令）、`status`、`attach`、`stop`。`start` 只代表创建进程，应通过状态和数据确认就绪。日志放根目录 `assets/bringup/`；已有窗口不覆盖，stop 发送 Ctrl-C，等待退出。硬件窗口相互独立，一个设备退出不连带停止其它设备。停止应用与停止硬件是两次独立操作。

## 底盘硬件主机

复制 `examples/control/hardware.env.example` 到 `configs/hardware.env`，填写本机环境激活方式、稳定串口路径和 ROS 网络设置。仅 IMU 接收波特率设为460800；200Hz由设备本身配置，脚本不发送 AT 设置命令。

```bash
# 三个窗口：stm32 / imu / bluesea
bash examples/control/start_hardware.sh start

# 或只启动所需的两个设备
bash examples/control/start_hardware.sh start stm32 imu
# 后续独立补开雷达
bash examples/control/start_hardware.sh start bluesea

bash examples/control/start_hardware.sh status
# 只停止雷达，另外两个窗口继续运行
bash examples/control/start_hardware.sh stop bluesea
```

前两条是不同的启动选择。默认 `RSIM_HARDWARE_MOTION=false`，STM32 仅接受零速。准备实际控制时，在硬件主机环境文件中设置 true，单独停止并重新启动 stm32 窗口。`STM32_ROS_ARGS`、`IMU_ROS_ARGS`、`BLUESEA_ROS_ARGS` 是原生 ROS 参数数组，可传 `--params-file`、`-p`、`-r`；参数改变后只重启相应设备。

## 控制服务主机

复制 `examples/control/chassis.env.example` 到 `configs/chassis.env`，复制 `chassis_topics.example.yaml` 到 `configs/chassis_topics.yaml`。这里填写已有话题、安装 Pose 和算法参数，**不填写串口或硬件启动选项**。

```bash
bash examples/control/start_chassis.sh start
bash examples/control/start_chassis.sh status
bash examples/control/start_chassis.sh stop
# 硬件仍运行；准备实际控制时独立开启 controller 的运动许可
bash examples/control/start_chassis.sh start --enable-motion
```

服务使用 ROS2Topic 接收 IMU、轮式 odom 和诊断，以 RSim Signal 组装里程计；速度命令通过已有 STM32 的 ROS 服务发送。两端需编译相同版本的 `rsim_stm32` 接口。跨主机先通过 Clock 服务建立单调时钟下界，再把剩余有效期转换到驱动时钟；传输耗时不延长有效期。驱动重启后旧实例命令被拒绝，需重新连接。硬件线程自己的 deadman 和反馈超时仍在底盘执行。

`wheel_imu` 不依赖扫描；使用 `scan_imu` 时在 YAML 加上 scan、scan_mount 和实测车体角区过滤，同时自行启动 BlueSea。所有 ROS 采样主机的系统时间应同步；命令时钟握手不替代 IMU/odom 的采样时钟同步。

服务入口会拒绝 `start_driver: true` 和会启动硬件的旧标定配方。独立的硬件 provider API 仍可在硬件侧使用；应用脚本没有硬件回退路径，缺少输入时等待/报错，不替用户打开设备。

## 键盘客户端主机

复制 `examples/control/remote.env.example` 到 `configs/remote.env`。DDS 对端填**控制服务主机**，不填硬件主机。控制 domain 默认42，硬件 ROS domain 默认0，两者独立。网卡、静态 peer、已有 NAT 映射的配置见[远程控制](remote-control.md)。

```bash
bash examples/control/start_remote.sh run command status
bash examples/control/start_remote.sh start        # 默认零速窗口
bash examples/control/start_remote.sh stop
bash examples/control/start_remote.sh start --live
```

实际运动需要硬件、controller、键盘三处都显式允许。窗口有焦点时 WASD 控制，空格制动，失焦停车，Esc退出。WSLg 使用本地显示环境，SSH X 转发沿用 SSH 的 DISPLAY；脚本把显示变量传给该 tmux 窗口，不修改全局显示设置。SSH X 转发窗口仍依赖 SSH 连接。

### 能连接但机器人不动

先区分每个进程实际读取的文件，修改后重启对应进程：

| 进程 | 本机配置 | 实际运动的必要设置 |
| --- | --- | --- |
| STM32 硬件驱动 | `configs/hardware.env` | `RSIM_HARDWARE_MOTION=true`，重启 stm32 窗口 |
| controller 服务 | `configs/chassis.env` | 启动时传 `--enable-motion` |
| pygame 客户端 | `configs/remote.env` | 启动脚本传 `--live`，窗口显示 `LIVE OUTPUT` |

服务端的 DDS 地址在 `chassis.env`；修改同机的 `remote.env` 只影响客户端。DDS 两端的 LOCAL/PEER 应分别指向各自本机与对端的可达地址，并使用相同控制 domain。ping 成功后，用 `start_remote.sh run command status` 验证 DDS 位姿返回，再用 `run command move 0` 验证零速控制链路。

服务日志的 `motion_enabled=true` 只表示 controller 允许运动，不能替代硬件许可。客户端 `state` 的 `motion_enabled` 和 `hardware.motion_enabled` 分别反映两层状态；硬件诊断还包含急停、碰撞与电机错误。STM32 重启后 controller CLI 会重新连接驱动实例，pygame 随之重建控制会话；库式使用需由调用方显式重建。

## 相机、3D 雷达与建图服务

在各硬件所在机器分别复制 camera/lidar/imu 的环境模板到 `configs/`，按需执行：

```bash
bash examples/camera/start_camera.sh start
bash examples/lidar/start_lidar.sh start
# 只在使用外挂 IMU 时启动它
bash examples/imu/start_imu.sh start
```

每个脚本对应一个硬件 provider，支持在命令末尾传原生 `--ros-args`。它们既发布 ROS2 话题，也可供同机 `rsim.devices` 客户端读取共享数据；跨机器 ROS 传输不等于跨机器 mmap 零拷贝。

在建图服务主机复制 `examples/mapping/mapping.env.example` 和 `mapping_topics.example.yaml`，填写已有话题、实测安装 Pose、激光时间偏移：

```bash
bash examples/mapping/start_mapping.sh start
bash examples/mapping/start_mapping.sh stop
```

该服务不打开相机、雷达或串口，也不启动底盘 controller。它仅启动建图算法，每次在根 `assets/mapping/` 创建独立数据库目录。默认订阅 `/iv_points`、`/rsim/d435/color/*` 和底盘 IMU/odom；可在 YAML 的 topics 改名。相机发布的 TF 必须与 body_camera 的子坐标系一致。硬件参数在硬件脚本配置；建图服务只配置算法。

最后在可视化主机配置原生 ROS 网络和 `configs/viewers.env`：

```bash
bash examples/ros2/start_viewers.sh start rqt rviz
```

rqt/rviz 通过 ROS DDS 订阅数据，使用 ROS 的 RMW profile；pygame 使用 RSim 控制连接的 Cyclone profile。两个配置不能互相替代。可视化窗口不启动建图或硬件。

带宽有限时，在数据源主机单独启动 `examples/ros2/start_preview.sh`，再让监看主机选择预览 domain 和 RViz 配置，见[点云预览示例](../examples/ros2/README.md#有限带宽点云预览)。硬件、预览、查看器仍分别启动，控制连接不随查看器开启。

## 单独使用 ROS2Topic 中继

需要在同机无 ROS 环境读取已有话题时，可独立运行 `examples/ros2/start_relays.sh`，配置见该目录的模板。控制服务已经创建自己的中继，不要求先运行它。相同 topic/kind/history/transport 的多个订阅共用现有中继；它们只订阅原生发布者，不获得硬件启动权。provider 租约可能由其它客户端持有，硬件维护时应先关闭相应客户端。
