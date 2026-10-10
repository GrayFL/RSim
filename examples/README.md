# 示例索引

示例按演示主题组织；公共设备组装、控制和模拟实现位于 `rsim/`。从仓库根目录运行 `python -m ...`，Notebook 可从各自目录打开。生成物统一写入根目录 `assets/`。

设备与应用使用各自独立的[本机 tmux 脚本](../docs/bringup.md)：底盘硬件 `control/start_hardware.sh`，控制服务 `control/start_chassis.sh`，键盘 `control/start_remote.sh`；相机、雷达、外挂 IMU、建图、ROS 查看器分别位于对应目录。环境配置保存在各机根 `configs/`，应用不会自动启动硬件。

| 目录 | 内容与入口 |
| --- | --- |
| [control](control/README.md) | 解耦服务、键盘、指令客户端与本机标定 |
| components | [Notebook](components/components.ipynb)，`python -m examples.components.components` |
| mapping | [Notebook](mapping/mapping.ipynb)，`python -m examples.mapping.mapping --help` |
| imu | [Notebook](imu/imu_rig.ipynb)，`python -m examples.imu.imu_rig --help` |
| camera | [D435](camera/d435.ipynb)、[无 ROS 客户端](camera/dds_client.ipynb)、UVC 脚本 |
| lidar | 直接读取、进程计算、相机/雷达组合演示 |
| ros1 | `python -m examples.ros1.chassis --help` |
| prototype | [早期实验 Notebook](prototype/prototype.ipynb) |
