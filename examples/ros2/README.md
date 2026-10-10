# 已有 ROS2 话题与可视化

`start_relays.sh` 只订阅已有 ROS2 话题并建立本机共享中继，配置为 `configs/relays.env` 与 `configs/relays.yaml`，参考同目录模板。它不打开硬件。

`start_viewers.sh start rqt rviz` 在本机 tmux 的两个窗口分别打开可视化程序，环境配置为 `configs/viewers.env`。先由用户独立启动数据源；可视化脚本不启动设备或服务。

底盘硬件和控制服务的独立启动顺序见[启动说明](../../docs/bringup.md)。

## 有限带宽点云预览

在数据源主机将 `preview.env.example` 复制到 `configs/preview.env`，配置环境和 ROS 网络后运行：

```bash
bash examples/ros2/start_preview.sh start
```

它只订阅已有 `/iv_points`，默认从硬件 domain 0 向预览 domain 43 发布 `/rsim/preview/points`，不启动雷达。每帧均匀抽样至最多1200点、60000字节，最多3 Hz；保留原始字段、时间戳与坐标系，历史深度1，不重发旧帧。原始点云不变；抽样预览不用于建图。

监看主机在 `configs/viewers.env` 设置 `ROS_DOMAIN_ID=43`、本机可达的 Fast DDS profile，以及：

```bash
RVIZ_ARGS=(-d "$RSIM_ROOT/examples/ros2/preview.rviz")
```

然后独立运行 `bash examples/ros2/start_viewers.sh start rviz`。模板固定坐标系为雷达默认 `seyond`，其他数据源须改成实际消息的 `header.frame_id`；仅点云预览不需要跨域转发 TF。RViz 设置 Best Effort、Keep Last 1。预览应用可通过参数改输入/输出 topic、domain、帧率和点数；两 domain 必须不同。原始 ROS 和 RSim 控制的 domain 不必调整。

修改已有监看环境后须关闭旧查看器再启动；一个旧 domain 的窗口不会随配置文件修改而迁移。完整地图或其他话题的监看使用各自的 domain 配置，预览不会自动复制所有 ROS 话题。网络带宽与网卡配置见[远程控制说明](../../docs/remote-control.md#大点云与控制共用网络)。
