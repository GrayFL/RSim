# 传感器组合示例

Notebook 演示单进程、协程与子进程组合。实际部署按设备和应用分开启动，见[独立启动说明](../../docs/bringup.md)：相机在 `../camera/start_camera.sh`，3D 雷达在 `../lidar/start_lidar.sh`，外挂 IMU 在 `../imu/start_imu.sh`。这些入口各自只启动一个设备，不自动启动其它硬件或应用。
