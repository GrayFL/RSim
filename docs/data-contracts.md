# 数据与控制契约

`rsim.core.contracts` 提供现有 dictionary payload 的 TypedDict 和验证函数，不替换
wire codec，不要求应用引入新的消息类。适配器应在边界验证测试样例；高频路径不
需要对每帧重复做矩阵特征值检查。`rsim.devices.ports.ChassisPorts` 是 pose、velocity、
state 端口的轻量组合，应用不依赖底盘的具体来源。

| 数据 | 必须明确的含义 |
|---|---|
| pose | 米、单位 xyzw 四元数、父/子坐标系；估计与 ground truth 区分 |
| velocity | 车体 +X，m/s；绕 +Z，rad/s；有限数值 |
| IMU | rad/s、m/s²、传感器 frame；重力是否已去除由显式 metadata/配置声明 |
| Odometry | pose 位于 header 父系，twist 位于 child 系；二维投影由算法承担 |
| LaserScan | 米/弧度、束数与角度一致、非负采样时间；强度可为空 |
| Image | encoding 与 shape/dtype 一致；RGB/BGR 不隐式交换 |
| Depth | 16UC1 为 mm、32FC1 为 m；optical_z 或 ray_distance 显式声明 |
| CameraInfo | 图像尺寸、optical frame 与 K 对应，同一次采样/标定版本配对 |

协方差为有限、按行展开的对称半正定矩阵。全零表示未知，不表示零误差；IMU
协方差首元素 -1 表示对应量不可用，此时不把占位 orientation 当测量。NaN、Inf
及超出有效量程的扫描返回值保留，消费者按契约过滤，不统一改成零或最大量程。

```python
from rsim.core.contracts import validate_imu, validate_odometry, validate_image

validate_imu(imu.data, gravity_removed=imu.metadata['gravity_removed'])
validate_odometry(odom.data, pose_kind='estimate', velocity_frame='base_footprint')
validate_image(depth.data, camera_info=info.data, depth_unit='m', depth_type='optical_z')
```

`Frame.stamp_ns / clock` 表示采样时钟，`received_ns` 表示原始主机接收时间。组合、
估计和重发布不能靠输出频率刷新过期输入的接收时间。命令 deadline 使用 provider
所在主机 monotonic 时钟，不能未经转换跨主机使用；provider 在最终应用点复验。
MuJoCo 的模拟时间和 speedup 不延长命令 TTL，物理线程暂停后恢复也先检查主机时间。

MujocoSim 项目侧绑定的 truth 输出保持 ground truth 标记；轮速里程计为 `wheel_odometry` 来源，
`PlanarOdometry.pose` 为 estimate。模拟器 reset 更新 `episode`，保持发布标识与
时间进展可辨识。现有平面估计器遇到 episode 改变明确失效，要求重建 Runtime，
不会把跳变后的真值隐式接到旧轨迹。地图 revision 只描述地图版本，不能代替
SampleId、时钟或仿真 episode。

内存模拟的应用契约见 `test_backend_contracts.py`；具体 MuJoCo 机器人与传感器
验证归 MujocoSim。RSim adapter 只测试协议桥接，不固定模拟构型或物理容差。
命令过期、provider 输出 fallback、设备反馈停止是三件事；软件仿真只验证前两项
及其模拟反馈，真实制动仍须在具备设备的安全环境单独验证。
