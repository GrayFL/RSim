# 底盘位姿融合与相对运动

`PlanarOdometry` 是输出 `pose` / `estimate` 的 Component；`ChassisController` 将它与速度端口组合，提供可 `await` 的相对运动。几何类型统一采用 **graphmap Pose v3**，使用前从 graphmap 项目安装该库及其声明的 NumPy/SciPy 依赖。RSim 的可选依赖组为 `motion`；仅使用其他传感器时不加载 graphmap。

## 接口

```python
from graphmap.pose import Pose
from rsim import Runtime
from rsim.motion import ChassisController

# chassis 是配置好的 Chassis 或具有相同 imu/odom/velocity 端口的组件。
# 必须使用设备的真实安装外参：下例仅演示坐标标签和传参形式。
T_body_imu = Pose(wrd_frame="body", ego_frame="imu")
robot = ChassisController(chassis, T_body_imu=T_body_imu)

async with Runtime(robot):
    frame = await robot.pose.get(timeout=10)
    T_odom_body = frame.data               # graphmap.pose.Pose
    await robot.move(0)                    # 零速测试
    await robot.rotate(yaw_deg=0)          # 零速测试
    await robot.stop()
```

默认 `motion_enabled=False`，非零目标抛出 `MotionError`，不会假装已经完成移动。需要运动时，在构造中显式设置 `motion_enabled=True`。示例 Notebook 的非零命令仅作用于内存模拟器。

| 方法 | 含义 |
| --- | --- |
| `await robot.move(distance_m, timeout=...)` | 沿开始时车体 X 轴移动指定米数；负数后退，并保持开始时航向 |
| `await robot.rotate(yaw_deg=..., timeout=...)` | 原地相对旋转角度；正数左转，负数右转 |
| `await robot.rotate(yaw_rad=..., timeout=...)` | 同上，弧度；两个单位必须且只能提供一个 |
| `await robot.stop()` | 中断活动运动，发送零速并等待 provider 应答 |

运动成功返回最终 `Pose`，不返回 ROS 消息。距离使用起始坐标系的 X 向投影，不是轮子累计路程；转角逐帧展开，支持超过 180° 及多圈，要求相邻位姿变化小于 180°。默认到达容差为 1 cm / 0.02 rad，连续 3 个新位姿满足条件才完成。默认限速 0.15 m/s、0.5 rad/s，可通过构造参数调整。速度按比例误差和剩余制动距离限幅；`linear_acceleration` / `angular_acceleration` 用于制动距离包络，并非执行器加速度保证。

同一时刻只允许一个运动，第二个请求立即报错。调用者取消、超时、位姿停更、provider 错误或 Runtime 退出时归零。`stop()` 会让当前运动抛出 `MotionError`；取消保留 `CancelledError`，期限到达抛出 `TimeoutError`。默认运动期限根据距离/角度和最大速度估算，也可以指定。到达、取消和关闭等待的是速度 provider 接受零速，不代表机械制动完成。

控制器独占 `velocity` 端口，使用已有 command claim 和 TTL，不应同时直接写这个端口。默认 TTL 0.25 s，底盘远端仍独立检查过期并执行 deadman。停止或故障时会尝试发送零速；通信已经断开时依赖 provider 的独立 TTL。控制循环是 Component 的 Metronome task，应用仍可并发执行其他协程。

## 坐标、时间与 EKF

- graphmap 约定右手系、X 前 / Y 左 / Z 上，米制；Pose 将局部点变到父坐标系，四元数为 `[x,y,z,w]`。输出标签来自 odom 的 `header.frame_id` / `child_frame_id`，即 `T_odom_body`。所有欧拉角构造显式使用 `degrees=False`。
- IMU 角速度先通过 `T_body_imu.rot_mat` 旋转，协方差同样旋转。若 IMU 与 odom child 的标签不同，必须提供带准确标签的刚体外参；不隐式假设两个坐标系对齐。此算法只使用角速度，安装平移不影响刚体角速度变换。
- 六维状态为 `[x, y, yaw, v, omega, gyro_bias]`，采用平面非完整约束、中点运动预测与 EKF Jacobian，协方差使用 Joseph 更新。odom 提供 x/y/yaw 观测，IMU 提供 `omega + gyro_bias` 观测。odom 的 twist 未被重复当作独立观测，IMU 加速度和绝对朝向未被融合。
- 两路必须属于同一个源时间域；按源时间戳处理。输出锚定 odom 时刻，等待 IMU 流覆盖该时刻，最近此前 IMU 须在 `max_skew` 内。有界队列、重复/迟到样本计数可在 `estimate` 查看。任一路停止都不会不断发布外推位姿；源时间回退需重启估计器，超过 `max_gap` 的时间跳变直接报错。
- 接收时间保留为本次所用两路观测中较早的时间，控制器据此检查 `pose_timeout`，不比较主机与底盘的墙上时钟。网络中本身已经积压但刚收到的消息仍需上游限制延迟。
- 使用消息中的有效协方差；全零或个别缺失方差使用 `odom_std` / `gyro_std`，标记不可用或非法协方差报错。过程噪声与观测噪声均可配置。应输入未与该 IMU 预先融合的轮式 odom，避免相关观测重复加权。

`robot.odometry.estimate` 额外提供 6×6 协方差、前向速度、角速度、零偏估计和丢弃计数。独立使用或把已有定位结果接入控制器：

```python
from rsim.odometry import PlanarOdometry
from rsim.motion import ChassisController
from rsim import ProcessPlacement

localization = PlanarOdometry(chassis.odom, chassis.imu, T_body_imu=T_body_imu)
robot = ChassisController(pose=localization.pose, velocity=chassis.velocity)
async with Runtime(robot, placement={localization: ProcessPlacement("localization")}):
    pose = (await robot.pose.get(timeout=15)).data
    await robot.move(0)
```

Pose 在跨进程时使用封闭的 translation/quaternion/scale/frame-label schema 重建；协方差数组沿用 mmap，接收环境须安装 graphmap。控制器自身留在调用进程，`move/rotate` 是本地异步方法，不是自动 RPC。Pose 和其数组发布后应视为只读。

这是平面相对运动原型，未实现避障、全局定位、坡道三维姿态或轮滑补偿。轮式里程计与 IMU 不能消除长期漂移；传感器噪声和控制参数需要按设备标定。估计器没有外点门控，输入突变可能影响闭环。实现参考 [robot_localization 的传感器配置原则](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst)，采用独立的简化平面模型。

运行示例见 [底盘控制 Notebook](../examples/chassis_motion.ipynb)。默认不连接硬件；实机单元格读取调用方配置，固定关闭运动，只执行零速请求。
