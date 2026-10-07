# 用配置组装传感器

`load_rig()` 从 YAML 创建 Component 图，进入 Runtime 才打开硬件。设备启动参数、数据端口和固定安装外参分别保存；分组可嵌套，同名传感器被多个组引用时只构建一次。使用者可以只选择当前在线的设备。

```python
from rsim import Runtime
from rsim.config import load_rig

rig = load_rig("configs/sensors.yaml", select="imu")
async with Runtime(rig):
    frame = await rig["imu"].imu.get(timeout=15)

rig = load_rig("configs/sensors.yaml", select="robot")
assert rig["robot"]["perception"]["imu"] is rig["inertial"]["imu"]
T_rgb_lidar = rig.transform("rgb", "lidar3d")   # LiDAR 点 -> 相机 FLU 坐标
```

`rig["rgb"].image`、`rig["lidar3d"].points`、`rig["imu"].imu` 是原数据源的 Signal 别名，保留时间戳和历史，不额外复制 payload。相机的 color/depth 两个视图仍由现有 SharedSensor 复用物理设备。

## YAML 结构

```yaml
rsim:
  base_frame: base_footprint
  geometry_file: calibration.yaml
  sensors:
    rgb:
      driver: d435
      parameters:
        stream: color
        parameters: {publish_tf: false}  # 原生 ROS 参数继续透传
      mount:
        from: rgb
        frame: camera
        status: reference
    imu:
      driver: hipnuc
      parameters:
        mode: serial
        port: /dev/ttyUSB0
        baudrate: 115200
      mount:
        frame: imu
        position: [null, 0, null]
        rotation: null
        status: unknown
  assemblies:
    perception: [rgb, imu]
    inertial: [imu]
    robot: [perception, inertial]
```

`geometry_file` 相对于当前 YAML 所在目录解析，其 `sensor_params.rgb` 等块可包含 position、rotation 和其他相机参数；本层读取 position/rotation，不把参考内参自动覆盖到硬件驱动。也可以直接在当前文件的 `sensor_params` 中存放几何信息。旧配置的 ROS1 `sensors.module` 名称不会被动态导入；新增 `rsim.sensors` 明确列出本库驱动工厂。

内置驱动名为 `d435`、`robinw`、`camera`、`hipnuc`。`parameters` 是相应工厂的参数字典，原生 ROS 参数仍放在它内部的 `parameters` / `ros_args`。可用 `factories={name: callable}` 显式注入其他 Component 工厂；不执行 YAML 指定的任意模块或类。重复 YAML 键、未知驱动、循环分组、缺失引用或错误几何会报错。

运行时覆盖无需改写参考配置：

```python
rig = load_rig("configs/sensors.yaml", select=["imu"],
               overrides={"imu": {"mode": "ros2", "port": serial_device,
                                  "parameters": {"timeout": 5.0}}})
```

覆盖是传感器构造参数级的浅覆盖；嵌套字典整体替换。默认 `select=None` 创建文件中全部传感器和组合，离线设备应通过 select 排除。`providers=False` 创建连接已有 provider 的客户端，忽略只属于驱动的构造项；IMU 客户端必须明确指定 port，公共设备标识/profile/history 须与 provider 的有效配置一致。

## 外参与未知标定

Mount 表示 **`T_base_sensor`：传感器局部坐标 → base_frame**。position 为米，rotation 为 graphmap 的 xyz 欧拉角、单位度；统一 X 前/Y 左/Z 上。使用 `rig.transform(target, source)` 得到 `(~T_base_target) * T_base_source`，输出仍是带坐标标签的 graphmap Pose。

| status | 含义与使用规则 |
| --- | --- |
| measured | 已测量，必须完整，可用于变换 |
| reference | 从参考配置读取的完整几何，可用于变换，不代表本轮重新标定 |
| estimated | 明确给出的完整估计值，需 `allow_estimated=True` 才可用于变换 |
| unknown | 未标定或部分已知，可以启动传感器，但几何变换抛出 `UncalibratedMount` |

未知位置或姿态不会被默认补成零。中心线附近的信息可以保留为 `[null, 0, null]`，在 status=unknown 下不当作精确外参。安装固定不等于外参已知；相关传感器可以先组合采集，测量后再补全配置。

参考相机外参使用 graphmap 相机 FLU 坐标。ROS Image 的 optical frame 标签继续保留在原始数据中，组装层不重命名或悄悄换轴；从 optical 点云变换前，应通过 graphmap conventions 显式转为相同局部坐标约定。深度与彩色的光学中心不默认视作相同，额外深度视图的外参需要标定。

## 运行示例

```bash
python -m examples.imu.imu_rig --mode serial
python -m examples.imu.imu_rig --mode ros2
python -m examples.imu.imu_rig --select robot --frames 30
```

默认只读取 IMU，不依赖底盘在线。配置示例见 `configs/sensors.yaml`，交互示例见 [Notebook](../examples/imu/imu_rig.ipynb)；程序报告和采样数组保存到根目录 assets。各个传感器保留自己的时间域，此处的组合不会自动完成跨设备时钟同步或融合。
