# 原生 ROS 驱动参数

`rsim.drivers.D435`、`rsim.drivers.RobinW` 和低层 `rsim.ros` 对应适配器接受 `parameters` 与 `ros_args`。参数名不设白名单：设备驱动支持的节点参数均可传入，具体名称、类型和取值范围以所安装驱动为准。

## Python 调用

```python
from rsim import Runtime
from rsim.drivers import D435

camera = D435(
    parameters={
        "publish_tf": False,
        "enable_color": False,
        "depth_module.emitter_enabled": 1,
    },
    ros_args=["--log-level", "warn"],
    log_path="assets/camera-driver.log",
)
async with Runtime(camera):
    frame = await camera.get(timeout=30)
```

创建日志前确保 `assets/` 已存在。Notebook 中可直接 `await`；普通脚本将上下文放入异步函数，再调用 `asyncio.run()`。

`parameters` 是以原生节点参数名为键的字典。支持 Python `bool`、64 位 `int`、有限 `float`、`str` 及同类型标量组成的列表/元组；嵌套参数用点分名称，例如 `"rgb_camera.enable_auto_exposure"`。字符串会保留引号语义，`"false"`、`"00123"` 不会意外转换为布尔值或数字。混合类型数组、字典值和 `None` 会在启动前报错。

`ros_args` 是 argv 字符串序列，不是 shell 命令。开头的 `--ros-args` 可省略；支持原生 `-p/--param`、`--params-file`、`-r/--remap`、`--log-level` 等选项。例如：

```python
from rsim.drivers import RobinW

lidar = RobinW(
    ip=lidar_ip,
    parameters={"frame_id": "front_lidar", "frame_topic": "cloud"},
    ros_args=["-r", "__ns:=/rig", "-r", "cloud:=points"],
    log_path="assets/lidar-driver.log",
)
```

此例驱动与适配器均使用 `/rig/points`。已有的 `history`、`transport` 等 RSim 选项继续作为工厂关键字参数传入。

## 命令行

将 RSim 选项放在 `--ros-args` **之前**；之后的内容交给原生 ROS 解析，不需要为每个驱动参数增加一个 RSim 开关：

```bash
python -m rsim.drivers d435 --log-path assets/camera-driver.log \
  --ros-args -p publish_tf:=false -p enable_color:=false \
  -p depth_module.emitter_enabled:=1 --log-level warn

python -m rsim.drivers robin --ip "$LIDAR_IP" \
  --ros-args -p frame_id:=front_lidar -p frame_topic:=cloud \
  -r __ns:=/rig -r cloud:=points
```

也可以在原生参数中传 `lidar_ip`，此时可省略 `--ip`。仅开彩色流时选择 `--stream color`，并传 `-p enable_depth:=false`。`--stream` 选择 provider 的返回视图；启用哪些流由 `enable_color`、`enable_depth` 决定。

ROS 参数文件沿用标准节点选择语法：

```yaml
/**:
  ros__parameters:
    publish_tf: false
    rgb_camera.enable_auto_exposure: true
```

```bash
python -m rsim.drivers d435 \
  --ros-args --params-file camera.yaml -p enable_sync:=true
```

对应函数调用为 `D435(ros_args=["--params-file", "camera.yaml", "-p", "enable_sync:=true"])`。文件路径在创建工厂时转成绝对路径；文件内容参与共享配置检查。创建工厂后若修改文件，需要重新创建工厂；不会默默以变更后的文件启动旧配置。

## 覆盖、路由与共享

优先级从低到高为：现有快捷参数（如 `depth_profile`、`ip`）→ `parameters` → `ros_args` 中的参数赋值/参数文件。同一原生参数重复指定时按 ROS 的顺序覆盖；remap 使用首个匹配规则，用户 remap 优先于 RSim 默认的 node/namespace。节点限定、YAML 和话题重映射由 ROS 自身解析，语法见 [ROS 2 命令行设计](https://design.ros2.org/articles/ros_command_line_arguments.html) 和 [重映射规则](https://design.ros2.org/articles/static_remapping.html)。带 `ros_args` 的设备工厂会创建并立即销毁一个无参数服务的临时 ROS 节点以解析配置，不占用硬件，也不把 ROS 句柄传给应用端。

- D435 自动跟随 `__node`、`__ns` 及图像 topic remap。`camera_name` 是驱动参数，不能替代节点重命名；用 `-r __node:=front` 修改节点名。
- RobinW 自动跟随有效 `lidar_ip`、`frame_topic` 和 topic remap。
- 关闭 D435 一路流时只采集剩余流。请求已关闭流的 provider 会立即报错；应用侧请求不存在的流会传播任务异常。两路均关闭无法提供该图像抽象，创建时拒绝。
- 原生参数覆盖 `serial_no`、`lidar_ip` 或 D435 profile 时，共享设备标识、profile 和采集节拍使用覆盖后的值。D435 profile 仍需显式指定正整数的宽、高、帧率。应用端的 `serial`、profile、history 需与有效配置一致；其他驱动参数无需传给无 ROS 的 `rsim.D435()` / `rsim.RobinW()`。
- 多个 provider 申请同一设备时，完整启动配置必须一致；参数、原生 argv 或参数文件内容冲突会报错，已有源不被重启。相同配置继续复用一个驱动和共享数组。连接型客户端不参与驱动专属配置比较。

此入口直接启动原生节点，相当于 `ros2 run` 的节点选项，不执行厂商 launch 文件。只属于 launch 的开关（如 RealSense 的 `camera_namespace`）需用相应节点选项表达（`-r __ns:=...`）。原生参数可启用额外 ROS 输出，但不会自动增加新的 RSim 数据模型：D435 当前返回 color/depth Image，RobinW 返回 PointCloud。改变输出模式或改用厂商多设备配置文件时，应使用低层 `Driver` + `RosSensor` 显式配置订阅与组合。

通用 `rsim.ros.Driver(package, executable, parameters, key=..., ros_args=..., remappings=...)` 也提供同样的参数编码与 argv 通道，适合自定义原生节点。`Camera` 当前通过 OpenCV/UVC 采集，没有原生 ROS 驱动启动参数；CLI 对其 `--ros-args` 明确报错。
