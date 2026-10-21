# Windows 的 DDS 与跨进程部署

Windows 保持 RSim 原有的 Component、Signal、CommandSink 和部署 API。DDS 传递描述信息，数组通过同机只读文件映射共享；不把图像转成 DDS JSON，也不把跨进程改成同进程模拟。

## 环境条件

- Python 版本满足 pyproject.toml（≥3.12），两侧安装同一版 RSim 接口；提供者和客户端可以使用不同 Python 版本。
- 安装 `rsim[dds]`，确认 Cyclone DDS 可实际创建 DomainParticipant 和 RSim 描述信息端点。Python 3.14 还需包含相应 IDL 兼容修复的绑定，见 [DDS 文档](dds.md)。
- Windows 10/11、本地 NTFS 文件系统及用户自己的临时目录。本实现依赖 Windows 的 POSIX 文件删除语义；没有把网络共享盘、FAT 或旧 Windows 列为支持环境。
- 数组共享与注册表限于同一主机、同一用户；name/key、domain、接口声明匹配。不同主机的底盘小消息控制仍使用已有 DDS 服务方案，但不能直接共享本机 mmap 路径。

MuJoCo 提供者另外安装 `rsim[mujoco,dds]`，RGBD 需要 OpenGL；仅控制/读取端安装 DDS 及所用数据模型依赖。需要 Pose 时准备 graphmap，保存 PNG 时准备 Pillow。

## 原有 API

```python
import asyncio
from rsim import Runtime, Map, ProcessPlacement
from rsim.components.synthetic import CounterArray

async def main():
    source = CounterArray(size=1024)
    analysis = Map(source.output, lambda values: values ** 2)
    async with Runtime(analysis.output, placement={analysis: ProcessPlacement("compute")}):
        frame = await analysis.output.get(timeout=15)
        print(type(frame.data), frame.data.shape)

if __name__ == "__main__":
    asyncio.run(main())
```

`ProcessSensor` 支持原有嵌套工厂；`SharedSensor` 支持单输出共享源；`SharedProvider/SharedComponent` 支持多端口、精确订阅和命令写入。连接者不执行提供者的工厂。接口细节及租约语义见 [多端口共享](shared-components.md)。`inprocess` 仍仅限一个事件循环，跨进程请选择 Cyclone DDS。

## MuJoCo 三个进程的示例

终端 A（具备 MuJoCo 的环境）：

```sh
python -m rsim.apps.mujoco_service --scene "data/scene.glb" --name mujoco --domain 0 --enable-motion
```

终端 B（可以使用另一 Python 环境）：

```sh
python -m rsim.apps.mujoco_motion --connect mujoco --domain 0 --output assets/mujoco/client
```

数据与控制链路为：

```text
应用进程：devices.Chassis + SharedComponent 传感器视图
  ↕ DDS 控制请求/反馈、数据描述信息 + mmap
控制服务进程：MotionService + ChassisController
  ↕ DDS Pose / CommandSink
物理 worker：MuJoCoSource、轮地接触、雷达与 RGBD
```

物理 worker 还具有独立的租约监督进程。应用继续执行相同的 `move(.2)`、`rotate(yaw_deg=30)`、`move(-.1)`、`stop()`。服务输出 service_pid/physics_pid；客户端 motion_summary 输出 client_pid/physics_pid、解释器版本、图像尺寸和共享映射标记。

物理是 SharedProvider 的源租约：服务退出后，若独立传感器客户端仍持有源租约，物理 worker 继续存在；速度短期租约过期后归零。最后一位使用者退出才清理源。ProcessPlacement 则始终跟随所属 Runtime。

## Windows 实现与 Linux 对照

| 机制 | Linux | Windows |
|---|---|---|
| 数组存储 | `/dev/shm` tmpfs | 用户临时目录的 NPY 文件 + mmap |
| 保留帧与淘汰 | unlink 后现有映射继续有效 | FILE_SHARE_DELETE + POSIX 删除语义，保留映射继续有效 |
| 完整映射转发 | 硬链接复用 inode | NTFS 硬链接复用文件 ID |
| 本地注册表/源租约 | Unix socket | 127.0.0.1 TCP + 每端点随机令牌 |
| 私有 Runtime 租约 | 继承管道描述符 | 显式继承管道 HANDLE |
| 源启动互斥 | flock | LockFileEx，串行化启动和最后租约关闭 |
| worker 树回收 | 进程组 / 父进程死亡信号 | 挂起创建、加入 Job Object、恢复执行；最后 Job 句柄关闭时回收进程树 |
| 正常停止 | SIGTERM | 停止文件触发协程清理；超时终止 Job |
| 时钟实例身份 | boot UUID | boot GUID |

Windows 映射使用文件作为后备存储，可以产生磁盘 I/O；没有宣称与 tmpfs 的性能相同。首次数组进入共享存储仍可能复制，完整的只读映射转发复用文件页；不代表传感器驱动端到端零拷贝。

原语位于 `rsim.platforms.windows`，两个平台共用 `runtime.supervision/shared_launch/shared_supervisor` 策略。路径、startup/shutdown 参数、私有 ACL、SID namespace 和诊断入口见 [跨平台运行时](runtime.md)。`doctor --require shared-mapping --deep` 会在实际存储卷测试独立子进程保留映射和硬链接，并报告文件系统。已有开发环境中的双解释器检查见 [本地测试](validation.md)。

监督进程与计算进程隔离，计算循环挂起也不阻碍监督者接收最后租约断开。worker 在运行任何 Python 代码前就被加入 Job；监督者被强制结束时，OS 负责关闭 Job。独立共享源的监督者显式脱离上层 Job，保持“创建者退出、其他使用者仍可继续”的契约。

Windows 进程、IPC、文件和锁分别位于 `platforms/windows/process.py`、`ipc.py`、`files.py` 和 `locks.py`；Linux 原语位于 `platforms/linux/`。共享数组编码位于 `transport/files.py`，运行时直接使用所选平台的原语。

原语依据：[Microsoft Job Objects](https://learn.microsoft.com/en-us/windows/win32/procthread/job-objects)、[SetFileInformationByHandle](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-setfileinformationbyhandle)。

## 验证

在安装测试依赖后执行：

```sh
python -m pytest tests/test_process.py tests/test_parent_death.py tests/test_placement.py tests/test_host.py tests/test_host_cross_process.py tests/test_shared.py tests/test_shared_component.py tests/test_windows_runtime.py -q -k "not ros2"
```

跨环境检查在 PowerShell 中显式选择已安装 DDS、NumPy、cloudpickle 和 RSim 的另一个解释器：

```powershell
$env:RSIM_TEST_CLIENT_PYTHON = "path/to/client/python.exe"
python -m pytest tests/test_cross_environment.py tests/test_shared_component.py -q -k "cross_environment and cyclonedds"
```

测试覆盖真实进程 PID、相同文件标识、映射在淘汰和退出后可读、命令 TTL、客户端死亡、worker 崩溃/挂起、父进程死亡、Job 所有者死亡、配置冲突、重启身份隔离和 ROS 导入禁止。未安装 ROS2 的环境会跳过 ROS2 后端互通测试，不能据此宣称已验证 ROS2 本身。

本机解释器版本、命令、测试报告和实际场景结果放在 PROJECT.md 与 assets/，不作为其他机器的路径配置。
