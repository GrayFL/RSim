# 本地开发测试

使用已有 Python 开发环境，从仓库根目录直接运行源码和 pytest。依赖按
`pyproject.toml` 中实际使用的功能准备；不需要构建 wheel、创建固定 venv 或运行
GitHub workflow。两套跨环境解释器都应能导入当前源码、NumPy、cloudpickle 和
可工作的 Cyclone DDS。机器路径与环境名记录在本机 `PROJECT.md`。

## 日常检查

```sh
python -m pytest tests/test_architecture.py tests/test_core.py tests/test_component_signal.py tests/test_sync.py tests/test_commands.py tests/test_inprocess_transport.py tests/test_runtime_contracts.py tests/test_data_contracts.py -q
python -m examples.components.components --local --frames 3
python -m examples.components.components --frames 3
```

第一个示例在当前进程运行，第二个使用真实子进程和 DDS；两者的执行器均为内存模拟。
按需诊断当前环境：

```sh
python -m rsim.apps.doctor --require process,shared-mapping,dds --deep --json
```

运行时改动可检查进程、共享源、租约与文件映射：

```sh
python -m pytest tests/test_platform_contracts.py tests/test_supervision_contracts.py tests/test_process.py tests/test_placement.py tests/test_shared.py tests/test_shared_component.py tests/test_host.py tests/test_host_cross_process.py tests/test_parent_death.py tests/test_transport.py -q -k "not ros2"
```

Windows 上另运行 `python -m pytest tests/test_windows_runtime.py -q`。Linux 原语和
Linux 专项在 Linux 主机执行；不在 Windows 模拟 Linux。只运行与本次改动有关的检查。

## 跨解释器

先用两套解释器分别执行 `python -c "import sys, rsim, cyclonedds; print(sys.executable, rsim.__file__)"`，
确认导入的是当前源码。然后指定另一套解释器：

```powershell
$env:RSIM_TEST_CLIENT_PYTHON = "C:/path/to/other/python.exe"
python -m pytest tests/test_cross_environment.py tests/test_shared_component.py -q -k "cross_environment and cyclonedds"
Remove-Item Env:RSIM_TEST_CLIENT_PYTHON
```

Linux 的等价用法：

```sh
RSIM_TEST_CLIENT_PYTHON=/path/to/other/bin/python python -m pytest tests/test_cross_environment.py tests/test_shared_component.py -q -k 'cross_environment and cyclonedds'
```

交换 provider/client 解释器后再运行，可覆盖两个方向。测试使用真实 DDS、只读 mmap、
嵌套进程、命令和源租约，客户端禁止导入 ROS/MuJoCo。具备 ROS2 的 Linux provider
环境可将筛选改为 `cross_environment and ros2`。

## 可选后端

| 环境 | 按需执行 |
|---|---|
| MuJoCo adapter | `python -m pytest tests/test_mujoco_adapter.py -q`；具体物理与渲染归 MujocoSim |
| Linux ROS2 | `python -m pytest tests/test_ros_context.py tests/test_ros_arrays.py tests/test_ros_topic_conversions.py tests/test_chassis_service.py tests/test_service_separation.py -q` |

graphmap 需要项目使用的 Pose v3 API。缺少后端、设备或环境时，跳过项表示未验证；
需要验证的后端应先准备好依赖，再执行对应测试。无设备时不执行实机验证。

默认只在终端查看结果；需要留存时增加 `--junitxml=assets/tests.xml`。示例输出、临时
日志和报告使用后清理，开发结论保留在 `PROJECT.md`，不将测试环境与源码快照长期留在仓库。

共享文件的内存与磁盘使用可按需运行 [本地基准](benchmark.md)，其中区分逻辑数据量、
进程 I/O 和整机设备写入，并提供临时文件提示、直接共享分配及存储位置的对照。
