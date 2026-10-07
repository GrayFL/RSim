# 开发与文档约定

## 文档分工

| 位置 | 内容 |
| --- | --- |
| `README.md` | 项目定位、依赖条件、公共接口、使用示例和通用验证方式 |
| `docs/` | 可复用的设计、接口和开发说明 |
| 根目录 `assets/` | 图像、图表、采集结果、日志和测试报告等生成资产；不提交 Git |

通用文档描述需要满足的能力和接口，不依赖某台机器的目录、解释器位置、环境名、kernel 名或设备地址。版本约束以包元数据为准，硬件配置由使用者提供。开发日志和某次实验结果不能替代对当前实现的验证。

## 模块与依赖

源码归属和扩展步骤见 [包结构与扩展](modules.md)。基础契约放 `core`，进程执行放 `runtime`，数据传输放 `transport`；新设备按协议适配、provider 工厂与客户端视图区分，算法使用端口而不依赖具体设备。

包内优先相对导入，跨包显式导入所属模块；库内部不经 `rsim` 公共便捷入口取基础类型。`__init__.py` 只组织导出，设备不得在导入阶段启动。跨进程入口使用完整模块名并通过 `python -m` 执行。包间依赖由 `tests/test_architecture.py` 检查；调整允许方向时应说明职责变化，不能仅为通过测试开放反向依赖。

新增子包使用 `__init__.py`，由现有 setuptools 配置递归发现。一个文件能清楚表达的设备或算法不必再拆目录；稳定公共入口与实现模块分开维护。删除或移动实现模块时，同步检查子进程命令、cloudpickle 工厂引用、ROS 启动脚本、示例和文档。

## 绘图与显示

统一从 `scipykit.mtp_initializer` 使用绘图工具。CV 场景可使用 `scipykit.task.cv`，它重导出同一套绘图入口。`scipykit` 是绘图和实验工作流依赖，不是传感器核心运行时的必需依赖；使用相应工具前应在所选环境中准备好该库。

以下示例从项目根目录运行，命名显示将图像外置保存到 `assets/` 下：

```python
import os
from pathlib import Path
from scipykit.mtp_initializer import subplots, disp, plt

assets = Path("assets")
assets.mkdir(parents=True, exist_ok=True)
os.environ["NOTEBOOK_ASSETS_ROOT"] = str(assets.resolve())

fig, ax = subplots()
ax.plot([0, 1, 2], [0, 1, 0])
disp(fig, "signal")
plt.close(fig)
```

使用已有初始化、样式和导出工具，避免重复配置。需要图像显示时，可使用 `show_image(image, key="sample")`。具名输出的具体子目录由绘图库的 Notebook 命名机制决定，资产根目录始终应指向项目根目录的 `assets/`。

## 资产路径

从子目录运行时，不能直接假定当前工作目录就是项目根目录。对于放在 `examples/` 下的普通脚本，可通过脚本位置定位：

```python
from pathlib import Path

project_root = next(p for p in Path(__file__).resolve().parents
                    if (p / 'pyproject.toml').is_file() and (p / 'rsim').is_dir())
assets = project_root / "assets"
assets.mkdir(parents=True, exist_ok=True)
```

Notebook 没有 `__file__`，从当前工作目录向上找到同时包含 `pyproject.toml` 和 `rsim/` 的项目根，再将 `NOTEBOOK_ASSETS_ROOT` 设置为该目录下 `assets/` 的绝对路径。不要依赖示例的固定目录层数。设置环境变量只影响使用它的显示工具，其他写文件操作仍需显式使用同一个资产目录。

## 验证与记录

在具备项目依赖的环境中，从项目根目录运行：

```bash
mkdir -p assets
python -m pytest -q --junitxml=assets/tests.xml
```

仅执行与改动有关的检查。文档整理检查链接、命令、路径和本地信息边界；运行时改动验证相应行为；硬件实验应在设备及驱动可用时单独执行。生成的日志、图片和测试报告保存到 `assets/`，本地实验条件和结论记录到 `PROJECT.md`。

提交文档或分享 Notebook 前，检查输出与元数据是否包含本机路径、设备地址或环境信息；具名外置图像需要同时提供相应资产。公开文档不能依赖未提交的本地文件才能理解或使用。
