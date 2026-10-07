"""Keep foundational packages independent as adapters and integrations grow."""
import ast
import importlib.util
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
ALLOWED = {
    "core": {"core"},
    "transport": {"core", "transport"},
    "runtime": {"core", "transport", "runtime"},
    "adapters": {"core", "runtime", "adapters"},
    "devices": {"core", "runtime", "devices"},
    "drivers": {"core", "runtime", "transport", "adapters", "devices", "drivers", "components"},
    "components": {"core", "transport", "components"},
    "config": {"core", "devices", "drivers", "config"},
    "apps": {"core", "runtime", "transport", "adapters", "devices", "drivers", "components", "config", "apps"},
}


def test_package_dependency_boundaries():
    violations = []
    for path in (ROOT / "rsim").rglob("*.py"):
        parts = path.relative_to(ROOT).with_suffix("").parts
        if parts == ("rsim", "__init__"):
            continue  # The public convenience facade may export every layer.
        assert len(parts) >= 3, f"place {path.name} in its owning package"
        assert not path.stem.startswith("_") or path.stem in {"__init__", "__main__"}
        layer = parts[1]
        package = ".".join(parts[:-1])
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                imports = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                module = importlib.util.resolve_name(
                    "." * node.level + (node.module or ""), package
                ) if node.level else node.module
                imports = [module + "." + alias.name for alias in node.names]
            else:
                continue
            for name in imports:
                if name == "rsim" or (name.startswith("rsim.")
                                      and name.split(".")[1] not in ALLOWED[layer]):
                    violations.append(f"{path.relative_to(ROOT)}:{node.lineno}: {name}")
    assert not violations, "dependency boundary violations:\n" + "\n".join(violations)


GUARD = '''
import sys
from importlib.abc import MetaPathFinder
class NoOptionalDependencies(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {
            'rclpy', 'rospy', 'std_msgs', 'sensor_msgs', 'geometry_msgs',
            'nav_msgs', 'ament_index_python', 'cyclonedds', 'serial',
            'cv2', 'graphmap', 'yaml',
        }:
            raise AssertionError('eager optional dependency: ' + fullname)
sys.meta_path.insert(0, NoOptionalDependencies())
'''


def run_isolated(code):
    result = subprocess.run([sys.executable, "-c", GUARD + code], cwd=ROOT,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_core_and_public_clients_work_without_optional_dependencies():
    run_isolated('''
import asyncio
import rsim
from rsim.core import Component, Signal, Frame
assert not any(name.startswith(('rsim.runtime', 'rsim.transport', 'rsim.adapters',
                               'rsim.devices', 'rsim.drivers', 'rsim.config',
                               'rsim.components')) for name in sys.modules)
from rsim import *
from rsim.runtime import Runtime as CanonicalRuntime
from rsim.devices import D435 as Client
assert rsim.Component is Component and rsim.Frame is Frame and rsim.Signal is Signal
assert rsim.Runtime is CanonicalRuntime and rsim.D435 is Client
assert 'ChassisController' in dir(rsim)  # Discoverable without importing graphmap.
camera = Client()
assert camera.source.producer.factory is None
assert not any(name.startswith(('rsim.drivers', 'rsim.adapters.ros2')) for name in sys.modules)
class Sample(Component):
    def __init__(self):
        super().__init__()
        self.sample = self.signal('sample')
    async def open(self):
        await self.sample.publish(42, stamp_ns=1, clock='test')
async def run():
    source = Sample()
    async with rsim.Runtime(source.sample):
        frame = await source.sample.get(timeout=1)
        assert frame.data == 42
        assert await source.sample.get(timestamp_ns=1, clock='test') is frame
asyncio.run(run())
''')


def test_driver_package_entrypoint_help_without_optional_dependencies():
    run_isolated('''
import runpy
sys.argv = ['rsim.drivers', '--help']
try:
    runpy.run_module('rsim.drivers', run_name='__main__')
except SystemExit as error:
    assert error.code == 0
else:
    raise AssertionError('driver CLI did not run')
''')
