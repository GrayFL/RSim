"""Opt-in integration: RSIM_TEST_CLIENT_PYTHON selects a separate Python environment."""
import asyncio
import json
import os
from pathlib import Path
import uuid

import numpy as np
import pytest

from rsim import Image, PointCloud, Runtime, Sensor, SharedSensor, TransportConfig, allocate


class ModelSource(Sensor):
    async def open(self):
        self.counter = 0
        self.task("produce", self.produce, hz=10)

    async def produce(self):
        self.counter += 1
        image = allocate((8, 12, 3), np.uint8)
        image[:] = self.counter % 256
        points = allocate((20,), np.dtype([(k, "f4") for k in ("x", "y", "z")]))
        for k in points.dtype.names:
            points[k] = self.counter
        await self.publish({"image": Image(image, "rgb8", "camera"),
                            "cloud": PointCloud(points, "lidar"),
                            "image_inode": os.stat(image.filename).st_ino,
                            "points_inode": os.stat(points.filename).st_ino},
                           stamp_ns=self.counter, clock="synthetic")


GUARD = '''
import sys
from importlib.abc import MetaPathFinder
sys.path[:] = [p for p in sys.path if '/opt/ros/' not in p]
class NoROS(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('rclpy', 'std_msgs', 'sensor_msgs', 'ament_index_python') or fullname == 'rsim.adapters.ros2':
            raise AssertionError('ROS import attempted in client: ' + fullname)
sys.meta_path.insert(0, NoROS())
'''

CLIENT = '''
import asyncio, json, os, sys
from pathlib import Path
import numpy as np
from rsim import Runtime, SharedSensor, ProcessSensor, Map, TransportConfig, Image, PointCloud, HistoryMiss
key,ready,go,result=sys.argv[1:]
config=TransportConfig('cyclonedds', 76)
def source():
    return SharedSensor(key=key, history=16, transport=config)
def graph():
    return Map(source(), lambda data: dict(data, mean=float(data['image'].pixels.mean())))
def mapped_inode(array):
    # The path may already be evicted while the retained mapping stays valid.
    address=array.ctypes.data
    for line in Path('/proc/self/maps').read_text().splitlines():
        fields=line.split()
        start,end=(int(n,16) for n in fields[0].split('-'))
        if start <= address < end: return int(fields[4])
    raise AssertionError('array has no memory mapping')
def verify(frame):
    data=frame.data
    assert type(frame).__module__ == 'rsim.core.model'
    assert isinstance(data['image'], Image) and isinstance(data['cloud'], PointCloud)
    for name,attr,inode in [('image','pixels','image_inode'),('cloud','points','points_inode')]:
        array=getattr(data[name],attr)
        assert isinstance(array,np.memmap) and not array.flags.writeable
        assert mapped_inode(array) == data[inode]
async def main():
    direct=source()
    nested=ProcessSensor(lambda: ProcessSensor(graph, transport=config), transport=config)
    async with Runtime(direct,nested):
        first,deep=await asyncio.gather(direct.get(timeout=20),nested.get(timeout=20))
        verify(first);verify(deep)
        assert deep.data['mean'] == deep.data['image'].pixels.mean()
        Path(ready).write_text(json.dumps({'pid':direct.worker_pid,'directory':str(direct.directory)}))
        async with asyncio.timeout(15):
            while not Path(go).exists(): await asyncio.sleep(.03)
        fresh=first
        for _ in range(5): fresh=await direct.get(after=fresh.sequence,timeout=5)
        assert fresh.stamp_ns>first.stamp_ns
        verify(fresh)
        assert await direct.get(timestamp_ns=fresh.stamp_ns,clock=fresh.clock) is fresh
        try: await direct.get(timestamp_ns=-1,clock=fresh.clock)
        except HistoryMiss: pass
        else: raise AssertionError('history miss did not raise')
        retained=first.data['image'].pixels
        checksum=int(retained.sum())
        worker=direct.worker_pid
        compute=nested.worker_pid
    assert int(retained.sum())==checksum
    assert not any(n.split('.')[0] in ('rclpy','std_msgs','sensor_msgs') or n=='rsim.adapters.ros2' for n in sys.modules)
    Path(result).write_text(json.dumps({'python':sys.version,'source_pid':worker,'compute_pid':compute,
        'native_dds':True,'ros_imports':False,'shared_inodes':True,'history':True,'survived_provider_exit':True}))
asyncio.run(main())
'''


@pytest.mark.parametrize("backend", ["ros2", "cyclonedds"])
def test_ros_provider_and_ros_free_client_share_models_memory_and_lifetime(tmp_path, backend):
    executable = os.environ.get("RSIM_TEST_CLIENT_PYTHON")
    if not executable:
        pytest.skip("set RSIM_TEST_CLIENT_PYTHON to test another interpreter")
    if backend == "ros2":
        pytest.importorskip("rclpy", exc_type=ImportError)

    async def run():
        key = "cross-env:" + uuid.uuid4().hex
        source = SharedSensor(ModelSource, key=key, history=16,
                              transport=TransportConfig(backend, 76))
        provider = Runtime(source)
        ready, go, result = (tmp_path / name for name in ("ready.json", "go", "result.json"))
        (tmp_path / "sitecustomize.py").write_text(GUARD)
        env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(tmp_path), str(Path.cwd()))))
        for name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_DISTRO", "ROS_VERSION"):
            env.pop(name, None)
        client = None
        try:
            await provider.__aenter__()
            await source.get(timeout=20)
            client = await asyncio.create_subprocess_exec(executable, "-c", CLIENT,
                key, str(ready), str(go), str(result), env=env,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            async with asyncio.timeout(30):
                while not ready.exists():
                    if client.returncode is not None:
                        out, err = await client.communicate()
                        pytest.fail((out + err).decode())
                    await asyncio.sleep(.05)
            attached = json.loads(ready.read_text())
            assert attached["pid"] == source.worker_pid
            assert attached["directory"] == str(source.directory)
            await provider.aclose()
            go.touch()
            out, err = await asyncio.wait_for(client.communicate(), 20)
            assert client.returncode == 0, (out + err).decode()
            evidence = json.loads(result.read_text())
            assert evidence["shared_inodes"] and evidence["survived_provider_exit"]
            assert evidence["source_pid"] != evidence["compute_pid"]
            async with asyncio.timeout(10):
                while source.directory.exists():
                    await asyncio.sleep(.05)
        finally:
            if client is not None and client.returncode is None:
                client.kill()
                await client.wait()
            await provider.aclose()
    asyncio.run(run())
