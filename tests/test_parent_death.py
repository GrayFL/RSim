import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import time


from process_support import descendants, alive, kill, command


def test_parent_sigkill_reaps_nested_workers_and_stores(tmp_path):
    async def run():
        ready = tmp_path / "ready.json"
        script = '''
import asyncio, json, sys
from pathlib import Path
from rsim import Runtime
from rsim.runtime.process import ProcessSensor
from rsim.components.synthetic import CounterArray
async def main():
    sensor = ProcessSensor(lambda: ProcessSensor(lambda: CounterArray()))
    async with Runtime(sensor):
        await sensor.get(timeout=20)
        Path(sys.argv[1]).write_text(json.dumps(str(sensor.directory)))
        await asyncio.Future()
asyncio.run(main())
'''
        process = await asyncio.create_subprocess_exec(sys.executable, "-c", script, str(ready))
        children = set()
        try:
            async with asyncio.timeout(25):
                while not ready.exists():
                    assert process.returncode is None
                    await asyncio.sleep(0.05)
            children = descendants(process.pid)
            assert len(children) >= 4  # two supervisors + two asyncio workers
            stores = []
            for pid in children:
                args = command(pid)
                if "rsim.runtime.supervisor" in args:
                    stores.append(Path(args[args.index("rsim.runtime.supervisor") + 1]))
            assert len(stores) == 2
            process.kill()
            await process.wait()
            async with asyncio.timeout(12):
                while any(alive(pid) for pid in children) or any(p.exists() for p in stores):
                    await asyncio.sleep(0.1)
        finally:
            if process.returncode is None:
                process.kill()
                await process.wait()
            for pid in children:
                if alive(pid):
                    kill(pid)
    asyncio.run(run())


def test_parent_sigkill_reclaims_placement_workers_and_local_export_store(tmp_path):
    async def run():
        ready = tmp_path / "deployment.json"
        script = '''
import asyncio, json, sys
from pathlib import Path
from rsim import Runtime, Map, ProcessPlacement
from rsim.components.synthetic import CounterArray
async def main():
    source = CounterArray()
    first, second = Map(source, lambda x: x), Map(source, lambda x: x)
    async with Runtime(first.output, second.output, placement={
            first: ProcessPlacement("first"), second: ProcessPlacement("second")}) as runtime:
        await asyncio.gather(first.get(timeout=20), second.get(timeout=20))
        Path(sys.argv[1]).write_text(json.dumps(str(runtime._binding_plan.directory)))
        await asyncio.Future()
asyncio.run(main())
'''
        process = await asyncio.create_subprocess_exec(sys.executable, "-c", script, str(ready))
        children = set()
        try:
            async with asyncio.timeout(25):
                while not ready.exists():
                    assert process.returncode is None
                    await asyncio.sleep(.05)
            directory = Path(json.loads(ready.read_text()))
            children = descendants(process.pid)
            assert len(children) >= 5
            assert any((directory / "local" / "frames").iterdir())
            process.kill()
            await process.wait()
            async with asyncio.timeout(12):
                while any(alive(pid) for pid in children) or directory.exists():
                    await asyncio.sleep(.1)
        finally:
            if process.returncode is None:
                process.kill()
                await process.wait()
            for pid in children:
                if alive(pid):
                    kill(pid)
    asyncio.run(run())
