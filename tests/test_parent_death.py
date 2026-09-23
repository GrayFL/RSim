import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import time


def descendants(parent):
    parents = {}
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            stat = path.read_text().rsplit(")", 1)[1].split()
            parents[int(path.parent.name)] = int(stat[1])
        except (FileNotFoundError, ProcessLookupError):
            pass
    found = {parent}
    while True:
        added = {pid for pid, ppid in parents.items() if ppid in found} - found
        if not added:
            return found - {parent}
        found |= added


def alive(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except FileNotFoundError:
        return False


def test_parent_sigkill_reaps_nested_workers_and_stores(tmp_path):
    async def run():
        ready = tmp_path / "ready.json"
        script = '''
import asyncio, json, sys
from pathlib import Path
from rsim import Runtime
from rsim.process import ProcessSensor
from rsim.synthetic import CounterArray
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
                try:
                    args = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
                except FileNotFoundError:
                    continue
                if b"rsim._supervisor" in args:
                    stores.append(Path(os.fsdecode(args[args.index(b"rsim._supervisor") + 1])))
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
                    os.kill(pid, signal.SIGKILL)
    asyncio.run(run())
