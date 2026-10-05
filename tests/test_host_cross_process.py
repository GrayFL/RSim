import asyncio
import json
from pathlib import Path
import sys
import uuid

from rsim import Runtime, SharedSensor
from rsim.components.synthetic import CounterArray


def test_independent_client_death_does_not_stop_other_consumer(tmp_path):
    async def run():
        key = "cross-process:" + uuid.uuid4().hex
        ready = tmp_path / "ready.json"
        script = '''
import asyncio,json,sys
from pathlib import Path
from rsim import Runtime,SharedSensor
from rsim.components.synthetic import CounterArray
async def main():
    sensor=SharedSensor(lambda: CounterArray(),key=sys.argv[1])
    async with Runtime(sensor):
        await sensor.get(timeout=20)
        Path(sys.argv[2]).write_text(json.dumps({"pid":sensor.worker_pid,"directory":str(sensor.directory)}))
        await asyncio.Future()
asyncio.run(main())
'''
        owner = await asyncio.create_subprocess_exec(sys.executable, "-c", script, key, str(ready))
        try:
            async with asyncio.timeout(25):
                while not ready.exists():
                    assert owner.returncode is None
                    await asyncio.sleep(0.05)
            state = json.loads(ready.read_text())
            consumer = SharedSensor(lambda: CounterArray(), key=key)
            async with Runtime(consumer):
                first = await consumer.get(timeout=15)
                assert str(consumer.directory) == state["directory"]
                assert consumer.worker_pid == state["pid"]
                owner.kill()
                await owner.wait()
                fresh = await consumer.get(after=first.sequence, timeout=5)
                assert fresh.stamp_ns > first.stamp_ns
            async with asyncio.timeout(7):
                while consumer.directory.exists():
                    await asyncio.sleep(0.05)
        finally:
            if owner.returncode is None:
                owner.kill()
                await owner.wait()
    asyncio.run(run())
