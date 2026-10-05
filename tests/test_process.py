import asyncio
import os
import signal

import numpy as np
import pytest

from rsim import Runtime, SensorError
from rsim.core.compose import Map
from rsim.runtime.process import ProcessSensor
from rsim.components.synthetic import CounterArray


def nested_factory():
    return Map(ProcessSensor(lambda: CounterArray(size=2048)), lambda data: data)


def test_nested_process_receives_dds_descriptors_and_keeps_mapping_alive():
    async def run():
        sensor = ProcessSensor(nested_factory)
        async with Runtime(sensor):
            frame = await sensor.get(timeout=20)
            assert sensor.worker_pid != os.getpid()
            assert isinstance(frame.data, np.memmap)
            assert not frame.data.flags.writeable
            assert frame.data.shape == (2048,)
            before = float(frame.data[0])
            directory = sensor.directory
            next_frame = await sensor.get(after=frame.sequence, timeout=5)
            assert next_frame.sequence > frame.sequence
            assert await sensor.get(timestamp_ns=frame.stamp_ns, clock=frame.clock) is frame
        assert not directory.exists()
        # Closing both child levels unlinks files; the retained frame remains valid.
        assert float(frame.data[0]) == before
        assert sensor.process.returncode == 0
    asyncio.run(run())


def test_worker_crash_propagates_and_cleans():
    async def run():
        sensor = ProcessSensor(lambda: CounterArray())
        async with Runtime(sensor):
            await sensor.get(timeout=20)
            os.kill(sensor.worker_pid, signal.SIGKILL)
            async with asyncio.timeout(3):
                while sensor._failure is None:
                    await asyncio.sleep(0.02)
            with pytest.raises(SensorError):
                await sensor.get()
        assert not sensor.directory.exists()
    asyncio.run(run())


def test_producer_and_two_process_boundaries_share_one_physical_array():
    def record(data):
        return {"array": data, "producer_inode": os.stat(data.filename).st_ino}

    async def run():
        sensor = ProcessSensor(lambda: ProcessSensor(lambda: Map(CounterArray(), record)))
        async with Runtime(sensor):
            frame = await sensor.get(timeout=20)
            assert os.stat(frame.data["array"].filename).st_ino == frame.data["producer_inode"]
            assert not frame.data["array"].flags.writeable
    asyncio.run(run())
