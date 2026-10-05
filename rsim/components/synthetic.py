"""Deterministic source for testing without hardware."""
import time

import numpy as np

from rsim.core.component import PrimaryComponent
from rsim.transport.shared import allocate


class CounterArray(PrimaryComponent):
    def __init__(self, *, size=1024, hz=30, history=16):
        super().__init__(history=history)
        self.size, self.hz, self.count = size, hz, 0

    async def open(self):
        self.task("produce", self.produce, hz=self.hz)

    async def produce(self):
        self.count += 1
        array = allocate((self.size,), dtype=np.float64)
        array.fill(self.count)
        array.flags.writeable = False
        await self.publish(array, stamp_ns=time.time_ns(), clock="host:unix")
