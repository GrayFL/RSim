"""Monotonic task pacing."""
import asyncio
import math

class Metronome:
    """Monotonic deadlines; skip missed ticks instead of catch-up bursts."""

    def __init__(self, hz: float):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("hz must be finite and positive")
        self.period = 1 / hz
        self.deadline = None
        self.missed = 0

    async def tick(self):
        loop = asyncio.get_running_loop()
        now = loop.time()
        if self.deadline is None:
            self.deadline = now
        else:
            self.deadline += self.period
            if self.deadline < now:
                skipped = math.ceil((now - self.deadline) / self.period)
                self.missed += skipped
                self.deadline += skipped * self.period
        await asyncio.sleep(max(0, self.deadline - loop.time()))
