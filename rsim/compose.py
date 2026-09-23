"""Small building blocks. Factories may contain nested ProcessSensor instances."""
import inspect
import time

from .core import Frame, Sensor


class Map(Sensor):
    def __init__(self, source, function, *, hz=30, history=16):
        super().__init__(source, history=history)
        self.function, self.hz = function, hz
        self.previous = 0

    async def open(self):
        self.previous = 0
        self.task("transform", self.transform, hz=self.hz)

    async def transform(self):
        frame = await self.children[0].get(after=self.previous)
        result = self.function(frame.data)
        if inspect.isawaitable(result):
            result = await result
        if isinstance(result, Frame):
            await self.publish(result.data, stamp_ns=result.stamp_ns, clock=result.clock,
                               received_ns=result.received_ns)
        else:
            await self.publish(result, stamp_ns=frame.stamp_ns, clock=frame.clock,
                               received_ns=frame.received_ns)
        self.previous = frame.sequence


class Bundle(Sensor):
    """Latest samples, preserving each clock. This is not calibrated sensor fusion."""
    def __init__(self, *, hz=10, history=16, **sources):
        if not sources:
            raise ValueError("Bundle requires at least one source")
        super().__init__(*sources.values(), history=history)
        self.names, self.hz = tuple(sources), hz

    async def open(self):
        self.task("bundle", self.combine, hz=self.hz)

    async def combine(self):
        values = {}
        for name, child in zip(self.names, self.children):
            frame = await child.get()
            values[name] = {"data": frame.data, "stamp_ns": frame.stamp_ns,
                            "clock": frame.clock, "received_ns": frame.received_ns}
        await self.publish(values, stamp_ns=time.time_ns(), clock="host:unix")
