import uuid
import math
import time
from rsim.core.component import Component

class RosContext(Component):
    process_local = True

    def __init__(self, *, hz=1000, max_callbacks=1, max_poll_s=.002):
        super().__init__(key="rsim:ros-context")
        if not isinstance(max_callbacks, int) or isinstance(max_callbacks, bool) or max_callbacks < 1:
            raise ValueError('max_callbacks must be a positive integer')
        if not math.isfinite(max_poll_s) or max_poll_s <= 0:
            raise ValueError('max_poll_s must be positive and finite')
        self.hz = hz
        self.max_callbacks, self.max_poll_s = max_callbacks, max_poll_s
        self.node = self.context = self.executor = None

    def configuration(self):
        return super().configuration(), self.hz, self.max_callbacks, self.max_poll_s

    async def open(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        self.context = Context()
        rclpy.init(
            args=[],
            context=self.context,
            signal_handler_options=SignalHandlerOptions.NO
            )
        self.node = rclpy.create_node(
            "rsim_" + uuid.uuid4().hex, context=self.context
            )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.task("dds-poll", self.poll, hz=self.hz)

    async def poll(self):
        # A busy multi-sensor node needs several ready callbacks per tick.
        # Bound both the batch size and elapsed time to yield to asyncio tasks.
        # This budget cannot preempt an individual blocking ROS callback.
        deadline = time.monotonic() + self.max_poll_s
        for _ in range(self.max_callbacks):
            self.executor.spin_once(timeout_sec=0)
            if time.monotonic() >= deadline:
                break

    async def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None:
            self.context.try_shutdown()
        self.node = self.context = self.executor = None
