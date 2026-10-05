import uuid
from rsim.core.component import Component

class RosContext(Component):
    process_local = True

    def __init__(self, *, hz=1000):
        super().__init__(key="rsim:ros-context")
        self.hz = hz
        self.node = self.context = self.executor = None

    def configuration(self):
        return super().configuration(), self.hz

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
        self.executor.spin_once(timeout_sec=0)

    async def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None:
            self.context.try_shutdown()
        self.node = self.context = self.executor = None
