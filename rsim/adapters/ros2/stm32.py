"""ROS2 serial chassis ports with same-host, monotonic command envelopes."""
import asyncio
from collections import deque
import time

from rsim.core.component import PrimaryComponent, ComponentError
from rsim.core.commands import CommandSink, VelocityCommand, CommandRejected
from .context import RosContext


class Stm32Chassis(PrimaryComponent):
    def __init__(self, *, topics, driver=None, history=128, max_ttl=.5):
        super().__init__(RosContext(), *((driver,) if driver else ()),
                         key="stm32:" + topics["odom"], history=history,
                         output_name="odom", clock="ros:system")
        self.odom = self.output
        self.topics = dict(topics)
        self.state = self.signal("state", history=history, clock="ros:system")
        self.velocity_feedback = self.signal("velocity_feedback", history=history, clock="host:monotonic")
        self.velocity = CommandSink(self, "velocity", self._apply_velocity, safe=self._safe_velocity,
                                    fallback=VelocityCommand(), max_ttl=max_ttl, hz=200)
        self.pending = deque(maxlen=256)
        self.subscriptions = []
        self.client = self.stop_client = None
        self.last_command = None

    def configuration(self):
        return (super().configuration(), tuple(sorted(self.topics.items())), self.velocity.guard.max_ttl_ns,
                tuple(child.configuration() for child in self.children))

    async def open(self):
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        from rsim_stm32.srv import SetVelocity, Stop
        node = self.children[0].node
        self.pending.clear()
        self.last_command = None
        for name, kind in (("odom", Odometry), ("diagnostics", DiagnosticArray)):
            self.subscriptions.append(node.create_subscription(kind, self.topics[name],
                lambda msg, name=name: self.pending.append((name, msg, time.time_ns())), 100))
        self.client = node.create_client(SetVelocity, self.topics["set_velocity"])
        self.stop_client = node.create_client(Stop, self.topics["stop"])
        self.task("samples", self._samples, hz=500)
        async with asyncio.timeout(10):
            while not (self.client.service_is_ready() and self.stop_client.service_is_ready()):
                await asyncio.sleep(.01)

    async def _samples(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        while self.pending:
            name, message, received = self.pending.popleft()
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            if name == "odom":
                await self.odom.publish(message_to_ordereddict(message), stamp_ns=stamp,
                                        clock="ros:system", received_ns=received)
            elif message.status:
                status = message.status[0]
                value = {item.key: item.value for item in status.values}
                value.update(normal=status.level == status.OK, message=status.message)
                await self.state.publish(value, stamp_ns=stamp, clock="ros:system", received_ns=received)

    async def _rpc(self, client, request):
        future = client.call_async(request)
        try:
            async with asyncio.timeout(.3):
                # rclpy owns the future; its executor is an independent metered task.
                while not future.done():
                    await asyncio.sleep(.001)
            return future.result()
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    async def _apply_velocity(self, envelope):
        from rsim_stm32.srv import SetVelocity
        value = envelope.value
        if not isinstance(value, VelocityCommand):
            raise CommandRejected("velocity sink requires VelocityCommand")
        request = SetVelocity.Request(controller_id=envelope.controller_id,
            controller_epoch=envelope.controller_epoch, sequence=envelope.sequence,
            deadline_ns=envelope.deadline_ns, linear_x=value.linear_x, angular_z=value.angular_z)
        response = await self._rpc(self.client, request)
        if not response.accepted:
            # A zero from an already acknowledged owner can stop even after TTL.
            if (value == VelocityCommand() and self.last_command is not None and
                (envelope.controller_id, envelope.controller_epoch) ==
                (self.last_command.controller_id, self.last_command.controller_epoch)):
                await self._safe_velocity(value)
                return {"stopped": True}
            raise CommandRejected(response.reason,
                                  reason="expired" if response.reason == "expired" else "invalid")
        self.last_command = envelope
        ack = dict(transmitted_ns=response.transmitted_ns, transmit_sequence=response.transmit_sequence,
                   meaning="host serial write completed; not a motor acknowledgement")
        await self.velocity_feedback.publish(dict(command=value, ack=ack), stamp_ns=time.monotonic_ns(),
                                             clock="host:monotonic")
        return ack

    async def _safe_velocity(self, _):
        from rsim_stm32.srv import Stop
        if self.last_command is None or self.stop_client is None:
            return
        command = self.last_command
        response = await self._rpc(self.stop_client, Stop.Request(controller_id=command.controller_id,
                                                                 controller_epoch=command.controller_epoch))
        if not response.stopped:
            raise ComponentError(response.reason)

    async def close(self):
        # Runtime closes command sinks before the ROS context/native driver.
        node = self.children[0].node
        for subscription in self.subscriptions:
            node.destroy_subscription(subscription)
        self.subscriptions.clear()
        for client in (self.client, self.stop_client):
            if client is not None:
                node.destroy_client(client)
        self.client = self.stop_client = None
        self.pending.clear()
