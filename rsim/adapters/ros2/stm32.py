"""ROS2 chassis ports; remote deadlines are bounded in the driver's clock."""
import asyncio
from collections import deque
import time

from rsim.core.component import PrimaryComponent, ComponentError
from rsim.core.commands import CommandSink, VelocityCommand, CommandRejected
from .context import RosContext


class DriverClock:
    def __init__(self):
        self.instance = None
        self.offset = self.received = 0

    def observe(self, instance, server_ns, sent, received):
        if self.instance is not None and instance != self.instance:
            raise ComponentError('STM32 driver restarted; reconnect explicitly')
        if received - sent > 50_000_000:
            if not self.received or received - self.received > 3_000_000_000:
                raise TimeoutError('no sufficiently fresh STM32 clock sample')
            return
        if not instance or server_ns <= 0 or received < sent:
            raise ComponentError('invalid STM32 clock sample')
        self.instance = instance
        # Subtract the entire response delay; transport never extends TTL.
        self.offset, self.received = server_ns - received, received

    def deadline(self, deadline, now):
        if not self.received or not 0 <= now - self.received <= 3_000_000_000:
            raise CommandRejected('STM32 clock sample expired')
        return deadline + self.offset


class Stm32Chassis(PrimaryComponent):
    def __init__(self, *, topics, driver=None, history=128, max_ttl=.5, ros=None,
                 remote_clock=False, feedback_sources=None):
        self.feedback_sources = dict(feedback_sources or {})
        if self.feedback_sources and set(self.feedback_sources) != {'odom', 'state'}:
            raise ValueError('feedback_sources requires odom and state Signals')
        super().__init__(ros if ros is not None else RosContext(), *((driver,) if driver else ()),
                         key="stm32:" + topics["odom"], history=history,
                         output_name="odom", clock="ros:system", inputs=tuple(self.feedback_sources.values()))
        self.odom = self.output
        self.topics = dict(topics)
        self.state = self.signal("state", history=history, clock="ros:system")
        self.velocity_feedback = self.signal("velocity_feedback", history=history, clock="host:monotonic")
        self.velocity = CommandSink(self, "velocity", self._apply_velocity, safe=self._safe_velocity,
                                    fallback=VelocityCommand(), max_ttl=max_ttl, hz=200)
        self.pending = deque(maxlen=256)
        self.subscriptions = []
        self.client = self.stop_client = self.clock_client = None
        self.remote_clock = remote_clock
        self.clock_bound = DriverClock()
        self.last_command = None

    def configuration(self):
        return (super().configuration(), tuple(sorted(self.topics.items())), self.velocity.guard.max_ttl_ns, self.remote_clock,
                tuple(child.configuration() for child in self.children))

    async def open(self):
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        from rsim_stm32.srv import SetVelocity, Stop
        node = self.children[0].node
        self.pending.clear()
        self.last_command = None
        self.previous = dict.fromkeys(self.feedback_sources, 0)
        self.clock_bound = DriverClock()
        for name, kind in (() if self.feedback_sources else (("odom", Odometry), ("diagnostics", DiagnosticArray))):
            self.subscriptions.append(node.create_subscription(kind, self.topics[name],
                lambda msg, name=name: self.pending.append((name, msg, time.time_ns())), 100))
        self.client = node.create_client(SetVelocity, self.topics["set_velocity"])
        self.stop_client = node.create_client(Stop, self.topics["stop"])
        if self.remote_clock:
            from rsim_stm32.srv import Clock
            self.clock_client = node.create_client(Clock, self.topics['clock'])
        self.task("samples", self._samples, hz=500)
        async with asyncio.timeout(10):
            while not (self.client.service_is_ready() and self.stop_client.service_is_ready()):
                await asyncio.sleep(.01)
            if self.remote_clock:
                while not self.clock_client.service_is_ready():
                    await asyncio.sleep(.01)
                while self.clock_bound.instance is None:
                    try:
                        await self.synchronize()
                    except TimeoutError:
                        await asyncio.sleep(.05)
        if self.remote_clock:
            self.task('driver-clock', self.synchronize, hz=1)

    async def synchronize(self):
        from rsim_stm32.srv import Clock
        sent = time.monotonic_ns()
        response = await self._rpc(self.clock_client, Clock.Request())
        self.clock_bound.observe(response.instance, response.monotonic_ns, sent, time.monotonic_ns())

    async def _samples(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        for name, source in self.feedback_sources.items():
            if not source.frames:
                continue
            await source.get(timeout=.01)  # propagate upstream failure
            for frame in source.frames:
                if frame.sequence <= self.previous[name]:
                    continue
                self.previous[name] = frame.sequence
                value = frame.data
                if name == 'state':
                    statuses = value['statuses']
                    if not statuses:
                        continue
                    status = statuses[0]
                    value = {**status['values'], 'normal': status['level'] == 0, 'message': status['message']}
                await getattr(self, name).publish(value, stamp_ns=frame.stamp_ns,
                    clock=frame.clock, received_ns=frame.received_ns)
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
        if self.remote_clock:
            request.deadline_ns = self.clock_bound.deadline(envelope.deadline_ns, time.monotonic_ns())
            request.driver_instance = self.clock_bound.instance
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
        for client in (self.client, self.stop_client, self.clock_client):
            if client is not None:
                node.destroy_client(client)
        self.client = self.stop_client = self.clock_client = None
        self.pending.clear()
