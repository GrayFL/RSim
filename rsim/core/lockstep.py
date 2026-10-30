"""Explicit single-process stage order, independent of the resource DAG."""
import asyncio
from collections.abc import Mapping
from dataclasses import asdict, dataclass
import math
import time
import uuid


@dataclass(frozen=True)
class StepContext:
    session: str
    episode: int
    step_index: int

    def __post_init__(self):
        if (not isinstance(self.session, str) or not self.session
                or type(self.episode) is not int or self.episode < 0
                or type(self.step_index) is not int or self.step_index < 0):
            raise ValueError('context requires a session and nonnegative integer episode/step_index')

    @classmethod
    def from_mapping(cls, value):
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping) or set(value) != {'session', 'episode', 'step_index'}:
            raise ValueError('invalid step context')
        return cls(**value)

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class StepAck:
    context: StepContext
    task_id: str


@dataclass(frozen=True)
class CommandIdentity:
    controller_id: str
    controller_epoch: str
    sequence: int

    def __post_init__(self):
        if (any(not isinstance(value, str) or not value
                for value in (self.controller_id, self.controller_epoch))
                or type(self.sequence) is not int or self.sequence < 1):
            raise ValueError('command identity requires controller id, epoch and positive sequence')

    @classmethod
    def from_mapping(cls, value):
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping) or set(value) != {'controller_id', 'controller_epoch', 'sequence'}:
            raise ValueError('invalid command identity')
        return cls(**value)

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class CommandAccepted:
    """Owner confirmation of a staged command, never evidence of advancement."""
    context: StepContext
    command_id: CommandIdentity
    deadline_ns: int
    source_deadline_ns: int | None = None
    port: str | None = None

    def __post_init__(self):
        if (not isinstance(self.context, StepContext)
                or not isinstance(self.command_id, CommandIdentity)
                or type(self.deadline_ns) is not int
                or (self.source_deadline_ns is not None and type(self.source_deadline_ns) is not int)
                or (self.port is not None and (not isinstance(self.port, str) or not self.port))):
            raise ValueError('invalid CommandAccepted ACK')

    @classmethod
    def from_mapping(cls, value):
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping) or value.get('kind') != 'CommandAccepted':
            raise ValueError('expected CommandAccepted ACK')
        try:
            return cls(StepContext.from_mapping(value['context']),
                       CommandIdentity.from_mapping(value['command_id']), value['deadline_ns'],
                       value.get('source_deadline_ns'), value.get('port'))
        except (KeyError, TypeError) as error:
            raise ValueError('invalid CommandAccepted ACK') from error

    def to_dict(self):
        return dict(kind='CommandAccepted', **asdict(self))


@dataclass(frozen=True)
class PhysicsAdvanced:
    """Committed control boundary after all fixed-dt substeps and publication.

    ``state`` is an opaque provider summary; RSim does not interpret its model.
    Aborted or partially completed transactions cannot construct this ACK.
    """
    request_context: StepContext
    context: StepContext
    command_id: CommandIdentity
    requested_substeps: int
    actual_substeps: int
    physical_substeps: int
    stamp_ns: int
    phase: str = 'READY'
    state: object = None

    def __post_init__(self):
        if (not isinstance(self.request_context, StepContext) or not isinstance(self.context, StepContext)
                or not isinstance(self.command_id, CommandIdentity)
                or type(self.requested_substeps) is not int or self.requested_substeps < 1
                or type(self.actual_substeps) is not int or self.actual_substeps != self.requested_substeps
                or type(self.physical_substeps) is not int or self.physical_substeps < self.actual_substeps
                or type(self.stamp_ns) is not int or self.phase != 'READY'
                or self.context != StepContext(self.request_context.session, self.request_context.episode,
                                               self.request_context.step_index + 1)):
            raise ValueError('invalid or incomplete PhysicsAdvanced ACK')

    @classmethod
    def from_mapping(cls, value):
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping) or value.get('kind') != 'PhysicsAdvanced':
            raise ValueError('expected PhysicsAdvanced ACK')
        try:
            return cls(StepContext.from_mapping(value['request_context']),
                       StepContext.from_mapping(value['context']),
                       CommandIdentity.from_mapping(value['command_id']), value['requested_substeps'],
                       value['actual_substeps'], value['physical_substeps'], value['stamp_ns'],
                       value['phase'], value.get('state'))
        except (KeyError, TypeError) as error:
            raise ValueError('invalid PhysicsAdvanced ACK') from error

    def to_dict(self):
        return dict(kind='PhysicsAdvanced', **asdict(self))


class SingleProcessStepper:
    """sample(k) -> compute(k) -> accepted command(k) -> advance(k+1).

    Callbacks own physics and timestamps; this class never infers order from a
    graph. The legacy mode accepts StepAck(context, 'command'). Strict mode
    requires a source context, requested_substeps, CommandAccepted and a matching
    PhysicsAdvanced. Its advance(context, accepted) callback receives the exact
    accepted identity. JSON owner replies can be passed without conversion.
    On timeout/failure/cancellation stop() is awaited and the session is aborted.
    Determinism additionally requires stable algorithms, seeds and logical
    budgets; this class makes no claim of cross-process barrier support.
    """

    def __init__(self, *, sample, compute, accept, advance, stop, timeout=5.,
                 context=None, require_physics_ack=False, requested_substeps=None):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('step timeout must be finite and positive')
        self.sample, self.compute, self.accept, self.advance, self.stop = sample, compute, accept, advance, stop
        self.timeout = timeout
        if context is not None and not isinstance(context, StepContext):
            raise ValueError('context must be the source StepContext')
        if require_physics_ack and (context is None or type(requested_substeps) is not int
                                    or requested_substeps < 1):
            raise ValueError('strict lockstep requires a source context and positive requested_substeps')
        if not require_physics_ack and requested_substeps is not None:
            raise ValueError('requested_substeps requires strict physics ACKs')
        self.context = context or StepContext(uuid.uuid4().hex, 0, 0)
        self.require_physics_ack, self.requested_substeps = require_physics_ack, requested_substeps
        self.last_command_ack = self.last_physics_ack = None
        self.needs_synchronization = False
        self.aborted = False
        self.trace = ()
        self.timings = ()
        self._lock = asyncio.Lock()

    async def _complete_transaction(self, callback):
        """Repeated caller cancellation cannot interrupt a safety transaction."""
        async def bounded():
            async with asyncio.timeout(self.timeout):
                return await callback()
        transaction = asyncio.create_task(bounded())
        cancellation = None
        while not transaction.done():
            try:
                await asyncio.shield(transaction)
            except asyncio.CancelledError as error:
                cancellation = error
        result = transaction.result()
        if cancellation is not None:
            raise cancellation
        return result

    async def step(self):
        async with self._lock:
            if self.aborted:
                raise RuntimeError('lockstep session aborted; reset explicitly')
            context, trace, timings = self.context, [], []
            loop = asyncio.get_running_loop()
            deadline = loop.time() + self.timeout
            def check_deadline():
                # Blocking callbacks or callbacks swallowing cancellation can
                # outlive asyncio.timeout; never let them authorize a step.
                if loop.time() >= deadline:
                    raise TimeoutError('lockstep stage exceeded its host deadline')
            async def stage(name, callback, *args):
                started = time.monotonic_ns()
                try:
                    return await callback(*args)
                finally:
                    timings.append((name, started, time.monotonic_ns()))
            try:
                async with asyncio.timeout(self.timeout):
                    state = await stage('sample', self.sample, context)
                    trace.append(('state', context))
                    check_deadline()
                    command = await stage('compute', self.compute, context, state)
                    trace.append(('compute', context))
                    check_deadline()
                    acknowledgement = await stage('accept', self.accept, context, command)
                    if self.require_physics_ack:
                        acknowledgement = CommandAccepted.from_mapping(acknowledgement)
                        if acknowledgement.context != context:
                            raise ValueError('CommandAccepted ACK context differs from the current source step')
                        if acknowledgement.deadline_ns <= time.monotonic_ns():
                            raise TimeoutError('CommandAccepted ACK already expired on host')
                        self.last_command_ack = acknowledgement
                    elif acknowledgement != StepAck(context, 'command'):
                        raise ValueError('command ACK does not match the current session/episode/step/task')
                    trace.append(('accepted', context))
                    check_deadline()
                    if self.require_physics_ack:
                        # A missing/invalid ACK cannot establish how far the
                        # owner progressed. Abort then query/reset the owner.
                        self.needs_synchronization = True
                        advanced = PhysicsAdvanced.from_mapping(await stage(
                            'advance', self.advance, context, acknowledgement))
                        if (advanced.request_context != context
                                or advanced.command_id != acknowledgement.command_id
                                or advanced.requested_substeps != self.requested_substeps):
                            raise ValueError('PhysicsAdvanced ACK does not match the accepted command/step')
                        previous = self.last_physics_ack
                        if previous is not None and (advanced.stamp_ns <= previous.stamp_ns
                                or advanced.physical_substeps != previous.physical_substeps + self.requested_substeps):
                            raise ValueError('PhysicsAdvanced ACK did not advance the source monotonically')
                        if (acknowledgement.source_deadline_ns is not None
                                and advanced.stamp_ns > acknowledgement.source_deadline_ns):
                            raise ValueError('PhysicsAdvanced ACK exceeded the fixed source command bound')
                        self.last_physics_ack = advanced
                        self.needs_synchronization = False
                    else:
                        await stage('advance', self.advance, context)
                    trace.append(('advance', context))
                    check_deadline()
            except BaseException:
                self.aborted = True
                self.trace = tuple(trace)
                self.timings = tuple(timings)
                await self._complete_transaction(self.stop)
                raise
            self.trace = tuple(trace)
            self.timings = tuple(timings)
            self.context = StepContext(context.session, context.episode, context.step_index + 1)
            return state, command

    async def reset(self, reset_source):
        async with self._lock:
            self.aborted = True
            async def transaction():
                await self.stop()
                fresh = await reset_source()
                if self.require_physics_ack:
                    fresh = StepContext.from_mapping(fresh)
                    if (fresh.step_index != 0 or (fresh.session == self.context.session
                                                and fresh.episode <= self.context.episode)):
                        raise ValueError('reset must return a new source generation at step zero')
                    self.context = fresh
                else:
                    self.context = StepContext(self.context.session, self.context.episode + 1, 0)
                self.last_command_ack = self.last_physics_ack = None
                self.needs_synchronization = False
                self.aborted, self.trace, self.timings = False, (), ()
            await self._complete_transaction(transaction)
