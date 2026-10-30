"""Explicit compute completion, with bounded idempotence and host deadlines.

Input revisions are opaque strings. The domain callback must verify the actual
snapshot represented by that revision; receiving a request is not completion.
"""
import asyncio
from collections import OrderedDict
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math
import time

from .lockstep import StepContext


@dataclass(frozen=True)
class StageRequest:
    context: StepContext
    task_id: str
    input_revision: str

    def __post_init__(self):
        if (not isinstance(self.context, StepContext)
                or not isinstance(self.task_id, str) or not self.task_id
                or not isinstance(self.input_revision, str) or not self.input_revision):
            raise ValueError('stage request requires context, task id and input revision')

    @classmethod
    def from_mapping(cls, value):
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping) or set(value) != {'kind', 'context', 'task_id', 'input_revision'}:
            raise ValueError('invalid StageRequest')
        if value['kind'] != 'StageRequest':
            raise ValueError('expected StageRequest')
        return cls(StepContext.from_mapping(value['context']), value['task_id'], value['input_revision'])

    def to_dict(self):
        return dict(kind='StageRequest', **asdict(self))


@dataclass(frozen=True)
class StageAck:
    context: StepContext
    task_id: str
    input_revision: str
    started_ns: int
    finished_ns: int
    result: object = None

    def __post_init__(self):
        StageRequest(self.context, self.task_id, self.input_revision)
        if (type(self.started_ns) is not int or type(self.finished_ns) is not int
                or self.started_ns < 0 or self.finished_ns < self.started_ns):
            raise ValueError('stage ACK requires ordered host timestamps')
        # Results describe outputs, not arbitrary executable objects or arrays.
        json.dumps(self.result, allow_nan=False)

    @property
    def request(self):
        return StageRequest(self.context, self.task_id, self.input_revision)

    @classmethod
    def from_mapping(cls, value):
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping) or value.get('kind') != 'StageAck':
            raise ValueError('expected StageAck')
        try:
            return cls(StepContext.from_mapping(value['context']), value['task_id'],
                       value['input_revision'], value['started_ns'], value['finished_ns'], value.get('result'))
        except (KeyError, TypeError) as error:
            raise ValueError('invalid StageAck') from error

    def to_dict(self):
        return dict(kind='StageAck', **asdict(self))


class StageFailed(ValueError):
    def __init__(self, message, *, reason='invalid'):
        super().__init__(message)
        self.reason = reason


class StageExecutor:
    """One worker-local stage; callbacks finish before a success is cached.

    Callback failures are terminal for the generation. A reset to a genuinely
    new source generation is required. Cancelled callers wait for the bounded
    transaction, then receive cancellation; retries never compute twice.
    """
    def __init__(self, task_id, callback, *, timeout=5., ack_capacity=32):
        if not isinstance(task_id, str) or not task_id or not callable(callback):
            raise ValueError('stage requires a task id and callable')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('stage timeout must be finite and positive')
        if type(ack_capacity) is not int or ack_capacity < 1:
            raise ValueError('ACK capacity must be a positive integer')
        self.task_id, self.callback = task_id, callback
        self.timeout, self.ack_capacity = timeout, ack_capacity
        self.context = None
        self.phase, self.error, self.high_water = 'READY', None, -1
        self._results = OrderedDict()
        self._retired_sessions = set()
        self._lock = asyncio.Lock()
        self._pending = set()
        self._closed = False

    def __getstate__(self):
        raise TypeError('StageExecutor is worker-local; construct it in Component.open')

    def describe(self):
        return dict(task_id=self.task_id, context=None if self.context is None else self.context.to_dict(),
                    phase=self.phase, high_water=self.high_water, error=self.error,
                    ack_cache_size=len(self._results), ack_capacity=self.ack_capacity,
                    last_ack=next((value[1].to_dict() for value in reversed(self._results.values())
                                   if isinstance(value[1], StageAck)), None))

    def _generation(self, context):
        return context.session, context.episode

    async def reset(self, context, *, deadline_ns=None):
        context = StepContext.from_mapping(context)
        if context.step_index != 0:
            raise StageFailed('stage reset requires source step zero')
        async with self._lock:
            if self._closed:
                raise StageFailed('stage closed')
            if deadline_ns is not None and time.monotonic_ns() >= deadline_ns:
                raise StageFailed('stage reset request expired', reason='expired')
            if self.context is not None:
                if context == self.context:
                    return self.describe()  # Idempotent reset cannot rewind computation.
                if (context.session in self._retired_sessions
                        or (context.session == self.context.session and context.episode <= self.context.episode)):
                    raise StageFailed('stage reset requires a new source generation', reason='stale_context')
                if context.session != self.context.session:
                    self._retired_sessions.add(self.context.session)
            self.context = context
            self._results.clear()
            self.phase, self.error, self.high_water = 'READY', None, -1
            return self.describe()

    async def execute(self, request, *, deadline_ns=None):
        request = StageRequest.from_mapping(request)
        if request.task_id != self.task_id:
            raise StageFailed('request task id does not match stage')
        if deadline_ns is not None and type(deadline_ns) is not int:
            raise StageFailed('stage deadline must be integer host nanoseconds')
        task = asyncio.create_task(self._execute(request, deadline_ns))
        self._pending.add(task)
        cancelled = False
        try:
            while not task.done():
                try:
                    await asyncio.shield(task)
                except asyncio.CancelledError:
                    if task.cancelled():
                        raise
                    cancelled = True
            result = task.result()
            if cancelled:
                raise asyncio.CancelledError
            return result
        finally:
            self._pending.discard(task)

    async def _execute(self, request, deadline_ns):
        async with self._lock:
            if self._closed:
                raise StageFailed('stage closed')
            if self.context is None:
                if request.context.step_index != 0:
                    raise StageFailed('new stage must start at source step zero')
                self.context = request.context
            if self._generation(request.context) != self._generation(self.context):
                raise StageFailed('stage generation differs; reset explicitly', reason='stale_context')
            cached = self._results.get(request.context)
            if cached is not None:
                original, result = cached
                if original != request:
                    raise StageFailed('stage request identity was reused with different input revision')
                if isinstance(result, StageAck):
                    return deepcopy(result)
                raise StageFailed(result[0], reason=result[1])
            if self.phase == 'ABORTED':
                raise StageFailed('stage aborted; reset to a new source generation')
            if request.context.step_index != self.high_water + 1:
                raise StageFailed('stage step is out of order or no longer cached', reason='stale_context')
            started = time.monotonic_ns()
            expires = started + round(self.timeout * 1e9)
            if deadline_ns is not None:
                expires = min(expires, deadline_ns)
            self.phase = 'COMPUTING'
            try:
                if started >= expires:
                    raise TimeoutError('stage request expired before computation')
                async with asyncio.timeout((expires-started)/1e9):
                    result = await self.callback(request)
                finished = time.monotonic_ns()
                if finished >= expires:
                    raise TimeoutError('stage callback exceeded its host deadline')
                ack = StageAck(request.context, request.task_id, request.input_revision,
                               started, finished, deepcopy(result))
                self.phase = 'COMPLETED'
                self._remember(request, ack)
                return deepcopy(ack)
            except BaseException as error:
                self.phase, self.error = 'ABORTED', str(error) or type(error).__name__
                reason = 'expired' if isinstance(error, TimeoutError) else getattr(error, 'reason', 'invalid')
                self._remember(request, (self.error, reason))
                if isinstance(error, asyncio.CancelledError):
                    raise
                raise StageFailed(self.error, reason=reason) from error

    def _remember(self, request, result):
        self.high_water = request.context.step_index
        self._results[request.context] = (request, result)
        while len(self._results) > self.ack_capacity:
            self._results.popitem(last=False)

    async def aclose(self):
        self._closed = True
        if self._pending:
            await asyncio.gather(*tuple(self._pending), return_exceptions=True)


class StageBarrier:
    """Explicit ordered stage ACKs, composable inside a strict stepper compute.

    Call begin(context), then call(sink, request) for each declared task, then
    complete(). Callbacks on the source owner remain separate. Only identical
    requests are retried after a lost response; a worker's callback is cached.
    """
    def __init__(self, task_ids, *, context, timeout=1., retries=1, request_ttl=.5):
        self.task_ids = tuple(task_ids)
        if (not self.task_ids or len(set(self.task_ids)) != len(self.task_ids)
                or any(not isinstance(task, str) or not task for task in self.task_ids)):
            raise ValueError('barrier task ids must be nonempty and unique')
        if (not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(request_ttl)
                or request_ttl <= 0 or type(retries) is not int or retries < 0):
            raise ValueError('invalid barrier timeout, request TTL or retry count')
        self.context = StepContext.from_mapping(context)
        self.timeout, self.retries, self.request_ttl = timeout, retries, request_ttl
        self.phase, self.index = 'READY', 0
        self.acks, self.timings = [], []
        self._lock = asyncio.Lock()
        self._retired_sessions = set()

    def __getstate__(self):
        raise TypeError('StageBarrier is coordinator-local; construct it after Runtime opens')

    def begin(self, context):
        if self.phase != 'READY' or context != self.context:
            raise StageFailed('barrier is not ready at this source context')
        self.phase, self.index = 'COMPUTING', 0
        self.acks, self.timings = [], []

    async def call(self, sink, request):
        async with self._lock:
            return await self._call(sink, request)

    async def _call(self, sink, request):
        started, attempts = time.monotonic_ns(), 0
        try:
            request = StageRequest.from_mapping(request)
            if (self.phase != 'COMPUTING' or self.index >= len(self.task_ids)
                    or request.context != self.context or request.task_id != self.task_ids[self.index]):
                raise StageFailed('barrier stage order or context mismatch')
            deadline = started + round(self.timeout*1e9)
            for attempt in range(self.retries+1):
                attempts += 1
                remaining = (deadline-time.monotonic_ns())/1e9
                if remaining <= 0:
                    raise TimeoutError('stage barrier host deadline exceeded')
                try:
                    # Each retry has a fresh transport envelope, but precisely
                    # the same logical request. It cannot renew a computation.
                    async with asyncio.timeout(remaining/(self.retries+1-attempt)):
                        reply = await sink.set(request.to_dict(), ttl=min(self.request_ttl, remaining))
                    ack = StageAck.from_mapping(reply)
                    if ack.request != request:
                        raise StageFailed('stage ACK context/task/input revision mismatch')
                    if time.monotonic_ns() >= deadline:
                        raise TimeoutError('stage ACK arrived after barrier deadline')
                    self.acks.append(ack)
                    self.index += 1
                    return ack
                except TimeoutError:
                    if attempt == self.retries:
                        raise
            raise AssertionError('unreachable stage retry loop')
        except BaseException:
            self.phase = 'ABORTED'
            raise
        finally:
            self.timings.append(dict(started_ns=started, finished_ns=time.monotonic_ns(), attempts=attempts))

    def complete(self):
        if self.phase != 'COMPUTING' or self.index != len(self.task_ids):
            self.phase = 'ABORTED'
            raise StageFailed('not all computation stages acknowledged completion')
        self.phase = 'READY'
        self.context = StepContext(self.context.session, self.context.episode, self.context.step_index+1)
        return tuple(self.acks)

    def reset(self, context):
        if self._lock.locked():
            raise StageFailed('wait for the current barrier transaction before resetting')
        context = StepContext.from_mapping(context)
        if (context.step_index != 0 or context.session in self._retired_sessions
                or (context.session == self.context.session
                                      and context.episode <= self.context.episode)):
            raise StageFailed('barrier reset requires a new source generation at step zero')
        if context.session != self.context.session:
            self._retired_sessions.add(self.context.session)
        self.context, self.phase, self.index = context, 'READY', 0
        self.acks, self.timings = [], []
