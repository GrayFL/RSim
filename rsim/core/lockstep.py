"""Explicit single-process stage order, independent of the resource DAG."""
import asyncio
from dataclasses import dataclass
import math
import uuid


@dataclass(frozen=True)
class StepContext:
    session: str
    episode: int
    step_index: int


@dataclass(frozen=True)
class StepAck:
    context: StepContext
    task_id: str


class SingleProcessStepper:
    """sample(k) -> compute(k) -> accepted command(k) -> advance(k+1).

    Callbacks own physics and timestamps; this class never infers order from a
    graph. accept must return StepAck(context, 'command') after actual acceptance.
    On timeout/failure/cancellation stop() is awaited and the session is aborted.
    Determinism additionally requires stable algorithms, seeds and logical
    budgets; this class makes no claim of cross-process barrier support.
    """

    def __init__(self, *, sample, compute, accept, advance, stop, timeout=5.):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('step timeout must be finite and positive')
        self.sample, self.compute, self.accept, self.advance, self.stop = sample, compute, accept, advance, stop
        self.timeout = timeout
        self.context = StepContext(uuid.uuid4().hex, 0, 0)
        self.aborted = False
        self.trace = ()
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
            context, trace = self.context, []
            loop = asyncio.get_running_loop()
            deadline = loop.time() + self.timeout
            def check_deadline():
                # Blocking callbacks or callbacks swallowing cancellation can
                # outlive asyncio.timeout; never let them authorize a step.
                if loop.time() >= deadline:
                    raise TimeoutError('lockstep stage exceeded its host deadline')
            try:
                async with asyncio.timeout(self.timeout):
                    state = await self.sample(context)
                    trace.append(('state', context))
                    check_deadline()
                    command = await self.compute(context, state)
                    trace.append(('compute', context))
                    check_deadline()
                    acknowledgement = await self.accept(context, command)
                    if acknowledgement != StepAck(context, 'command'):
                        raise ValueError('command ACK does not match the current session/episode/step/task')
                    trace.append(('accepted', context))
                    check_deadline()
                    await self.advance(context)
                    trace.append(('advance', context))
                    check_deadline()
            except BaseException:
                self.aborted = True
                self.trace = tuple(trace)
                await self._complete_transaction(self.stop)
                raise
            self.trace = tuple(trace)
            self.context = StepContext(context.session, context.episode, context.step_index + 1)
            return state, command

    async def reset(self, reset_source):
        async with self._lock:
            self.aborted = True
            async def transaction():
                await self.stop()
                await reset_source()
                self.context = StepContext(self.context.session, self.context.episode + 1, 0)
                self.aborted, self.trace = False, ()
            await self._complete_transaction(transaction)
