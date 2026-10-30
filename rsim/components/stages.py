"""A generic compute stage exposed through a declared command port."""
from rsim.core.component import Component
from rsim.core.commands import CommandRejected, CommandSink
from rsim.core.stages import StageExecutor, StageFailed, StageRequest


class StageEndpoint(Component):
    """Subclass compute(request); declare its input/dependency Components.

    The callback must wait for and verify the requested input revision, perform
    the calculation, publish any outputs, and return a small JSON summary.
    Methods use the declared port, so they also work with ProcessPlacement.
    """
    def __init__(self, task_id, *dependencies, inputs=(), timeout=5., ack_capacity=32, max_ttl=.5):
        super().__init__(*dependencies, inputs=inputs)
        self.task_id, self.stage_timeout, self.ack_capacity = task_id, timeout, ack_capacity
        self.executor = None
        # Validate the pure configuration without opening resources.
        StageExecutor(task_id, self.compute, timeout=timeout, ack_capacity=ack_capacity)
        self.stage_request = CommandSink(self, 'stage_request', self._request,
                                        fallback=None, safe=lambda _: None, max_ttl=max_ttl, hz=500)

    async def open(self):
        self.executor = StageExecutor(self.task_id, self.compute, timeout=self.stage_timeout,
                                      ack_capacity=self.ack_capacity)

    async def compute(self, request):
        raise NotImplementedError('implement real input validation, computation and publication')

    async def _request(self, envelope):
        value = envelope.value
        if value is None:
            return None
        try:
            if isinstance(value, dict) and set(value) == {'kind'} and value['kind'] == 'StageDescribe':
                return self.executor.describe()
            if isinstance(value, dict) and set(value) == {'kind', 'context'} and value['kind'] == 'StageReset':
                return await self.executor.reset(value['context'], deadline_ns=envelope.deadline_ns)
            return (await self.executor.execute(StageRequest.from_mapping(value),
                                                 deadline_ns=envelope.deadline_ns)).to_dict()
        except (StageFailed, ValueError, TypeError) as error:
            reason = getattr(error, 'reason', 'invalid')
            if reason == 'retired':
                # Input generation retirement does not retire this management
                # client's transport controller; it must still be able to reset.
                reason = 'stale_context'
            raise CommandRejected(str(error), reason=reason) from error

    async def run_stage(self, request, *, ttl=None):
        return await self.stage_request.set(StageRequest.from_mapping(request).to_dict(), ttl=ttl)

    async def reset_stage(self, context, *, ttl=None):
        from rsim.core.lockstep import StepContext
        return await self.stage_request.set(dict(kind='StageReset', context=StepContext.from_mapping(context).to_dict()), ttl=ttl)

    async def describe_stage(self, *, ttl=None):
        return await self.stage_request.set(dict(kind='StageDescribe'), ttl=ttl)

    async def close(self):
        if self.executor is not None:
            await self.executor.aclose()
