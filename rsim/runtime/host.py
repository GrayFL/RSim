"""Legacy single-output view over the common shared-source lease strategy."""
from __future__ import annotations

from rsim.core.errors import ComponentError
from .process import ProcessSensor


class SharedSensor(ProcessSensor):
    """`key` identifies the physical source; `version` identifies its recipe.

    All clients with a key must specify the same version and history. Only the
    first client's factory runs. Use a configuration digest for version when a
    factory has parameters. Omitting factory makes this a connection-only client:
    it never starts a source or loads a provider's factory. Factories are trusted
    local Python code, serialized only inside the environment that launches them.

    provider_version optionally identifies driver-only settings: competing
    providers must match it, while connection-only clients need not know it.
    """

    def __init__(self, factory=None, *, key, version="1", history=16, hz=200, transport=None,
                 provider_version=None, output_name="output", options=None):
        super().__init__(factory, history=history, hz=hz, transport=transport, output_name=output_name, options=options)
        self.source_key, self.version = key, version
        self.provider_version = provider_version
        self._reader = self._writer = None

    async def open(self):
        from .shared_launch import open_sensor
        if self.transport.backend == 'inprocess':
            raise ValueError('SharedSensor requires a real cross-process DDS backend')
        self.options = (self.options or self._runtime.options).resolved()
        await open_sensor(self)

    async def receive(self):
        if self._reader.at_eof():
            raise ComponentError("shared source supervisor disconnected")
        await super().receive()

    async def close(self):
        if self._writer is not None:
            self._writer.close()
            await self._writer.wait_closed()
            self._writer = None
        self._reader = None
        initial = getattr(self, "_initial_client", None)
        if initial is not None:
            initial.close()
            self._initial_client = None
        if self.subscription is not None:
            self.subscription.close()
            self.subscription = None
        self.pending.clear()
