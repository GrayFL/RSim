"""Reset/seek isolate timestamp history locally and at imported boundaries."""
import asyncio
import json

import pytest

from rsim.core import Component, HistoryMiss, Frame
from rsim.runtime import Runtime
from rsim.runtime.port_binding import PortImporter, SharedMemoryChannel
from rsim.transport.shared import SharedStore


def test_reset_clears_timestamp_history_without_rewinding_waiter_cursor():
    async def run():
        source = Component()
        output = source.signal('output', clock='simulation')
        async with Runtime(output):
            old = await output.publish('old', stamp_ns=100, clock='simulation', metadata={'episode': 0})
            waiter = asyncio.create_task(output.get(after=old.sequence, timeout=1))
            await output.clear_history()
            with pytest.raises(HistoryMiss):
                await output.get(timestamp_ns=100, clock='simulation')
            new = await output.publish('new', stamp_ns=0, clock='simulation', metadata={'episode': 1})
            assert await waiter is new
            assert new.sequence > old.sequence
            assert new.sample_id.publication_sequence > old.sample_id.publication_sequence
            assert old.data == 'old'
    asyncio.run(run())


def test_episode_change_isolates_remote_imported_history(tmp_path):
    async def run():
        transport = Component()
        importer = PortImporter(SharedMemoryChannel(str(tmp_path), 8, 'simulation'), transport)
        # Exercise the receive path without DDS: descriptors are exactly those
        # used by a real PortExporter; the transport lease has separate tests.
        importer._closed = False
        store = SharedStore(tmp_path)
        try:
            old = Frame('old', 100, 'simulation', sequence=1, metadata={'episode': 0})
            importer._enqueue(json.dumps(store.put(old)))
            await importer.receive()
            new = Frame('new', 0, 'simulation', sequence=2, metadata={'episode': 1})
            importer._enqueue(json.dumps(store.put(new)))
            await importer.receive()
            assert len(importer.output.frames) == 1
            assert (await importer.output.get()).data == 'new'
            with pytest.raises(HistoryMiss):
                await importer.output.get(timestamp_ns=100, clock='simulation')
        finally:
            store.close()
    asyncio.run(run())


def test_ordinary_timestamp_history_and_received_wall_time_are_unchanged():
    async def run():
        source = Component()
        output = source.signal('output', clock='source')
        async with Runtime(output):
            first = await output.publish(1, stamp_ns=1, clock='source', received_ns=123)
            await output.publish(2, stamp_ns=2, clock='source')
            assert await output.get(timestamp_ns=1, clock='source') is first
            assert first.received_ns == 123
    asyncio.run(run())
