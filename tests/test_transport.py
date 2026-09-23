import asyncio
import uuid

import pytest

from rsim import Runtime, TransportConfig
from rsim.transport import DescriptorTransport


@pytest.mark.parametrize("writer_backend,reader_backend", [
    ("cyclonedds", "cyclonedds"), ("ros2", "cyclonedds"),
    ("cyclonedds", "ros2"), ("ros2", "ros2"),
])
def test_backends_interoperate_and_deliver_retained_sample(writer_backend, reader_backend):
    if "ros2" in (writer_backend, reader_backend):
        pytest.importorskip("rclpy", exc_type=ImportError)

    async def run():
        publisher = DescriptorTransport(TransportConfig(writer_backend, 71))
        subscriber = DescriptorTransport(TransportConfig(reader_backend, 71))
        topic = "/rsim/test/p" + uuid.uuid4().hex
        received = []
        async with Runtime(publisher):
            writer = publisher.publisher(topic)
            writer.publish('retained: 中文 {"sequence": 1}')
            async with Runtime(subscriber):
                reader = subscriber.subscribe(topic, received.append)
                async with asyncio.timeout(10):
                    while not received:
                        await asyncio.sleep(0.01)
                assert received == ['retained: 中文 {"sequence": 1}']
                writer.publish("live: 2")
                async with asyncio.timeout(5):
                    while received[-1] != "live: 2":
                        await asyncio.sleep(0.01)
                reader.close()
            writer.close()
    asyncio.run(run())


def test_different_domains_do_not_exchange_descriptors():
    async def run():
        one = DescriptorTransport(TransportConfig("cyclonedds", 72))
        two = DescriptorTransport(TransportConfig("cyclonedds", 73))
        received = []
        async with Runtime(one, two):
            topic = "/rsim/test/p" + uuid.uuid4().hex
            writer = one.publisher(topic)
            two.subscribe(topic, received.append)
            writer.publish("isolated")
            await asyncio.sleep(0.2)
            assert not received
    asyncio.run(run())
