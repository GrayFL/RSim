"""Factory declarations are checked before resources open and before readiness."""
import asyncio
from dataclasses import dataclass, replace
import uuid

import pytest

from rsim.core import Component, CommandSink, ComponentError, VelocityCommand
from rsim.devices import ChassisPortsDevice
from rsim.runtime import Runtime, SharedProvider, PortSpec, ProcessPlacement, component_ports, validate_component_ports


class Fixture(Component):
    def __init__(self, mutation=None, placed=False):
        super().__init__()
        if placed:
            self.placement = ProcessPlacement('factory-owner')
        self.mutation = mutation
        self.pose = self.signal('pose', clock='fixture')
        self.state = self.signal('state', clock='fixture')
        self.extra = self.signal('extra', clock='fixture')
        self.velocity = CommandSink(self, 'velocity', self.apply, fallback=VelocityCommand())

    async def apply(self, command):
        await self.state.publish(command.value, stamp_ns=1, clock='fixture')

    async def open(self):
        if self.mutation == 'clock':
            self.pose.clock = 'changed'
        elif self.mutation == 'ports':
            self.surprise = self.signal('surprise', clock='fixture')
        elif self.mutation == 'fallback':
            self.velocity.fallback = VelocityCommand(.1)
        await self.pose.publish(1, stamp_ns=1, clock=self.pose.clock)
        await self.state.publish(0, stamp_ns=1, clock='fixture')
        await self.extra.publish(2, stamp_ns=1, clock='fixture')


@dataclass(frozen=True)
class Factory:
    mutation: str | None = None
    placed: bool = False

    def build(self):
        return Fixture(self.mutation, self.placed)

    def specs(self):
        return component_ports(Fixture())

    def shared(self, *, key, **kwargs):
        return SharedProvider(factory=self.build, ports=self.specs(), key=key,
                              interface_version='chassis-ports-v1', **kwargs)


@pytest.mark.parametrize('field,value', [('schema', 'unknown'), ('clock', 'wrong'),
                                        ('history_capacity', 4), ('direction', 'sink')])
def test_declared_output_fields_have_actionable_errors(field, value):
    source = Fixture()
    specs = component_ports(source)
    extra = {'max_ttl': .5} if field == 'direction' else {}
    specs['pose'] = replace(specs['pose'], **{field: value}, **extra)
    with pytest.raises(ValueError, match=rf"pose.*{field}.*expected.*actual"):
        validate_component_ports(source, specs)


def test_complete_port_set_and_safety_configuration_are_checked():
    source = Fixture()
    specs = component_ports(source)
    del specs['extra']
    with pytest.raises(ValueError, match='port set mismatch.*extra'):
        validate_component_ports(source, specs)
    specs = component_ports(source)
    specs['velocity'] = replace(specs['velocity'], max_ttl=.1)
    with pytest.raises(ValueError, match='velocity.*max_ttl.*expected.*actual'):
        SharedProvider(source, ports=specs, key='unused')
    specs['velocity'] = replace(specs['velocity'], max_ttl=.5, fallback=VelocityCommand(1))
    with pytest.raises(ValueError, match='velocity.*fallback.*expected.*actual'):
        validate_component_ports(source, specs)


@pytest.mark.parametrize('mutation', ['clock', 'ports', 'fallback'])
@pytest.mark.parametrize('placed', [False, True])
def test_open_cannot_change_declared_ports_before_worker_ready(tmp_path, mutation, placed):
    async def run():
        log = tmp_path / 'worker.log'
        provider = Factory(mutation, placed).shared(key='bad-factory:' + uuid.uuid4().hex, log_path=log)
        with pytest.raises(ComponentError, match='provider port'):
            async with Runtime(provider):
                pytest.fail('worker became ready with changed ports')
        assert 'ValueError: provider port' in log.read_text()
        assert not provider.directory.exists()
    asyncio.run(run())


@pytest.mark.parametrize('placed', [False, True])
def test_bound_factory_spawns_and_public_client_connects_subset(placed):
    async def run():
        key = 'factory:' + uuid.uuid4().hex
        provider = Factory(placed=placed).shared(key=key)
        client = ChassisPortsDevice(key=key, clock='fixture', history=3)
        assert client.connection is None
        assert not hasattr(client, 'factory')
        async with Runtime(provider, _root_ports=False), Runtime(client, client.velocity):
            assert (await client.pose.get(timeout=10)).data == 1
            assert set(client.port_specs) == {'pose', 'state', 'velocity'}
            assert set(provider.manifest['ports']) == {'pose', 'state', 'extra', 'velocity'}
            await client.velocity.set(VelocityCommand(.2), ttl=.2)
            frame = await client.state.get(timeout=5)
            while not isinstance(frame.data, VelocityCommand):
                frame = await client.state.get(after=frame.sequence, timeout=5)
            assert frame.data == VelocityCommand(.2)
            wrong = ChassisPortsDevice(key=key, clock='wrong')
            with pytest.raises(ComponentError, match='pose.*clock.*expected.*actual'):
                async with Runtime(wrong):
                    pass
    asyncio.run(run())


def test_local_view_and_connection_construction_do_not_open_resources(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('connection construction started a provider')
    monkeypatch.setattr(SharedProvider, 'open', forbidden)
    source = Fixture()
    ports = ChassisPortsDevice.from_component(source)
    assert ports.pose is source.pose and ports.velocity is source.velocity
    assert source._closed and source._runtime is None
    client = ChassisPortsDevice(key='not-running', clock='fixture')
    assert client.connection is None
    source.state.clock = 'other'
    with pytest.raises(ValueError, match='same source clock'):
        ChassisPortsDevice.from_component(source)


def test_mutable_fallbacks_are_not_shared_between_views():
    from rsim.runtime import SharedComponent
    fallback = {'values': [0]}
    specs = {'command': PortSpec(direction='sink', max_ttl=.5, fallback=fallback)}
    first, second = (SharedComponent(key='unused', ports=specs) for _ in range(2))
    first.port_specs['command'].fallback['values'].append(1)
    assert second.port_specs['command'].fallback == {'values': [0]}
    assert fallback == {'values': [0]}
