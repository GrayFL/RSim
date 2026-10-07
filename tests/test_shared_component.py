import asyncio
import os
from pathlib import Path
import uuid

import numpy as np
import pytest

from rsim import (Component, Runtime, SharedProvider, SharedComponent, PortSpec,
                  ProcessPlacement, LocalPlacement, PortNotBound, CommandSink, VelocityCommand,
                  CommandEnvelope, CommandRejected)
from rsim.runtime.registry import Connection, registry_paths


class MappingFixture(Component):
    def __init__(self):
        super().__init__()
        self.pose = self.signal('pose', clock='test')
        self.map = self.signal('map', clock='test', history=2)
        self.debug = self.signal('debug', clock='test')

    async def open(self):
        await self.map.publish(np.arange(32), stamp_ns=10, clock='test',
                               metadata={'session_id': 'example', 'revision': 1})
        self.task('pose', self.tick, hz=20)

    async def tick(self):
        await self.pose.publish({'pid': os.getpid(), 'map_shared': isinstance(self.map.frames[0].data, np.memmap)},
                                stamp_ns=20, clock='test')


def view(key):
    return SharedComponent(key=key, ports={'pose': PortSpec(clock='test'),
                                          'map': PortSpec(clock='test', history_capacity=4)})


@pytest.mark.parametrize('placed', [False, True])
def test_exact_dynamic_ports_retained_identity_and_independent_leases(placed):
    async def run():
        key = 'test:' + uuid.uuid4().hex
        source = MappingFixture()
        wrapper = Component()
        wrapper.expose('pose', source.pose)
        wrapper.expose('map', source.map)
        provider = SharedProvider(wrapper, key=key,
            placement={source: ProcessPlacement('compute') if placed else LocalPlacement()})
        launcher = Runtime(provider, _root_ports=False)
        await launcher.__aenter__()
        directory = provider.directory
        first, second = view(key), view(key)
        try:
            async with Runtime(first.pose):
                pose = await first.pose.get(timeout=15)
                assert pose.data['pid'] != os.getpid()
                assert (pose.data['pid'] != provider.manifest['pid']) == placed
                assert pose.data['map_shared'] is False
                assert not list(directory.rglob('*.npy'))
                actual_store = Path(first.bindings['pose']['storage_descriptor']['directory'])
                if placed:
                    assert not list(actual_store.parents[2].rglob('*.npy'))
                with pytest.raises(PortNotBound):
                    await first.map.get(timeout=1)
                async with Runtime(second.map):
                    frame = await second.map.get(timeout=10)
                    assert frame.sample_id is not None and frame.metadata['revision'] == 1
                    another = view(key)
                    async with Runtime(another.map):
                        copy = await another.map.get(timeout=10)
                        assert second.bindings['map']['binding_id'] == another.bindings['map']['binding_id']
                        assert frame.sample_id == copy.sample_id
                        a, b = os.stat(frame.data.filename), os.stat(copy.data.filename)
                        assert (a.st_dev, a.st_ino) == (b.st_dev, b.st_ino)
                        # Launcher is just another source lease.
                        await launcher.aclose()
                        assert (await first.pose.get(after=pose.sequence, timeout=2)).sample_id != pose.sample_id
                    binding = second.bindings['map']
                # Last map subscription stopped its exporter, but not pose/source.
                await asyncio.sleep(.15)
                again = view(key)
                async with Runtime(again.map):
                    retained = await again.map.get(timeout=10)
                    assert retained.sample_id == frame.sample_id
                    assert again.bindings['map']['binding_id'] != binding['binding_id']
                    assert again.bindings['map']['channel_generation'] > binding['channel_generation']
                    # Dynamic materialization must not rewrite the retained source Frame.
                    assert (await first.pose.get(timeout=2)).data['map_shared'] is False
        finally:
            await launcher.aclose()
        async with asyncio.timeout(10):
            while directory.exists():
                await asyncio.sleep(.05)
        assert np.array_equal(frame.data, np.arange(32))
    asyncio.run(run())


def test_shared_commands_keep_provider_exclusivity_deadlines_and_disconnect_stop():
    import time
    class Actuator(Component):
        def __init__(self):
            super().__init__()
            self.feedback = self.signal('feedback', clock='test')
            self.velocity = CommandSink(self, 'velocity', self.apply, fallback=VelocityCommand())

        async def apply(self, envelope):
            await self.feedback.publish(envelope.value, stamp_ns=1, clock='test')

    async def run():
        key = 'commands:' + uuid.uuid4().hex
        source = Actuator()
        provider = SharedProvider(source, key=key)
        specs = {'velocity': PortSpec(direction='sink', clock='host:monotonic', max_ttl=.5,
                                      fallback=VelocityCommand()), 'feedback': PortSpec(clock='test')}
        a, b = (SharedComponent(key=key, ports=specs) for _ in range(2))
        async with Runtime(provider, _root_ports=False), Runtime(a.velocity, a.feedback), Runtime(b.velocity):
            await asyncio.wait_for(a.endpoints['velocity'].ready.wait(), 5)
            command = CommandEnvelope(VelocityCommand(.1), 'a', 'one', 1, time.monotonic_ns() + 400_000_000)
            await a.velocity.set(command)
            accepted = await a.feedback.get(timeout=1)
            assert accepted.data.linear_x == .1
            with pytest.raises(CommandRejected, match='exclusive'):
                await b.velocity.set(VelocityCommand(.2), ttl=.3)
            with pytest.raises(CommandRejected, match='replayed'):
                await a.velocity.set(command)
            with pytest.raises(CommandRejected, match='expired'):
                await a.velocity.set(CommandEnvelope(VelocityCommand(), 'a', 'one', 2, time.monotonic_ns()-1))
            # Dropping the lease is independent of client-side safe callbacks.
            await a.connection.close()
            await asyncio.sleep(.1)
            await b.velocity.set(VelocityCommand(), ttl=.3)
            assert b.endpoints['velocity'].channel.instance_id == provider.manifest['instance_id']
    asyncio.run(run())


def test_manifest_only_and_simultaneous_views_share_one_importer():
    async def run():
        key = 'aliases:' + uuid.uuid4().hex
        provider = SharedProvider(MappingFixture(), key=key)
        async with Runtime(provider, _root_ports=False):
            assert provider.bindings == {}
            assert not list(provider.directory.rglob('*.npy'))
            a, b = view(key), view(key)
            async with Runtime(a.pose, b.pose):
                frame = await a.pose.get(timeout=5)
                assert await b.pose.get() is frame
                assert b._canonical is a
                assert set(a.bindings) == {'pose'}
    asyncio.run(run())


def test_independent_guard_cleans_blocked_provider_after_launcher_dies():
    import signal
    async def run():
        key = 'blocked:' + uuid.uuid4().hex
        provider = SharedProvider(MappingFixture(), key=key)
        launcher = Runtime(provider, _root_ports=False)
        await launcher.__aenter__()
        pid, directory = provider.manifest['pid'], provider.directory
        os.kill(pid, signal.SIGSTOP)
        # Model a client crash: drop the socket without cooperative provider work.
        await launcher.aclose()
        async with asyncio.timeout(12):
            while directory.exists():
                await asyncio.sleep(.05)
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
    asyncio.run(run())


@pytest.mark.parametrize('backend', ['cyclonedds', 'ros2'])
def test_v2_cross_environment_client_and_placement_do_not_import_ros(tmp_path, backend):
    import json
    from rsim import TransportConfig
    from test_cross_environment import GUARD
    executable = os.environ.get('RSIM_TEST_CLIENT_PYTHON')
    if not executable:
        pytest.skip('set RSIM_TEST_CLIENT_PYTHON to select the independent client interpreter')
    code = '''
import asyncio, json, os, sys
from pathlib import Path
import numpy as np
from rsim import Runtime, SharedComponent, PortSpec, ProcessPlacement, Map, TransportConfig
key = sys.argv[1]
source = SharedComponent(key=key, ports={'map': PortSpec(clock='test')}, transport=TransportConfig('cyclonedds', 78))
compute = Map(source.map, lambda array: array)
async def run():
    async with Runtime(source.map, compute.output, placement={compute: ProcessPlacement('compute', TransportConfig('cyclonedds', 78))}):
        first, second = await asyncio.gather(source.map.get(timeout=20), compute.get(timeout=20))
        assert isinstance(first.data, np.memmap) and isinstance(second.data, np.memmap)
        a,b = os.stat(first.data.filename), os.stat(second.data.filename)
        assert (a.st_dev,a.st_ino) == (b.st_dev,b.st_ino)
        assert first.metadata['revision'] == 1
        assert first.sample_id != second.sample_id  # Explicit computation is a new publication.
        assert not any(n.startswith(('rclpy','rsim.adapters.ros2')) for n in sys.modules)
        print(json.dumps({'same_inode': True, 'python': sys.version, 'no_ros': True}))
asyncio.run(run())
'''
    async def run():
        key = 'cross-v2:' + uuid.uuid4().hex
        provider = SharedProvider(MappingFixture(), key=key, transport=TransportConfig(backend, 78))
        (tmp_path / 'sitecustomize.py').write_text(GUARD)
        env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(tmp_path), str(Path.cwd()))))
        async with Runtime(provider, _root_ports=False):
            child = await asyncio.create_subprocess_exec(executable, '-c', code, key, env=env,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            try:
                out, err = await asyncio.wait_for(child.communicate(), 35)
                assert child.returncode == 0, (out+err).decode()
                assert json.loads(out)['no_ros']
            finally:
                if child.returncode is None:
                    child.kill()
                    await child.wait()
    asyncio.run(run())


def test_registry_requests_are_idempotent_and_client_cannot_start_provider():
    async def run():
        key = 'test:' + uuid.uuid4().hex
        client = view(key)
        with pytest.raises(Exception, match='not running'):
            async with Runtime(client.pose):
                pass
        provider = SharedProvider(MappingFixture(), key=key)
        async with Runtime(provider, _root_ports=False):
            connection = await Connection.connect(registry_paths(key, provider.transport.domain_id)[0])
            try:
                manifest = await connection.request('describe')
                for _ in range(2):
                    result = await connection.request('subscribe', request_id='one', port='map',
                                                       instance_id=manifest['instance_id'])
                    assert result['subscription_id'] == 'one'
                await connection.request('unsubscribe', subscription_id='one')
                await connection.request('unsubscribe', subscription_id='one')
                with pytest.raises(Exception, match='instance changed'):
                    await connection.request('subscribe', port='map', instance_id='old')
            finally:
                await connection.close()
    asyncio.run(run())


def test_alias_port_codec_error_does_not_kill_other_provider_ports():
    class BadMap(MappingFixture):
        async def open(self):
            await self.map.publish(object(), stamp_ns=10, clock='test')
            self.task('pose', self.tick, hz=20)

    async def run():
        key = 'port-error:' + uuid.uuid4().hex
        source, wrapper = BadMap(), Component()
        wrapper.expose('pose', source.pose)
        wrapper.expose('map', source.map)
        provider, client = SharedProvider(wrapper, key=key), view(key)
        async with Runtime(provider, _root_ports=False), Runtime(client.pose, client.map):
            pose = await client.pose.get(timeout=5)
            with pytest.raises(Exception, match="port 'map' failed"):
                await client.map.get(timeout=3)
            assert (await client.pose.get(after=pose.sequence, timeout=2)).sequence > pose.sequence
            assert client._failure is None
    asyncio.run(run())


def test_provider_restart_changes_identity_and_never_silently_rebinds_old_client():
    import signal
    async def run():
        key = 'restart:' + uuid.uuid4().hex
        provider, client = SharedProvider(MappingFixture(), key=key), view(key)
        async with Runtime(provider, _root_ports=False), Runtime(client.map):
            frame = await client.map.get(timeout=5)
            old_instance = client.manifest['instance_id']
            os.kill(provider.manifest['pid'], signal.SIGKILL)
            async with asyncio.timeout(5):
                while client._failure is None or provider.directory.exists():
                    await asyncio.sleep(.05)
            replacement, fresh = SharedProvider(MappingFixture(), key=key), view(key)
            async with Runtime(replacement, _root_ports=False), Runtime(fresh.map):
                new_frame = await fresh.map.get(timeout=5)
                assert fresh.manifest['instance_id'] != old_instance
                assert new_frame.sample_id != frame.sample_id
                with pytest.raises(Exception, match='task failed'):
                    await client.map.get(timeout=1)
    asyncio.run(run())


@pytest.mark.parametrize('placed', [False, True])
def test_nested_shared_provider_forwards_only_demanded_port_and_original_identity(placed):
    async def run():
        raw_key, derived_key = ('nested:' + uuid.uuid4().hex for _ in range(2))
        raw = SharedProvider(MappingFixture(), key=raw_key)
        upstream, assembly = view(raw_key), Component()
        assembly.expose('pose', upstream.pose)
        assembly.expose('map', upstream.map)
        derived = SharedProvider(assembly, key=derived_key,
            placement={assembly: ProcessPlacement('nested')} if placed else None)
        direct, nested = view(raw_key), view(derived_key)
        async with Runtime(raw, _root_ports=False), Runtime(derived, _root_ports=False):
            async with Runtime(nested.pose):
                await nested.pose.get(timeout=10)
                assert not list(raw.directory.rglob('*.npy'))
            async with Runtime(direct.map), Runtime(nested.map):
                first, second = await asyncio.gather(direct.map.get(timeout=10), nested.map.get(timeout=10))
                assert first.sample_id == second.sample_id
                assert first.metadata == second.metadata
                a, b = os.stat(first.data.filename), os.stat(second.data.filename)
                assert (a.st_dev, a.st_ino) == (b.st_dev, b.st_ino)
            await asyncio.sleep(.15)
            async with Runtime(nested.map):
                assert (await nested.map.get(timeout=10)).sample_id == first.sample_id
    asyncio.run(run())
