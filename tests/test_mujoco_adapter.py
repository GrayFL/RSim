"""Protocol adapter only; no robot geometry, scenes, DDS or estimator tests."""
import asyncio
from functools import partial
import threading
import time

import pytest

from rsim.runtime import Runtime
from rsim.adapters.mujoco import MuJoCoSource, Observation, CommandSpec
from rsim.core import CommandRejected


class Backend:
    def __init__(self, fail=False):
        self.thread = threading.get_ident()
        self.calls = ['factory']
        self.fail = fail
        self.time = 0.
        self.applied = {}
        self.demands = []

    def mark(self, name):
        assert threading.get_ident() == self.thread
        self.calls.append(name)

    def open(self):
        self.mark('open')
        if self.fail:
            raise RuntimeError('backend initialization failed')

    def step(self, duration, commands):
        self.mark('step')
        self.time += duration
        self.applied = dict(commands)

    def sample(self, demand, *, force=False):
        self.mark('sample')
        self.demands.append(demand.ports)
        values = {'feedback': dict(self.applied)}
        if demand.has('camera'):
            values['camera'] = ('immutable', self.time)
        return Observation(self.time, values, {'source': 'test'}, {'feedback': {'unit': 'N'}})

    def reset(self):
        self.mark('reset')
        self.time = 0.
        self.applied = {}

    def stop(self):
        self.mark('stop')
        self.applied = {'force': 0., 'grip': 0.}

    def close(self): self.mark('close')


def zero(value): return value == 0.


def source(factory=Backend, **options):
    return MuJoCoSource(factory, outputs={'feedback': 8, 'camera': 2},
                        commands={name: CommandSpec(0., zero) for name in ('force', 'grip')}, **options)


def test_generic_commands_demand_thread_ownership_and_reset():
    async def run():
        component = source(motion_enabled=True)
        assert component.backend is None
        async with Runtime(component.feedback):
            assert component.backend.thread != threading.get_ident()
            assert not component.camera.frames
            assert all('camera' not in demand for demand in component.backend.demands)
            await component.force.set(2., ttl=.05)
            await component.grip.set(3., ttl=.4)
            # Stall the owner past one deadline; force expires independently.
            await component.inspect(lambda backend: time.sleep(.09))
            frame = await component.feedback.get()
            frame = await component.feedback.get(after=frame.sequence, timeout=1)
            assert frame.data == {'force': 0., 'grip': 3.}
            assert frame.metadata['unit'] == 'N'
            stamp, episode = frame.stamp_ns, frame.metadata['episode']
            await component.reset()
            reset = await component.feedback.get()
            assert reset.stamp_ns > stamp
            assert reset.metadata['episode'] == episode + 1
            assert component.current_commands == {'force': 0., 'grip': 0.}
        assert component.backend.calls[-2:] == ['stop', 'close']
        assert component._executor is None
    asyncio.run(run())


def test_guarded_outputs_and_startup_failure_cleanup():
    async def run():
        component = source()
        async with Runtime(component.camera):
            assert (await component.camera.get()).data[0] == 'immutable'
            with pytest.raises(CommandRejected, match='motion_enabled'):
                await component.force.set(1.)
            await component.force.set(0.)
        component = source(partial(Backend, fail=True))
        with pytest.raises(Exception, match='backend initialization failed'):
            async with Runtime(component.feedback):
                pass
        assert component.backend.calls[-2:] == ['stop', 'close']
        assert component._executor is None
    asyncio.run(run())


def test_adapter_imports_without_simulation_or_geometry_packages():
    import subprocess
    import sys
    code = '''
import sys
from importlib.abc import MetaPathFinder
class Block(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco','mujocosim','trimesh','graphmap','PIL'}:
            raise AssertionError(fullname)
sys.meta_path.insert(0, Block())
from rsim.adapters.mujoco import MuJoCoSource, CommandSpec, Observation
from rsim.drivers import MuJoCo
component = MuJoCo(lambda: None, outputs={'angle': 2})
assert component.backend is None
'''
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
