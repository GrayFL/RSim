"""Real Shared/DDS process integration; the only actuator is ROS-free simulation."""
import asyncio
import json
import os
import time
import uuid

import pytest

pytest.importorskip('cyclonedds')
pytest.importorskip('pyrobo')

from rsim import Runtime, SharedProvider, ProcessPlacement
from rsim.apps.navigation.service import NavigationService
from rsim.apps.navigation.simulation import SimulationBridge
from rsim.apps.navigation.client import NavigationClient


def test_shared_navigation_commands_maps_pids_and_manual_priority(tmp_path):
    async def run():
        service = NavigationService(SimulationBridge(), {'optional_scans': [], 'max_compute_ms': 450}, simulated=True, workers=True)
        provider = SharedProvider(service, key='navigation-test:' + uuid.uuid4().hex,
                                  placement={service.workers.local: ProcessPlacement('local'),
                                             service.workers.global_: ProcessPlacement('global')},
                                  log_path=tmp_path/'controller.log')
        client = NavigationClient(provider, service.config, simulated=True, motion_enabled=True)
        async with Runtime(client):
            async with asyncio.timeout(25):
                while client.map is None or client.status()['hardware']['health'] != 'ok': await asyncio.sleep(.05)
            assert client.map.grid.size == 10000
            assert client.status()['controller_pid'] != os.getpid()
            lease = await client.claim()
            sequence = 0
            async def command(op, **args):
                nonlocal sequence
                sequence += 1
                return await client.command(dict(op=op, lease=lease, sequence=sequence,
                                                  deadline_ms=time.monotonic()*1000+490, **args))
            await command('arm')
            await command('goal', x=2, y=0, yaw=0)
            for _ in range(12):
                await command('heartbeat')
                await asyncio.sleep(.1)
            await command('manual', throttle=.2, steering=.2)
            await command('goal', x=0, y=2, yaw=1.5)
            await command('manual', throttle=.2, steering=.2)
            await command('resume')
            for _ in range(7):
                began = time.monotonic()
                await command('manual', throttle=.2, steering=.2)
                await asyncio.sleep(max(0, .08-(time.monotonic()-began)))
            state = client.status()
            assert state['mode'] == 'manual', state
            assert state['published'][0] > 0, json.dumps(state)
            pids = {os.getpid(), state['controller_pid'], state['workers']['local']['pid'], state['workers']['global']['pid']}
            assert len(pids) == 4
            await client.stop()
            # Observe the confirmed stop and simulated actuator feedback, not
            # an arbitrary delay followed by a possibly retained old frame.
            async with asyncio.timeout(2):
                while True:
                    stopped = client.status()
                    if (stopped.get('last_stop', {}).get('reason') == 'operator_stop'
                            and stopped['hardware'].get('published') == [0., 0.]): break
                    await asyncio.sleep(.02)
            assert stopped['published'] == [0., 0.] and not stopped['armed'], json.dumps(stopped)
            # A pure-yaw route has zero XY length. The placed local worker
            # must keep it so differential navigation can finish orientation.
            async with asyncio.timeout(4):
                while max(map(abs, client.status()['measured'])) > .015:
                    await command('heartbeat')
                    await asyncio.sleep(.1)
            await command('drive_mode', drive_mode='differential')
            await client.relocalize(0, 0, 0)
            async with asyncio.timeout(2):
                while max(map(abs, client.status()['pose'])) > .01: await asyncio.sleep(.05)
            await command('arm')
            await command('goal', x=0, y=0, yaw=.5)
            async with asyncio.timeout(12):
                while client.status()['reason'] != 'arrived':
                    await command('heartbeat')
                    await asyncio.sleep(.1)
            assert abs(client.status()['pose'][2] - .5) < .2
            assert not client.status()['armed']
            await command('arm')
            await command('goal', x=.6, y=.1, yaw=.5)
            seen_navigation = False
            async with asyncio.timeout(30):
                while True:
                    current = client.status()
                    seen_navigation |= current.get('goal') is not None
                    if seen_navigation and current['reason'] == 'arrived' and current.get('goal') is None: break
                    await command('heartbeat')
                    await asyncio.sleep(.1)
            assert abs(client.status()['pose'][0] - .6) < .18
            assert abs(client.status()['pose'][1] - .1) < .18
        # SharedProvider supervisor owns the actual workers. Runtime close must
        # release the provider; it is not enough to set a stopped Python flag.
        async with asyncio.timeout(15):
            while provider.directory.exists(): await asyncio.sleep(.05)

    asyncio.run(run())


@pytest.mark.parametrize('delay_stage', ['solve', 'accept'])
def test_slow_global_solver_does_not_block_manual_control(tmp_path, delay_stage):
    from pathlib import Path
    from rsim.apps.navigation.planning import PlannerPair, PlanningWorker

    class SlowGlobal(PlanningWorker):
        async def accept(self, envelope):
            if delay_stage == 'accept':
                (tmp_path/'global-busy').write_text(str(os.getpid()))
                await asyncio.sleep(1.5)
            return await super().accept(envelope)

        def compute_job(self, job):
            if delay_stage == 'solve':
                (tmp_path/'global-busy').write_text(str(os.getpid()))
                time.sleep(1.5)
            return super().compute_job(job)

    async def run():
        workers = PlannerPair()
        workers.global_ = SlowGlobal('global')
        service = NavigationService(SimulationBridge(), {'optional_scans': [], 'max_compute_ms': 450}, simulated=True, workers=workers)
        provider = SharedProvider(service, key='navigation-stall:' + uuid.uuid4().hex,
                                  placement={workers.local: ProcessPlacement('local'), workers.global_: ProcessPlacement('global')},
                                  log_path=tmp_path/'isolation.log')
        client = NavigationClient(provider, service.config, simulated=True, motion_enabled=True)
        async with Runtime(client):
            async with asyncio.timeout(25):
                while client.map is None or client.status()['hardware']['health'] != 'ok': await asyncio.sleep(.05)
            lease, sequence = await client.claim(), 0
            async def command(op, **data):
                nonlocal sequence
                sequence += 1
                await client.command(dict(op=op, lease=lease, sequence=sequence,
                                          deadline_ms=time.monotonic()*1000+490, **data))
            await command('arm')
            await command('goal', x=2, y=0, yaw=0)
            async with asyncio.timeout(3):
                while not (tmp_path/'global-busy').exists():
                    await command('heartbeat')
                    await asyncio.sleep(.05)
            start = time.monotonic()
            while time.monotonic()-start < .8:
                await command('manual', throttle=.15, steering=0)
                if client.status()['published'][0] > 0: break
                await asyncio.sleep(.05)
            assert client.status()['mode'] == 'manual'
            assert client.status()['published'][0] > 0, json.dumps(client.status())
            assert time.monotonic()-start < .8  # The global worker is still asleep.
            await client.stop()
    asyncio.run(run())
