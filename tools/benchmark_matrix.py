"""Reproducible functional/performance matrix for the optional four-library stack.

Run with the development interpreter: python -m tools.benchmark_matrix run
--output RESULTS --baseline REF_TREE --repeat 3. Missing optional libraries are
errors, not implicit skips. The source trees are selected by subprocess cwd and
PYTHONPATH only in this benchmark launcher, never in the product libraries.
"""
from __future__ import annotations

import argparse
import asyncio
from dataclasses import asdict, dataclass
import hashlib
import importlib.metadata
import importlib.util
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback
import uuid

from rsim import Component, CommandSink, ProcessPlacement, VelocityCommand
from tools.performance_observer import (ObservedRuntime, Probe, ProcessTreeSampler, distribution,
    install_hooks, resource_window, summarize_instrumentation, write_json)


class Counter(Component):
    def __init__(self, hz=500):
        super().__init__()
        self.hz, self.index = hz, 0
        self.output = self.signal('output', history=128, clock='benchmark:source')

    async def open(self):
        self.task('publish', self.publish, hz=self.hz)

    async def publish(self):
        await self.output.publish(self.index, stamp_ns=round(self.index*1e9/self.hz),
            clock='benchmark:source', metadata=dict(session='counter', episode=0, step_index=self.index))
        self.index += 1


class Echo(Component):
    def __init__(self):
        super().__init__()
        self.request = CommandSink(self, 'request', self.apply, safe=lambda _: None, fallback=None, hz=500)

    async def apply(self, envelope):
        return dict(kind='EchoAck', value=envelope.value, pid=os.getpid())


async def core_case(args):
    from tools.benchmark_pacing import Pulse, throughput
    from rsim.core import Metronome
    pulses = [Pulse(1000, 1024) for _ in range(4)]
    async def busy():
        pace = Metronome(100)
        while True:
            await pace.tick()
            end = time.perf_counter()+.0005
            while time.perf_counter()<end:
                pass
    async with ObservedRuntime(*(pulse.output for pulse in pulses)):
        task = asyncio.create_task(busy())
        try:
            await asyncio.sleep(args.warmup)
            start = time.monotonic_ns()
            await asyncio.sleep(args.seconds)
            end = time.monotonic_ns()
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
    return dict(valid=True, steady_window=[start, end], micro=await throughput(),
                scenario='4x1000Hz, 1024 float64, 0.5ms CPU busy work at 100Hz',
                warmup_s=args.warmup)


async def ipc_case(args):
    from rsim import SignalPacer
    source, echo = Counter(), Echo()
    samples, replies = [], []
    async with ObservedRuntime(source.output, echo.request,
            placement={source: ProcessPlacement('counter'), echo: ProcessPlacement('echo')}):
        pacer = SignalPacer(source.output, hz=100, deduplicate=True)
        async def consume():
            while True:
                tick = await pacer.tick()
                samples.append(dict(host_ns=time.monotonic_ns(), stamp_ns=tick.stamp_ns,
                    step=tick.step_index, source_skipped=pacer.source_skipped, skipped=pacer.skipped))
        consumer = asyncio.create_task(consume())
        try:
            await asyncio.sleep(args.warmup)
            start = time.monotonic_ns()
            while (time.monotonic_ns()-start)/1e9 < args.seconds:
                before = time.monotonic_ns()
                reply = await echo.request.set(len(replies), ttl=.5)
                after = time.monotonic_ns()
                assert reply['value'] == len(replies) and reply['pid'] != os.getpid()
                replies.append(dict(started_ns=before, finished_ns=after, **reply))
                await asyncio.sleep(.005)
            end = time.monotonic_ns()
            counters = {name: getattr(pacer, name) for name in ('received', 'delivered', 'skipped', 'source_skipped', 'filtered')}
        finally:
            consumer.cancel()
            await asyncio.gather(consumer, return_exceptions=True)
            await pacer.aclose()
    steady = [row for row in samples if start <= row['host_ns'] <= end]
    deltas = [b['stamp_ns']-a['stamp_ns'] for a, b in zip(steady, steady[1:])]
    assert deltas and min(deltas)>0
    return dict(valid=True, steady_window=[start, end], signal_samples=samples, command_samples=replies,
                source_dt_ns=distribution(deltas), counters=counters,
                command_rtt_ns=distribution(row['finished_ns']-row['started_ns'] for row in replies))


async def pacer_case(args):
    from rsim import TickSource, SignalPacer, FrameTickPacer
    from rsim.components.replay import ReplaySource, ReplaySample
    fanouts, replays, rates = [], [], []
    for count in (1, 8, 32):
        source = TickSource(clock='benchmark:source')
        subscriptions = [source.subscribe(policy='every', capacity=8) for _ in range(count)]
        received = [[] for _ in subscriptions]
        async def consume(i, subscription):
            for expected in range(1000):
                tick = await subscription.tick()
                assert tick.step_index == expected
                received[i].append([tick.step_index, tick.stamp_ns, time.monotonic_ns()])
        tasks = [asyncio.create_task(consume(i, sub)) for i, sub in enumerate(subscriptions)]
        start = time.monotonic_ns()
        publication_wall, publication_samples = [], []
        for step in range(1000):
            before = time.monotonic_ns()
            await source.publish(stamp_ns=step*2_000_000, episode=0, step_index=step)
            after = time.monotonic_ns()
            publication_wall.append(after-before)
            publication_samples.append([step,before,after])
        await asyncio.gather(*tasks)
        end = time.monotonic_ns()
        fanouts.append(dict(subscribers=count, steps=1000, start_ns=start, end_ns=end,
            received=received, publication_wall_ns=publication_wall,
            publication_samples=publication_samples, warmup_steps=100,
            steady_window=[publication_samples[100][1],end],
            skipped=[sub.skipped for sub in subscriptions], steps_per_s=900e9/(end-publication_samples[100][1])))
        await source.aclose()
    for rate in (1., 2., 10.):
        source = ReplaySource([ReplaySample(i*2_000_000, i) for i in range(150)],
            clock='benchmark:recorded', rate=rate, paused=True, history=256)
        async with ObservedRuntime(source.output):
            await asyncio.sleep(.02)
            assert not source.output.frames
            await source.resume()
            async with asyncio.timeout(5):
                while not source.output.frames or source.output.frames[-1].data != 149:
                    await asyncio.sleep(.001)
            first = [[f.data, f.stamp_ns, f.metadata['host_monotonic_ns']] for f in source.output.frames]
            await source.pause()
            await source.seek(75)
            await source.resume()
            async with asyncio.timeout(5):
                while not source.output.frames or source.output.frames[-1].data != 149:
                    await asyncio.sleep(.001)
            second = [[f.data, f.stamp_ns, f.metadata['host_monotonic_ns']] for f in source.output.frames]
            assert [row[0] for row in first] == list(range(150))
            assert [row[0] for row in second] == list(range(75, 150))
            replays.append(dict(rate=rate, initial=first, after_seek=second,
                warmup_steps=30,
                rtf=(first[-1][1]-first[30][1])/(first[-1][2]-first[30][2])))
    # Source-rate filters and bounded every Frame matching share real frames.
    owner = Component()
    signal = owner.signal('source', clock='benchmark:source', history=8)
    ticks = TickSource(clock='benchmark:source')
    exact = FrameTickPacer(signal, ticks.subscribe(policy='every', capacity=2))
    filters = [SignalPacer(signal, hz=hz, deduplicate=True) for hz in (30, 100)]
    pending = [asyncio.create_task(p.tick()) for p in filters]
    latest = ticks.subscribe(policy='latest')
    async with ObservedRuntime(signal):
        for step in range(501):
            frame = await signal.publish(step, stamp_ns=step*2_000_000, clock='benchmark:source',
                metadata=dict(session='pacer', episode=0, step_index=step))
            await ticks.publish(stamp_ns=frame.stamp_ns, episode=0, step_index=step, session='pacer')
            await exact.tick()
            assert exact.last_frame is frame
            async with asyncio.timeout(1):
                while any(p.received <= step for p in filters):
                    await asyncio.sleep(0)
            for i, task in enumerate(pending):
                if task.done():
                    tick = await task
                    rates.append(dict(hz=(30,100)[i], step=tick.step_index, stamp_ns=tick.stamp_ns))
                    pending[i] = asyncio.create_task(filters[i].tick())
        assert (await latest.tick()).step_index == 500
        coalesced = latest.skipped
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)
        for p in (*filters, exact):
            await p.aclose()
    await ticks.aclose()
    assert [sum(row['hz']==hz for row in rates) for hz in (30,100)] == [31,101]
    return dict(valid=True, fanout=fanouts, replay=replays, rate_samples=rates,
                latest_coalesced=coalesced, exact_matched_frames=501,
                scope='synthetic pacing, source timestamp matching and backpressure; no robot physics')


def make_source(backend, scheduling, rate, directory):
    if backend == 'memory':
        from pyrobo.rsim import MemoryChassisFactory
        factory = MemoryChassisFactory(scheduling=scheduling, speedup=rate, motion_enabled=True,
                                       bounds=(-3.,3.,-3.,3.))
        dt, substeps = .02, 5
    else:
        from mujocosim.rsim import MujocoChassisFactory
        from mujocosim.config import SensorConfig
        from mujocosim.robots.differential_drive import DifferentialDrive, DifferentialDriveConfig
        scene = directory/'room.json'
        scene.write_text(json.dumps({'format':'mujocosim-scene-v1','room':{'size':[6,6,2.6]},
                                     'spawn':[0,0,0], 'objects':[]}))
        factory = MujocoChassisFactory(scene=scene, cache_root=directory/'scene-cache',
            motion_enabled=True, spawn=(0.,0.,0.), scheduling=scheduling, speedup=rate,
            robot=DifferentialDrive(DifferentialDriveConfig(wheel_track=.4)),
            sensors=SensorConfig(width=80,height=60))
        dt, substeps = .002, 50
    return factory, factory.build(), dt, substeps


async def navigation_case(args):
    from pyrobo.apps import navigation_provider as app
    from pyrobo.apps import lockstep_navigation as lockstep
    from pyrobo import DWAConfig, Footprint
    from pyrobo.rsim import StackConfig
    from rsim.devices import ChassisPortsDevice
    app.Runtime = lockstep.Runtime = ObservedRuntime
    backend, mode = args.case.split('-', 1)
    is_lockstep = mode.startswith('lockstep')
    schedule = 'controlled' if is_lockstep else 'accelerated' if mode.startswith('accelerated') else 'realtime'
    factory, source, dt, substeps = make_source(backend, schedule, args.rate, args.output.parent)
    factory_configuration = asdict(factory)
    fingerprint = factory.fingerprint()
    if is_lockstep:
        report = await lockstep.run_lockstep(source, physics_timestep=dt, requested_substeps=substeps,
            map_data=app.room_map(spacing=.3), episodes=3,
            execution='process' if mode.endswith('process') else 'local')
        assert report['arrived'] and not report['collided']
        for episode in report['episodes']:
            for trace in episode['trace']:
                advanced = trace['advanced']
                assert advanced['actual_substeps'] == substeps
                assert advanced['command_id'] == trace['accepted']['command_id']
        return dict(valid=True, navigation=report, factory=factory_configuration, fingerprint=fingerprint,
            warmup_episodes=1,
            steady_windows=[[e['host_started_ns'], e['host_finished_ns']] for e in report['episodes'][1:]])
    config = StackConfig(clock='simulation', scheduling='realtime' if mode=='realtime' else 'environment',
        control_period=.2 if schedule=='accelerated' else .1,
        execution_period=.04 if schedule=='accelerated' else .02,
        controller=DWAConfig(prediction_time=.6,integration_dt=.1,linear_samples=3,angular_samples=3),
        footprint=Footprint(.6,.6),command_speed_limit=.45,command_yaw_limit=.75)
    episodes, windows = [], []
    for repetition in range(2):
        if repetition:
            _, source, _, _ = make_source(backend, schedule, args.rate, args.output.parent)
        ObservedRuntime.source = source
        report = await app.navigate(ChassisPortsDevice.from_component(source), config=config,
            map_data=app.room_map(spacing=.3), execution='process', duration=20.)
        assert report['arrived'] and not report['collided'], 'navigation did not arrive safely'
        lifecycle = ObservedRuntime.records[-1]
        final = lifecycle['source_final']
        final_stamp = final['stamp_ns']
        wall = (lifecycle['closed_ns']-lifecycle['open_started_ns'])/1e9
        report['complete_runtime_physical_s'] = final_stamp/1e9
        report['complete_runtime_rtf'] = final_stamp/1e9/wall
        report['complete_runtime_wall_s'] = wall
        report['source_initial_stamp_ns'] = 0
        episodes.append(report)
        if repetition:
            windows.append([report['samples'][0]['host_observed_ns'], report['samples'][-1]['host_observed_ns']])
    ObservedRuntime.source = None
    return dict(valid=True, navigation=episodes, config=asdict(config), fingerprint=fingerprint,
        factory=factory_configuration, warmup_episodes=1, steady_windows=windows,
        steady_scope='complete post-warmup navigation observation window; trajectory is not stationary')


def build_in_child(recipes, connection, directory):
    from rsim.runtime import component_ports
    hooks = install_hooks(directory)
    rows = []
    try:
        for recipe in recipes:
            start = time.monotonic_ns()
            graph = recipe.shared().factory()
            end = time.monotonic_ns()
            assert component_ports(graph) == recipe.shared().port_specs
            assert graph._closed and graph._runtime is None
            rows.append(dict(factory=type(recipe).__name__, started_ns=start, finished_ns=end,
                             fingerprint=recipe.fingerprint()))
        connection.send(dict(pid=os.getpid(), rows=rows, ready_ns=time.monotonic_ns()))
        connection.recv()
    finally:
        hooks.flush()
        connection.close()


class FixtureMotor(Component):
    """No hardware/ROS: isolates real NativeFactory composition and transport."""
    def __init__(self):
        super().__init__()
        self.state = self.signal('state', clock='ros:system', history=128)
        self.odom = self.signal('odom', clock='ros:system', history=512)
        self.velocity = CommandSink(self, 'velocity', self.apply, safe=lambda _: None,
                                    fallback=VelocityCommand(), max_ttl=.5)
        self.topics = {'odom':'/wheel'}

    async def apply(self, envelope):
        value = dict(sequence=envelope.sequence, deadline_ns=envelope.deadline_ns,
                     controller_epoch=envelope.controller_epoch, speed=envelope.value.linear_x)
        await self.state.publish(value, stamp_ns=time.time_ns(), clock='ros:system')
        return dict(kind='FixtureAccepted', pid=os.getpid(), **value)


class FixtureEstimate(Component):
    def __init__(self):
        super().__init__()
        self.pose = self.signal('pose', clock='ros:system', history=512)
        self.odometry = self.signal('odometry', clock='ros:system', history=512)


@dataclass(frozen=True)
class FixtureRecipe:
    recipe: object
    directory: str

    def build(self):
        graph = self.recipe.build(chassis=FixtureMotor(), odometry=FixtureEstimate(),
                                  imu=Component().signal('imu', clock='ros:system'))
        graph.dependencies = (Probe(self.directory), *graph.dependencies)
        return graph


async def factory_case(args):
    from rsim.drivers import (OdometryFactory, ScanOdometryFactory, NativeChassisFactory,
                              TopicChassisFactory, MappingFactory)
    from rsim.runtime import SharedProvider, component_ports
    from rsim.adapters.ros2.arguments import GeneratedParameters
    import yaml
    mount = dict(wrd_frame='base_footprint', ego_frame='imu')
    settings = [
        (OdometryFactory, dict(imu_mount=mount)),
        (ScanOdometryFactory, dict(mount=dict(wrd_frame='base_footprint',ego_frame='laser'))),
        (NativeChassisFactory, dict(stm32={'start_driver':False},imu={'topic':'/imu'},imu_mount=mount,motion_enabled=True)),
        (TopicChassisFactory, dict(stm32={},imu={'topic':'/imu'},imu_mount=mount)),
        (MappingFactory, dict(database=str(args.output.parent/'persistent.db'),
            mounts={name:dict(wrd_frame='base_footprint',ego_frame=child) for name,child in
                    [('body_imu','imu'),('body_lidar','lidar'),('body_camera','camera_optical')]},
            timing={'lidar':{'offset_s':0.}}))]
    rows, recipes = [], []
    for cls, values in settings:
        path = args.output.parent/(cls.__name__+'.yaml')
        values['directory'] = str(args.output.parent/'unopened')
        path.write_text(yaml.safe_dump(values))
        for iteration in range(60):
            begin = time.monotonic_ns()
            recipe = cls.from_config(path)
            parsed = time.monotonic_ns()
            graph = recipe.build()
            built = time.monotonic_ns()
            shared = recipe.shared()
            described = time.monotonic_ns()
            assert component_ports(graph) == shared.port_specs and graph._closed
            rows.append(dict(factory=cls.__name__, iteration=iteration, warmup=iteration<10,
                started_ns=begin, parsed_ns=parsed, built_ns=built, shared_ns=described))
        recipes.append(recipe)
    assert not (args.output.parent/'unopened').exists()
    context = mp.get_context('spawn')
    parent, child = context.Pipe()
    process = context.Process(target=build_in_child, args=(recipes,child,str(args.raw)))
    begin = time.monotonic_ns()
    process.start()
    child.close()
    try:
        ready = await asyncio.wait_for(asyncio.to_thread(parent.recv), 20)
        await asyncio.sleep(.15)
        close_start = time.monotonic_ns()
        parent.send('close')
        await asyncio.to_thread(process.join, 10)
        closed = time.monotonic_ns()
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(5)
        parent.close()
    native = recipes[2]
    key = 'benchmark-native-'+uuid.uuid4().hex
    provider = SharedProvider(factory=FixtureRecipe(native,str(args.raw)).build,key=key,
        ports=native.port_specs(),interface_version=native.INTERFACE_VERSION)
    client = native.device(key=key)
    async with ObservedRuntime(provider, _root_ports=False), ObservedRuntime(client,client.velocity):
        await asyncio.sleep(args.warmup)
        steady_start = time.monotonic_ns()
        for _ in range(100):
            reply = await client.velocity.set(VelocityCommand(.1),ttl=.5)
            assert reply['pid'] != os.getpid() and reply['speed'] == .1
        steady_end = time.monotonic_ns()
    async with asyncio.timeout(5):
        while provider.directory.exists():
            await asyncio.sleep(.02)
    sessions = []
    generated = GeneratedParameters({'frequency':50.,'publish_tf':False})
    for iteration in range(60):
        started = time.monotonic_ns()
        session = generated.materialize()
        materialized = time.monotonic_ns()
        path = session.path
        session.validate()
        session.close()
        assert not path.exists()
        sessions.append(dict(iteration=iteration,warmup=iteration<10,started_ns=started,
            materialized_ns=materialized,closed_ns=time.monotonic_ns()))
    return dict(valid=True, config_build=rows, parameter_sessions=sessions, worker=ready,
        spawn_started_ns=begin, worker_ready_ns=ready['ready_ns'], close_started_ns=close_start, closed_ns=closed,
        steady_window=[steady_start,steady_end],
        scope='5 real recipes parse/build/shared spawn; real Native composite + DDS with explicit fake hardware/estimator; generated-file lifecycle',
        unrun=['ROS2/native estimator/hardware activation performance'])


def provenance():
    import rsim
    result = dict(python=sys.version,executable=sys.executable,platform=platform.platform(),
        machine=platform.machine(),processor=platform.processor(),cpu_count=os.cpu_count(),
        affinity=sorted(os.sched_getaffinity(0)),rsim_source=rsim.__file__,repositories={},dependencies={})
    for name in ('numpy','scipy','torch','mujoco','cyclonedds','psutil','PyYAML'):
        try:
            result['dependencies'][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result['dependencies'][name] = None
    # Installed project metadata need not point at the editable symlink checkout.
    for name in ('rsim','pyrobo','graphmap','mujocosim'):
        spec = importlib.util.find_spec(name)
        if spec is None or spec.origin is None:
            result['repositories'][name] = None
            continue
        source = Path(spec.origin).resolve()
        root = subprocess.run(['git','-C',str(source.parent),'rev-parse','--show-toplevel'],capture_output=True,text=True)
        if root.returncode:
            result['repositories'][name] = dict(source=str(source))
            continue
        root = root.stdout.strip()
        git = lambda *cmd: subprocess.check_output(['git','-C',root,*cmd])
        result['repositories'][name] = dict(source=str(source),root=root,head=git('rev-parse','HEAD').decode().strip(),
            status=git('status','--short').decode(),diff_sha256=hashlib.sha256(git('diff')).hexdigest())
    result['environment'] = {name:os.environ.get(name) for name in
        ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','MUJOCO_GL','PYTHONPATH')}
    result['tool_sha256'] = {name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                            for name in ('benchmark_matrix.py','performance_observer.py')}
    return result


async def sample(args):
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.raw.mkdir(parents=True,exist_ok=True)
    hooks = install_hooks(args.raw)
    ObservedRuntime.directory, ObservedRuntime.records = str(args.raw), []
    result = dict(case=args.case,rate=args.rate,provenance=provenance(),started_ns=time.monotonic_ns())
    try:
        handler = {'core':core_case,'pacer':pacer_case,'ipc':ipc_case,'factory':factory_case}.get(args.case,navigation_case)
        result.update(await handler(args))
    except BaseException as error:
        result.update(valid=False,error=f'{type(error).__name__}: {error}',traceback=traceback.format_exc())
    finally:
        result['finished_ns'] = time.monotonic_ns()
        result['runtime'] = ObservedRuntime.records
        hooks.flush()
        write_json(args.output,result)
    print(json.dumps(dict(case=args.case,rate=args.rate,valid=result['valid'],error=result.get('error'))),flush=True)
    return 0 if result['valid'] else 1


def enrich(path, sampler, command, exitcode):
    data = json.loads(path.read_text()) if path.exists() else dict(valid=False,error='sample produced no result')
    raw = path.parent/'instrumentation'
    data['command'], data['exitcode'] = command, exitcode
    data['resources'] = sampler.report()
    data['instrumentation'] = summarize_instrumentation(raw.glob('process-*.json'))
    observed = {row['pid'] for row in data['resources']['processes']}
    instrumented = set(data['instrumentation']['instrumented_pids'])
    data['resource_coverage_complete'] = instrumented <= observed
    data['instrumented_pids_not_sampled'] = sorted(instrumented-observed)
    windows = data.get('steady_windows', [data['steady_window']] if 'steady_window' in data else [])
    data['steady'] = [dict(window=window,resources=sampler.report(*window),
        instrumentation=summarize_instrumentation(raw.glob('process-*.json'),*window)) for window in windows]
    write_json(path,data)


def run(args):
    args.output.mkdir(parents=True,exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    candidate = args.candidate.resolve() if args.candidate is not None else root
    cases = [('pacer',1.),('ipc',1.),('factory',1.)]
    for backend in ('memory','mujoco'):
        cases += [(backend+'-realtime',1.),(backend+'-environment',1.)]
        cases += [(backend+'-accelerated',rate) for rate in (1.,2.,10.)]
        cases += [(backend+'-lockstep-local',1.),(backend+'-lockstep-process',1.)]
    if args.cases:
        requested = set(args.cases.split(','))
        cases = [row for row in cases if row[0] in requested]
    plan = []
    for repeat in range(args.repeat):
        if not args.cases or 'core' in args.cases.split(','):
            paired = ([('baseline',args.baseline.resolve())] if args.baseline is not None else []) + [('candidate',candidate)]
            if repeat%2:
                paired.reverse()
            plan += [dict(case='core',rate=1.,repeat=repeat,side=label,source=str(source)) for label,source in paired]
        ordered = cases if repeat%2==0 else list(reversed(cases))
        plan += [dict(case=case,rate=rate,repeat=repeat,side='candidate',source=str(candidate)) for case,rate in ordered]
    if not plan:
        raise ValueError('no recognized benchmark cases selected')
    manifest = dict(plan=plan,created_unix_s=time.time(),provenance=provenance(),runs=[],
                    sample_interval_s=.02,seconds=args.seconds,warmup=args.warmup)
    write_json(args.output/'manifest.json',manifest)
    for entry in plan:
        name = f"{entry['repeat']+1:02d}-{entry['case']}-{entry['rate']:g}-{entry['side']}"
        directory = args.output/name
        directory.mkdir()
        output = directory/'result.json'
        environment = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',MUJOCO_GL='egl',
                           PYTHONPATH=os.pathsep.join((entry['source'],str(root))))
        command = ['taskset','-c',args.cpus,sys.executable,'-m','tools.benchmark_matrix','sample',
            '--case',entry['case'],'--rate',str(entry['rate']),'--seconds',str(args.seconds),
            '--warmup',str(args.warmup),'--output',str(output.resolve()),'--raw',str((directory/'instrumentation').resolve())]
        with (directory/'run.log').open('w') as log:
            process = subprocess.Popen(command,cwd=entry['source'],env=environment,stdout=log,stderr=subprocess.STDOUT)
            sampler = ProcessTreeSampler(process.pid)
            started = time.monotonic()
            while process.poll() is None:
                sampler.sample()
                if time.monotonic()-started > 180:
                    process.terminate()
                    break
                time.sleep(.02)
            try:
                code = process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                code = process.wait(timeout=5)
            sampler.sample()
        # Leased supervisors can finish just after their parent. Observe that
        # bounded teardown without pretending a normal parent exit proves it.
        cleanup_deadline = time.monotonic()+5
        survivors = sampler.living_pids()
        while survivors and time.monotonic()<cleanup_deadline:
            sampler.sample()
            time.sleep(.02)
            survivors = sampler.living_pids()
        write_json(directory/'resources-raw.json',sampler.samples)
        enrich(output,sampler,command,code)
        measurement = json.loads(output.read_text())
        outcome = dict(entry,output=str(output),exitcode=code,elapsed_s=time.monotonic()-started,
                       resource_coverage_complete=measurement['resource_coverage_complete'],
                       surviving_pids=survivors)
        manifest['runs'].append(outcome)
        write_json(args.output/'manifest.json',manifest)
        print(json.dumps(outcome),flush=True)
    return int(any(row['exitcode'] or row['surviving_pids'] or not row['resource_coverage_complete']
                   for row in manifest['runs']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command',required=True)
    child = commands.add_parser('sample')
    child.add_argument('--case',required=True)
    child.add_argument('--rate',type=float,default=1.)
    child.add_argument('--raw',type=Path,required=True)
    parent = commands.add_parser('run')
    parent.add_argument('--baseline',type=Path)
    parent.add_argument('--candidate',type=Path,help='defaults to this checkout; use baseline again for self comparison')
    parent.add_argument('--repeat',type=int,default=3)
    parent.add_argument('--cpus',default='4-7')
    parent.add_argument('--cases',help='comma-separated case names, useful for a smoke run')
    for sub in (child,parent):
        sub.add_argument('--output',type=Path,required=True)
        sub.add_argument('--seconds',type=float,default=5.)
        sub.add_argument('--warmup',type=float,default=1.)
    args = parser.parse_args()
    if args.seconds<=0 or args.warmup<0 or getattr(args,'repeat',1)<1:
        parser.error('positive duration/repeat and nonnegative warmup required')
    return asyncio.run(sample(args)) if args.command=='sample' else run(args)


if __name__ == '__main__':
    raise SystemExit(main())
