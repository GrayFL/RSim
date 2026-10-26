"""RSim-owned local Python 2 ROS1 process and shared-file ports.

ROS packages are never imported into the RSim interpreter. Every configured
subscription is a Signal and velocity is a CommandSink, so ordinary Runtime,
Connect and optional SharedProvider bindings can be used without special cases.
"""
from __future__ import annotations
import asyncio
import json
import os
from pathlib import Path
import shlex
import socket
import time
import uuid

from ..core.component import Component
from ..core.commands import CommandSink, VelocityCommand
from .ros1 import decode_message
from .ros1_wire import VERSION, path_for, read_packet, write_packet, notification_address


class LocalROS1Bridge(Component):
    def __init__(self, config, *, directory='/dev/shm/rsim-ros1',
                 python='/usr/bin/python2', setup=('/opt/ros/melodic/setup.bash',),
                 agent=None, motion_enabled=False, history=4):
        super().__init__()
        self.config = json.loads(json.dumps(config))
        self.root_directory = Path(directory)
        self.directory = self.root_directory
        self.python, self.setup = str(python), tuple(map(str, setup))
        self.agent = Path(agent) if agent else Path(__file__).resolve().parents[2] / 'compat' / 'ros1_local_agent.py'
        self.motion_enabled = bool(motion_enabled)
        self.armed = False
        self.manual_direct = False
        self.footprint_rear = .81
        self.drive_mode = self.config.get('drive_mode', 'towing')
        self.process = self.log = None
        self.processes, self.logs = [], []
        self.updates = self.command_notify = None
        self.session, self.command_sequence, self.action_sequence = None, 0, 0
        self.signatures = {}
        self.packet_sequences = {}
        self.poll_lock = asyncio.Lock()
        names = set(self.config['topics']) | {'pose', 'status', 'action_result'}
        for name in names:
            if hasattr(self, name):
                raise ValueError('ROS port name conflicts with Component: ' + name)
            setattr(self, name, self.signal(name, history=1 if name == 'map' else history, clock='ros:system'))
        self.velocity = CommandSink(self, 'velocity', self.apply, fallback=VelocityCommand(), safe=self.stop, max_ttl=.5)

    async def open(self):
        self.session = uuid.uuid4().hex
        self.directory = self.root_directory / self.session
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=False)
        self.command_sequence = self.action_sequence = 0
        self.signatures.clear()
        self.packet_sequences.clear()
        self.armed = False
        self.config_path = self.directory / ('config-' + self.session + '.json')
        self.config_path.write_text(json.dumps(self.config), encoding='utf8')
        os.chmod(self.config_path, 0o600)
        await self.heartbeat()
        self.updates = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.updates.bind(notification_address(self.session, 'updates'))
        self.updates.setblocking(False)
        self.command_notify = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.command_notify.setblocking(False)
        arguments = [self.python, '-u', str(self.agent), '--directory', str(self.directory),
                     '--session', self.session, '--config', str(self.config_path)]
        if self.motion_enabled: arguments.append('--enable-motion')
        script = 'set -e\n' + ''.join('source ' + shlex.quote(p) + ' >&2\n' for p in self.setup)
        # Do not leak conda's Python/loader variables into the ROS Python 2 child.
        environment = {'HOME': os.environ.get('HOME', '/root'), 'LANG': 'C.UTF-8',
                       'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
                       'ROS_MASTER_URI': self.config.get('master_uri', 'http://127.0.0.1:11311'),
                       'ROS_IP': self.config.get('ros_ip', '127.0.0.1'),
                       'ROBOT_TYPE': self.config.get('robot_type', '')}
        try:
            roles = ('command', 'pose', 'sensors') if self.config.get('split_processes', False) else ('combined',)
            for role in roles:
                args = arguments + ['--role', role, '--lockfile', str(self.root_directory / ('agent-' + role + '.lock'))]
                log = (self.directory / ('agent-' + role + '.log')).open('wb')
                self.logs.append(log)
                process = await asyncio.create_subprocess_exec('/bin/bash', '--noprofile', '--norc', '-c',
                    script + 'exec ' + shlex.join(args), env=environment, stdout=log, stderr=log)
                self.processes.append(process)
            self.process, self.log = self.processes[0], self.logs[0]
            self.task('heartbeat', self.heartbeat, hz=10)
            async with asyncio.timeout(20):
                while True:
                    if any(p.returncode is not None for p in self.processes):
                        raise RuntimeError('ROS1 relay exited; inspect ' + str(self.directory))
                    ready = [await asyncio.to_thread(read_packet, str(self.directory),
                              'ready' if role == 'combined' else 'ready_' + role) for role in roles]
                    if all(p and p.get('session') == self.session for p in ready): break
                    await asyncio.sleep(.05)
            await self.poll()
            self._tasks.add(asyncio.create_task(self.receive_updates(), name='ROS1:updates'))
            # Recovery for a dropped datagram and child-death detection only.
            self.task('mailbox-recovery', self.poll, hz=2)
        except BaseException:
            await self.close()
            raise

    async def heartbeat(self):
        await asyncio.to_thread(write_packet, str(self.directory), 'heartbeat',
                                {'session': self.session, 'stamp_ns': time.monotonic_ns()})

    def read_changed(self, names=None):
        changed = []
        for name in (self.outputs if names is None else names):
            if name not in self.outputs: continue
            # Notifications already identify changed mailboxes; avoid a stat
            # syscall per frame (especially costly under PRoot).
            if names is None:
                try: info = Path(path_for(str(self.directory), name)).stat()
                except FileNotFoundError: continue
                signature = (info.st_mtime_ns, info.st_size, info.st_ino)
                if signature == self.signatures.get(name): continue
                self.signatures[name] = signature
            packet = read_packet(str(self.directory), name)
            if (packet and packet.get('session') == self.session and packet.get('version') == VERSION
                    and packet.get('sequence', 0) > self.packet_sequences.get(name, 0)):
                changed.append((name, packet))
                self.packet_sequences[name] = packet['sequence']
        return changed

    async def receive_updates(self):
        loop = asyncio.get_running_loop()
        while True:
            names = {(await loop.sock_recv(self.updates, 128)).decode('ascii')}
            try:
                while True: names.add(self.updates.recv(128).decode('ascii'))
            except BlockingIOError: pass
            await self.poll(names)

    async def poll(self, names=None):
        # Recovery and datagram delivery share one cursor. Reading/publishing
        # concurrently could deliver an older snapshot after a newer frame.
        async with self.poll_lock:
            if any(p.returncode is not None for p in self.processes):
                raise RuntimeError('ROS1 child exited with code ' + str(self.process.returncode))
            for name, packet in await asyncio.to_thread(self.read_changed, names):
                data = decode_message(packet['data'])
                metadata = {k: v for k, v in packet.items() if k != 'data'}
                metadata['bridge_read_ns'] = time.monotonic_ns()
                await self.outputs[name].publish(data, stamp_ns=packet['stamp_ns'], clock='ros:system', metadata=metadata)

    def wake_command(self):
        if self.command_notify:
            try: self.command_notify.sendto(b'command', notification_address(self.session, 'command'))
            except (FileNotFoundError, BlockingIOError, ConnectionRefusedError): pass

    async def apply(self, envelope):
        command = envelope.value
        if not isinstance(command, VelocityCommand): raise TypeError('velocity requires VelocityCommand')
        self.command_sequence += 1
        await asyncio.to_thread(write_packet, str(self.directory), 'command',
            {'version': VERSION, 'session': self.session, 'sequence': self.command_sequence,
             'controller_epoch': self.session, 'deadline_ns': envelope.deadline_ns,
             'linear_x': command.linear_x, 'angular_z': command.angular_z, 'armed': self.armed,
             'footprint_rear': self.footprint_rear,
             'manual_direct': self.manual_direct,
             'drive_mode': self.drive_mode})
        self.wake_command()

    async def stop(self, fallback=None):
        if not self.session: return
        self.armed = False
        self.command_sequence += 1
        await asyncio.to_thread(write_packet, str(self.directory), 'command',
            {'version': VERSION, 'session': self.session, 'sequence': self.command_sequence,
             'controller_epoch': self.session, 'deadline_ns': time.monotonic_ns() + 250000000,
             'linear_x': 0., 'angular_z': 0., 'armed': False})
        self.wake_command()

    async def relocalize(self, x, y, yaw):
        """Explicit localization command, separate from all velocity mailboxes."""
        await self.stop()
        self.action_sequence += 1
        seq = self.action_sequence
        await asyncio.to_thread(write_packet, str(self.directory), 'action',
            dict(session=self.session, sequence=seq, deadline_ns=time.monotonic_ns() + 2000000000,
                 op='initialpose', x=x, y=y, yaw=yaw))
        after = None
        async with asyncio.timeout(3):
            while True:
                frame = await self.action_result.get(after=after)
                after = frame.sequence
                if frame.data['sequence'] == seq: return frame.data

    async def close(self):
        try:
            await self.stop()
            for process in self.processes:
                if process.returncode is None: process.terminate()
            for process in self.processes:
                try:
                    await asyncio.wait_for(process.wait(), 3)
                except TimeoutError:
                    process.kill()
                    await process.wait()
        finally:
            for log in self.logs: log.close()
            for sock in (self.updates, self.command_notify):
                if sock: sock.close()
            self.updates = self.command_notify = None
            self.processes, self.logs = [], []
            self.process = self.log = None
            if hasattr(self, 'config_path'): self.config_path.unlink(missing_ok=True)


def ROS1Topic(bridge, name):
    """Select a declared native topic port; Runtime owns its bridge dependency."""
    if not isinstance(bridge, LocalROS1Bridge): raise TypeError('pass LocalROS1Bridge')
    return bridge.outputs[name]
