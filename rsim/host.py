"""One cooperating source per user/host/ROS domain, leased by Unix sockets."""
from __future__ import annotations

import asyncio
import fcntl
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading

import cloudpickle

from .core import SensorError
from .process import ProcessSensor, descriptor_qos


def registry_directory():
    directory = Path(tempfile.gettempdir()) / f"rsim-host-{os.getuid()}"
    directory.mkdir(mode=0o700, exist_ok=True)
    if directory.is_symlink() or directory.stat().st_uid != os.getuid():
        raise SensorError("source registry is not owned by the current user")
    os.chmod(directory, 0o700)
    return directory


class SharedSensor(ProcessSensor):
    """`key` identifies the physical source; `version` identifies its recipe.

    All clients with a key must specify the same version and history. Only the
    first client's factory runs. Use a configuration digest for version when a
    factory has parameters. Factories are trusted local Python code.
    """

    def __init__(self, factory, *, key, version="1", history=16, hz=200):
        super().__init__(factory, history=history, hz=hz)
        self.source_key, self.version = key, version
        self._reader = self._writer = None

    async def open(self):
        from std_msgs.msg import String
        identity = f"{os.environ.get('ROS_DOMAIN_ID', '0')}:{self.source_key}"
        digest = hashlib.sha256(identity.encode()).hexdigest()[:32]
        registry = registry_directory()
        lock_path = registry / (digest + ".lock")
        state_path = registry / (digest + ".json")
        socket_path = registry / (digest + ".sock")
        signature = {"version": self.version, "history": self._history.maxlen}
        lock = lock_path.open("a+")
        try:
            while True:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    await asyncio.sleep(0.01)
            state = None
            if state_path.exists():
                candidate = json.loads(state_path.read_text())
                try:
                    reader, writer = await asyncio.wait_for(
                        asyncio.open_unix_connection(str(socket_path)), 1)
                except (FileNotFoundError, ConnectionRefusedError, TimeoutError):
                    pass
                else:
                    self._reader, self._writer = reader, writer
                    await asyncio.wait_for(reader.readexactly(1), 3)
                    if candidate["signature"] != signature:
                        raise ValueError(f"conflicting shared source configuration: {self.source_key}")
                    state = candidate
            if state is None:
                state = self._launch(signature, socket_path, lock_path, state_path)
                self._reader, self._writer = await asyncio.open_connection(sock=self._initial_client)
                self._initial_client = None
                await asyncio.wait_for(self._reader.readexactly(1), 5)
            self.directory = Path(state["directory"])
            self._remote_sequence = 0
            self.pending.clear()
            self.subscription = self.children[0].node.create_subscription(
                String, state["topic"], lambda msg: self.pending.append(msg.data), descriptor_qos())
            self.task("receive", self.receive, hz=self.hz)
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)
            lock.close()

    def _launch(self, signature, socket_path, lock_path, state_path):
        import shutil
        import uuid
        directory = Path(tempfile.mkdtemp(prefix=f"rsim-{os.getuid()}-", dir="/dev/shm"))
        topic = "/rsim/frames/p" + uuid.uuid4().hex
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        initial_server, initial_client = socket.socketpair()
        try:
            socket_path.unlink(missing_ok=True)
            listener.bind(str(socket_path))
            listener.listen(128)
            (directory / "factory.pkl").write_bytes(cloudpickle.dumps(self.factory))
            (directory / "config.json").write_text(json.dumps({
                "history": self._history.maxlen, "topic": topic,
                "sys_path": [str(Path(p).resolve()) for p in sys.path]}))
            state = {"directory": str(directory), "topic": topic, "signature": signature}
            temporary = state_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(state))
            temporary.replace(state_path)
            with (directory / "worker.log").open("w") as log:
                process = subprocess.Popen([
                    sys.executable, "-m", "rsim._shared_supervisor", str(directory),
                    str(listener.fileno()), str(initial_server.fileno()),
                    str(socket_path), str(lock_path),
                ], pass_fds=(listener.fileno(), initial_server.fileno()),
                    start_new_session=True, stdout=log, stderr=log)
            # Reap the daemon if this client stays alive after handing ownership
            # to other clients. This thread never handles sensor data or the GIL.
            threading.Thread(target=process.wait, daemon=True, name="rsim-source-reaper").start()
            self._initial_client = initial_client
            return state
        except BaseException:
            initial_client.close()
            shutil.rmtree(directory, ignore_errors=True)
            raise
        finally:
            listener.close()
            initial_server.close()

    async def receive(self):
        if self._reader.at_eof():
            raise SensorError("shared source supervisor disconnected")
        await super().receive()

    async def close(self):
        if self._writer is not None:
            self._writer.close()
            await self._writer.wait_closed()
            self._writer = None
        initial = getattr(self, "_initial_client", None)
        if initial is not None:
            initial.close()
            self._initial_client = None
        if self.subscription is not None:
            self.children[0].node.destroy_subscription(self.subscription)
            self.subscription = None
        self.pending.clear()
