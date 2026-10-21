"""Lifecycle policy shared by both platforms and ownership strategies."""
from dataclasses import dataclass, field
from enum import Enum
import subprocess
import time

from rsim.platforms import get_platform
from rsim.platforms.files import open_shared
from .options import ShutdownPolicy
import asyncio
import json
from pathlib import Path
from rsim.core.errors import ComponentError
from rsim.core.errors import CleanupFailed


class State(str, Enum):
    NEW = 'NEW'
    STARTING = 'STARTING'
    RUNNING = 'RUNNING'
    STOPPING = 'STOPPING'
    CLOSED = 'CLOSED'
    FAILED = 'FAILED'


@dataclass
class Lifecycle:
    ownership: str
    state: State = State.NEW
    failure: str | None = None
    cleanup: list = field(default_factory=list)
    transitions: list = field(default_factory=list)

    def transition(self, state, error=None):
        allowed = {State.NEW: {State.STARTING, State.CLOSED, State.FAILED},
                   State.STARTING: {State.RUNNING, State.STOPPING, State.FAILED},
                   State.RUNNING: {State.STOPPING, State.FAILED},
                   State.STOPPING: {State.CLOSED, State.FAILED},
                   State.CLOSED: {State.STARTING}, State.FAILED: {State.STARTING, State.STOPPING}}
        if state == self.state:
            return
        if state not in allowed[self.state]:
            raise RuntimeError(f'invalid lifecycle transition {self.state} -> {state}')
        self.state = state
        if error is not None:
            self.failure = f'{type(error).__name__}: {error}'
        self.transitions.append(dict(state=state.value, monotonic_s=time.monotonic()))


def stop_worker(worker, directory, policy=None):
    """Cooperate, wait, escalate, then release ownership; never wait indefinitely."""
    if worker is None:
        return
    policy = policy or ShutdownPolicy()
    native = get_platform().process
    try:
        if worker.poll() is None:
            native.request_stop(worker, directory)
            try:
                worker.wait(timeout=policy.grace_s)
            except subprocess.TimeoutExpired:
                native.kill_tree(worker)
                worker.wait(timeout=policy.kill_wait_s)
    finally:
        native.close_worker(worker)


def read_status(directory):
    try:
        with open_shared(Path(directory) / 'status.json') as file:
            return json.load(file)
    except FileNotFoundError:
        return {}


def check_cleanup_report(log_path):
    if log_path is None:
        return
    report = Path(log_path).with_suffix('.report.json')
    try:
        with open_shared(report) as file:
            state = json.load(file)
    except FileNotFoundError:
        return
    if state.get('cleanup'):
        raise CleanupFailed(f'supervisor cleanup failed; report: {report}', phase='close', path=str(report))


async def wait_ready(process, directory, timeout):
    directory = Path(directory)
    async with asyncio.timeout(timeout):
        while True:
            state = read_status(directory)
            if 'error' in state:
                raise ComponentError(state['error'])
            if state.get('ready'):
                return state
            if process.returncode is not None:
                raise ComponentError(f'supervisor exited before readiness: {process.returncode}')
            await asyncio.sleep(.02)
