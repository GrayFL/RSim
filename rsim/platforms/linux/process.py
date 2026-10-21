"""Linux lease inheritance, process groups and managed exec trampoline."""
import asyncio
import os
from pathlib import Path
import signal
import subprocess
import sys


def supervisor_options():
    return dict(start_new_session=True)


def lease_options(read):
    return str(read), dict(pass_fds=(read,))


def lease_reader(argument):
    return int(argument)


def install_shutdown(stopped, directory):
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, stopped.set)


def spawn_worker(module, directory, *arguments):
    command = [sys.executable, '-m', module, str(directory), *map(str, arguments)]
    return subprocess.Popen([sys.executable, '-m', 'rsim.platforms.linux.exec', str(os.getpid()),
                             *command], start_new_session=True)


def request_stop(worker, directory):
    try:
        os.killpg(worker.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass


def kill_tree(worker):
    try:
        os.killpg(worker.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def close_worker(worker):
    # Also reclaim surviving descendants of an already-exited group leader.
    kill_tree(worker)
