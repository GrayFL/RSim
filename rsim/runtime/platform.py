"""Platform-specific ownership primitives; graph and port semantics stay shared."""
import asyncio
import getpass
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading


def user_id():
    if os.name != 'nt':
        return str(os.getuid())
    return hashlib.sha256((os.environ.get('USERDOMAIN', '') + '\\' + getpass.getuser()).encode()).hexdigest()[:16]


def boot_id():
    if os.name != 'nt':
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    import ctypes
    import uuid
    data = ctypes.create_string_buffer(32)
    query = ctypes.WinDLL('ntdll').NtQuerySystemInformation
    query.restype = ctypes.c_long
    query.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p]
    status = query(90, data, len(data), None)  # SystemBootEnvironmentInformation
    if status < 0:
        raise OSError(f'cannot read Windows boot identity: {status & 0xffffffff:#x}')
    return str(uuid.UUID(bytes_le=data.raw[:16]))


def supervisor_options():
    if os.name != 'nt':
        return dict(start_new_session=True)
    flags = subprocess.CREATE_NO_WINDOW
    if os.environ.get('RSIM_WINDOWS_JOB') == '1':
        flags |= subprocess.CREATE_BREAKAWAY_FROM_JOB
    return dict(creationflags=flags)


async def spawn_leased(module, directory, **options):
    read, write = os.pipe()
    kwargs = supervisor_options()
    argument = str(read)
    if os.name == 'nt':
        import msvcrt
        handle = msvcrt.get_osfhandle(read)
        os.set_handle_inheritable(handle, True)
        startup = subprocess.STARTUPINFO()
        startup.lpAttributeList = {'handle_list': [handle]}
        kwargs.update(startupinfo=startup, close_fds=True)
        argument = f'handle:{handle}'
    else:
        kwargs['pass_fds'] = (read,)
    try:
        process = await asyncio.create_subprocess_exec(
            sys.executable, '-m', module, str(directory), argument, **kwargs, **options)
        return process, write
    except BaseException:
        os.close(write)
        raise
    finally:
        os.close(read)


def lease_reader(argument):
    if argument.startswith('handle:'):
        import msvcrt
        handle = int(argument.split(':', 1)[1])
        os.set_handle_inheritable(handle, False)
        return msvcrt.open_osfhandle(handle, os.O_RDONLY | os.O_BINARY)
    return int(argument)


def watch_lease(fd):
    ended = threading.Event()
    def read():
        try:
            while os.read(fd, 1):
                pass
        except OSError:
            pass
        finally:
            ended.set()
    threading.Thread(target=read, daemon=True, name='rsim-lease').start()
    return ended


def install_shutdown(stopped, directory):
    loop = asyncio.get_running_loop()
    if os.name != 'nt':
        for sig in (signal.SIGTERM, signal.SIGINT):
            loop.add_signal_handler(sig, stopped.set)
    else:
        async def poll():
            while not stopped.is_set():
                if (Path(directory) / 'stop').exists():
                    stopped.set()
                    return
                await asyncio.sleep(.05)
        return asyncio.create_task(poll(), name='rsim-shutdown')


def spawn_worker(module, directory, *arguments):
    command = [sys.executable, '-m', module, str(directory), *map(str, arguments)]
    if os.name == 'nt':
        from .windows_process import JobProcess
        return JobProcess(command)
    return subprocess.Popen([sys.executable, '-m', 'rsim.runtime.exec', str(os.getpid()),
                             *command], start_new_session=True)


def stop_worker(worker, directory):
    if worker is None:
        return
    if worker.poll() is None:
        if os.name == 'nt':
            (Path(directory) / 'stop').touch()
        else:
            try:
                os.killpg(worker.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        try:
            worker.wait(timeout=5)
        except subprocess.TimeoutExpired:
            if os.name == 'nt':
                worker.kill()
            else:
                os.killpg(worker.pid, signal.SIGKILL)
            worker.wait()
    if os.name == 'nt':
        worker.close()
