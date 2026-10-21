"""Lease plumbing; ownership decisions remain in runtime supervision."""
import asyncio
import os
import sys
import threading

from .detect import get_platform
from .errors import CapabilityUnavailable, record_cleanup


async def spawn_leased(module, directory, **options):
    read, write = os.pipe()
    native = get_platform().process
    task = None
    try:
        argument, kwargs = native.lease_options(read)
        task = asyncio.create_task(asyncio.create_subprocess_exec(
            sys.executable, '-m', module, str(directory), argument,
            **native.supervisor_options(), **kwargs, **options))
        # Cancellation cannot leave an unobserved spawn owning an inherited lease.
        process = await asyncio.shield(task)
        return process, write
    except BaseException as error:
        os.close(write)
        if task is not None and not task.cancelled():
            process = None
            try:
                process = await task
                try:
                    await asyncio.wait_for(process.wait(), 12)
                except TimeoutError:
                    process.kill()
                    await asyncio.wait_for(process.wait(), 5)
            except (NotImplementedError, OSError, TimeoutError) as cleanup:
                if process is not None:
                    record_cleanup('spawn-rollback', directory, cleanup)
                    error.add_note(f'spawn rollback also failed: {cleanup}')
        if isinstance(error, NotImplementedError):
            loop = type(asyncio.get_running_loop()).__name__
            raise CapabilityUnavailable('subprocess',
                f'{loop} cannot spawn supervised processes; use a subprocess-capable loop '
                'or start an independent provider outside this kernel') from error
        raise
    finally:
        os.close(read)


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
