"""Device identity policy, using bounded native lock primitives."""
import asyncio
import hashlib
from rsim.core.errors import ComponentError
from rsim.platforms import get_platform
from .paths import registry_directory


async def locked(path, timeout=30):
    native = get_platform().locks
    lock = native.open_lock(path)
    try:
        async with asyncio.timeout(timeout):
            while True:
                try:
                    native.try_lock(lock)
                    return lock
                except BlockingIOError:
                    await asyncio.sleep(.02)
    except BaseException:
        lock.close()
        raise


def acquire_device(key):
    directory = registry_directory()
    name = 'device-' + hashlib.sha256(key.encode()).hexdigest() + '.lock'
    native = get_platform().locks
    lease = native.open_lock(directory / name)
    try:
        native.try_lock(lease)
    except BlockingIOError as error:
        lease.close()
        raise ComponentError(f'physical device already owned: {key}; use the shared source') from error
    except BaseException:
        lease.close()
        raise
    return lease
