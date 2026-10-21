"""Explicit filesystem probes using only stdlib and native file primitives."""
from functools import lru_cache
import json
import mmap
from pathlib import Path
import subprocess
import sys

from .detect import get_platform
from .files import open_shared, hardlink, unlink, remove_tree
from .paths import new_directory
from .errors import CapabilityUnavailable


@lru_cache(maxsize=32)
def probe_storage(root, deep=False):
    directory = new_directory(prefix='rsim-probe-', root=root)
    path, link = directory / 'mapped.bin', directory / 'linked.bin'
    child = None
    try:
        filesystem = get_platform().files.filesystem_info(directory)
        if filesystem['remote']:
            raise CapabilityUnavailable('shared-mapping', f'network filesystems are outside the local shared domain: {root}')
        with open_shared(path, write=True, create=True) as file:
            file.write(b'rsim-mapping-probe')
            file.flush()
        with open_shared(path) as file:
            identity = get_platform().files.identity(file)
            view = mmap.mmap(file.fileno(), 0, access=mmap.ACCESS_READ)
        try:
            hardlink(path, link)
            with open_shared(link) as file:
                if get_platform().files.identity(file) != identity:
                    raise CapabilityUnavailable('shared-mapping', 'hardlink changed backing identity')
            if deep:
                # The other interpreter holds its own mapping while the parent unlinks.
                child = subprocess.Popen([sys.executable, '-m', 'rsim.platforms.probe', str(link)],
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                         text=True)
                # communicate is bounded; a marker file avoids blocking readline.
                import time
                deadline = time.monotonic() + 10
                while not (directory / 'mapped.ready').exists():
                    if child.poll() is not None or time.monotonic() >= deadline:
                        raise CapabilityUnavailable('shared-mapping', 'child mapping failed to become ready')
                    time.sleep(.01)
            unlink(path)
            unlink(link)
            if view[:] != b'rsim-mapping-probe':
                raise CapabilityUnavailable('shared-mapping', 'retained mapping changed after unlink')
            if child is not None:
                stdout, stderr = child.communicate('read\n', timeout=10)
                if child.returncode != 0 or stdout.strip() != 'rsim-mapping-probe':
                    raise CapabilityUnavailable('shared-mapping', f'child retained mapping failed: {stderr[-1000:]}')
            return dict(mapped_file_unlink=True, hardlink_reuse=True, child_mapping=deep,
                        directory=str(Path(root).absolute()), filesystem=filesystem)
        finally:
            view.close()
    except OSError as error:
        raise CapabilityUnavailable('shared-mapping', f'shared mapping unavailable at {root}: {error}') from error
    finally:
        if child is not None and child.poll() is None:
            child.kill()
            child.wait(timeout=5)
        remove_tree(directory)


if __name__ == '__main__':
    path = Path(sys.argv[1])
    with open_shared(path) as file:
        view = mmap.mmap(file.fileno(), 0, access=mmap.ACCESS_READ)
    try:
        (path.parent / 'mapped.ready').touch()
        sys.stdin.readline()
        print(view[:].decode(), flush=True)
    finally:
        view.close()
