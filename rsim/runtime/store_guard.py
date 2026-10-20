"""Reclaim local stores even if the Runtime owner is forcibly terminated."""
import os
from pathlib import Path
import signal
import sys
import time

from .platform import lease_reader, watch_lease
from rsim.transport.files import remove_tree


def main():
    directory, lease = Path(sys.argv[1]), lease_reader(sys.argv[2])
    ended = watch_lease(lease)
    signal.signal(signal.SIGTERM, lambda *_: ended.set())
    signal.signal(signal.SIGINT, lambda *_: ended.set())
    try:
        ended.wait()
        deadline = time.monotonic() + 7
        while time.monotonic() < deadline and any(directory.glob('p[0-9]*')):
            time.sleep(.05)
    finally:
        os.close(lease)
        remove_tree(directory, ignore_errors=True)


if __name__ == '__main__':
    main()
