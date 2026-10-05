"""Reclaim a deployment's local stores even if its Python owner is SIGKILLed."""
import os
from pathlib import Path
import selectors
import shutil
import signal
import sys
import time


def main():
    directory, lease = Path(sys.argv[1]), int(sys.argv[2])
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(lease, selectors.EVENT_READ)
            while not stopped:
                if selector.select(timeout=.1) and not os.read(lease, 1):
                    break
        # Each worker has its own independent lease supervisor. Let those stop
        # blocked workers and remove their directories before the final sweep.
        deadline = time.monotonic() + 7
        while time.monotonic() < deadline and any(directory.glob("p[0-9]*")):
            time.sleep(.05)
    finally:
        os.close(lease)
        shutil.rmtree(directory, ignore_errors=True)


if __name__ == "__main__":
    main()
