"""Reclaim local stores even if the Runtime owner is forcibly terminated."""
import os
from pathlib import Path
import signal
import sys
import time
import json

from rsim.platforms import get_platform
from rsim.platforms.process import watch_lease
from rsim.transport.files import remove_tree
from .options import RuntimeOptions


def main():
    directory, lease = Path(sys.argv[1]), get_platform().process.lease_reader(sys.argv[2])
    ended = watch_lease(lease)
    signal.signal(signal.SIGTERM, lambda *_: ended.set())
    signal.signal(signal.SIGINT, lambda *_: ended.set())
    try:
        options = RuntimeOptions.from_dict(json.loads((directory / 'guard.json').read_text(encoding='utf-8')))
        ended.wait()
        deadline = time.monotonic() + options.shutdown.grace_s + options.shutdown.kill_wait_s + 1
        while time.monotonic() < deadline and any(directory.glob('p[0-9]*')):
            time.sleep(.05)
    finally:
        os.close(lease)
        remove_tree(directory, ignore_errors=True)


if __name__ == '__main__':
    main()
