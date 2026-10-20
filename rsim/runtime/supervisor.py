"""Own a worker and its files until the parent lease closes."""
import json
import os
from pathlib import Path
import signal

from .platform import lease_reader, watch_lease, spawn_worker, stop_worker
from rsim.transport.files import remove_tree


def main():
    import sys
    directory, lease = Path(sys.argv[1]), lease_reader(sys.argv[2])
    ended = watch_lease(lease)
    signal.signal(signal.SIGTERM, lambda *_: ended.set())
    signal.signal(signal.SIGINT, lambda *_: ended.set())
    worker = None
    try:
        config = json.loads((directory / 'config.json').read_text(encoding='utf-8'))
        worker = spawn_worker(config.get('worker_module', 'rsim.runtime.worker'), directory)
        reported = False
        while not ended.wait(.1):
            if worker.poll() is not None and not reported:
                remove_tree(directory / 'frames', ignore_errors=True)
                status = directory / 'status.json'
                state = json.loads(status.read_text(encoding='utf-8')) if status.exists() else {}
                if 'error' not in state:
                    temporary = directory / 'supervisor-status.tmp'
                    temporary.write_text(json.dumps({'pid': worker.pid, 'error': f'worker exited: {worker.returncode}'}), encoding='utf-8')
                    temporary.replace(status)
                reported = True
    finally:
        stop_worker(worker, directory)
        os.close(lease)
        remove_tree(directory, ignore_errors=True)


if __name__ == '__main__':
    main()
