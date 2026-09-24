"""Own a worker and its files until all parent lease handles are closed.

This small independent process keeps reacting even when the worker's event loop
or GIL is blocked. Nested workers each have their own supervisor/lease pair.
"""
import os
from pathlib import Path
import selectors
import shutil
import signal
import subprocess
import sys
import json


def main():
    directory, lease = Path(sys.argv[1]), int(sys.argv[2])
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    worker = None
    try:
        config = json.loads((directory / "config.json").read_text())
        module = config.get("worker_module", "rsim._worker")
        worker = subprocess.Popen([
            sys.executable, "-m", "rsim._exec", str(os.getpid()),
            sys.executable, "-m", module, str(directory),
        ], start_new_session=True)
        with selectors.DefaultSelector() as selector:
            selector.register(lease, selectors.EVENT_READ)
            while not stopping:
                if worker.poll() is not None:
                    shutil.rmtree(directory / "frames", ignore_errors=True)
                    status = directory / "status.json"
                    # A hard crash cannot write its own traceback.
                    state = json.loads(status.read_text()) if status.exists() else {}
                    if "error" not in state:
                        temporary = directory / "supervisor-status.tmp"
                        temporary.write_text(json.dumps({"pid": worker.pid,
                            "error": f"worker exited: {worker.returncode}"}))
                        temporary.replace(status)
                if selector.select(timeout=0.1):
                    if not os.read(lease, 1):
                        break
    finally:
        if worker is not None and worker.poll() is None:
            try:
                os.killpg(worker.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(worker.pid, signal.SIGKILL)
                worker.wait()
        os.close(lease)
        shutil.rmtree(directory, ignore_errors=True)


if __name__ == "__main__":
    main()
