"""Own a worker and its files until the parent lease closes."""
import json
import os
from pathlib import Path
import signal
from dataclasses import asdict

from rsim.platforms import get_platform
from rsim.platforms.process import watch_lease
from rsim.transport.files import remove_tree
from .options import RuntimeOptions
from .supervision import Lifecycle, State, read_status, stop_worker


def main():
    import sys
    native = get_platform().process
    directory, lease = Path(sys.argv[1]), native.lease_reader(sys.argv[2])
    ended = watch_lease(lease)
    signal.signal(signal.SIGTERM, lambda *_: ended.set())
    signal.signal(signal.SIGINT, lambda *_: ended.set())
    worker = None
    options = RuntimeOptions()
    lifecycle = Lifecycle('private-runtime-lease')
    lifecycle.transition(State.STARTING)
    report = None
    def save():
        if report is not None:
            temporary = report.with_suffix('.tmp')
            temporary.write_text(json.dumps(asdict(lifecycle)), encoding='utf-8')
            temporary.replace(report)
    try:
        config = json.loads((directory / 'config.json').read_text(encoding='utf-8'))
        options = RuntimeOptions.from_dict(config.get('options'))
        report = Path(config['log_path']).with_suffix('.report.json')
        save()
        worker = native.spawn_worker(config.get('worker_module', 'rsim.runtime.worker'), directory)
        reported = False
        import time
        deadline = time.monotonic() + options.startup_timeout_s
        while not ended.wait(.1):
            state = read_status(directory)
            if lifecycle.state == State.STARTING:
                if state.get('ready'):
                    lifecycle.transition(State.RUNNING)
                    save()
                elif time.monotonic() >= deadline:
                    raise TimeoutError('private worker initialization timed out')
            if worker.poll() is not None and not reported:
                lifecycle.cleanup.extend(remove_tree(directory / 'frames', ignore_errors=True))
                status = directory / 'status.json'
                if 'error' not in state:
                    temporary = directory / 'supervisor-status.tmp'
                    temporary.write_text(json.dumps({'pid': worker.pid, 'error': f'worker exited: {worker.returncode}'}), encoding='utf-8')
                    temporary.replace(status)
                reported = True
                lifecycle.transition(State.FAILED, RuntimeError(state.get('error', f'worker exited: {worker.returncode}')))
                save()
    except BaseException as error:
        lifecycle.transition(State.FAILED, error)
        raise
    finally:
        lifecycle.transition(State.STOPPING)
        try:
            stop_worker(worker, directory, options.shutdown)
        except Exception as error:
            lifecycle.cleanup.append(dict(operation='stop-worker', error=f'{type(error).__name__}: {error}'))
        finally:
            os.close(lease)
            lifecycle.cleanup.extend(remove_tree(directory, ignore_errors=True))
            lifecycle.transition(State.FAILED if lifecycle.failure or lifecycle.cleanup else State.CLOSED)
            save()


if __name__ == '__main__':
    main()
