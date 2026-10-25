"""Own one complete navigation process tree inside a dedicated tmux pane."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def write(path, value):
    temporary = path.with_suffix('.next')
    temporary.write_text(json.dumps(value), encoding='utf8')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--data-dir', default='assets/navigation')
    parser.add_argument('--simulate', action='store_true')
    args = parser.parse_args()
    root = Path(args.data_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    run = Path(os.environ.get('RSIM_RUN_DIR', '/dev/shm/rsim-service'))
    run.mkdir(parents=True, exist_ok=True)
    # flock belongs to this runner, not to a PID file that can become stale.
    import fcntl
    lock = (run / 'lifecycle.lock').open('a+')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    state, restart, stop = (run / name for name in ('lifecycle.json', 'restart.request', 'stop.request'))
    restart.unlink(missing_ok=True)
    stop.unlink(missing_ok=True)
    stopping = False
    def terminate(_signum, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    env = dict(os.environ, RSIM_LIFECYCLE_STATE=str(state), RSIM_RESTART_REQUEST=str(restart),
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    command = [sys.executable, '-m', 'rsim.apps.navigation', '--config', args.config, '--data-dir', str(root)]
    if args.simulate: command.append('--simulate')
    generation = 0
    try:
        while not stopping:
            generation += 1
            logfile = root / 'web.log'
            if logfile.exists() and logfile.stat().st_size > 5_000_000:
                logfile.replace(root / 'web.previous.log')
            with (root / 'web.log').open('ab', buffering=0) as log:
                child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                reason = 'running'
                while child.poll() is None:
                    write(state, dict(runner_pid=os.getpid(), child_pid=child.pid, generation=generation,
                                      updated_at=time.time(), config=str(Path(args.config).resolve()), status=reason))
                    if stop.exists(): stopping = True
                    if stopping: reason = 'stopping'; break
                    if restart.exists():
                        try: request = json.loads(restart.read_text())
                        except (OSError, ValueError): request = {}
                        restart.unlink(missing_ok=True)
                        if request.get('child_pid') == child.pid and time.time() - request.get('requested_at', 0) < 15:
                            reason = 'restarting'; break
                    time.sleep(.25)
                if child.poll() is None:
                    child.terminate()
                    try: child.wait(timeout=25)
                    except subprocess.TimeoutExpired:
                        # Only the group created by this runner is eligible.
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=5)
                write(state, dict(runner_pid=os.getpid(), child_pid=None, generation=generation,
                                  updated_at=time.time(), status=reason, exit_code=child.returncode))
            if not stopping: time.sleep(2)
    finally:
        stop.unlink(missing_ok=True)


if __name__ == '__main__': main()
