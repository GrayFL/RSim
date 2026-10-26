"""Manually manage only the retired 18765 stack; never the OEM ROS group."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

SUPERVISOR = ['/usr/bin/python', '/usr/local/bin/supervisorctl', '-c', '/etc/supervisord.conf']
SCRIPTS = {'/root/DataBags/zbz/onboard_nav/server.py', '/root/DataBags/zbz/onboard_nav/handle_scale.py',
           '/root/DataBags/zbz/zbz_obs_view.py', '/root/DataBags/zbz/zbz_web_teleop_bridge.py'}
HELPERS = {'zbz_onboard_batt', 'zbz_onboard_spd', 'zbz_onboard_handle', 'zbz_handle_scale'}


def processes():
    result = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            argv = (path.parent/'cmdline').read_bytes().decode(errors='replace').split('\0')
            result[int(path.parent.name)] = (int(fields[1]), fields[19], argv)
        except (OSError, ValueError): pass
    return result


def manage(operation):
    if operation not in ('start', 'stop', 'status'): raise ValueError('invalid operation')
    env = dict(os.environ, PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin')
    for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH'): env.pop(key, None)
    if operation == 'stop':
        before = processes()
        owned = {pid for pid, (_, _, argv) in before.items() if SCRIPTS.intersection(argv) or HELPERS.intersection(argv)}
        for _ in range(12): owned.update(pid for pid, (parent, _, _) in before.items() if parent in owned)
        subprocess.run(SUPERVISOR + ['stop', 'onboard-nav'], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
        for sig in (signal.SIGTERM, signal.SIGKILL):
            current = processes()
            for pid in owned:
                if pid in current and before[pid][1] == current[pid][1]:
                    try: os.kill(pid, sig)
                    except ProcessLookupError: pass
            time.sleep(.4)
    elif operation == 'start':
        response = subprocess.run(SUPERVISOR + ['start', 'onboard-nav'], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=25)
        if response.returncode and b'ALREADY_STARTED' not in response.stdout:
            raise RuntimeError(response.stdout.decode(errors='replace')[:200])
    response = subprocess.run(SUPERVISOR + ['status', 'onboard-nav'], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
    return dict(ok=True, running=b'RUNNING' in response.stdout, status=response.stdout.decode().strip())


if __name__ == '__main__':
    try: result = manage(sys.argv[1])
    except Exception as error: result = dict(ok=False, error=str(error))
    print(json.dumps(result))
    sys.exit(0 if result['ok'] else 1)
