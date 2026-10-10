"""Bringup must select one driver owner and preserve graceful tmux shutdown."""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
import uuid

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(shutil.which('tmux') is None, reason='tmux not installed')
def test_tmux_stop_delivers_sigint_and_does_not_replace_live_session(tmp_path):
    session = 'rsim-test-' + uuid.uuid4().hex[:10]
    marker = tmp_path/'graceful stop.txt'
    child = tmp_path/'child.py'
    child.write_text('import signal,time\nfrom pathlib import Path\n'
        f'def stop(*_):\n time.sleep(.1)\n print("cleanup complete",flush=True)\n Path({str(marker)!r}).write_text("stopped"); raise SystemExit(0)\n'
        'signal.signal(signal.SIGINT,stop)\nprint("ready",flush=True)\n'
        'while True: time.sleep(.05)\n')
    script = tmp_path/'entry.sh'
    script.write_text('launch_entry=$(realpath "${BASH_SOURCE[0]}")\n'
        f'source {shlex.quote(str(ROOT/"examples/_launch.sh"))}\n'
        'launch_help() { :; }\n'
        f'launch_command() {{ CMD=({shlex.quote(sys.executable)} {shlex.quote(str(child))}); }}\n'
        'launch_main test "$@"\n')
    env = {**os.environ, 'RSIM_TMUX_SESSION': session, 'RSIM_ENV_FILE': str(tmp_path/'absent.env'),
           'RSIM_CONDA_ENV': ''}
    def invoke(action):
        return subprocess.run(['bash', str(script), action], env=env, capture_output=True, text=True, timeout=20)
    try:
        assert invoke('start').returncode == 0
        deadline = time.monotonic()+5
        while 'ready' not in invoke('status').stdout:
            assert time.monotonic() < deadline
            time.sleep(.05)
        duplicate = invoke('start')
        assert duplicate.returncode != 0 and 'Session exists' in duplicate.stderr
        assert not marker.exists()
        stopped = invoke('stop')
        assert stopped.returncode == 0, stopped.stderr
        assert marker.read_text() == 'stopped'
    finally:
        subprocess.run(['tmux', 'kill-session', '-t', '='+session], capture_output=True)


@pytest.mark.skipif(shutil.which('tmux') is None, reason='tmux not installed')
def test_hardware_windows_have_independent_lifetimes(tmp_path):
    session='rsim-test-'+uuid.uuid4().hex[:10]
    child=tmp_path/'child.py'
    child.write_text('import time\nprint("ready",flush=True)\nwhile True:time.sleep(.1)\n')
    script=tmp_path/'group.sh'
    script.write_text('launch_entry=$(realpath "${BASH_SOURCE[0]}")\n'
        f'source {shlex.quote(str(ROOT/"examples/_launch.sh"))}\n'
        'launch_group=1\nLAUNCH_WINDOWS=(motor imu)\nlaunch_help() { :; }\n'
        f'launch_command() {{ CMD=({shlex.quote(sys.executable)} {shlex.quote(str(child))}); }}\n'
        'launch_main test "$@"\n')
    env={**os.environ,'RSIM_TMUX_SESSION':session,'RSIM_ENV_FILE':str(tmp_path/'absent.env'),'RSIM_CONDA_ENV':''}
    def invoke(*args):return subprocess.run(['bash',str(script),*args],env=env,capture_output=True,text=True,timeout=20)
    try:
        assert invoke('start').returncode==0
        deadline=time.monotonic()+5
        while invoke('status').stdout.count('ready')<2:
            assert time.monotonic()<deadline
            time.sleep(.05)
        assert invoke('stop','imu').returncode==0
        assert 'dead=0' in invoke('status','motor').stdout
        assert invoke('start','imu').returncode==0
        assert invoke('stop','motor').returncode==0
        assert 'dead=0' in invoke('status','imu').stdout
        assert invoke('stop','imu').returncode==0
    finally:
        subprocess.run(['tmux','kill-session','-t','='+session],capture_output=True)
