"""Real executable lifecycle checks; no ROS installation or hardware required."""
import asyncio
import json
from pathlib import Path
import sys
from types import ModuleType

import pytest

from rsim import Runtime
from rsim.adapters.ros2 import Driver, GeneratedParameters


def install_probe(tmp_path, monkeypatch):
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_prefix = lambda _: str(tmp_path)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    executable = tmp_path / 'lib' / 'probe' / 'node'
    executable.parent.mkdir(parents=True)
    executable.write_text(f'#!{sys.executable}\n' + '''
import json, signal, sys, time
from pathlib import Path
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
path = Path(sys.argv[sys.argv.index('--params-file') + 1])
values = json.loads(path.read_text())['/**']['ros__parameters']
Path(values['output']).write_text(json.dumps({'values': values, 'path': str(path)}))
while True:
    time.sleep(.01)
''')
    executable.chmod(0o755)


async def wait_output(path):
    async with asyncio.timeout(10):
        while not path.exists():
            await asyncio.sleep(.01)
    return json.loads(path.read_text())


def test_generated_parameters_are_snapshot_and_private_session_files(tmp_path, monkeypatch):
    install_probe(tmp_path, monkeypatch)
    output = tmp_path / 'received.json'
    external = tmp_path / 'external.yaml'
    external.write_text('/**:\n  ros__parameters:\n    explicit: true\n')
    values = {'output': str(output), 'Grid/3D': 'false', 'flags': [True, False], 'number': 1.5}
    document = GeneratedParameters(values)
    values['flags'].append(True)
    values['number'] = 99
    driver = Driver('probe', 'node', key='probe:' + str(tmp_path),
                    log_path=tmp_path / 'logs' / 'probe.log', generated_parameters=document,
                    ros_args=['--params-file', str(external)])
    signature = driver.configuration()
    assert not (tmp_path / 'logs').exists()
    assert driver._parameter_file is None

    async def run():
        paths = []
        for _ in range(2):
            async with Runtime(driver):
                received = await wait_output(output)
                path = Path(received['path'])
                assert path.exists()
                paths.append(path)
                assert received['values']['flags'] == [True, False]
                assert received['values']['number'] == 1.5
                assert received['values']['Grid/3D'] == 'false'
                assert driver.configuration() == signature
            assert driver.process.returncode == 0
            assert not path.parent.exists()
            assert external.read_text().endswith('explicit: true\n')
            output.unlink()
        assert paths[0] != paths[1]
    asyncio.run(run())


def test_generated_files_cleaned_when_subprocess_launch_fails(tmp_path, monkeypatch):
    install_probe(tmp_path, monkeypatch)
    document = GeneratedParameters({'value': 1})
    materialized = []
    original = GeneratedParameters.materialize

    def materialize(self):
        file = original(self)
        materialized.append(file.path)
        return file

    async def fail(*args, **kwargs):
        assert materialized[0].is_file()
        raise OSError('probe failure')

    monkeypatch.setattr(GeneratedParameters, 'materialize', materialize)
    monkeypatch.setattr(asyncio, 'create_subprocess_exec', fail)
    driver = Driver('probe', 'node', key='failure:' + str(tmp_path), generated_parameters=document)
    with pytest.raises(OSError, match='probe failure'):
        asyncio.run(driver.open())
    assert not materialized[0].parent.exists()
    assert driver._device_lease is driver._parameter_file is None


def test_modified_external_parameters_fail_without_touching_user_file(tmp_path, monkeypatch):
    install_probe(tmp_path, monkeypatch)
    external = tmp_path / 'user.yaml'
    external.write_text('/**: {}\n')
    driver = Driver('probe', 'node', key='changed:' + str(tmp_path),
                    generated_parameters={'value': 1}, ros_args=['--params-file', str(external)])
    external.write_text('/changed: {}\n')
    with pytest.raises(ValueError, match='file changed'):
        asyncio.run(driver.open())
    assert driver._parameter_file is driver._device_lease is None
    assert external.read_text() == '/changed: {}\n'


def test_parallel_parameter_sessions_do_not_remove_each_others_files():
    document = GeneratedParameters({'value': [1, 2]})
    first, second = document.materialize(), document.materialize()
    try:
        assert first.path != second.path
        first.close()
        assert second.path.exists()
        second.path.write_text('changed')
        with pytest.raises(ValueError, match='changed'):
            second.validate()
    finally:
        first.close()
        second.close()


@pytest.mark.parametrize('values', [{'x': None}, {'x': [1, True]}, {'x': float('nan')}, {'': 1}])
def test_invalid_generated_parameters_rejected_before_files(values):
    with pytest.raises((ValueError, TypeError)):
        GeneratedParameters(values)
