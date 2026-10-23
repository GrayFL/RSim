"""Run a named release gate, optionally from a wheel outside the checkout."""
import argparse
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from contextlib import contextmanager
import time


ROOT = Path(__file__).resolve().parents[1]
SUITES = {
    'core': ['test_core', 'test_component_signal', 'test_sync', 'test_commands', 'test_inprocess_transport',
             'test_runtime_contracts', 'test_data_contracts'],
    'runtime-dds': ['test_platform_contracts', 'test_supervision_contracts', 'test_process', 'test_placement',
                    'test_shared', 'test_shared_component', 'test_host', 'test_host_cross_process',
                    'test_parent_death', 'test_windows_runtime', 'test_transport'],
    'cross-python': ['test_cross_environment', 'test_shared_component'],
    'mujoco-adapter': ['test_mujoco_adapter'],
    'ros2-integration': ['test_ros_context', 'test_ros_arrays', 'test_ros_topic_conversions',
                        'test_transport', 'test_chassis_service', 'test_service_separation', 'test_mapping'],
}
REQUIRED = {
    'core': ['numpy', 'cloudpickle'],
    'runtime-dds': ['cyclonedds', 'psutil'],
    'cross-python': ['cyclonedds', 'psutil'],
    'mujoco-adapter': [],
    'ros2-integration': ['rclpy', 'sensor_msgs.msg', 'nav_msgs.msg', 'cyclonedds', 'graphmap.pose'],
}


@contextmanager
def temporary_workspace():
    directory = Path(tempfile.mkdtemp(prefix='rsim-gate-'))
    try:
        yield directory
    finally:
        # A shared lease closes asynchronously; on Windows a final supervisor
        # may still hold cwd while completing its bounded resource cleanup.
        deadline = time.monotonic() + 15
        while True:
            try:
                shutil.rmtree(directory)
                break
            except FileNotFoundError:
                break
            except PermissionError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', required=True, choices=SUITES)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--installed', action='store_true', help='require wheel import and copy tests outside checkout')
    parser.add_argument('--cross-backend', choices=('cyclonedds', 'ros2'), default='cyclonedds',
                        help='provider backend for the cross-python gate; clients always use native DDS')
    args = parser.parse_args()
    if args.installed and not sys.flags.isolated:
        # Codex shells, notebooks and developer profiles may inject the checkout
        # via PYTHONPATH. Start the gate isolated before its first RSim import.
        environment = dict(os.environ)
        environment.pop('PYTHONPATH', None)
        return subprocess.run([sys.executable, '-I', str(Path(__file__).resolve()), *sys.argv[1:]],
                              env=environment).returncode
    output = args.output.absolute()
    output.mkdir(parents=True, exist_ok=True)
    for name in ('tests.xml', 'doctor.json', 'environment.json', 'skipped.json', 'pytest.log',
                 'failure.log', 'gate-failure.xml'):
        (output / name).unlink(missing_ok=True)
    import rsim
    origin = Path(rsim.__file__).resolve()
    if args.installed and (origin.is_relative_to(ROOT / 'rsim') or not origin.is_relative_to(Path(sys.prefix))):
        raise RuntimeError(f'installed artifact gate imported unexpected RSim: {origin}')
    versions = dict(python=sys.version, executable=sys.executable, rsim_file=str(origin), suite=args.suite)
    versions['distributions'] = {}
    for distribution in ('rsim', 'numpy', 'cloudpickle', 'cyclonedds', 'pytest', 'psutil', 'mujoco', 'graphmap'):
        try:
            versions['distributions'][distribution] = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            pass
    required_modules = list(REQUIRED[args.suite])
    if args.suite == 'cross-python' and args.cross_backend == 'ros2':
        required_modules.extend(['rclpy', 'std_msgs.msg'])
    for module in required_modules:
        imported = importlib.import_module(module)  # required deps never importorskip
        versions[module] = dict(version=getattr(imported, '__version__', None), file=getattr(imported, '__file__', None))
    if args.suite == 'cross-python':
        client = os.environ.get('RSIM_TEST_CLIENT_PYTHON')
        if not client or Path(client).resolve() == Path(sys.executable).resolve():
            raise RuntimeError('cross-python requires a different RSIM_TEST_CLIENT_PYTHON interpreter')
        check = ('import sys,cyclonedds,rsim; from pathlib import Path; '
                 'assert Path(rsim.__file__).resolve().is_relative_to(Path(sys.prefix)); '
                 'print(sys.version); print(rsim.__file__)')
        result = subprocess.run([client, '-I', '-c', check],
                                capture_output=True, text=True, check=True, timeout=20)
        versions['client'] = result.stdout
    (output / 'environment.json').write_text(json.dumps(versions, indent=2), encoding='utf-8')
    required = {'runtime-dds': 'process,shared-mapping,dds', 'cross-python': 'shared-mapping,dds',
                'ros2-integration': 'dds'}.get(args.suite, '')
    environment = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1',
                       RSIM_LOG_ROOT=str(output / 'logs'))
    if args.installed:
        environment.pop('PYTHONPATH', None)
        environment['PYTHONNOUSERSITE'] = '1'
        environment['RSIM_TEST_INSTALLED'] = '1'
    with temporary_workspace() as workspace:
        workspace = Path(workspace)
        directory = workspace if args.installed else ROOT
        if args.installed:
            shutil.copytree(ROOT / 'tests', workspace / 'tests', ignore=shutil.ignore_patterns('__pycache__'))
            # Integration tests use versioned recipe fixtures. Copy data only;
            # never copy the rsim package or executable provider implementations.
            for recipe in (ROOT / 'examples').rglob('*.yaml'):
                target = workspace / recipe.relative_to(ROOT)
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(recipe.read_bytes())
        with (output / 'doctor.json').open('w', encoding='utf-8') as log:
            result = subprocess.run([sys.executable, '-m', 'rsim.apps.doctor', '--require', required, '--deep', '--json',
                                     '--backend', args.cross_backend if args.suite == 'cross-python' else 'cyclonedds'],
                                     cwd=directory, env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=90)
        if result.returncode:
            raise RuntimeError('required doctor probe failed; see doctor.json')
        command = [sys.executable, '-m', 'pytest', '-q', '-ra', '--junitxml=' + str(output / 'tests.xml')]
        command += [str(directory / 'tests' / (name + '.py')) for name in SUITES[args.suite]]
        if args.suite in ('runtime-dds', 'cross-python'):
            selection = ('not ros2 and not ros_provider' if args.suite == 'runtime-dds' else
                         'cross_environment and ' + ('ros2' if args.cross_backend == 'ros2' else 'not ros2'))
            command += ['-k', selection]
        with (output / 'pytest.log').open('w', encoding='utf-8') as log:
            result = subprocess.run(command, cwd=directory, env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=600)
        report = ET.parse(output / 'tests.xml')
        cases = report.findall('.//testcase')
        skipped = [dict(name=case.attrib.get('name'), reason=case.find('skipped').attrib.get('message'))
                   for case in cases if case.find('skipped') is not None]
        (output / 'skipped.json').write_text(json.dumps(skipped, indent=2), encoding='utf-8')
        print(json.dumps(dict(suite=args.suite, tests=len(cases), skipped=len(skipped), exit_code=result.returncode,
                              output=str(output)), indent=2))
        if not cases or len(cases) == len(skipped):
            raise RuntimeError('gate collected no executed tests')
        # Native Job cases on Linux and ROS2-only cases outside ROS runners are
        # enumerated in skipped.json. Required imports/probes above cannot skip.
        return result.returncode


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception:
        import traceback
        detail = traceback.format_exc()
        if '--output' in sys.argv:
            output = Path(sys.argv[sys.argv.index('--output') + 1]).absolute()
            output.mkdir(parents=True, exist_ok=True)
            (output / 'failure.log').write_text(detail, encoding='utf-8')
            suite = ET.Element('testsuite', name='gate-preflight-or-cleanup', tests='1', failures='1')
            case = ET.SubElement(suite, 'testcase', name='gate')
            ET.SubElement(case, 'failure', message='gate failed').text = detail
            ET.ElementTree(suite).write(output / 'gate-failure.xml', encoding='utf-8', xml_declaration=True)
            for name, record in [('skipped.json', []), ('doctor.json', {'ok': False, 'errors': [detail]}),
                                 ('environment.json', {'python': sys.version, 'executable': sys.executable})]:
                if not (output / name).exists():
                    (output / name).write_text(json.dumps(record, indent=2), encoding='utf-8')
        print(detail, file=sys.stderr)
        raise SystemExit(1)
