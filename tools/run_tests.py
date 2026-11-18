"""Run host tests: fast by default, --full for Hoppino, --suite to select an area."""
import argparse
import ast
import json
import os
import shlex
from pathlib import Path
import subprocess
import sys
import time

from host_build import HostBuild

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ['src/configuration/' + name + '.cpp' for name in
          ('RecoveryConfiguration', 'RecoveryRuleEngine', 'ConfigurationService')]
CONFIG += ['src/storage/ConfigurationStore.cpp']
NAVIGATION = ['src/navigation/' + name + '.cpp' for name in
              ('AttitudeEstimator', 'VerticalKalman', 'FlightDetector', 'Navigation')]
SENSORS = [*NAVIGATION, 'src/navigation/SensorPipeline.cpp',
           'src/hardware/HardwareSensorSource.cpp', 'src/flight/ArmingCalibration.cpp',
           'src/flight/ImuCalibration.cpp', 'src/storage/ImuCalibrationStore.cpp']
STORAGE = ['src/storage/' + name + '.cpp' for name in
           ('FlightDirectory', 'FlightHeaderStore', 'FlightSampleStore', 'FlightSummaryStore',
            'FlightRecorder', 'QueuedRecorder')]
RUNTIME = [*CONFIG, *SENSORS, *STORAGE, 'src/control/AirbrakeLaw.cpp',
           *('src/flight/' + name + '.cpp' for name in ('RunController', 'FlightRuntime', 'ArmingController', 'OutputExecutor')),
           *('src/communication/' + name + '.cpp' for name in ('CommandService', 'CommandRouter', 'ArmingCommands')),
           *('src/simulation/' + name + '.cpp' for name in
             ('EstimateReplayCommands', 'SensorHilCommands', 'HilSession', 'SimulatedSensorSource',
              'SimulationSession', 'SimulationCommands')),
           'src/configuration/ConfigurationCommands.cpp', 'src/storage/FlightCommands.cpp']
CPP = {
    'bluetooth': ['src/hardware/BluetoothName.cpp', 'src/hardware/Jdy34Link.cpp'],
    'configuration': [*CONFIG, 'src/flight/RunController.cpp'],
    'navigation': SENSORS,
    'flight': RUNTIME,
    'storage_format': [*CONFIG, *STORAGE],
    'storage_metadata': [*CONFIG, *STORAGE],
    'storage_recording': [*CONFIG, *STORAGE],
    'storage_recovery': [*CONFIG, *STORAGE],
    'commands': RUNTIME,
    'simulation': RUNTIME,
}
PYTHON = {
    'clients': [],
    'replay': ['hoppino_replay'],
    'recorded_replay': ['command_device', 'hoppino_replay', 'navigation_sensor_replay', 'onboard_simulation'],
    'recording': ['command_device'],
    'desktop': ['command_device'],
}
FIXTURES = {
    'command_device': [*RUNTIME, 'test/fixtures/command_device.cpp'],
    'hoppino_replay': ['src/configuration/RecoveryConfiguration.cpp',
                       'src/configuration/RecoveryRuleEngine.cpp', 'tools/replay/hoppino_replay.cpp'],
    'navigation_sensor_replay': [*NAVIGATION, 'tools/replay/navigation_sensor_replay.cpp'],
    'onboard_simulation': [*NAVIGATION,'src/navigation/SensorPipeline.cpp','src/simulation/SimulatedSensorSource.cpp','test/fixtures/onboard_simulation.cpp'],
}
RECORDED = ('recorded_replay', 'recording')
DESKTOP_TEST = 'test/integration/desktop_test.py'
DESKTOP_MAP = ROOT / '.pio/tests/desktop_map.json' # Written by desktop runs; read by --changed.
RUNNER_FILES = {'tools/run_tests.py', 'tools/host_build.py', 'tools/test_watchdog.py'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', nargs='+', choices=[*CPP, *PYTHON], help='Run only these areas')
    parser.add_argument('--full', action='store_true', help='Include recorded Hoppino integrations')
    parser.add_argument('--desktop', action='store_true', help='Also run the optional Qt desktop area')
    parser.add_argument('--area', nargs='+', metavar='AREA', help='Run only these desktop areas (implies desktop; see test/integration/desktop_test.py --list)')
    parser.add_argument('--timeout', type=float, default=120, help='Seconds before a stuck suite is stopped; Python suites print the stack of every thread (default 120)')
    parser.add_argument('--changed', nargs='?', const='HEAD', metavar='REF', help='Run only suites/areas affected by files changed since REF (default HEAD: uncommitted and new files)')
    parser.add_argument('--list', action='store_true', help='List available areas without building')
    parser.add_argument('--quick', nargs='?', const='HEAD', metavar='REF', help='Affected host tests only; defer desktop and recorded-flight integrations to the final check')
    parser.add_argument('--plan', action='store_true', help='Show selected tests without compiling or running them')
    parser.add_argument('--compiler', default='g++')
    parser.add_argument('--jobs', type=int, default=min(4, os.cpu_count() or 1), help='Parallel compiler processes (default max 4)')
    parser.add_argument('--rebuild', action='store_true', help='Ignore the C++ build cache')
    parser.add_argument('--verbose', action='store_true', help='Show individual scenario output')
    parser.add_argument('--skip-recorded-data', action='store_true', help='Exclude recorded integrations, including with --full')
    args = parser.parse_args()
    if args.quick is not None:
        if args.changed or args.suite or args.area or args.desktop or args.full:
            parser.error('--quick selects affected host tests; do not combine with other selection options')
        args.changed=args.quick
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    if args.list:
        for name in (*CPP, *PYTHON):
            print(name + ('  (full: requires Hoppino CSV)' if name in RECORDED else '  (optional: Qt desktop environment)' if name=='desktop' else ''))
        return 0
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    if args.changed and (args.suite or args.area):
        parser.error('--changed chooses suites itself; do not combine it with --suite or --area')
    desktop_python=ROOT/'.venv/Scripts/python.exe'
    default = [] if args.area else [*CPP, *(n for n in PYTHON if n!='desktop')] # --area alone runs only those areas.
    if args.changed and args.quick is None and desktop_python.exists(): default.append('desktop')
    selected = list(dict.fromkeys(args.suite or default))
    if (args.desktop or args.area) and 'desktop' not in selected: selected.append('desktop')
    if (not args.full and not args.suite) or args.skip_recorded_data:
        selected = [name for name in selected if name not in RECORDED]
    areas = args.area or list(desktop_areas())
    if args.quick is not None:
        print('Quick check: desktop and recorded-flight integrations are deferred; finish with --changed.')
    if args.changed:
        selected, areas = select_changed(args.changed, selected, areas)
        if not selected:
            print('No host tests are affected by the changed files.')
            return 0
    elif not selected:
        parser.error('No suites remain after excluding recorded-data tests')
    if 'desktop' in selected:
        unknown = [area for area in areas if area not in desktop_areas()]
        if unknown:
            parser.error('Unknown desktop areas: ' + ', '.join(unknown) + '. Available: ' + ', '.join(desktop_areas()))
    if args.plan:
        names=[item for name in selected for item in (
            [f'desktop/{area}' for area in areas] if name=='desktop' else [name])]
        print(f'Plan ({len(names)} runs): ' + ', '.join(names))
        return 0
    start = time.perf_counter()
    if 'desktop' in selected and not desktop_python.exists():
        parser.error('Create .venv and install perpetual_app/requirements.txt first (see README.md)')
    builder = HostBuild(args.compiler, args.jobs, args.rebuild)
    targets = {}
    for name in selected:
        if name in CPP:
            targets[name] = list(dict.fromkeys([*CPP[name], f'test/host/{name}_test.cpp']))
        else:
            for fixture in PYTHON[name]:
                targets[fixture] = FIXTURES[fixture]
    build_start = time.perf_counter()
    executables = builder.build_many(targets)
    print(f'Build: {builder.compiled} compiled, {builder.linked} linked in {time.perf_counter()-build_start:.2f}s', flush=True)
    logs = builder.directory / 'logs'
    logs.mkdir(exist_ok=True)
    runs = []
    for name in selected:
        if name in CPP:
            runs.append((name, [str(executables[name])]))
        elif name == 'desktop':
            # One process per area: a fresh app and firmware fixture, and a failure names its area.
            runs += [(f'desktop/{area}', watched(desktop_python, args.timeout, DESKTOP_TEST, area)) for area in areas]
        else:
            runs.append((name, watched(sys.executable, args.timeout, f'test/integration/{name}_test.py')))
    used_dir = builder.directory / 'used'
    used_dir.mkdir(exist_ok=True)
    for name, command in runs:
        began = time.perf_counter()
        area = name.split('/', 1)[1] if name.startswith('desktop/') else None
        used = used_dir / f'{area}.json' if area else None
        output, status = run_with_timeout(command, args.timeout, used)
        if used and status == 'PASS': remember_used_files(area, used)
        log = logs / (name + '.log')
        log.parent.mkdir(exist_ok=True)
        log.write_text(output, encoding='utf-8')
        print(f'{status} {name}: {time.perf_counter()-began:.2f}s', flush=True)
        if args.verbose or status != 'PASS':
            print(output, end='' if output.endswith('\n') else '\n')
        if status != 'PASS':
            if status == 'TIMEOUT':
                print(f'Stopped after {args.timeout:g}s: the stack above shows where it was stuck (raise the limit with --timeout).')
            print(f'Log: {log}')
            return 1
    print(f'{len(runs)} runs passed in {time.perf_counter()-start:.2f}s. Logs: {logs}', flush=True)
    return 0


def watched(python, timeout, script, *arguments):
    """Python command whose stacks are printed if it is still running after the timeout."""
    return [str(python), '-u', 'tools/test_watchdog.py', f'{timeout:g}', script, *arguments]


def desktop_areas():
    """Desktop area -> test module, read from desktop_test.py without importing Qt."""
    tree = ast.parse((ROOT / DESKTOP_TEST).read_text(encoding='utf-8'))
    table = next(node.value for node in tree.body if isinstance(node, ast.Assign) and node.targets[0].id == 'AREAS')
    return {area: target.split(':')[0] for area, target in ast.literal_eval(table).items()}


def remember_used_files(area, used):
    """Keep, per desktop area, the project files whose code it ran (used by --changed)."""
    try: mapping = json.loads(DESKTOP_MAP.read_text(encoding='utf-8'))
    except (OSError, ValueError): mapping = {}
    try: mapping[area] = json.loads(used.read_text(encoding='utf-8'))
    except (OSError, ValueError): mapping.pop(area, None) # Unknown usage: --changed will run it.
    DESKTOP_MAP.write_text(json.dumps(mapping, indent=1, sort_keys=True), encoding='utf-8')


def changed_files(ref):
    """Files changed since REF (working tree included), plus new untracked files."""
    names = set()
    for command in (['git', 'diff', '--name-only', ref], ['git', 'ls-files', '--others', '--exclude-standard']):
        names.update(subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=True).stdout.splitlines())
    return names


def cpp_inputs(sources):
    """Sources plus every header the last build recorded for them (g++ -MMD); None if never built."""
    inputs = set()
    for source in sources:
        inputs.add(source)
        try:
            rule = (ROOT / '.pio/tests/obj' / (source + '.d')).read_text().replace('\\\n', '').splitlines()[0]
        except (OSError, IndexError):
            return None
        inputs.update(shlex.split(rule.split(': ', 1)[1]))
    return inputs


def select_changed(ref, selected, areas):
    """Keep only suites/areas whose inputs changed. Anything unknown runs: a skip must be proven safe."""
    changed = changed_files(ref)
    print(f'Changed since {ref}: {len(changed)} files')
    if changed & RUNNER_FILES:
        print('Test runner changed: running everything selected.')
        return selected, areas
    covered = set()
    def touches(inputs):
        if inputs is None: return True
        covered.update(changed & inputs); return bool(changed & inputs)
    python_files = {name for name in changed if name.startswith(('tools/', 'test/fixtures/', 'test/integration/'))
                    and not name.startswith('test/integration/desktop_')}
    keep = []
    for name in selected:
        if name in CPP:
            hit = touches(cpp_inputs([*CPP[name], f'test/host/{name}_test.cpp']))
        elif name == 'desktop':
            continue # Decided per area below.
        else: # Python suites are few and fast: any tool/test-script change runs them all.
            hit = any([touches(cpp_inputs(FIXTURES[fixture])) for fixture in PYTHON[name]]) or bool(python_files)
            covered.update(python_files)
        if hit: keep.append(name)
    kept_areas = []
    if 'desktop' in selected:
        modules = desktop_areas()
        try: used = json.loads(DESKTOP_MAP.read_text(encoding='utf-8'))
        except (OSError, ValueError): used = {}
        everything = touches(cpp_inputs(FIXTURES['command_device'])) or bool(changed & {DESKTOP_TEST, 'test/integration/desktop_support.py'})
        app_files = {name for name in changed if name.startswith('perpetual_app/') and not name.endswith('.md')}
        known = set().union(*used.values()) if used else set()
        if app_files - known: # Assets, entry points or code no area has run yet.
            everything = True; covered.update(app_files)
        for area in areas:
            module = f'test/integration/{modules.get(area, "")}.py'
            files = used.get(area)
            if everything or files is None or module in changed or changed & set(files):
                kept_areas.append(area)
            covered.update(changed & ({module} | set(files or ())))
        if everything: covered.update({DESKTOP_TEST, 'test/integration/desktop_support.py'} & changed)
        if kept_areas: keep.append('desktop')
    uncovered = sorted(name for name in changed - covered if not name.endswith('.md'))
    if uncovered:
        print('Not exercised by this selection (deferred or requiring board/manual checks):\n  ' + '\n  '.join(uncovered))
    print(f'Selected: {", ".join(n for n in keep if n != "desktop") or "no suites"}'
          + (f'; desktop areas: {", ".join(kept_areas)}' if kept_areas else ''), flush=True)
    return keep, kept_areas


def run_with_timeout(command, timeout, used_files=None):
    """Return (output, PASS/FAIL/TIMEOUT). The watchdog exits Python itself; kill only as a last resort."""
    environment = dict(os.environ, PYTHONIOENCODING='utf-8')
    if used_files: environment['PERPETUAL_USED_FILES'] = str(used_files)
    process = subprocess.Popen(command, cwd=ROOT, text=True, encoding='utf-8', errors='replace',
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=environment)
    began = time.monotonic()
    try:
        output, _ = process.communicate(timeout=timeout + 15)
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()
        return output + f'\nKilled after {timeout + 15:g}s without output from the watchdog.\n', 'TIMEOUT'
    if process.returncode == 0:
        return output, 'PASS'
    timed_out = time.monotonic() - began >= timeout and 'Timeout (' in output
    return output, 'TIMEOUT' if timed_out else 'FAIL'


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as error:
        raise SystemExit(str(error)) from error
