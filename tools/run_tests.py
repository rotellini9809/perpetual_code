"""Run host tests: fast by default, --full for Hoppino, --suite to select an area."""
import argparse
import os
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
           'src/hardware/HardwareSensorSource.cpp', 'src/flight/ArmingCalibration.cpp']
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', nargs='+', choices=[*CPP, *PYTHON], help='Run only these areas')
    parser.add_argument('--full', action='store_true', help='Include recorded Hoppino integrations')
    parser.add_argument('--desktop', action='store_true', help='Also run the optional Qt desktop area')
    parser.add_argument('--list', action='store_true', help='List available areas without building')
    parser.add_argument('--compiler', default='g++')
    parser.add_argument('--jobs', type=int, default=min(4, os.cpu_count() or 1), help='Parallel compiler processes (default max 4)')
    parser.add_argument('--rebuild', action='store_true', help='Ignore the C++ build cache')
    parser.add_argument('--verbose', action='store_true', help='Show individual scenario output')
    parser.add_argument('--skip-recorded-data', action='store_true', help='Exclude recorded integrations, including with --full')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    if args.list:
        for name in (*CPP, *PYTHON):
            print(name + ('  (full: requires Hoppino CSV)' if name in RECORDED else '  (optional: Qt desktop environment)' if name=='desktop' else ''))
        return 0
    selected = list(dict.fromkeys(args.suite or [*CPP, *(n for n in PYTHON if n!='desktop')]))
    if args.desktop and 'desktop' not in selected: selected.append('desktop')
    if (not args.full and not args.suite) or args.skip_recorded_data:
        selected = [name for name in selected if name not in RECORDED]
    if not selected:
        parser.error('No suites remain after excluding recorded-data tests')
    start = time.perf_counter()
    desktop_python=ROOT/'.pio/perpetual_app_venv/Scripts/python.exe'
    if 'desktop' in selected and not desktop_python.exists():
        parser.error('Build/install the desktop environment first: powershell -ExecutionPolicy Bypass -File perpetual_app/build.ps1')
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
    for name in selected:
        command = [str(executables[name])] if name in CPP else [str(desktop_python) if name=='desktop' else sys.executable, f'test/integration/{name}_test.py']
        began = time.perf_counter()
        result = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        log = logs / (name + '.log')
        log.write_text(result.stdout, encoding='utf-8')
        print(f'{"PASS" if result.returncode == 0 else "FAIL"} {name}: {time.perf_counter()-began:.2f}s', flush=True)
        if args.verbose or result.returncode:
            print(result.stdout, end='')
        if result.returncode:
            print(f'Log: {log}')
            return 1
    print(f'{len(selected)} suites passed in {time.perf_counter()-start:.2f}s. Logs: {logs}', flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as error:
        raise SystemExit(str(error)) from error
