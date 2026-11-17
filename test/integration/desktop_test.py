"""Optional Qt tests: actual firmware configuration/storage plus asynchronous views.

Each area is independent: it creates its own window and PC firmware fixture. tools/run_tests.py runs every
area in a separate process; run this file directly to run all areas here, or name some:
    .venv/Scripts/python.exe test/integration/desktop_test.py [--list] [AREA ...]
"""
import os
import sys
import time

# Kept free of Qt imports so --list stays instant.
AREAS = {
    'sigint': 'desktop_basics:check_sigint',
    'sensor_history': 'desktop_basics:check_sensor_history',
    'arm_button': 'desktop_arm_button:check_arm_button',
    'airbrakes': 'desktop_airbrakes:check_airbrakes',
    'setting_drafts': 'desktop_settings:check_setting_drafts',
    'imu': 'desktop_imu:check_imu',
    'connections': 'desktop_connections:check_connections',
    'config_transactions': 'desktop_basics:check_config_transactions',
    'dialogs': 'desktop_basics:check_dialogs',
    'bistage_settings': 'desktop_dashboard:check_bistage_settings',
    'rule_selection': 'desktop_dashboard:check_rule_selection',
    'orientation_and_pyros': 'desktop_dashboard:check_orientation_and_pyros',
    'servo_preparation': 'desktop_dashboard:check_servo_preparation',
    'sensors_and_power': 'desktop_dashboard:check_sensors_and_power',
    'simulation_and_arming': 'desktop_simulation:check_simulation_and_arming',
    'config_roundtrip': 'desktop_simulation:check_config_roundtrip',
    'recorded_flights': 'desktop_flights:check_recorded_flights',
    'maintenance': 'desktop_maintenance:check_maintenance',
    'source_loss': 'desktop_maintenance:check_source_loss',
    'bluetooth_name': 'desktop_bluetooth:check_bluetooth_name',
    'translations': 'desktop_i18n:check_translations',
    'language_switch': 'desktop_i18n:check_language_switch',
}


def record_used_files(path):
    """Record which project files run code during this process (feeds run_tests.py --changed)."""
    import atexit, json
    from pathlib import Path
    root=Path(__file__).resolve().parents[2]; prefix=str(root).lower(); seen=set()
    monitor=sys.monitoring; tool=monitor.PROFILER_ID
    def started(code,offset):
        # CO_OPTIMIZED marks functions/lambdas: module and class bodies only run on import.
        if code.co_flags & 1: seen.add(code.co_filename)
        return monitor.DISABLE # Each function is reported once, so the tests keep their speed.
    monitor.use_tool_id(tool,'perpetual-used-files')
    monitor.register_callback(tool,monitor.events.PY_START,started)
    monitor.set_events(tool,monitor.events.PY_START) # Covers every thread.
    def save():
        monitor.set_events(tool,0); used=set()
        for name in seen:
            if not name.lower().startswith(prefix): continue
            relative=Path(name).resolve().relative_to(root).as_posix()
            if relative.startswith(('perpetual_app/','tools/','test/')): used.add(relative) # Not .venv libraries.
        Path(path).write_text(json.dumps(sorted(used)),encoding='utf-8')
    atexit.register(save)


def main(argv):
    if argv[:1]==['--list']:
        print('\n'.join(AREAS)); return 0
    if os.environ.get('PERPETUAL_USED_FILES'): record_used_files(os.environ['PERPETUAL_USED_FILES'])
    unknown=[name for name in argv if name not in AREAS]
    if unknown:
        print('Unknown desktop areas: '+', '.join(unknown)+'\nAvailable: '+', '.join(AREAS)); return 2
    import importlib, inspect
    from desktop_support import Native,start_app,make_wait,dispose
    app=start_app(); wait=make_wait(app)
    for name in argv or AREAS:
        module,function=AREAS[name].split(':')
        check=getattr(importlib.import_module(module),function)
        available=dict(app=app,wait=wait,native=Native) # Pass only what each check declares.
        began=time.perf_counter()
        check(**{parameter:available[parameter] for parameter in inspect.signature(check).parameters})
        print(f'desktop/{name} passed in {time.perf_counter()-began:.2f}s',flush=True)
    dispose(app)
    return 0


if __name__=='__main__': sys.exit(main(sys.argv[1:]))
