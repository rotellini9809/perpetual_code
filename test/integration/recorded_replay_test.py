"""Recorded replay regression scenarios; run through tools/run_tests.py."""
import contextlib
import csv
import io
from pathlib import Path
import tempfile
import json
import math
import subprocess

import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'test/fixtures'))
sys.path.insert(0, str(ROOT / 'tools'))
from client_fixture import ProcessLink, load_tool
from device_client import Client

def check_navigation_replay():
    module = load_tool('tools/replay/replay_navigation.py')
    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        with contextlib.redirect_stdout(io.StringIO()):
            actual = module.replay(module.DEFAULT_INPUT, folder / 'actual.csv')
        assert len(actual) == 6600 and all(row['imu_valid'] == '1' for row in actual)
        transitions = {}
        for row in actual:
            transitions.setdefault(int(row['phase']), int(row['t_ms']))
        assert transitions == {0: 0, 1: 8202, 2: 10194, 3: 12553, 4: 30223}, transitions
        assert sum(row['baro_updated'] == '1' for row in actual) == 1506
        subprocess.run([sys.executable,str(ROOT/'tools/replay/trim_hoppino.py'),'--check'],check=True,capture_output=True)
        # Compare physical rotations, not Euler components near pitch=-90 degrees.
        with contextlib.redirect_stdout(io.StringIO()):
            full=module.replay(ROOT/'datasets/hoppino_VOLO_UFFICIALE_original.csv',folder/'full.csv')
        original={int(r['source_t_us']):r for r in full}
        def quaternion(row):
            a,b,c=(float(row[k])/2 for k in ('roll_rad','pitch_rad','yaw_rad'))
            cr,sr,cp,sp,cy,sy=math.cos(a),math.sin(a),math.cos(b),math.sin(b),math.cos(c),math.sin(c)
            return (cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)
        differences=[]
        for row in actual:
            if row['phase']=='0': continue
            before=original[int(row['t_us'])+2003000] # The excerpt starts 2.003 s into the original log.
            assert before['phase']==row['phase']
            dot=abs(sum(a*b for a,b in zip(quaternion(row),quaternion(before))))
            differences.append(math.degrees(2*math.acos(min(1,dot))))
        assert max(differences)<.1, max(differences)
        assert int(actual[-1]['t_ms'])-transitions[4]>=2500
        print(f'Full-flight crop: launch through landing preserved; maximum attitude difference {max(differences):.4f} degrees.')
        subprocess.run([sys.executable,str(ROOT/'tools/replay/generate_hoppino_data.py'),'--check'],check=True,capture_output=True)
        embedded=subprocess.run([str(ROOT/'.pio/tests/bin/onboard_simulation.exe')],check=True,capture_output=True,text=True)
        onboard=list(csv.DictReader(io.StringIO(embedded.stdout)))
        assert len(onboard)==6600
        assert onboard==[{key:row[key] for key in onboard[0]} for row in actual], 'Embedded Hoppino differs from CSV sensor replay'
        print('Embedded Hoppino matches CSV navigation on all 6600 frames, including attitude and phase.')
        with module.DEFAULT_INPUT.open(newline='') as source:
            reader = csv.DictReader(source)
            fields, rows = reader.fieldnames, list(reader)
        # Change every recorded output/phase and switch explicit pressure units to Pa.
        altered = folder / 'altered.csv'
        with altered.open('w', newline='') as destination:
            writer = csv.DictWriter(destination, fieldnames=fields)
            writer.writeheader()
            for row in rows:
                for name in ('z_est', 'vz_est', 'roll', 'pitch', 'yaw', 'az_est'):
                    row[name] = '9999'
                row['state'] = '0'
                row['pressure'] = repr(float(row['pressure']) * 100)
                writer.writerow(row)
        with contextlib.redirect_stdout(io.StringIO()):
            independent = module.replay(altered, folder / 'independent.csv', pressure_unit='pa')
        columns = [name for name in actual[0] if not name.startswith('recorded_')]
        for first, second in zip(actual, independent):
            assert all(first[name] == second[name] for name in columns)
        print('Navigation sensor replay passed: 6600 samples; recorded estimates/phases cannot affect calculations; Pa/hPa equivalent.')
    return actual


def check_sensor_hil_usb(navigation_reference):
    module = load_tool('tools/replay/replay_sensors_usb.py')


    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        prepared = module.prepare(module.navigation_replay.DEFAULT_INPUT)
        output = folder / 'hil.csv'
        process = subprocess.Popen([str(ROOT / '.pio/tests/bin/command_device.exe')],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                final = module.run(module.transport.Client(ProcessLink(process)), prepared, output)
        finally:
            process.stdin.close()
            process.wait(timeout=10)
            process.stdout.close()
        assert process.returncode == 0 and final['samples'] == 6600
        reference = navigation_reference
        with output.open(newline='') as source:
            actual = list(csv.DictReader(source))
        assert len(actual) == len(reference)
        for hil, pc in zip(actual, reference):
            assert hil['phase'] == pc['phase'] and hil['t_ms'] == pc['t_ms']
            assert (hil['imu_valid'] == 'True') == (pc['imu_valid'] == '1')
            assert (hil['baro_updated'] == 'True') == (pc['baro_updated'] == '1')
            for name in ('z_est', 'vz_est', 'ba_est', 'az_est'):
                assert abs(float(hil[name]) - float(pc[name])) < 1e-5, (name, hil[name], pc[name])
        starts = [start for row in actual for start in json.loads(row['starts'])]
        assert starts == [[10, 'pyro', 1, 12553], [10, 'servo', 1, 12553], [20, 'pyro', 2, 18500]], starts
        assert final['cancelled_pending'] == 0
        print('Sensor HIL integration passed: all 6600 navigation results match PC replay; 3 virtual starts.')

def check_replay_hoppino_usb():
    module = load_tool('tools/replay/replay_hoppino_usb.py')


    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        for content in ['', 't_ms,state,z_est,vz_est\n',
                        't_ms,state,z_est,vz_est\n0,6,0,0\n',
                        't_ms,state,z_est,vz_est\n1,0,0,0\n1,1,1,1\n']:
            source = folder / 'invalid.csv'
            source.write_text(content)
            try:
                module.load_samples(source)
                raise AssertionError('Invalid input accepted')
            except ValueError:
                pass
        samples = module.load_samples(module.DEFAULT_INPUT)
        output = folder / 'usb_events.csv'
        process = subprocess.Popen([str(ROOT / '.pio/tests/bin/command_device.exe')],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                module.run(Client(ProcessLink(process)), samples, output)
        finally:
            process.stdin.close()
            process.wait(timeout=10)
            process.stdout.close()
        assert process.returncode == 0
        reference = folder / 'reference.csv'
        subprocess.run([str(ROOT / '.pio/tests/bin/hoppino_replay.exe'), str(module.DEFAULT_INPUT), str(reference)],
                       check=True, capture_output=True)
        with output.open(newline='') as source:
            actual = list(csv.DictReader(source))
        with reference.open(newline='') as source:
            expected = list(csv.DictReader(source))
        assert actual == expected, (actual, expected)
        assert len(samples) == 6600
        print(f'USB client / C++ replay matches PC reference: {len(samples)} samples, {len(actual)} events.')

if __name__ == "__main__":
    navigation_reference = check_navigation_replay()
    check_sensor_hil_usb(navigation_reference)
    check_replay_hoppino_usb()
