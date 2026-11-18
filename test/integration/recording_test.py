"""Recording regression scenarios; run through tools/run_tests.py."""
import contextlib
import base64
import zlib
import csv
import io
import json
import math
from pathlib import Path
import subprocess
import tempfile

import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'test/fixtures'))
from client_fixture import ProcessLink, load_tool

def check_recording_usb():


    replay = load_tool('tools/replay/replay_sensors_usb.py')
    export = load_tool('tools/replay/download_flight.py')
    prepared = replay.prepare(replay.navigation_replay.DEFAULT_INPUT, include_temperature=True)
    with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
        navigation_reference = replay.navigation_replay.replay(replay.navigation_replay.DEFAULT_INPUT, Path(directory) / 'reference.csv')


    class FakeClock:
        def __init__(self):
            self.value = 100.0

        def clock(self):
            return self.value

        def sleep(self, seconds):
            self.value += seconds


    class CheckingClient:
        def __init__(self, client, queued):
            self.client, self.queued = client, queued

        def request(self, command, **kwargs):
            result = self.client.request(command, **kwargs)
            if command.startswith('hil begin'):
                status = self.client.request('flights status')
                assert status['ready'] and status['active'] and status['directory_error'] == 'None'
                for blocked in ('config begin', 'flights info 1', 'flights timing reset'):
                    try:
                        self.client.request(blocked)
                        raise AssertionError(f'Active recording accepted {blocked}')
                    except RuntimeError as error:
                        assert 'BUSY' in str(error)
                if self.queued:
                    try:
                        self.client.request('flights timing')
                        raise AssertionError('Read live worker buffers without ownership')
                    except RuntimeError as error:
                        assert 'BUSY' in str(error)
            return result


    def check(queued):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            arguments = [str(ROOT / '.pio/tests/bin/command_device.exe')]
            if queued:
                arguments.append('--queued')
            process = subprocess.Popen(arguments,
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            try:
                client = CheckingClient(replay.transport.Client(ProcessLink(process)), queued)
                client.request('flights init')
                client.request('config begin')
                waits=[(10,'pyro',1,75),(10,'servo',1,200),(20,'pyro',2,325)]
                for rule,kind,channel,delay in waits:
                    client.request(f'output unassign {rule} {kind} {channel}')
                    client.request(f'output assign {rule} {kind} {channel} {delay}')
                for command in ('servo set 3 1100 1900','airbrake channels 4','airbrake add phase 68','airbrake add altitude > 20'):
                    client.request(command)
                client.request('config apply')
                client.request('config airbrakes on')
                pacer_class = replay.ReplayPacer
                fake = FakeClock()
                if queued:
                    replay.ReplayPacer = lambda hz: pacer_class(hz, clock=fake.clock, sleep=fake.sleep)
                try:
                    with contextlib.redirect_stdout(io.StringIO()):
                        final = replay.run(client, prepared, folder / 'live.csv', record_bytes=1048576, measure_timing=True, realtime=queued)
                finally:
                    replay.ReplayPacer = pacer_class
                assert final['saved_samples'] == 6600 and final['dropped_samples'] == 0 and final['recording_error'] == 'None', final
                assert final['recording_sequence'] == '1' and not final['recording_active']
                assert final['recording_async'] == queued and final['pending_samples'] == 0
                timings = json.loads((folder / 'live.timing.json').read_text())
                assert timings['sequence'] == '1' and not timings['hil']['timing_valid'] and not timings['flash']['timing_valid']
                assert timings['hil']['compute'] == timings['hil']['storage'] == timings['hil']['step'] == [6600, 0, 0]
                assert timings['hil']['over_budget'] == 0 and timings['flash']['failures'] == [0, 0, 0]
                assert timings['flash']['program'][0] > 328*4 and timings['flash']['erase'][0] == 257
                if queued:
                    assert 0 < timings['flash']['queue_peak'] <= 64 and timings['flash']['queue_capacity'] == 64
                    assert timings['flash']['worker'][0] >= 6600
                    assert timings['pacing']['samples'] == 6600 and timings['pacing']['sends_late_by_period'] == 0
                    assert abs(timings['pacing']['elapsed_ms']-timings['pacing']['recorded_duration_ms']) < 1e-5
                # Applied settings may change later; downloaded history must retain its frozen snapshot.
                for command in ('config begin', 'rule rename 10 "Changed after flight"', 'config apply'):
                    client.request(command)
                with contextlib.redirect_stdout(io.StringIO()):
                    metadata = export.download(client, '1', folder / 'stored.csv')
                # Packed transport preserves the old diagnostic read exactly (float32
                # versus JSON decimals), and fits the existing serial line limit.
                packed=client.request('flights packed 1 0 0')
                decoded=export.unpack_samples(packed)
                assert 5<=packed['count']<=9 and len(json.dumps(packed,separators=(',',':')))<1024
                legacy=[]
                for offset in range(0,9,2): legacy.extend(client.request(f'flights read 1 0 {offset}')['rows'])
                for a,b in zip(decoded,legacy):
                    assert len(a)==len(b)==25
                    for x,y in zip(a,b):
                        assert x is y or (x is not None and y is not None and math.isclose(x,y,rel_tol=1e-8,abs_tol=1e-8))
                for damaged in (dict(packed,crc32=packed['crc32']^1),dict(packed,data='!'),dict(packed,count=12)):
                    try: export.unpack_samples(damaged); raise AssertionError('Corrupt packed data accepted')
                    except RuntimeError: pass
                raw=base64.b64decode(packed['data'])
                for payload in (raw[:-1],raw+b'\0'):
                    damaged=dict(packed,data=base64.b64encode(payload).decode(),crc32=zlib.crc32(payload))
                    try: export.unpack_samples(damaged); raise AssertionError('Malformed packed data accepted')
                    except RuntimeError: pass
                assert metadata['configuration']['rules'][0]['name'] == 'Example apogee'
                timestamps=[stamp-prepared[1][0][0] for stamp,_ in prepared[1]]
                expected=[next(t for t in timestamps if t>=start+delay) for start,delay in zip((12553,12553,18500),(75,200,325))]
                assert [s['t_ms'] for s in metadata['output_starts']] == expected
                assert [a['delay_ms'] for r in metadata['configuration']['rules'] for a in r['outputs']]==[75,200,325]
                assert all(s['state'] == 'recorded' for s in metadata['output_starts'])
                assert metadata['sensor_mode'] == 1 and metadata['output_mode'] == 1
                calibration=metadata['calibration']
                assert not calibration['gyro_calibrated'] and not calibration['sensor_axes']
                assert calibration['gyro_bias_radps'] is None and calibration['accel_bias_mps2'] is None and calibration['up_axis'] is None
                assert calibration['airbrake_law_version']==1
                air=metadata['configuration']['airbrakes']
                assert air['channel_mask']==4 and air['logic']=='and' and len(air['conditions'])==2
                assert math.isfinite(calibration['reference_altitude_m'])
                summary = metadata['recording_summary']
                assert summary['state'] == 'Valid' and summary['version'] == 1 and summary['reason'] == 'Completed'
                assert summary['saved_samples'] == 6600 and summary['dropped_samples'] == 0
                assert summary['queue_capacity'] == (64 if queued else 0)
                assert client.request('flights list 0')['sequence'] == '1'
                assert not client.request('flights list 1')['available']
                with (folder / 'live.csv').open(newline='') as source:
                    live = list(csv.DictReader(source))
                with (folder / 'stored.csv').open(newline='') as source:
                    stored = list(csv.DictReader(source))
                assert len(live) == len(stored) == 6600
                enabled=[bool(int(row['airbrake_flags'])&2) for row in stored]
                assert any(enabled) and not enabled[0] and not enabled[-1]
                assert any(a!=b for a,b in zip(enabled,enabled[1:]))
                for row,active in zip(stored,enabled):
                    assert int(row['airbrake_channel_mask'])==4 and float(row['airbrake_3_pct'])==0
                    assert all(math.isnan(float(row[f'airbrake_{i}_pct'])) for i in (1,2,4))
                    assert active==(row['state']=='2' and float(row['z_est'])>20)
                    if active: assert int(row['airbrake_failed_conditions'])==0
                assert all(float(row['battery_v'])==8.402 for row in stored)
                old_rows = prepared[0]
                for a, b, old, reference in zip(live, stored, old_rows, navigation_reference):
                    assert a['t_ms'] == b['t_ms'] and a['phase'] == b['state']
                    for field in ('z_est', 'vz_est', 'az_est'):
                        assert abs(float(a[field])-float(b[field])) < 1e-5
                    # Unique checks from the removed manual HIL-to-recorder bridge
                    # now exercise actual USB recording and the production downloader.
                    for field in ('accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ', 'pressure', 'temp'):
                        actual, expected = float(b[field]), float(old.get(field) or 'nan')
                        assert (math.isnan(actual) and math.isnan(expected)) or math.isclose(actual, expected, rel_tol=2e-6, abs_tol=1e-5), field
                    for field in ('z_est', 'vz_est', 'az_est'):
                        assert math.isclose(float(b[field]), float(reference[field]), rel_tol=2e-6, abs_tol=1e-5), field
                    for field in ('roll', 'pitch', 'yaw'):
                        assert math.isclose(float(b[field]), math.degrees(float(reference[field+'_rad'])), rel_tol=2e-6, abs_tol=2e-5), field
                assert json.loads((folder / 'stored.json').read_text()) == metadata
                try:
                    export.download(client, '1', folder / 'stored.csv')
                    raise AssertionError('Existing exports were overwritten')
                except FileExistsError:
                    pass
                assert export.sequence_text('18446744073709551615') == '18446744073709551615'
                for invalid in ('0', '-1', '18446744073709551616', '1 extra'):
                    try:
                        export.sequence_text(invalid)
                        raise AssertionError('Invalid selection accepted')
                    except ValueError:
                        pass
                if queued:
                    client.request('hil begin 0 200 logged_body record 4096')
                    client.request('hil sample 0 0 0 9.80665 0 0 0 101325')
                    process.stdin.write(b'__disconnect__\n')
                    process.stdin.flush()
                    assert json.loads(process.stdout.readline()) == {}
                    replay.wait_for_recording(client,client.request('hil status'))
                    info = client.request('flights info 2')
                    assert info['samples'] == 1 and info['status'] == 2
                    assert client.request('hil status')['pending_samples'] == 0
                    assert client.request('flights summary 2')['summary']['reason'] == 'Disconnected'
                    # Inject actual sample corruption through the host fixture only.
                    # Production USB flags the closed flight and still exports healthy history.
                    process.stdin.write(b'__damage_current__\n')
                    process.stdin.flush()
                    assert json.loads(process.stdout.readline()) == {}
                    try:
                        export.download(client,'2',folder/'damaged.csv')
                        raise AssertionError('Damaged flight exported as healthy')
                    except RuntimeError as error:
                        assert 'Corrupt' in str(error)
                    assert not (folder/'damaged.json').exists()
                    assert client.request('flights recover')['corrupt_flights'] == 1
                    entry = client.request('flights list 0')
                    assert entry['sequence'] == '2' and entry['corrupt'] and export.STATUS[entry['status']] == 'CORRUPT'
                    with contextlib.redirect_stdout(io.StringIO()):
                        export.download(client,'1',folder/'healthy_after_corrupt.csv')
                    assert (folder/'healthy_after_corrupt.csv').read_bytes() == (folder/'stored.csv').read_bytes()
                # Delete a previously cached/downloaded selection; no stale rows survive reuse.
                client.request('flights info 1')
                assert client.request('flights delete 1')['deleted']
                for command in ('flights info 1', 'flights summary 1', 'flights read 1 0 0'):
                    try:
                        client.request(command)
                        raise AssertionError('Deleted recording remained readable')
                    except RuntimeError as error:
                        assert 'StaleSelection' in str(error)
                client.request('hil begin 0 200 logged_body record 4096')
                client.request('hil sample 0 0 0 9.80665 0 0 0 101325')
                replay.wait_for_recording(client,client.request('hil stop'))
                latest = '3' if queued else '2'
                assert client.request(f'flights info {latest}')['samples'] == 1
                assert client.request(f'flights summary {latest}')['summary']['reason'] == 'Stopped'
                assert client.request('flights list 0')['sequence'] == latest
                # Deleting the newest flight and remounting must not reuse its identity.
                client.request(f'flights delete {latest}')
                client.request('flights recover')
                started=client.request('hil begin 0 200 logged_body record 4096')
                client.request('hil sample 0 0 0 9.80665 0 0 0 101325')
                replay.wait_for_recording(client,client.request('hil stop'))
                assert started['recording_sequence']==str(int(latest)+1)
                assert client.request('flights list 0')['sequence']==started['recording_sequence']
            finally:
                process.stdin.close()
                process.wait(timeout=10)
                process.stdout.close()
            assert process.returncode == 0

    for queued in (False, True):
        check(queued)
    print('Synchronous/queued USB recording/download passed: all 6600 samples, temperature, frozen configuration, three starts and paced-client path.')

if __name__ == "__main__":
    check_recording_usb()
