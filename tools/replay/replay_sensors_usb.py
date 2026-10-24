"""HIL sensor source -> shared processing pipeline -> virtual outputs on RP2350."""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]


def sibling(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client as transport
navigation_replay = sibling('replay_navigation')


def compact_float(value):
    """Shortest tested representation preserving the exact input float32."""
    if not math.isfinite(value):
        return 'nan' if math.isnan(value) else ('inf' if value > 0 else '-inf')
    bits = struct.pack('<f', value)  # Reject overflow before connecting.
    for digits in range(1, 10):
        text = format(value, f'.{digits}g')
        try:
            if struct.pack('<f', float(text)) == bits:
                return text
        except OverflowError:
            continue
    raise ValueError('Cannot encode float32')


def prepare(input_path, pressure_unit='hpa', include_temperature=False):
    rows, samples, reference, hz = navigation_replay.load_sensor_samples(input_path, pressure_unit)
    origin = samples[0][0]
    commands = []
    for row, (stamp, values) in zip(rows, samples):
        if include_temperature:
            values = [*values, float(row.get('temp') or 'nan')]
        command = f'hil sample {stamp-origin} ' + ' '.join(compact_float(value) for value in values)
        if len(('@4294967295 ' + command).encode('ascii')) > 128:
            raise ValueError('A sensor sample exceeds the 128-byte command limit; no samples sent')
        commands.append(command)
    return rows, samples, reference, hz, commands


class ReplayPacer:
    """Absolute recorded-time deadlines; host delays never shift later deadlines."""
    def __init__(self, hz, clock=None, sleep=None):
        self.clock, self.sleep = clock or time.perf_counter, sleep or time.sleep
        self.period_us = int(1000000 / hz)
        self.start = self.clock()
        self.count = self.late_periods = self.max_late_us = 0
        self.total_round_trip_us = self.max_round_trip_us = 0

    def wait(self, time_ms):
        deadline = self.start + time_ms/1000
        now = self.clock()
        while now < deadline:
            self.sleep(min(deadline-now, 0.05))
            now = self.clock()
        late = max(0, round((now-deadline)*1000000))
        self.count += 1
        self.max_late_us = max(self.max_late_us, late)
        self.late_periods += late > self.period_us
        return now, late

    def replied(self, sent):
        elapsed = max(0, round((self.clock()-sent)*1000000))
        self.total_round_trip_us += elapsed
        self.max_round_trip_us = max(self.max_round_trip_us, elapsed)
        return elapsed

    def report(self, duration_ms):
        return dict(source='PC clock over USB', samples=self.count, nominal_period_us=self.period_us,
                    recorded_duration_ms=duration_ms, elapsed_ms=(self.clock()-self.start)*1000,
                    sends_late_by_period=self.late_periods, max_send_late_us=self.max_late_us,
                    mean_round_trip_us=self.total_round_trip_us/self.count if self.count else 0,
                    max_round_trip_us=self.max_round_trip_us)


def wait_for_queue(client, capacity):
    # Development USB flow control only. Timed replay deliberately omits it:
    # throttling a periodic producer would hide a slow worker/overflow.
    while True:
        status = client.request('hil status')
        if status.get('recording_error') != 'None':
            raise RuntimeError(f'Recording worker failed: {status.get("recording_error")}')
        if status.get('pending_samples', 0) <= capacity//4:
            return
        time.sleep(0.001)


def wait_for_recording(client, status, timeout=60, clock=time.monotonic, sleep=time.sleep):
    """Wait on the PC; a stopped sensor run may still have a closing flash worker."""
    deadline = clock() + timeout
    sequence = status['recording_sequence']
    while status.get('recording_busy', False):
        if clock() >= deadline:
            raise TimeoutError('Recording worker has not released flash; do not start another storage operation')
        sleep(0.005)
        progress = client.request('hil status')
        if progress.get('recording_sequence') != sequence:
            raise RuntimeError('Recording changed while waiting for close')
        status.update(progress)
    return status


def save_timing(client, output, sequence, pacing=None):
    report = {'sequence': sequence, 'hil': client.request('hil timing'), 'flash': client.request('flights timing')}
    if pacing is not None:
        report['pacing'] = pacing
    report_path = Path(output).with_suffix('.timing.json')
    with report_path.open('x', encoding='utf-8') as destination:
        json.dump(report, destination, indent=2, allow_nan=False)
        destination.write('\n')
    hil, flash = report['hil'], report['flash']
    if hil['timing_valid'] and flash['timing_valid']:
        print(f'Recording preparation: {hil["prepare_us"]/1000:.3f} ms; close submission: {hil["finish_us"]/1000:.3f} ms.')
        for field, label in (('compute', 'Navigation/rules'), ('storage', 'Storage submission'), ('step', 'Compute + storage submission')):
            count, total, peak = hil[field]
            print(f'{label}: max {peak} us; mean {total/count if count else 0:.1f} us ({count} calls).')
        print(f'Steps over nominal {hil["nominal_period_us"]} us period: {hil["over_budget"]}/{hil["step"][0]} '
              '(USB, parsing and formatting excluded).')
        for index, field in enumerate(('read', 'program', 'erase')):
            count, total, peak = flash[field]
            print(f'Flash {field}: {count} calls; max {peak} us; mean {total/count if count else 0:.1f} us; '
                  f'{flash["failures"][index]} failures.')
        if 'worker' in flash:
            count, total, peak = flash['worker']
            print(f'Storage worker: max {peak} us; mean {total/count if count else 0:.1f} us; '
                  f'queue peak {flash["queue_peak"]}/{flash["queue_capacity"]}.')
    else:
        print('Board timing unavailable: no microsecond clock; operation counts only.')
    if pacing is not None:
        print(f'PC-paced replay: {pacing["elapsed_ms"]/1000:.3f} s for {pacing["recorded_duration_ms"]/1000:.3f} s recorded; '
              f'{pacing["sends_late_by_period"]} sends late by more than one period; '
              f'max send lateness {pacing["max_send_late_us"]} us; max USB round trip {pacing["max_round_trip_us"]} us.')
    print(f'Timing report: {report_path}')
    return report


def run(client, prepared, output, record_bytes=0, measure_timing=False, realtime=False):
    rows, samples, reference, hz, commands = prepared
    measure_timing = measure_timing or realtime
    if measure_timing and Path(output).with_suffix('.timing.json').exists():
        raise FileExistsError(Path(output).with_suffix('.timing.json'))
    if record_bytes:
        print('Mount/recover recording storage; existing flights retained.', flush=True)
        recovered = client.request('flights recover', timeout=600)
        if recovered.get('corrupt_flights', 0):
            print(f'Warning: {recovered["corrupt_flights"]} corrupt flight(s) retained; their space remains reserved.', flush=True)
    view = client.request('config show')
    if not view.get('available') or view.get('draft_active'):
        raise RuntimeError('Apply a valid plan and close any draft before HIL')
    names = {rule: client.request(f'rule show {rule}')['name'] for rule in view['rule_ids']}
    if measure_timing:
        client.request('flights timing reset')
    fields = ['t_ms', 'source_t_ms', 'phase', 'imu_valid', 'baro_updated', 'z_est', 'vz_est',
              'ba_est', 'az_est', 'compute_us', 'triggered', 'starts', 'mode',
              'recorded_z_est', 'recorded_vz_est', 'recorded_legacy_state']
    if realtime:
        fields.extend(('send_late_us', 'round_trip_us'))
    labels = {0: 'PAD', 1: 'BOOST', 2: 'COAST', 3: 'DESCENT', 4: 'LANDED'}
    started = False
    with Path(output).open('x', newline='', encoding='utf-8') as destination:
        writer = csv.DictWriter(destination, fieldnames=fields)
        writer.writeheader()
        try:
            begin_command = f'hil begin {reference:.9g} {hz:.9g} logged_body'
            if record_bytes:
                print(f'Preparing {record_bytes} sample bytes on flash; this can take a while.', flush=True)
                begin = client.request(begin_command + f' record {record_bytes}', timeout=600)
            else:
                begin = client.request(begin_command)
            if begin.get('output_mode') != 'VIRTUAL' or begin.get('frame') != 'logged_body':
                raise RuntimeError('Unexpected HIL mode')
            started = True
            if record_bytes:
                print(f'Recording sequence: {begin["recording_sequence"]}', flush=True)
            print('HIL sensor source: logged BODY IMU + pressure -> onboard navigation/rules; VIRTUAL only.')
            queued = record_bytes and begin.get('recording_async', False)
            capacity = begin.get('queue_capacity', 0)
            if queued and capacity < 4:
                raise RuntimeError('Unexpected recording queue capacity')
            if queued:
                print(f'Storage worker enabled; {capacity}-sample queue; {"PC-paced input" if realtime else "USB flow control"}.')
            pacer = ReplayPacer(hz) if realtime else None
            previous = None
            starts = invalid = 0
            for index, (command, old) in enumerate(zip(commands, rows)):
                expected_time = samples[index][0] - samples[0][0]
                if pacer:
                    sent, late = pacer.wait(expected_time)
                elif queued and index and index % max(1, capacity//4) == 0:
                    wait_for_queue(client, capacity)
                response = client.request(command)
                if pacer:
                    round_trip = pacer.replied(sent)
                if response.get('output_mode') != 'VIRTUAL' or response.get('t_ms') != expected_time:
                    raise RuntimeError('Unexpected output mode or sample time')
                phase = response['phase']
                if phase != previous:
                    print(f'{expected_time} ms: computed {labels[phase]}')
                    previous = phase
                for rule in response['triggered']:
                    if rule not in names:
                        raise RuntimeError('Unknown triggered rule')
                    print(f'{expected_time} ms: rule {rule} triggered ({names[rule]})')
                for rule, kind, channel, stamp in response['starts']:
                    if rule not in names or stamp != expected_time:
                        raise RuntimeError('Invalid output start association/time')
                    starts += 1
                    print(f'{stamp} ms: VIRTUAL {kind} {channel}')
                invalid += not response['imu_valid']
                result = {key: response[key] for key in fields[:1] + fields[2:10]}
                result.update(source_t_ms=old['t_ms'], triggered=json.dumps(response['triggered']),
                              starts=json.dumps(response['starts']), mode='VIRTUAL',
                              recorded_z_est=old.get('z_est', ''), recorded_vz_est=old.get('vz_est', ''),
                              recorded_legacy_state=old.get('state', ''))
                if pacer:
                    result.update(send_late_us=late, round_trip_us=round_trip)
                writer.writerow(result)
                if (index + 1) % 1000 == 0:
                    print(f'{index + 1}/{len(samples)} sensor samples')
            pacing = pacer.report(samples[-1][0]-samples[0][0]) if pacer else None
            final = client.request('hil stop completed', timeout=60) if record_bytes else client.request('hil stop completed')
            started = False
            if record_bytes:
                final = wait_for_recording(client, final)
            print(f'{len(samples)} samples, {starts} virtual starts, {invalid} rejected navigation frames; '
                  f'{final["cancelled_pending"]} pending starts cancelled at EOF.')
            if final['timing_valid']:
                print(f'Max sensor processing + navigation + rule evaluation: {final["max_compute_us"]} us '
                      '(excludes flash writes, USB, parsing and reply formatting).')
            if record_bytes:
                print(f'Flash sequence {final["recording_sequence"]}: {final["saved_samples"]} saved, '
                      f'{final["dropped_samples"]} dropped; storage {final["recording_error"]}.')
            if measure_timing:
                save_timing(client, output, final['recording_sequence'], pacing)
            if record_bytes:
                if final['recording_error'] != 'None':
                    raise RuntimeError('Recording failed; HIL finished, recover storage before downloading the durable prefix')
            return final
        finally:
            if started:
                try:
                    stopped = client.request('hil stop', timeout=60) if record_bytes else client.request('hil stop')
                    if record_bytes:
                        wait_for_recording(client, stopped)
                except Exception as error:
                    print(f'Stop not acknowledged: {error}; close the connection to terminate HIL.', file=sys.stderr)
                print(f'HIL incomplete; partial CSV retained at {output}', file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--input', type=Path, default=navigation_replay.DEFAULT_INPUT)
    parser.add_argument('--pressure-unit', choices=['hpa', 'pa'], default='hpa')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--record', action='store_true', help='Record HIL to board flash; requires flights init once on blank storage')
    parser.add_argument('--record-bytes', type=int, default=1048576, help='Reserved sample capacity, multiple of 4096; used with --record')
    parser.add_argument('--timing', action='store_true', help='Reset flash timing before the run and save board timing statistics beside the HIL CSV')
    parser.add_argument('--realtime', action='store_true', help='Pace sensor input by recorded timestamps using the PC clock; enables --timing')
    args = parser.parse_args()
    if args.record and (args.record_bytes <= 0 or args.record_bytes % 4096):
        raise ValueError('--record-bytes must be a positive multiple of 4096')
    prepared = prepare(args.input, args.pressure_unit, include_temperature=args.record)
    output = args.output or ROOT / '.pio' / f'hoppino_hil_{uuid.uuid4().hex}.csv'
    if output.exists():
        raise FileExistsError(output)
    if (args.timing or args.realtime) and output.with_suffix('.timing.json').exists():
        raise FileExistsError(output.with_suffix('.timing.json'))
    output.parent.mkdir(parents=True, exist_ok=True)
    with transport.DeviceConnection(args.port) as client:
        run(client, prepared, output, args.record_bytes if args.record else 0, measure_timing=args.timing, realtime=args.realtime)
    print(f'HIL CSV: {output}')


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'HIL failed: {error}', file=sys.stderr)
        sys.exit(1)
