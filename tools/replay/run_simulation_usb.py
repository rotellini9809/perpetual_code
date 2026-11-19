"""Start the onboard Hoppino sensor replay; USB/SPP only controls and observes it.

--rate480 runs the stress profile: Hoppino interpolated at 480 Hz, 12 s longer PAD and a
half-speed descent (about 61 s, grows the recording), and also reports `diag`."""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client as transport


def run(client, output, record=False, clock=time.monotonic, sleep=time.sleep, rate480=False):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    if record:
        print('Mount/recover storage; existing recordings retained.', flush=True)
        recovered = client.request('flights recover', timeout=600)
        if recovered.get('corrupt_flights', 0):
            print(f'Warning: {recovered["corrupt_flights"]} corrupt flight(s) retained; their space remains reserved.', flush=True)
        client.request('flights timing reset')
    source = 'hoppino480' if rate480 else 'hoppino'
    print(f'Starting onboard {source} sensor replay; VIRTUAL outputs only.', flush=True)
    started = False
    try:
        command = 'sim begin' + (' record' if record else '') + (' 480' if rate480 else '')
        status = client.request(command, timeout=600 if record else 5)
        started = True
        if status.get('source')!=source: raise RuntimeError(f'Firmware does not provide onboard {source}; upload current MAIN')
        duration=status['duration_ms']; samples=status['total_samples']
        deadline = clock() + duration/1000 + 20
        previous = None
        while status['active'] or status.get('recording_busy', False):
            if clock() >= deadline:
                raise TimeoutError('Hoppino replay or recording worker did not finish within its duration plus 20 seconds')
            if status['phase'] != previous:
                print(f'{status["t_ms"]} ms: {("PAD", "BOOST", "COAST", "DESCENT", "LANDED")[status["phase"]]}', flush=True)
                previous = status['phase']
            sleep(0.5)
            status = client.request('sim status')
        started = False
        report = dict(status=status, timing=client.request('sim timing'), events=client.request('sim events'))
        if record:
            report['flash'] = client.request('flights timing')
        if rate480:
            report['diag'] = client.request('diag')
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as target:
            json.dump(report, target, indent=2, allow_nan=False)
            target.write('\n')
        print(f'Report: {output}')
        timing = report['timing']
        count, total, peak = timing['cycle']
        print(f'{status["samples"]} samples; {timing["missed_ticks"]} missed ticks; '
              f'max cycle {peak} us; mean {total/count if count else 0:.1f} us; '
              f'max lateness {timing["max_lateness_us"]} us.')
        for phase, stamp in report['events']['phases']:
            print(f'{stamp} ms: computed {("PAD", "BOOST", "COAST", "DESCENT", "LANDED")[phase]}')
        for rule, kind, channel, stamp in report['events']['starts']:
            print(f'{stamp} ms: VIRTUAL {kind} {channel} (rule {rule})')
        if record:
            print(f'Flash sequence {status["recording_sequence"]}: {status["saved_samples"]} saved, '
                  f'{status["dropped_samples"]} dropped; storage {status["recording_error"]}.')
            flash = report['flash']
            if 'worker' in flash and 'erase' in flash: print(f'Storage: {flash.get("growths", "?")} growth(s), {flash.get("overwritten_flights", "?")} older flight(s) overwritten; '
                  f'longest worker job {flash["worker"][2]/1000:.1f} ms, longest erase {flash["erase"][2]/1000:.1f} ms, '
                  f'queue peak {flash["queue_peak"]}/{flash["queue_capacity"]}.')
        if rate480:
            print_diag(report['diag'])
        if (status['output_mode'] != 'VIRTUAL' or status['reason'] != 'completed' or status['error'] != 'None'
                or status.get('source') != 'hoppino' or status['invalid_frames'] or status['t_ms'] != duration
                or status['samples'] + timing['missed_ticks'] != samples or count != status['samples']
                or [event[0] for event in report['events']['phases']] != list(range(status['phase']+1))):
            raise RuntimeError('Simulation result failed validation; report retained')
        # PAD samples are reduced to the pre-launch history (0.5 s full rate + 10 Hz), so
        # fewer samples are saved than processed; none may be dropped.
        if record and (not 0 < status['saved_samples'] <= status['samples'] or status['dropped_samples'] or status['recording_error'] != 'None'):
            raise RuntimeError('Recording incomplete; report retained')
        if timing['missed_ticks'] or timing['over_budget']:
            raise RuntimeError('Measured deadline violations; report retained')
        return report
    finally:
        if started:
            try:
                client.request('sim stop', timeout=60)
            except Exception as error:
                print(f'Stop not acknowledged: {error}. Standalone simulation may still be running.', file=sys.stderr)


def print_diag(diag):
    (used0, size0), (used1, size1) = diag['stack_bytes']
    print(f'Stack high-water: core 0 {used0}/{size0} bytes, core 1 {used1}/{size1} bytes'
          + (' (EXHAUSTED)' if any(diag['stack_exhausted']) else '') + '.')
    count, total, peak = diag['loop']
    print(f'Core 0 loop: {count} periods, mean {total/count if count else 0:.0f} us, max {peak} us, '
          f'{diag["loop_over_period"]} longer than {diag["loop_period_us"]} us.')
    print(f'Run {diag["run"]}: step max {diag["step"][2]} us, compute max {diag["compute"][2]} us, '
          f'storage max {diag["storage"][2]} us, over budget {diag["over_budget"]}; frame dt '
          f'{diag["frame_dt_us"][0]}..{diag["frame_dt_us"][1]} us, sequence gaps {diag["sequence_gaps"]}.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--record', action='store_true', help='Record to board flash (512 KiB erased at start, growing as needed)')
    parser.add_argument('--rate480', action='store_true', help='480 Hz stress profile (about 61 s) and diag report')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or ROOT / '.pio' / f'simulation_{uuid.uuid4().hex}.json'
    if output.exists():
        raise FileExistsError(output)
    with transport.DeviceConnection(args.port) as client:
        run(client, output, args.record, rate480=args.rate480)


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Simulation check failed: {error}', file=sys.stderr)
        sys.exit(1)
