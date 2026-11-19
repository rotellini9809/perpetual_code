"""Start the onboard Hoppino sensor replay; USB/SPP only controls and observes it."""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client as transport


def run(client, output, record=False, clock=time.monotonic, sleep=time.sleep):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    if record:
        print('Mount/recover storage; existing recordings retained.', flush=True)
        recovered = client.request('flights recover', timeout=600)
        if recovered.get('corrupt_flights', 0):
            print(f'Warning: {recovered["corrupt_flights"]} corrupt flight(s) retained; their space remains reserved.', flush=True)
        client.request('flights timing reset')
    print('Starting onboard Hoppino sensor replay; VIRTUAL outputs only.', flush=True)
    started = False
    try:
        status = client.request('sim begin record' if record else 'sim begin', timeout=600 if record else 5)
        started = True
        if status.get('source')!='hoppino': raise RuntimeError('Firmware does not provide onboard Hoppino; upload current MAIN')
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--record', action='store_true', help='Record to board flash (512 KiB erased at start, growing as needed)')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or ROOT / '.pio' / f'simulation_{uuid.uuid4().hex}.json'
    if output.exists():
        raise FileExistsError(output)
    with transport.DeviceConnection(args.port) as client:
        run(client, output, args.record)


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Simulation check failed: {error}', file=sys.stderr)
        sys.exit(1)
