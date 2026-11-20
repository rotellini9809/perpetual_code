"""Bench check of the initial vertical alignment and tilt estimate (physical arming, stays in PAD).

Arms the board, waits for PAD, then asks you to hold the board at a series of angles. At each
position it reads `status` for a few seconds and prints the estimated tilt (angle between BODY +X,
the rocket axis, and the upward vertical) with its spread, and the upward vertical in BODY axes.
Compare with a protractor or a phone level. Disarms at the end; a run that stays in PAD is not
recorded. In PAD the accelerometer keeps correcting the attitude, so this checks the alignment and
the tilt reading, not gyroscope drift (which matters in BOOST/COAST, where that correction is off). Run from a terminal:

    python tools/replay/bench_tilt_usb.py --port COM11
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client as transport

STEPS = (
    ('upright', 0, 'Board upright (rocket axis vertical), still'),
    ('20 deg', 20, 'Tilt to about 20 deg and hold'),
    ('40 deg', 40, 'Tilt to about 40 deg (the bistage abort limit) and hold'),
    ('90 deg', 90, 'Lay it horizontal (90 deg) and hold'),
    ('20 deg other side', 20, 'Stand it upright first, then tilt about 20 deg towards a different side '
                              '(e.g. a quarter turn around the rocket axis from the first tilt) and hold'),
    ('upright again', 0, 'Back upright, still'),
)


def wait_pad(client, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.request('status')
        if status['arming_state'] == 'armed' and status.get('phase') == 0 and status.get('attitude'):
            return status
        time.sleep(0.5)
    raise RuntimeError(f'Board did not reach PAD with an attitude (state {status["arming_state"]})')


def measure(client, seconds):
    tilts, ups = [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        status = client.request('status')
        if status.get('phase') != 0:
            raise RuntimeError(f'Phase changed to {status.get("phase")}: the board left PAD')
        tilts.append(status['attitude']['tilt_deg']); ups.append(status['attitude']['up_body'])
        time.sleep(0.1)
    mean_up = [sum(u[i] for u in ups) / len(ups) for i in range(3)]
    return dict(samples=len(tilts), mean=sum(tilts) / len(tilts), min=min(tilts), max=max(tilts),
                up_body=[round(v, 3) for v in mean_up])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=3, help='Reading time per position')
    args = parser.parse_args()
    report = dict(result='incomplete', steps={})
    path = ROOT / '.pio' / f'bench_tilt_{uuid.uuid4().hex[:8]}.json'
    with transport.DeviceConnection(args.port) as client:
        if client.request('status')['arming_state'] != 'disarmed':
            raise RuntimeError('Board must be disarmed')
        preflight = client.request('preflight')
        if not preflight.get('ready'):
            raise RuntimeError(f'Physical preflight not ready: {preflight.get("reasons")}')
        input('Place the board UPRIGHT and still, then press Enter to arm...')
        print('Arming: keep still for the gyro calibration (about 5 s)...', flush=True)
        client.request('arm', timeout=30); wait_pad(client)
        try:
            for name, expected, prompt in STEPS:
                input(f'\n{prompt}. Press Enter when in position...')
                r = measure(client, args.seconds); r['expected'] = expected; report['steps'][name] = r
                print(f'  {name}: tilt {r["mean"]:.1f} deg (spread {r["min"]:.1f}..{r["max"]:.1f}, expected ~{expected}), '
                      f'up_body {r["up_body"]}', flush=True)
            report['result'] = 'done'
        finally:
            try:
                if client.request('status')['arming_state'] == 'armed':
                    client.request('disarm'); print('\nDisarmed.', flush=True)
            except Exception as error:
                print(f'Disarm refused: {error}', flush=True); report['disarm_error'] = str(error)
            path.write_text(json.dumps(report, indent=2))
            print(f'Report: {path}', flush=True)


if __name__ == '__main__':
    main()
