"""Bench check of the initial vertical alignment and tilt estimate (physical arming, stays in PAD).

Arms the board, waits for PAD, then asks you to hold the board at a series of angles. At each
position it reads `status` for a few seconds and prints the estimated tilt (angle between BODY +X,
the rocket axis, and the upward vertical) with its spread, and the upward vertical in BODY axes.
Compare with a protractor or a phone level. Disarms at the end; a run that stays in PAD is not
recorded. In PAD the accelerometer keeps correcting the attitude, so this checks the alignment and
the tilt reading, not gyroscope drift (which matters in BOOST/COAST, where that correction is off).

`--gyro` (firmware built with IMU_IDENTITY_TEST) checks that case: `nav gyro start` copies the
attitude into a second estimator that integrates the gyroscope alone, as in BOOST/COAST, and the
script compares it with the normal tilt: drift while still, known rotations, fast motion. Run from
a terminal:

    python tools/replay/bench_tilt_usb.py --port COM11
    python tools/replay/bench_tilt_usb.py --port COM11 --gyro
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

GYRO_STEPS = (
    ('40 deg', 40, 'Tilt to about 40 deg and hold'),
    ('90 deg', 90, 'Lay it horizontal (90 deg) and hold'),
    ('upright', 0, 'Back upright and hold'),
    ('20 deg other side', 20, 'Tilt about 20 deg towards a different side (a quarter turn around the rocket axis) and hold'),
    ('upright after rotations', 0, 'Back upright and hold'),
    ('upright after fast motion', 0, 'Twist and swing it quickly for about 5 s (rotations, NO hard shakes or knocks), '
                                     'then put it back upright and hold'),
)


def wait_pad(client, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.request('status')
        if status['arming_state'] == 'armed' and status.get('phase') == 0 and status.get('attitude'):
            return status
        time.sleep(0.5)
    raise RuntimeError(f'Board did not reach PAD with an attitude (state {status["arming_state"]})')


def summary(tilts, ups):
    mean_up = [sum(u[i] for u in ups) / len(ups) for i in range(3)]
    return dict(samples=len(tilts), mean=sum(tilts) / len(tilts), min=min(tilts), max=max(tilts),
                up_body=[round(v, 3) for v in mean_up])


def measure(client, seconds, gyro=False):
    tilts, ups, gyro_tilts, gyro_ups = [], [], [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        status = client.request('status')
        if status.get('phase') != 0:
            raise RuntimeError(f'Phase changed to {status.get("phase")}: the board left PAD')
        attitude = status['attitude']
        tilts.append(attitude['tilt_deg']); ups.append(attitude['up_body'])
        if gyro:
            if 'gyro_only' not in attitude:
                raise RuntimeError('status has no gyro_only attitude')
            gyro_tilts.append(attitude['gyro_only']['tilt_deg']); gyro_ups.append(attitude['gyro_only']['up_body'])
        time.sleep(0.1)
    result = summary(tilts, ups)
    if gyro:
        result['gyro_only'] = summary(gyro_tilts, gyro_ups)
    return result


def angle_between(a, b):
    dot = sum(x * y for x, y in zip(a, b)) / (math.sqrt(sum(x * x for x in a)) * math.sqrt(sum(y * y for y in b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def gyro_test(client, seconds, still_seconds, report):
    input('\nUpright and still: press Enter to start the gyro-only attitude...')
    client.request('nav gyro start')
    started = time.monotonic()
    first = measure(client, seconds, True)
    print(f'  start: tilt {first["mean"]:.1f} deg, gyro-only {first["gyro_only"]["mean"]:.1f} deg', flush=True)
    print(f'Hold still for {still_seconds:.0f} s...', flush=True)
    time.sleep(max(0.0, still_seconds - 2 * seconds))
    last = measure(client, seconds, True)
    minutes = (time.monotonic() - started - seconds) / 60
    drift = angle_between(first['gyro_only']['up_body'], last['gyro_only']['up_body'])
    report['still'] = dict(start=first, end=last, minutes=minutes, drift_deg=drift, drift_deg_per_min=drift / minutes)
    print(f'  still: gyro-only vertical moved {drift:.2f} deg in {minutes * 60:.0f} s ({drift / minutes:.2f} deg/min); '
          f'tilt {last["mean"]:.1f}, gyro-only {last["gyro_only"]["mean"]:.1f}', flush=True)
    for name, expected, prompt in GYRO_STEPS:
        input(f'\n{prompt}. Press Enter when in position...')
        r = measure(client, seconds, True); r['expected'] = expected
        r['difference_deg'] = angle_between(r['up_body'], r['gyro_only']['up_body'])
        r['elapsed_s'] = time.monotonic() - started; report['steps'][name] = r
        print(f'  {name}: tilt {r["mean"]:.1f}, gyro-only {r["gyro_only"]["mean"]:.1f} deg (expected ~{expected}); '
              f'verticals {r["difference_deg"]:.1f} deg apart after {r["elapsed_s"]:.0f} s', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=3, help='Reading time per position')
    parser.add_argument('--gyro', action='store_true', help='Gyroscope-only attitude test (IMU_IDENTITY_TEST firmware)')
    parser.add_argument('--still-seconds', type=float, default=60, help='--gyro: still time for the drift check')
    args = parser.parse_args()
    report = dict(result='incomplete', steps={})
    path = ROOT / '.pio' / f'bench_tilt{"_gyro" if args.gyro else ""}_{uuid.uuid4().hex[:8]}.json'
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
            if args.gyro:
                gyro_test(client, args.seconds, args.still_seconds, report)
            else:
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
