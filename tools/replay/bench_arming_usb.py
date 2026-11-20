"""Bench test of a physical arming with the real sensors: USB load and, optionally, a real recording.

Part A (board still on the bench, stays in PAD): arm, then three phases of increasing USB load,
reading `diag` between them to see which load delays the flight loop or loses IMU frames:
  quiet  - no requests at all;
  app    - what the desktop app sends while armed: status + control status every 4 s, ping every 2 s;
  heavy  - app load plus `sensors` every 0.25 s (the live sensor window over USB).
`sequence_gaps` and `fifo_overruns` are counts (per-phase deltas); loop/step maxima only grow, so
a phase that raises them is the one that caused them.

Part B (--motion, interactive): after Part A, shake the board hard (> 3 g) to trigger BOOST.
It then records at full rate and grows in the ring; after --record-seconds you lower it about 1 m
and set it down still until LANDED, when the run closes by itself. Prints growths, erase and
worker maxima and downloads the flight. After BOOST the board refuses to disarm until LANDED; if
it stays in COAST, unplugging it is the only way out (the recording is then recovered as
Interrupted at the next boot).

Physical arming drives real outputs. Run Part B only with a plan that assigns no pyro channel.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import device_client as transport
import download_flight

PHASES = ('PAD', 'BOOST', 'COAST', 'DESCENT', 'LANDED', 'BOOST 2', 'COAST 2', 'ABORT')


class AppLoad:
    """Requests at the desktop app's intervals; `sensors_every` adds the live sensor window."""
    def __init__(self, client, sensors_every=None):
        self.client, self.sensors_every = client, sensors_every
        self.next = dict(ping=0.0, status=0.0, sensors=0.0)
        self.requests = 0

    def tick(self, now):
        if now >= self.next['status']:
            self.client.request('status'); self.client.request('control status'); self.requests += 2
            self.next['status'] = now + 4
        elif now >= self.next['ping']:
            self.client.request('ping'); self.requests += 1
            self.next['ping'] = now + 2
        elif self.sensors_every and now >= self.next['sensors']:
            self.client.request('sensors'); self.requests += 1
            self.next['sensors'] = now + self.sensors_every


def summary(diag, before=None):
    gaps = diag['sequence_gaps'] - (before['sequence_gaps'] if before else 0)
    fifo = diag['fifo_overruns'] - (before['fifo_overruns'] if before else 0)
    return dict(sequence_gaps=gaps, fifo_overruns=fifo, loop_max_us=diag['loop'][2], loop_over_period=diag['loop_over_period'],
                step_max_us=diag['step'][2], compute_max_us=diag['compute'][2], frame_dt_us=diag['frame_dt_us'],
                stack_core0=diag['stack_bytes'][0][0], frames=diag['step'][0], loop_max_at=diag.get('loop_max_at'))


def at(where):
    if not where: return ''
    phase = PHASES[where['phase']] if 0 <= where['phase'] < len(PHASES) else 'no phase'
    return f' at {where["run_ms"]/1000:.1f} s in {phase}' + (', USB input' if where['usb_input'] else '')


def show(name, s):
    print(f'  {name:6s}: {s["frames"]} frames, gaps +{s["sequence_gaps"]}, FIFO overruns +{s["fifo_overruns"]}, '
          f'loop max {s["loop_max_us"]} us{at(s["loop_max_at"])} ({s["loop_over_period"]} over period), step max {s["step_max_us"]} us, '
          f'frame dt {s["frame_dt_us"][0]}..{s["frame_dt_us"][1]} us, stack {s["stack_core0"]} B', flush=True)


def wait_armed(client, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.request('status')
        if status['arming_state'] == 'armed':
            return status
        time.sleep(0.5)
    raise RuntimeError(f'Board did not reach armed (state {status["arming_state"]})')


def part_a(client, seconds):
    results = {}
    before = client.request('diag')
    for name, load in (('quiet', None), ('app', AppLoad(client)), ('heavy', AppLoad(client, sensors_every=0.25))):
        print(f'Phase {name}: {seconds} s...', flush=True)
        end = time.monotonic() + seconds
        while (now := time.monotonic()) < end:
            if load: load.tick(now)
            time.sleep(0.02)
        diag = client.request('diag')
        if diag['run'] != 'physical' or not diag['run_active']:
            raise RuntimeError(f'Run ended during phase {name}: {diag["run"]}, active {diag["run_active"]}')
        results[name] = dict(summary(diag, before), requests=load.requests if load else 0)
        show(name, results[name]); before = diag
    return results


def part_b(client, record_seconds, folder):
    load = AppLoad(client)
    input('\nPart B. Press Enter, then SHAKE the board hard (> 3 g) to trigger BOOST...')
    phase, deadline = None, time.monotonic() + 120
    while time.monotonic() < deadline:
        load.tick(time.monotonic())
        status = client.request('status'); phase = status.get('phase')
        if phase is not None and phase != 0:
            break
        time.sleep(0.2)
    else:
        print('No BOOST detected in 120 s: still in PAD, disarming.'); return None
    print(f'{PHASES[phase]} detected. Leave the board still: recording at full rate for {record_seconds} s...', flush=True)
    end = time.monotonic() + record_seconds
    while (now := time.monotonic()) < end:
        load.tick(now); time.sleep(0.05)
    input('Now LOWER the board about 1 m, set it down and keep it still; press Enter when it is down...')
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        load.tick(time.monotonic())
        status = client.request('status')
        if status['arming_state'] == 'disarmed':
            break
        time.sleep(0.5)
    else:
        print(f'Still {status["arming_state"]} in phase {PHASES[status["phase"]] if status.get("phase") is not None else "?"}: '
              'it may be stuck in COAST. Unplugging it ends the run; the recording is recovered as Interrupted.')
        return dict(stuck=True, status=status)
    while client.request('flights status').get('active'):
        time.sleep(0.5)
    flash, diag = client.request('flights timing'), client.request('diag')
    newest = client.request('flights list 0')
    result = dict(flash=flash, diag=diag, flight=newest)
    print(f'Recording: flight {newest.get("sequence")}, {flash.get("growths")} growth(s); longest erase {flash["erase"][2]/1000:.1f} ms, '
          f'longest worker job {flash["worker"][2]/1000:.1f} ms (preparation at arming {flash.get("prepare_us", 0)/1000:.0f} ms), '
          f'queue peak {flash["queue_peak"]}/{flash["queue_capacity"]}', flush=True)
    show('flight', summary(diag))
    if newest.get('available'):
        metadata = download_flight.download(client, newest['sequence'], folder / f'flight_{newest["sequence"]}.csv')
        result['download'] = dict(samples=metadata['sample_count'], status=metadata['status'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--phase-seconds', type=int, default=60)
    parser.add_argument('--motion', action='store_true', help='Part B: shake for BOOST and record (interactive)')
    parser.add_argument('--record-seconds', type=int, default=60)
    args = parser.parse_args()
    folder = ROOT / '.pio' / f'bench_arming_{uuid.uuid4().hex[:8]}'
    folder.mkdir(parents=True)
    report = dict(result='incomplete')
    # No heartbeat: the quiet phase must send nothing; each phase sends its own requests.
    with transport.DeviceConnection(args.port, heartbeat_interval=3600) as client:
        status = client.request('status')
        if status['arming_state'] != 'disarmed':
            raise RuntimeError('Board must be disarmed')
        preflight = client.request('preflight')
        if not preflight.get('ready'):
            raise RuntimeError(f'Physical preflight not ready: {preflight.get("reasons")}')
        report['preflight'] = preflight
        print('Arming (keep the board still for the gyro calibration)...', flush=True)
        client.request('arm', timeout=30); wait_armed(client)
        try:
            report['part_a'] = part_a(client, args.phase_seconds)
            if args.motion:
                report['part_b'] = part_b(client, args.record_seconds, folder)
        finally:
            try:
                if client.request('status')['arming_state'] == 'armed':
                    client.request('disarm'); print('Disarmed.', flush=True)
            except Exception as error:  # e.g. DISARM_PROHIBITED_IN_FLIGHT after BOOST
                print(f'Disarm refused: {error}', flush=True); report['disarm_error'] = str(error)
            report['final_diag'] = client.request('diag')
            report['result'] = 'done'
            (folder / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
            print(f'Report: {folder / "report.json"}')


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Bench arming test failed: {error}', file=sys.stderr)
        sys.exit(1)
