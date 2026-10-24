"""Temporary recorded-estimate replay. Uses the board's persisted plan; virtual only."""
import argparse
import csv
import math
from pathlib import Path
import sys
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from device_client import Client, DeviceConnection

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = ROOT / 'datasets/hoppino_VOLO_UFFICIALE.csv'
PHASES = {0: 'PAD', 1: 'BOOST', 2: 'COAST', 3: 'DESCENT', 4: 'DESCENT', 5: 'LANDED'}


def load_samples(path):
    samples = []
    with Path(path).open(newline='', encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        names = reader.fieldnames or []
        if len(names) != len(set(names)) or not {'t_ms', 'state', 'z_est', 'vz_est'} <= set(names):
            raise ValueError('Missing or duplicate CSV columns')
        for line, row in enumerate(reader, 2):
            try:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError('Wrong column count')
                if not row['t_ms'].isascii() or not row['t_ms'].isdigit():
                    raise ValueError('Invalid timestamp')
                if row['state'] not in ('0', '1', '2', '3', '4', '5'):
                    raise ValueError('Unknown legacy state')
                stamp, state = int(row['t_ms']), int(row['state'])
                altitude, velocity = float(row['z_est']), float(row['vz_est'])
                if any(math.isfinite(value) and abs(value) > 3.402823466e38 for value in (altitude, velocity)):
                    raise ValueError('Estimate exceeds float32 range')
                if samples and stamp <= samples[-1][0]:
                    raise ValueError('Timestamps must strictly increase')
                samples.append((stamp, state, altitude, velocity))
            except (ValueError, TypeError) as error:
                raise ValueError(f'CSV line {line}: {error}') from error
    if not samples:
        raise ValueError('No samples')
    if samples[-1][0] - samples[0][0] > 0xFFFFFFFF:
        raise ValueError('Replay exceeds uint32 millisecond range')
    return samples


def run(client, samples, output):
    view = client.request('config show')
    if not view.get('available'):
        raise RuntimeError('Board has no valid applied configuration; configure/apply first')
    if view.get('draft_active'):
        raise RuntimeError('A draft is active; apply or discard it before replay')
    names = {}
    for rule_id in view['rule_ids']:
        names[rule_id] = client.request(f'rule show {rule_id}')['name']
    origin = samples[0][0]
    fields = ['t_ms', 'source_t_ms', 'event', 'phase', 'rule_id', 'rule_name', 'output_kind', 'channel', 'mode']
    # Exclusive creation prevents overwriting recordings, aliases or earlier exports.
    with Path(output).open('x', newline='', encoding='utf-8') as destination:
        writer = csv.DictWriter(destination, fieldnames=fields)
        writer.writeheader()
        started = False
        complete = False
        try:
            client.request('replay begin')
            started = True
            print('Board replay: applied configuration; recorded estimates; VIRTUAL outputs only.')
            last_phase = None
            starts = 0
            for index, (stamp, state, altitude, velocity) in enumerate(samples):
                t_ms = stamp - origin
                response = client.request(f'replay sample {t_ms} {state} {altitude:.9g} {velocity:.9g}')
                if response.get('output_mode') != 'VIRTUAL' or response.get('t_ms') != t_ms:
                    raise RuntimeError('Unexpected replay mode/time')
                phase = PHASES[state]

                def event(kind, rule_id=0, output_kind='', channel=0):
                    writer.writerow(dict(t_ms=t_ms, source_t_ms=stamp, event=kind, phase=phase,
                                         rule_id=rule_id, rule_name=names.get(rule_id, ''),
                                         output_kind=output_kind, channel=channel, mode='VIRTUAL'))

                if phase != last_phase:
                    event('phase')
                    print(f'{t_ms} ms: {phase}')
                    last_phase = phase
                for rule_id in response['triggered']:
                    event('rule_trigger', rule_id)
                    print(f'{t_ms} ms: rule {rule_id} triggered ({names.get(rule_id, "")})')
                for rule_id, kind, channel, actual_time in response['starts']:
                    if actual_time != t_ms or rule_id not in names:
                        raise RuntimeError('Invalid output-start association/time')
                    event('output_start', rule_id, kind, channel)
                    print(f'{t_ms} ms: VIRTUAL {kind} {channel}')
                    starts += 1
                if (index + 1) % 1000 == 0:
                    print(f'{index + 1}/{len(samples)} samples')
            final = client.request('replay stop')
            started = False
            complete = True
            print(f'{len(samples)} samples, {starts} virtual starts; {final["cancelled_pending"]} pending actions cancelled at EOF.')
        finally:
            if started:
                try:
                    client.request('replay stop')
                except Exception as error:
                    print(f'Stop could not be acknowledged: {error}. Close the connection to terminate replay.', file=sys.stderr)
            if not complete:
                print(f'Replay incomplete; partial events retained at {output}', file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='USB or outgoing SPP port, e.g. COM9/COM11; close other terminals first')
    parser.add_argument('--input', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    samples = load_samples(args.input)  # Validate before connecting or starting replay.
    output = args.output or ROOT / '.pio' / f'hoppino_usb_events_{uuid.uuid4().hex}.csv'
    if output.exists():
        raise ValueError('Output already exists')
    output.parent.mkdir(parents=True, exist_ok=True)
    with DeviceConnection(args.port) as client:
        run(client, samples, output)
    print(f'Events: {output}')


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Replay failed: {error}', file=sys.stderr)
        sys.exit(1)
