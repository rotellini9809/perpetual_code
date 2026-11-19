"""Bench test of the flight ring on the board: growth, wrap-around and overwrite of old flights.

Repeats the onboard 480 Hz stress recording (`sim begin record 480`, VIRTUAL outputs, about
61 s and 1 MiB each) until the ring has wrapped, then a few more times. After every run it
reads the retained flights and checks that:
- the new flight is Completed, with no dropped samples and no storage error;
- it was placed right after the previous newest flight (circularly) and grew at least once;
- retained extents never overlap on the ring;
- the flights that disappeared are exactly the older ones its blocks overlapped, oldest first,
  and nothing else changed.
It downloads and decodes the first and the last test flight and the oldest one still retained.
Timing (`sim timing`, `flights timing`, `diag`) is stored per run; deadline misses are reported
as findings, storage-contract violations stop the test.

Existing flights WILL be overwritten: --allow-overwrite is required when the board holds any.
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
from run_simulation_usb import print_diag

RING_BEGIN, BLOCK_BYTES, RING_BLOCKS = 0x030000, 65536, 253
STATUS = {1: 'Recording', 2: 'Completed', 3: 'Interrupted', 4: 'StoppedFull', 5: 'Invalidated', 6: 'Corrupt'}


class Failure(RuntimeError):
    pass


def retained(client):
    flights = []
    for index in range(64):
        entry = client.request(f'flights list {index}')
        if not entry.get('available'):
            break
        flights.append(dict(sequence=int(entry['sequence']), status=entry['status'],
                            first=(entry['start_address'] - RING_BEGIN) // BLOCK_BYTES,
                            blocks=entry['allocated_bytes'] // BLOCK_BYTES, committed=entry['committed_bytes']))
    return sorted(flights, key=lambda f: f['sequence'])


def blocks_of(flight):
    return {(flight['first'] + i) % RING_BLOCKS for i in range(flight['blocks'])}


def check_disjoint(flights):
    seen = {}
    for flight in flights:
        for block in blocks_of(flight):
            if block in seen:
                raise Failure(f'Flights {seen[block]} and {flight["sequence"]} both own ring block {block}')
            seen[block] = flight['sequence']


def record_once(client, run, timeout_margin=90):
    client.request('flights timing reset')
    status = client.request('sim begin record 480', timeout=600)
    if status.get('source') != 'hoppino480':
        raise Failure('Firmware does not provide the 480 Hz profile; upload current MAIN')
    deadline = time.monotonic() + status['duration_ms'] / 1000 + timeout_margin
    try:
        while status['active'] or status.get('recording_busy', False):
            if time.monotonic() > deadline:
                raise Failure(f'Run {run}: simulation or recording worker did not finish in time')
            time.sleep(1)
            status = client.request('sim status')
    except BaseException:
        try:
            client.request('sim stop', timeout=60)
        except Exception as error:
            print(f'Stop not acknowledged: {error}', file=sys.stderr)
        raise
    return dict(status=status, timing=client.request('sim timing'), flash=client.request('flights timing'),
                diag=client.request('diag'))


def check_run(run, result, before, after, ours=()):
    """Storage contract after one recording; returns the new flight and the overwritten sequences."""
    status, flash = result['status'], result['flash']
    if status['reason'] != 'completed' or status['error'] != 'None' or status['recording_error'] != 'None':
        raise Failure(f'Run {run}: simulation {status["reason"]}/{status["error"]}, storage {status["recording_error"]}')
    if status['dropped_samples'] or not status['saved_samples']:
        raise Failure(f'Run {run}: {status["saved_samples"]} saved, {status["dropped_samples"]} dropped')
    sequence = int(status['recording_sequence'])
    new = [f for f in after if f['sequence'] == sequence]
    if not new or STATUS.get(new[0]['status']) != 'Completed':
        raise Failure(f'Run {run}: flight {sequence} is not retained as Completed')
    new = new[0]
    if before and sequence <= before[-1]['sequence']:
        raise Failure(f'Run {run}: sequence {sequence} does not follow {before[-1]["sequence"]}')
    if before:
        newest = before[-1]
        expected = (newest['first'] + newest['blocks']) % RING_BLOCKS
        if new['first'] != expected:
            raise Failure(f'Run {run}: flight starts at block {new["first"]}, expected {expected} after flight {newest["sequence"]}')
    if not flash.get('growths'):
        raise Failure(f'Run {run}: the recording never grew beyond its first 8 blocks')
    check_disjoint(after)
    kept = {f['sequence']: f for f in after if f['sequence'] != sequence}
    previous = {f['sequence']: f for f in before}
    if set(kept) - set(previous):
        raise Failure(f'Run {run}: unexpected flights appeared: {sorted(set(kept) - set(previous))}')
    for seq, flight in kept.items():
        if flight != previous[seq]:
            raise Failure(f'Run {run}: retained flight {seq} changed: {previous[seq]} -> {flight}')
    removed = sorted(set(previous) - set(kept))
    overlapped = sorted(s for s, f in previous.items() if blocks_of(f) & blocks_of(new))
    if removed != overlapped:
        raise Failure(f'Run {run}: overwritten {removed}, but the new flight overlaps {overlapped}')
    # Test flights are contiguous in recording order, so their overwrite must go oldest first.
    # Pre-existing flights may sit anywhere: they go when (and only when) their blocks are needed.
    ours_removed, ours_kept = [s for s in removed if s in ours], [s for s in kept if s in ours]
    if ours_removed and ours_kept and max(ours_removed) > min(ours_kept):
        raise Failure(f'Run {run}: flight {max(ours_removed)} was overwritten before older flight {min(ours_kept)}')
    if flash.get('overwritten_flights') != len(removed):
        raise Failure(f'Run {run}: board reports {flash.get("overwritten_flights")} overwritten flights, list shows {len(removed)}')
    return new, removed


def findings(run, result):
    timing, diag, notes = result['timing'], result['diag'], []
    if timing['missed_ticks'] or timing['over_budget']:
        notes.append(f'{timing["missed_ticks"]} missed ticks, {timing["over_budget"]} cycles over the {timing["period_us"]} us period')
    if timing.get('sequence_gaps'):
        notes.append(f'{timing["sequence_gaps"]} frame sequence gaps')
    if any(diag['stack_exhausted']):
        notes.append('stack exhausted')
    return [f'Run {run}: {note}' for note in notes]


def post_mortem(port, attempts=15):
    """After a lost connection: reconnect and record whether the board rebooted, and why."""
    for _ in range(attempts):
        try:
            with transport.DeviceConnection(port) as client:
                diag = client.request('diag')
                return dict(uptime_ms=client.request('status').get('uptime_ms'), reset_reason=diag.get('reset_reason'),
                            reset_bits=diag.get('reset_bits'), stack_bytes=diag.get('stack_bytes'), flights=retained(client))
        except Exception as error:
            last = str(error)
            time.sleep(2)
    return dict(error=f'Board did not answer again: {last}')


def download(client, sequence, folder):
    path = folder / f'flight_{sequence}.csv'
    metadata = download_flight.download(client, sequence, path)
    return dict(sequence=sequence, samples=metadata['sample_count'], csv=str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--allow-overwrite', action='store_true', help='Proceed although the board holds flights (they will be lost)')
    parser.add_argument('--after-wrap', type=int, default=2, help='Runs after the first wrap-around (default 2)')
    parser.add_argument('--max-runs', type=int, default=24)
    parser.add_argument('--delete-after', action='store_true', help='Delete the flights this test recorded')
    parser.add_argument('--output', type=Path, help='Report folder (default .pio/ring_overwrite_<id>)')
    args = parser.parse_args()
    folder = args.output or ROOT / '.pio' / f'ring_overwrite_{uuid.uuid4().hex[:8]}'
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(runs=[], findings=[], downloads=[], result='incomplete')
    report_path = folder / 'report.json'

    def save():
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')

    with transport.DeviceConnection(args.port) as client:
        if client.request('status').get('arming_state') != 'disarmed':
            raise Failure('Board must be disarmed')
        recovered = client.request('flights recover', timeout=600)
        if recovered.get('corrupt_flights'):
            print(f'Warning: {recovered["corrupt_flights"]} corrupt flight(s) retained.', flush=True)
        flights = retained(client)
        report['initial'] = flights
        if flights and not args.allow_overwrite:
            raise Failure(f'The board holds {len(flights)} flight(s); they would be overwritten. '
                          'Download what you need, then rerun with --allow-overwrite.')
        print(f'{len(flights)} flight(s) on the board at start; ring of {RING_BLOCKS} blocks.', flush=True)
        recorded, used, wrapped_at = [], 0, None
        try:
            for run in range(1, args.max_runs + 1):
                print(f'Run {run}: recording the 480 Hz profile...', flush=True)
                result = record_once(client, run)
                after = retained(client)
                new, removed = check_run(run, result, flights, after, set(recorded))
                recorded.append(new['sequence'])
                used += new['blocks']
                if wrapped_at is None and flights and new['first'] < flights[-1]['first']:
                    wrapped_at = run
                if wrapped_at is None and new['first'] + new['blocks'] > RING_BLOCKS:
                    wrapped_at = run  # The flight itself crosses the end of the ring.
                notes = findings(run, result)
                report['findings'] += notes
                flash, diag = result['flash'], result['diag']
                report['runs'].append(dict(run=run, flight=new, overwritten=removed, retained=len(after), **result))
                save()
                print(f'  flight {new["sequence"]}: blocks {new["first"]}..{(new["first"] + new["blocks"] - 1) % RING_BLOCKS} '
                      f'({new["blocks"]}), {result["status"]["saved_samples"]} samples, {flash["growths"]} growth(s); '
                      f'overwritten {removed or "none"}; {len(after)} retained'
                      + ('; ring WRAPPED' if wrapped_at == run else ''), flush=True)
                print(f'  worker max {flash["worker"][2] / 1000:.1f} ms, erase max {flash["erase"][2] / 1000:.1f} ms, '
                      f'queue peak {flash["queue_peak"]}/{flash["queue_capacity"]}', flush=True)
                print_diag(diag)
                for note in notes:
                    print('  FINDING: ' + note, flush=True)
                flights = after
                if wrapped_at is not None and run - wrapped_at >= args.after_wrap:
                    break
            if wrapped_at is None:
                raise Failure(f'The ring did not wrap within {args.max_runs} runs ({used} blocks recorded)')
            print('Downloading and decoding the first, the last and the oldest retained test flights...', flush=True)
            targets = []
            for sequence in (recorded[0], flights[0]['sequence'], recorded[-1]):
                if sequence in (f['sequence'] for f in flights) and sequence not in targets:
                    targets.append(sequence)
            for sequence in targets:
                report['downloads'].append(download(client, sequence, folder))
            report['result'] = 'passed'
        except Failure as error:
            report['result'] = f'failed: {error}'
            raise
        except (transport.ConnectionLost, OSError, TimeoutError) as error:
            report['result'] = f'connection lost: {error}'
            lost = True
        else:
            lost = False
        finally:
            report['final'] = flights
            if report['result'].startswith('connection lost'):
                pass  # The connection is gone: no cleanup through it.
            elif args.delete_after:
                for sequence in recorded:
                    if any(f['sequence'] == sequence for f in flights):
                        client.request(f'flights delete {sequence}')
                report['deleted'] = recorded
            save()
    if lost:
        print('Connection lost: reconnecting to see whether the board rebooted...', flush=True)
        report['post_mortem'] = post_mortem(args.port); save()
        print(f'Post mortem: {report["post_mortem"]}')
        print(f'Report: {report_path}')
        raise Failure(report['result'])
    print(f'Report: {report_path}')
    print(f'Ring overwrite test passed: wrapped at run {wrapped_at}, {len(report["runs"])} runs, '
          f'{len(report["findings"])} timing finding(s).')


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Ring overwrite test failed: {error}', file=sys.stderr)
        sys.exit(1)
