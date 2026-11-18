"""Download a retained flight as CSV and configuration/output-start metadata JSON."""
import argparse
import base64
import binascii
import zlib
from concurrent.futures import CancelledError
import csv
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client as transport
sys.path.insert(0, str(Path(__file__).resolve().parent))
import flight_log
FIELDS = flight_log.CSV_FIELDS
STATUS = {1: 'recording', 2: 'completed', 3: 'interrupted', 4: 'stopped_full', 5: 'invalidated', 6: 'CORRUPT'}


def configuration(payload):
    offset = 0

    def read(fmt):
        nonlocal offset
        size = struct.calcsize('<' + fmt)
        if offset + size > len(payload):
            raise ValueError('Truncated configuration snapshot')
        values = struct.unpack_from('<' + fmt, payload, offset)
        offset += size
        return values[0] if len(values) == 1 else values

    count = read('B')
    if not 0 <= count <= 8:
        raise ValueError('Invalid rule count')
    result = {'pyros': [{'channel': i+1, 'pulse_ms': read('I')} for i in range(4)], 'servos': [], 'rules': []}
    for i in range(4):
        calibrated, initial, final = read('BHH')
        result['servos'].append(dict(channel=i+1, calibrated=bool(calibrated), initial_us=initial,
                                     final_us=final))
    for _ in range(count):
        rule = {'id': read('I'), 'name': read('48s').split(b'\0', 1)[0].decode('utf-8', errors='replace'),
                'delay_ms': read('I')}
        conditions, outputs = read('BB')
        if not 1 <= conditions <= 8 or not 1 <= outputs <= 8:
            raise ValueError('Invalid rule capacities')
        rule['conditions'] = []
        for _ in range(conditions):
            kind, comparison, phases, value, upper, elapsed, reference = read('BBBffII')
            rule['conditions'].append(dict(kind=kind, comparison=comparison, phase_mask=phases,
                                           value=value if math.isfinite(value) else None,
                                           upper=upper if math.isfinite(upper) else None,
                                           elapsed_ms=elapsed, reference=reference))
        rule['outputs'] = []
        for _ in range(outputs):
            kind, channel, delay = read('BBI')
            rule['outputs'].append(dict(kind='pyro' if kind == 0 else 'servo', channel=channel, delay_ms=delay))
        result['rules'].append(rule)
    channels,conditions=read('BB')
    if channels>15 or conditions>8 or (channels and not conditions): raise ValueError('Invalid airbrake rule')
    air=dict(channel_mask=channels,logic='and',conditions=[])
    for _ in range(conditions):
        kind,comparison,phases,value,upper,elapsed,reference=read('BBBffII')
        air['conditions'].append(dict(kind=kind,comparison=comparison,phase_mask=phases,value=value if math.isfinite(value) else None,
                                      upper=upper if math.isfinite(upper) else None,elapsed_ms=elapsed,reference=reference))
    result['airbrakes']=air
    if offset != len(payload):
        raise ValueError('Extra bytes in configuration snapshot')
    return result


def fetch_block(request, sequence, index, chunk_bytes):
    """Assemble one raw v4 block body from base64 chunks and decode it."""
    body = bytearray()
    frames = None
    while len(body) < flight_log.BODY_BYTES:
        response = request(f'flights block {sequence} {index} {len(body)}')
        try: chunk = base64.b64decode(response.get('data', ''), validate=True)
        except (ValueError, binascii.Error) as error: raise RuntimeError('Invalid block chunk encoding') from error
        if (response.get('sequence') != sequence or response.get('block') != index or response.get('offset') != len(body)
                or response.get('bytes') != len(chunk) or not 1 <= len(chunk) <= chunk_bytes
                or len(body) + len(chunk) > flight_log.BODY_BYTES or (frames is not None and response.get('frames') != frames)):
            raise RuntimeError('Unexpected block chunk identity/order')
        frames = response.get('frames'); body.extend(chunk)
    try: block = flight_log.decode_block(bytes(body))
    except flight_log.LogError as error: raise RuntimeError(f'Block {index}: {error}') from error
    if str(block['sequence']) != sequence or block['index'] != index or len(block['frames']) != frames:
        raise RuntimeError('Block identity mismatch')
    return block


def sequence_text(sequence):
    text = str(sequence)
    if not text.isascii() or not text.isdigit() or not 0 < int(text) <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError('Sequence must be a positive uint64')
    return str(int(text))


def download(client, sequence, destination, *, progress=None, cancelled=None, overwrite=False):
    """Export CSV/JSON; cancel between replies and remove only this export's partial files."""
    if overwrite:
        destination = Path(destination)
        metadata_path = destination.with_suffix('.json')
        if destination == metadata_path:
            raise ValueError('CSV and metadata destinations overlap')
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.flight-download-', dir=destination.parent) as folder:
            staged = Path(folder) / destination.name
            result = download(client, sequence, staged, progress=progress, cancelled=cancelled)
            if cancelled and cancelled(): raise CancelledError('Download cancelled')
            staged.with_suffix('.json').replace(metadata_path)
            staged.replace(destination)
            return result
    owned=[]
    try: return _download(client,sequence,destination,progress,cancelled,owned)
    except CancelledError:
        for path in owned: path.unlink(missing_ok=True)
        raise


def _download(client,sequence,destination,progress,cancelled,owned):
    def checkpoint():
        if cancelled and cancelled(): raise CancelledError('Download cancelled')
    def request(command,**kwargs):
        checkpoint()
        response=client.request(command,**kwargs)
        checkpoint() # Always consume an in-flight reply before stopping.
        return response
    checkpoint()
    sequence = sequence_text(sequence)
    destination = Path(destination)
    metadata_path = destination.with_suffix('.json')
    if destination == metadata_path or destination.exists() or metadata_path.exists():
        raise FileExistsError('CSV/metadata destination already exists or overlaps')
    info = request(f'flights info {sequence}', timeout=600)
    if info.get('sample_transport') != 'raw-v4' or info.get('block_body_bytes') != flight_log.BODY_BYTES:
        raise ValueError('Board firmware does not provide flight log v4; upload current MAIN firmware')
    if type(info.get('block_chunk_bytes')) is not int or not 1 <= info['block_chunk_bytes'] <= flight_log.BODY_BYTES:
        raise RuntimeError('Invalid block chunk size')
    if info.get('config_version') != 3:
        raise ValueError('Unsupported configuration export format; upload current MAIN firmware')
    response = request(f'flights summary {sequence}')
    calibration_response=request(f'flights calibration {sequence}')
    if calibration_response.get('sequence')!=sequence or 'calibration' not in calibration_response:
        raise RuntimeError('Invalid flight calibration snapshot')
    calibration=calibration_response['calibration']
    if calibration is not None:
        if type(calibration.get('bistage')) is not bool or type(calibration.get('minimum_coast_velocity')) not in (int,float) or not math.isfinite(calibration['minimum_coast_velocity']) or not 0<=calibration['minimum_coast_velocity']<=100000:
            raise RuntimeError('Invalid recorded state-machine settings')
        if (type(calibration.get('gyro_calibrated')) is not bool or type(calibration.get('sensor_axes')) is not bool
                or not isinstance(calibration.get('reference_altitude_m'),(int,float)) or not math.isfinite(calibration['reference_altitude_m'])):
            raise RuntimeError('Invalid flight calibration snapshot')
        if calibration['sensor_axes']:
            if calibration.get('up_axis') not in ('+X','-X','+Y','-Y','+Z','-Z'): raise RuntimeError('Invalid recorded orientation')
            for key in ('gyro_bias_radps','accel_bias_mps2','accel_scale'):
                values=calibration.get(key)
                if not isinstance(values,list) or len(values)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                    raise RuntimeError('Invalid recorded sensor bias')
        elif calibration.get('gyro_calibrated') or any(calibration.get(key) is not None for key in ('up_axis','gyro_bias_radps','accel_bias_mps2')):
            raise RuntimeError('Unexpected calibration for simulated BODY data')
    summary = response.get('summary', {})
    if response.get('sequence') != sequence or summary.get('state') not in ('Missing', 'Incomplete', 'Valid', 'Corrupt', 'Unsupported'):
        raise RuntimeError('Invalid recording summary')
    if summary['state'] == 'Valid':
        counts = [summary.get(field) for field in ('saved_samples', 'dropped_samples', 'queue_peak', 'queue_capacity')]
        if (summary.get('version') != 1 or any(type(value) is not int or not 0 <= value <= 0xFFFFFFFF for value in counts)
                or summary['saved_samples'] != info['samples'] or summary['queue_peak'] > summary['queue_capacity']
                or summary.get('reason') not in ('Completed', 'Stopped', 'Disconnected', 'StorageFull')):
            raise RuntimeError('Invalid committed recording summary')
    if info.get('sequence') != sequence or info.get('status') not in (2, 3, 4):
        raise RuntimeError('Flight is not a closed retained selection')
    if not 0 <= info['samples'] <= 0xFFFFFFFF or not 0 <= info['blocks'] <= 8192:
        raise RuntimeError('Invalid recording dimensions')
    if not 1 <= info['config_bytes'] <= 4096:
        raise RuntimeError('Invalid snapshot size')
    payload = bytearray()
    while len(payload) < info['config_bytes']:
        response = request(f'flights config {sequence} {len(payload)}')
        chunk = bytes.fromhex(response['hex'])
        if response.get('sequence') != sequence or response['offset'] != len(payload) or not 1 <= len(chunk) <= 128:
            raise RuntimeError('Invalid configuration chunk')
        payload.extend(chunk)
        if len(payload) > info['config_bytes']:
            raise RuntimeError('Oversized configuration snapshot')
    snapshot = configuration(payload)
    names = {rule['id']: rule['name'] for rule in snapshot['rules']}
    starts = []
    for rule, kind, channel, state, stamp in info['starts']:
        if rule not in names or kind not in (0, 1) or not 1 <= channel <= 4 or state not in (0, 2, 3):
            raise RuntimeError('Invalid output-start metadata')
        if (state == 2 and (not isinstance(stamp, int) or not 0 <= stamp <= 0xFFFFFFFF)) or (state != 2 and stamp is not None):
            raise RuntimeError('Invalid output-start timestamp')
        starts.append(dict(rule_id=rule, rule_name=names[rule], kind='pyro' if kind == 0 else 'servo',
                           channel=channel, state={0: 'empty', 2: 'recorded', 3: 'invalid'}[state], t_ms=stamp))
    if info['frame'] not in (0, 1) or (info['frame'] == 0) != bool(calibration and calibration.get('sensor_axes')):
        raise RuntimeError('Recorded frame convention does not match the calibration snapshot')
    calibrate = flight_log.Calibration(info['frame'], calibration)
    abort = None
    count, previous_time, previous_sequence = 0, None, None
    if progress: progress(0,info['samples'])
    destination.parent.mkdir(parents=True, exist_ok=True)
    checkpoint()
    with destination.open('x', newline='', encoding='utf-8') as target:
        owned.append(destination)
        writer = csv.writer(target)
        writer.writerow(FIELDS)
        for index in range(info['blocks']):
            block = fetch_block(request, sequence, index, info['block_chunk_bytes'])
            for frame in block['frames']:
                if previous_time is not None:
                    if frame['time_us'] <= previous_time or frame['sequence'] <= previous_sequence:
                        raise RuntimeError('Stored samples are not increasing')
                    # The first frame of a block carries absolute values; recompute its gap.
                    frame['gap'] = max(frame['gap'], frame['sequence'] - previous_sequence - 1)
                row = flight_log.csv_row(frame, calibrate, info['frame'] == 0)
                if frame['state']['abort'] and abort is None:
                    abort = dict(t_us=frame['time_us'], reason={1: 'Velocity', 2: 'Tilt'}[frame['state']['abort']])
                previous_time, previous_sequence = frame['time_us'], frame['sequence']
                writer.writerow(['' if value is None else value for value in row])
                count += 1
            if progress: progress(count,info['samples'])
    if count != info['samples']:
        raise RuntimeError('Stored sample count mismatch; partial CSV retained')
    metadata = dict(sequence=sequence, status=STATUS[info['status']], sample_count=count, log_format=4,
                    sensor_mode=info['sensor_mode'], output_mode=info['output_mode'], frame=info['frame'],
                    configuration=snapshot, configuration_version=3, configuration_hex=payload.hex(), output_starts=starts,
                    recording_summary=summary, calibration=calibration, abort=abort)
    checkpoint()
    with metadata_path.open('x', encoding='utf-8') as target:
        owned.append(metadata_path)
        json.dump(metadata, target, ensure_ascii=False, allow_nan=False, indent=2)
        target.write('\n')
    checkpoint()
    print(f'Flight {sequence}: {count} stored samples -> {destination}')
    print(f'Configuration/output starts: {metadata_path}')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--list', action='store_true')
    selection.add_argument('--sequence', type=sequence_text)
    selection.add_argument('--delete', type=sequence_text, help='Explicitly release this flight for reuse; its data will no longer be downloadable')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    with transport.DeviceConnection(args.port) as client:
        recovered = client.request('flights recover', timeout=600)
        if recovered.get('corrupt_flights', 0):
            print(f'Warning: {recovered["corrupt_flights"]} corrupt flight(s) retained; their space remains reserved.', flush=True)
        if args.list:
            for index in range(48):
                entry = client.request(f'flights list {index}')
                if not entry.get('available'):
                    break
                print(f'{index}: sequence {entry["sequence"]}, {STATUS[entry["status"]]}, '
                      f'{entry["committed_bytes"]} committed bytes')
        elif args.delete:
            response = client.request(f'flights delete {args.delete}', timeout=600)
            if response.get('sequence') != args.delete or response.get('deleted') is not True:
                raise RuntimeError('Deletion not acknowledged')
            print(f'Flight {args.delete} deleted; its space can be reused.')
        else:
            destination = args.output or ROOT / '.pio' / f'flight_{args.sequence}_{uuid.uuid4().hex}.csv'
            download(client, args.sequence, destination)


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Download failed: {error}', file=sys.stderr)
        sys.exit(1)
