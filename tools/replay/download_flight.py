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
FIELDS = ['t_ms', 'accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ', 'pressure',
          'temp', 'z_est', 'vz_est', 'roll', 'pitch', 'yaw', 'az_est', 'state', 'battery_v', 'abort_reason', *[f'airbrake_{i}_pct' for i in range(1,5)], 'airbrake_flags', 'airbrake_failed_conditions', 'airbrake_channel_mask']
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


def unpack_samples(response):
    """Decode bounded, checksummed little-endian sample records without float rounding."""
    count=response.get('count')
    if type(count) is not int or not 1<=count<=9 or not isinstance(response.get('data'),str) or len(response['data'])>600:
        raise RuntimeError('Invalid packed sample dimensions')
    try: payload=base64.b64decode(response['data'],validate=True)
    except (ValueError,binascii.Error) as error: raise RuntimeError('Invalid packed sample encoding') from error
    if len(payload)>448 or type(response.get('crc32')) is not int or zlib.crc32(payload)!=response['crc32']:
        raise RuntimeError('Packed sample checksum/length mismatch')
    rows=[]; offset=0; words=[0]*19
    for _ in range(count):
        if offset+4>len(payload): raise RuntimeError('Truncated packed sample mask')
        mask=struct.unpack_from('<I',payload,offset)[0]; offset+=4
        if mask>>19: raise RuntimeError('Invalid packed sample mask')
        for word in range(19):
            if mask & (1<<word):
                if offset+4>len(payload): raise RuntimeError('Truncated packed sample word')
                words[word]^=struct.unpack_from('<I',payload,offset)[0]; offset+=4
        stamp,*fields=struct.unpack('<I14fBHB4HHBB',struct.pack('<19I',*words))
        values=fields[:14]; phase,battery,reason=fields[14:17]; openings=fields[17:21]; flags,failed,channels=fields[21:]
        if channels>15 or flags & ~511 or any((u>10000 if channels&(1<<i) else u!=65535) for i,u in enumerate(openings)):
            raise RuntimeError('Invalid airbrake sample')
        if any(math.isinf(value) for value in values): raise RuntimeError('Invalid packed sample value')
        rows.append([stamp,*[None if math.isnan(value) else value for value in values],phase,None if battery==65535 else battery/1000,reason,*[None if u==65535 else u/100 for u in openings],flags,failed,channels])
    if offset!=len(payload): raise RuntimeError('Extra packed sample bytes')
    return rows


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
            if cancelled and cancelled(): raise CancelledError('Download annullato')
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
        if cancelled and cancelled(): raise CancelledError('Download annullato')
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
    if info.get('sample_transport') != 'xor32-v2':
        raise ValueError('Aggiorna il firmware della scheda per usare il download ottimizzato')
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
            for key in ('gyro_bias_radps','accel_bias_mps2'):
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
    if not 0 <= info['samples'] <= 0xFFFFFFFF or not 0 <= info['blocks'] <= 16384:
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
    abort=None
    count, previous = 0, None
    if progress: progress(0,info['samples'])
    destination.parent.mkdir(parents=True, exist_ok=True)
    checkpoint()
    with destination.open('x', newline='', encoding='utf-8') as target:
        owned.append(destination)
        writer = csv.writer(target)
        writer.writerow(FIELDS)
        for block in range(info['blocks']):
            offset = 0
            while True:
                response = request(f'flights packed {sequence} {block} {offset}')
                rows = unpack_samples(response)
                total = response['block_count']
                if response.get('sequence') != sequence or response['block'] != block or response['offset'] != offset or response['first'] != count:
                    raise RuntimeError('Unexpected sample chunk identity/order')
                if not 1 <= total <= 9 or not 1 <= len(rows) <= 9 or offset+len(rows) > total:
                    raise RuntimeError('Invalid sample chunk dimensions')
                for row in rows:
                    if len(row) != len(FIELDS) or not isinstance(row[0], int) or not 0 <= row[0] <= 0xFFFFFFFF or row[15] not in range(8) or row[17] not in range(3):
                        raise RuntimeError('Invalid stored sample')
                    if previous is not None and row[0] <= previous:
                        raise RuntimeError('Stored timestamps are not increasing')
                    if row[17] and abort is None: abort=dict(t_ms=row[0],reason={1:'Velocity',2:'Tilt'}[row[17]])
                    previous = row[0]
                    writer.writerow(['nan' if value is None else value for value in row])
                    count += 1
                offset += len(rows)
                if progress: progress(count,info['samples'])
                if offset == total:
                    break
    if count != info['samples']:
        raise RuntimeError('Stored sample count mismatch; partial CSV retained')
    metadata = dict(sequence=sequence, status=STATUS[info['status']], sample_count=count,
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
