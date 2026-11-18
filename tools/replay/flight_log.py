"""Flight log v4 decoder (docs/technical/STORAGE_FORMAT.md, "Samples and blocks v4").

The board stores and sends raw block bodies; only the PC decodes records and applies
the per-flight calibration, repeating the firmware's float32 arithmetic.
"""
import math
import struct
import zlib

MAGIC, VERSION, BODY_BYTES = 0x4B425350, 4, 1792
PAYLOAD_OFFSET, CRC_OFFSET = 64, 1788
MAX_FRAMES = (CRC_OFFSET - PAYLOAD_OFFSET) // 15
FRAME, BARO, STATUS, BATTERY, AIRBRAKE, GAP, TIME = range(1, 8)
EST_PRESENT, HISTORY, IMU_INVALID = 1, 2, 4
DELIBERATE = 1
DT_FROM_TIME = 0xFFFF
# MountingAxes in src/navigation/SensorPipeline.cpp, indexed by the recorded up axis.
MOUNTING = {'+X': (1, 2, 3), '-X': (-1, 2, -3), '+Y': (2, -1, 3), '-Y': (-2, 1, 3), '+Z': (3, 2, -1), '-Z': (-3, 2, 1)}

CSV_FIELDS = ['t_us', 'imu_seq', 'pad_history', 'gap_before',
              'raw_ax', 'raw_ay', 'raw_az', 'raw_gx', 'raw_gy', 'raw_gz',
              'accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ', 'pressure', 'temp',
              'z_est', 'vz_est', 'az_est', 'roll', 'pitch', 'yaw', 'state', 'abort_reason', 'battery_v',
              *[f'airbrake_{i}_pct' for i in range(1, 5)], 'airbrake_flags', 'airbrake_failed_conditions', 'airbrake_channel_mask']


class LogError(ValueError):
    pass


def f32(value):
    """Round to IEEE-754 binary32 like one firmware float operation."""
    return struct.unpack('<f', struct.pack('<f', value))[0]


ACCEL_MG = f32(0.976)
GRAVITY = f32(9.80665)
GYRO_MDPS = f32(70.0)
DEG_TO_RAD = f32(0.017453292519943295)


def accel_from_code(code):
    return f32(f32(f32(code * ACCEL_MG) * GRAVITY) / 1000.0)


def gyro_from_code(code):
    return f32(f32(f32(code * GYRO_MDPS) * DEG_TO_RAD) / 1000.0)


def _state(phase=0, abort=0, battery=0xFFFF, flags=0, failed=0, mask=0, openings=(0xFFFF,) * 4):
    return dict(phase=phase, abort=abort, battery=battery, flags=flags, failed=failed, mask=mask, openings=tuple(openings))


def decode_block(body):
    """Validate one 1792-byte body (header, CRC, record syntax, FF padding) and decode it."""
    if len(body) != BODY_BYTES:
        raise LogError('Block body must be 1792 bytes')
    magic, version, size = struct.unpack_from('<IHH', body, 0)
    if magic != MAGIC or version != VERSION or size != BODY_BYTES:
        raise LogError('Not a v4 flight log block')
    if struct.unpack_from('<I', body, CRC_OFFSET)[0] != zlib.crc32(body[:CRC_OFFSET]):
        raise LogError('Block CRC mismatch')
    sequence, index, address, first_time, first_seq, frames, used = struct.unpack_from('<QIIQIHH', body, 8)
    if not 1 <= frames <= MAX_FRAMES or not 1 <= used <= CRC_OFFSET - PAYLOAD_OFFSET:
        raise LogError('Invalid block dimensions')
    if any(b != 0xFF for b in body[56:PAYLOAD_OFFSET]) or any(b != 0xFF for b in body[PAYLOAD_OFFSET + used:CRC_OFFSET]):
        raise LogError('Invalid block padding')
    phase, abort, battery, flags, failed, mask = struct.unpack_from('<BBHHBB', body, 40)
    state = _state(phase, abort, battery, flags, failed, mask, struct.unpack_from('<4H', body, 48))
    if phase > 7 or abort > 2 or mask > 15:
        raise LogError('Invalid block state snapshot')
    result = dict(sequence=sequence, index=index, address=address, first_time_us=first_time,
                  first_sequence=first_seq, start=dict(state), frames=[])
    out = result['frames']
    pending = dict(gap=0, deliberate=False, baro=None)
    time_delta = None
    previous_time = previous_seq = 0
    p, end = PAYLOAD_OFFSET, PAYLOAD_OFFSET + used
    dangling = False

    def take(fmt):
        nonlocal p
        size = struct.calcsize(fmt)
        if p + size > end:
            raise LogError('Truncated record')
        values = struct.unpack_from(fmt, body, p)
        p += size
        return values

    while p < end:
        tag = body[p]; p += 1
        kind, flag = tag >> 5, tag & 31
        dangling = kind != FRAME
        if kind == STATUS:
            if flag: raise LogError('Invalid STATUS flags')
            phase, abort = take('<BB')
            if phase > 7 or abort > 2: raise LogError('Invalid STATUS values')
            state = dict(state, phase=phase, abort=abort)
        elif kind == BATTERY:
            if flag: raise LogError('Invalid BATTERY flags')
            state = dict(state, battery=take('<H')[0])
        elif kind == AIRBRAKE:
            if flag: raise LogError('Invalid AIRBRAKE flags')
            flags, failed, mask = take('<HBB')
            if mask > 15 or flags & ~511: raise LogError('Invalid AIRBRAKE values')
            openings = [0xFFFF] * 4
            for i in range(4):
                if mask & (1 << i): openings[i] = take('<H')[0]
            state = dict(state, flags=flags, failed=failed, mask=mask, openings=tuple(openings))
        elif kind == GAP:
            if flag & ~DELIBERATE: raise LogError('Invalid GAP flags')
            pending['gap'] += take('<H')[0]; pending['deliberate'] = bool(flag & DELIBERATE)
        elif kind == TIME:
            if flag or time_delta is not None: raise LogError('Invalid TIME record')
            time_delta = take('<I')[0]
        elif kind == BARO:
            if flag or pending['baro']: raise LogError('Invalid BARO record')
            age, = take('<H'); low, high = take('<HB'); temperature, = take('<h')
            pending['baro'] = (age, low | (high << 16), temperature)
        elif kind == FRAME:
            if flag & ~(EST_PRESENT | HISTORY | IMU_INVALID): raise LogError('Invalid FRAME flags')
            dt, *imu = take('<H6h')
            estimates = None
            if flag & EST_PRESENT:
                low, high = take('<Hb'); estimates = (low | (high << 16), *take('<5h'))
            if not out:
                if dt or time_delta is not None: raise LogError('First FRAME carries a time delta')
                stamp, seq = first_time, first_seq
            else:
                if (dt == DT_FROM_TIME) != (time_delta is not None): raise LogError('Inconsistent FRAME time')
                delta = time_delta if time_delta is not None else dt
                if not delta: raise LogError('Non-increasing FRAME time')
                stamp, seq = previous_time + delta, previous_seq + 1 + pending['gap']
            if len(out) == MAX_FRAMES: raise LogError('Too many frames')
            out.append(dict(time_us=stamp, sequence=seq, gap=pending['gap'], deliberate=pending['deliberate'],
                            history=bool(flag & HISTORY), imu_valid=not flag & IMU_INVALID, imu=tuple(imu),
                            estimates=estimates, baro=pending['baro'], state=dict(state)))
            pending = dict(gap=0, deliberate=False, baro=None); time_delta = None
            previous_time, previous_seq = stamp, seq
        else:
            raise LogError('Unknown record type')
    if len(out) != frames or dangling:
        raise LogError('Block record count mismatch')
    return result


class Calibration:
    """Rebuild calibrated BODY values from stored IMU values (firmware float32 order)."""

    def __init__(self, frame, calibration):
        self.sensor_axes = frame == 0
        if not self.sensor_axes:
            return
        if not calibration or not calibration.get('sensor_axes'):
            raise LogError('Sensor-axis flight without a calibration snapshot')
        self.mounting = MOUNTING[calibration['up_axis']]
        self.accel_bias = [f32(v) for v in calibration['accel_bias_mps2']]
        self.accel_scale = [f32(v) for v in calibration['accel_scale']]
        self.gyro_bias = [f32(v) for v in calibration['gyro_bias_radps']]

    def body(self, imu):
        if not self.sensor_axes:  # Already calibrated BODY values, quantized with the same LSBs.
            return [accel_from_code(c) for c in imu[:3]], [gyro_from_code(c) for c in imu[3:]]
        accel = [f32(f32(accel_from_code(imu[i]) - self.accel_bias[i]) * self.accel_scale[i]) for i in range(3)]
        gyro = [f32(gyro_from_code(imu[3 + i]) - self.gyro_bias[i]) for i in range(3)]
        remap = lambda vector: [math.copysign(1.0, m) * vector[abs(m) - 1] for m in self.mounting]
        return remap(accel), remap(gyro)


def _estimate(value, unit, invalid):
    return None if value == invalid else value * unit


def csv_row(frame, calibration, sensor_axes):
    """One CSV row (CSV_FIELDS order); None means an empty cell."""
    state = frame['state']
    raw = list(frame['imu']) if frame['imu_valid'] and sensor_axes else [None] * 6
    if frame['imu_valid']:
        accel, gyro = calibration.body(frame['imu'])
    else:
        accel = gyro = [None] * 3
    pressure = temperature = None
    if frame['baro']:
        pressure = frame['baro'][1] / 64 / 100  # 1/64 Pa -> hPa
        temperature = _estimate(frame['baro'][2], 0.01, -32768)
    estimates = [None] * 6
    if frame['estimates']:
        altitude, velocity, vertical, roll, pitch, yaw = frame['estimates']
        estimates = [_estimate(altitude, 0.01, -8388608), _estimate(velocity, 0.02, -32768), _estimate(vertical, 0.01, -32768),
                     _estimate(roll, 0.01, -32768), _estimate(pitch, 0.01, -32768), _estimate(yaw, 0.01, -32768)]
    z, vz, az, roll, pitch, yaw = estimates
    openings = [None if u == 0xFFFF else u / 100 for u in state['openings']]
    return [frame['time_us'], frame['sequence'], int(frame['history']), frame['gap'], *raw, *accel, *gyro,
            pressure, temperature, z, vz, az, roll, pitch, yaw, state['phase'], state['abort'],
            None if state['battery'] == 0xFFFF else state['battery'] / 1000, *openings,
            state['flags'], state['failed'], state['mask']]
