"""Recompute navigation from recorded BODY IMU/pressure, not recorded estimates."""
import argparse
import csv
import io
import math
from pathlib import Path
import statistics
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from host_build import HostBuild
DEFAULT_INPUT = ROOT / 'datasets/hoppino_VOLO_UFFICIALE.csv'
SOURCES = ['src/navigation/AttitudeEstimator.cpp', 'src/navigation/VerticalKalman.cpp',
           'src/navigation/FlightDetector.cpp', 'src/navigation/Navigation.cpp']


def load_sensor_samples(input_path, pressure_unit='hpa'):
    with Path(input_path).open(newline='', encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        required = {'t_ms', 'accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ', 'pressure'}
        names = reader.fieldnames or []
        if len(names) != len(set(names)) or not required <= set(names):
            raise ValueError('Missing/duplicate sensor columns')
        rows = list(reader)
    if not rows:
        raise ValueError('No recorded sensors')
    samples = []
    for index, row in enumerate(rows, 2):
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f'CSV line {index}: wrong column count')
        if not row['t_ms'].isascii() or not row['t_ms'].isdigit():
            raise ValueError(f'CSV line {index}: invalid timestamp')
        stamp = int(row['t_ms'])
        if samples and stamp <= samples[-1][0]:
            raise ValueError('Timestamps must strictly increase')
        values = [float(row[name]) for name in ('accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ', 'pressure')]
        if pressure_unit == 'hpa':
            values[-1] *= 100
        samples.append((stamp, values))
    origin = samples[0][0]
    if samples[-1][0] - origin > 0xFFFFFFFF:
        raise ValueError('Recording too long')
    reference_values = [44330 * (1 - (values[-1] / 101325) ** 0.1903)
                        for stamp, values in samples if stamp - origin <= 1000
                        and math.isfinite(values[-1]) and values[-1] > 0]
    if not reference_values:
        raise ValueError('No valid pressure in initial 1-second pad window')
    reference = statistics.mean(reference_values)
    hz = 1000 / statistics.median([samples[i][0] - samples[i-1][0] for i in range(1, len(samples))]) if len(samples) > 1 else 200
    return rows, samples, reference, hz


def replay(input_path, output_path, pressure_unit='hpa', compiler=None):
    rows, samples, reference, hz = load_sensor_samples(input_path, pressure_unit)
    origin = samples[0][0]
    output_path = Path(output_path)
    if output_path.exists():
        raise FileExistsError(output_path)
    executable = HostBuild(compiler).build('navigation_sensor_replay', [*SOURCES, 'tools/replay/navigation_sensor_replay.cpp'])
    lines = [f'{reference:.9g} {hz:.9g}']
    lines.extend(f'{stamp-origin} ' + ' '.join(f'{value:.9g}' for value in values) for stamp, values in samples)
    result = subprocess.run([str(executable)], input='\n'.join(lines)+'\n', capture_output=True, text=True, check=True)
    computed = list(csv.DictReader(io.StringIO(result.stdout)))
    if len(computed) != len(rows):
        raise RuntimeError('Navigation row count mismatch')
    fields = list(computed[0]) + ['source_t_ms', 'recorded_z_est', 'recorded_vz_est', 'recorded_legacy_state']
    with output_path.open('x', newline='', encoding='utf-8') as destination:
        writer = csv.DictWriter(destination, fieldnames=fields)
        writer.writeheader()
        for new, old in zip(computed, rows):
            new.update(source_t_ms=old['t_ms'], recorded_z_est=old.get('z_est', ''),
                       recorded_vz_est=old.get('vz_est', ''), recorded_legacy_state=old.get('state', ''))
            writer.writerow(new)
    labels = {0: 'PAD', 1: 'BOOST', 2: 'COAST', 3: 'DESCENT', 4: 'LANDED'}
    previous = None
    for row in computed:
        phase = int(row['phase'])
        if phase != previous:
            print(f'{row["t_ms"]} ms: computed {labels[phase]}')
            previous = phase
    for actual, recorded in [('z_est', 'recorded_z_est'), ('vz_est', 'recorded_vz_est')]:
        pairs = [(float(r[actual]), float(r[recorded])) for r in computed if r[recorded] and r['imu_valid'] == '1']
        errors = [a-b for a, b in pairs if math.isfinite(a) and math.isfinite(b)]
        if errors:
            print(f'{actual}: recorded-estimate RMSE {math.sqrt(sum(e*e for e in errors)/len(errors)):.3f}, max difference {max(map(abs, errors)):.3f}')
    print(f'{len(computed)} sensor samples; {sum(r["baro_updated"] == "1" for r in computed)} fresh barometer corrections.')
    print(f'Initial pad reference: {reference:.3f} m pressure altitude; nominal rate {hz:.2f} Hz.')
    print('Recorded estimates/phases are comparison columns only, never estimator inputs. No device or actuator access.')
    print(f'Navigation CSV: {output_path}')
    return computed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--pressure-unit', choices=['hpa', 'pa'], default='hpa', help='Explicit input CSV pressure unit; Hoppino is hPa')
    parser.add_argument('--compiler', default=None)
    args = parser.parse_args()
    replay(args.input, args.output or ROOT / '.pio' / f'hoppino_navigation_{uuid.uuid4().hex}.csv', args.pressure_unit, args.compiler)


if __name__ == '__main__':
    main()
