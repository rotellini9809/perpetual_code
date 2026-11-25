"""Read recorded CSV/metadata and prepare bounded plot data, without device operations."""
from pathlib import Path
import csv
import json
import math

PHASES = ['PAD', 'BOOST', 'COAST', 'DESCENT', 'LANDED', 'BOOST 2', 'COAST 2', 'ABORT']


def phase_name(value,bistage=False): return (PHASES[value]+' 1' if bistage and value in (1,2) else PHASES[value]) if type(value) is int and 0<=value<len(PHASES) else '—'


def load_flight(path):
    from .orientation import euler_x_up
    path = Path(path)
    rows = []; phases=[]; air_rows=[]; air_transitions=[]; air_mask=0; previous_air=None; total=0; first=None; last=None
    fields=('z_est','vz_est','az_est','roll','pitch','yaw','gyroX','gyroY','gyroZ','battery_v')
    def number(row,key):
        try:
            value=float(row.get(key,''))
            # CSV gyro fields are BODY rad/s; convert once for the degree/s plot.
            if key.startswith('gyro'): value=math.degrees(value)
        except (TypeError,ValueError): return None
        return value if math.isfinite(value) else None
    with path.open(newline='',encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        if not {'t_us','z_est','vz_est'} <= set(reader.fieldnames or []): raise ValueError('Incompatible CSV: t_us, z_est and vz_est are required (flight log v4)')
        previous = -1
        for row in reader:
            stamp = int(row['t_us'])
            if stamp <= previous: raise ValueError('CSV times are not increasing')
            previous = stamp; total+=1
            if first is None: first=stamp
            last=stamp
            phase=row.get('state') or ''
            phase=int(phase) if phase.isdigit() and 0<=int(phase)<len(PHASES) else None
            values=[number(row,key) for key in fields]
            # Stored Euler fields are already degrees in NAV Z-up. Re-express
            # the entire rotation in X-up before decimation, preserving gaps.
            values[3:6]=euler_x_up(*values[3:6])
            rows.append((stamp/1e6,*values))
            openings=[number(row,f'airbrake_{i}_pct') for i in range(1,5)]
            if 'airbrake_flags' in row:
                flags=int(row['airbrake_flags']); failed=int(row['airbrake_failed_conditions']); mask=int(row['airbrake_channel_mask'])
                if not 0<=flags<=511 or not 0<=failed<=255 or not 0<=mask<=15 or any(opening is not None and not 0<=opening<=100 for opening in openings):
                    raise ValueError('Invalid airbrake data in the CSV')
                air_mask|=mask; air_rows.append((stamp/1e6,*openings))
                observed=(flags,failed,mask)
                if observed!=previous_air:
                    air_transitions.append(dict(time=stamp/1e6,phase=phase,flags=flags,failed=failed,channel_mask=mask)); previous_air=observed
            name=PHASES[phase] if phase is not None else None
            if not phases or phases[-1][1]!=name: phases.append((stamp/1e6,name))
    valid=[r for r in rows if r[1] is not None and r[2] is not None]
    if not valid: raise ValueError('No valid samples in the CSV')
    metadata = {}
    if path.with_suffix('.json').exists(): metadata=json.loads(path.with_suffix('.json').read_text(encoding='utf-8'))
    if (metadata.get('calibration') or {}).get('bistage'):
        phases=[(stamp,{'BOOST':'BOOST 1','COAST':'COAST 1'}.get(name,name)) for stamp,name in phases]
    step = max(1,math.ceil(len(rows)/1200))
    plot=[]
    for i in range(0,len(rows),step):
        # Preserve gaps even when invalid readings fall between displayed points.
        bucket=rows[max(0,i-step+1):i+1]
        plot.append((rows[i][0],*(rows[i][column] if all(r[column] is not None for r in bucket) else None for column in range(1,len(fields)+1))))
    if (len(rows)-1)%step: plot.append(rows[-1])
    air_config=metadata.get('configuration',{}).get('airbrakes',{})
    air_mask|=air_config.get('channel_mask',0)
    air_step=max(1,math.ceil(len(air_rows)/1200))
    air_plot=air_rows[::air_step]
    if air_rows and (len(air_rows)-1)%air_step: air_plot.append(air_rows[-1])
    air_phases=[]
    for event in air_transitions:
        name='Control enabled' if event['flags']&2 else 'ABORT' if event['flags']&16 else 'Control not active' # Translated when drawn.
        if not air_phases or air_phases[-1][1]!=name: air_phases.append((event['time'],name))
    airbrakes=dict(rows=air_plot,transitions=air_transitions,phases=air_phases,channel_mask=air_mask)
    simulated_battery = metadata.get('sensor_mode') == 1
    if simulated_battery:
        # Illustrative 2S trace, separate from CSV battery_v and never exported as a measurement.
        duration=max(.001,(last-first)/1e6)
        plot=[(*r,round(8.4-.2*(r[0]-first/1e6)/duration,3)) for r in plot]
    return dict(path=str(path),rows=plot,phases=phases,airbrakes=airbrakes,samples=total,valid_samples=len(valid),duration=(last-first)/1e6,
                max_altitude=max(r[1] for r in valid),max_speed=max(abs(r[2]) for r in valid),metadata=metadata,simulated_battery=simulated_battery)


