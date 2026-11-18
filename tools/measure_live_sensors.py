"""Read-only idle acquisition benchmark; never arms or commands outputs."""
import argparse
import json
from pathlib import Path
import time

from device_client import Client, open_serial


def check_idle(client):
    state = client.request('status')
    if (state['arming_state'] != 'disarmed' or state['outputs_active']
            or state['simulation_active'] or state['sensor_hil_active']
            or state['estimate_replay_active'] or state['power_mode'] != 'active'):
        raise RuntimeError(f'Expected awake, disarmed, idle board: {state}')
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 3600:
        parser.error('seconds must be between 1 and 3600')
    if args.output.exists():
        parser.error('output already exists')
    with open_serial(args.port) as link:
        client = Client(link)
        before = check_idle(client)
        samples = [client.request('sensors')]
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            time.sleep(.5)
            check_idle(client)
            samples.append(client.request('sensors'))
        after = check_idle(client)
    first, last = samples[0], samples[-1]
    elapsed = (last['uptime_ms'] - first['uptime_ms']) / 1000
    if elapsed <= 0 or any(b['uptime_ms'] <= a['uptime_ms'] for a, b in zip(samples, samples[1:])):
        raise RuntimeError('Device clock reset or invalid measurement interval')
    polls = last['idle_poll'][0] - first['idle_poll'][0]
    read_us = (last['idle_poll'][1] - first['idle_poll'][1]) & 0xffffffff
    summary = dict(seconds=elapsed,
                   imu_hz=(last['idle_samples'][0]-first['idle_samples'][0])/elapsed,
                   baro_hz=(last['idle_samples'][1]-first['idle_samples'][1])/elapsed,
                   poll_hz=polls/elapsed,
                   read_mean_us=read_us/polls if polls else None,
                   read_max_us_since_boot=last['idle_poll'][2],
                   late_gaps=last['idle_gap'][0]-first['idle_gap'][0],
                   gap_max_us_since_boot=last['idle_gap'][1],
                   dt_min_max_us_since_boot=last['idle_imu'][:2],
                   sequence_gaps=last['idle_imu'][2]-first['idle_imu'][2],
                   fifo_overruns_errors=[last['idle_imu'][3]-first['idle_imu'][3], last['idle_imu'][4]-first['idle_imu'][4]],
                   acquisition_wall_percent=read_us/(elapsed*10000),
                   observed_invalid_snapshots=sum(not s['imu_sample_valid'] or not s['baro_sample_valid'] for s in samples))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(summary=summary, before=before, after=after, samples=samples), indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
