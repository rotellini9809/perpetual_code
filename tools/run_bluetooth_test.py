"""Test MAIN with --main, or probe BLUETOOTH_TEST; optionally use a paired SPP port."""
import argparse
import json
import re
from pathlib import Path
import time
import uuid

import serial
import device_client as transport


def check_main(usb_port, radio_port, report):
    """Exercise production protocol and RAM-only edits; never apply/write settings."""
    report['firmware'] = 'MAIN'
    with serial.Serial(usb_port, 115200, timeout=.1, write_timeout=2) as board:
        usb = transport.Client(board)
        deadline = time.monotonic()+8
        while True:
            status = usb.request('bluetooth status')
            if status['ready']:
                break
            if status['error'] != 'STARTING' or time.monotonic() >= deadline:
                raise RuntimeError('Bluetooth startup failed: '+str(status))
            time.sleep(.1)
        report['uart_verified'] = True
        if usb.request('control status')['owner'] != 'none':
            raise RuntimeError('An edit/run is already owned; finish it before testing')
        view = usb.request('config show')
        if view['draft_active']:
            raise RuntimeError('An existing draft must be finished before testing')
        with serial.Serial(radio_port, 115200, timeout=.1, write_timeout=2) as radio:
            bt = transport.Client(radio)
            info = bt.request('info',timeout=8)
            if info.get('device') != 'Perpetual':
                raise RuntimeError('Unexpected radio endpoint: '+str(info))
            report['radio_info'] = info
            if bt.request('control status')['transport'] != 'bluetooth':
                raise RuntimeError('Radio is not using the Bluetooth command session')
            if not usb.request('bluetooth status')['connected']:
                raise RuntimeError('Radio connection notification was not observed')
            owned = False
            try:
                started = bt.request('config begin')
                owned = True
                # Exact latest-ID retry must return the same response, without reexecuting.
                repeated = json.loads(request(radio, f'@{bt.request_id} config begin', '{', timeout=8))
                if repeated != started:
                    raise RuntimeError('Bluetooth retry response changed')
                try:
                    usb.request('config discard')
                except RuntimeError as error:
                    if str(error) != 'config discard: CONTROL_BUSY':
                        raise
                else:
                    raise RuntimeError('USB changed the Bluetooth-owned draft')
                if not usb.request('config show')['draft_active']:
                    raise RuntimeError('Bluetooth draft disappeared')
                # Observer USB loss must not discard the radio owner's draft.
                board.close(); time.sleep(.2); board.open(); time.sleep(.2)
                usb = transport.Client(board)
                if not usb.request('config show')['draft_active']:
                    raise RuntimeError('USB disconnect discarded the Bluetooth draft')
                # Module notifications must be recognized even after partial input.
                radio.write(b'@500 config discard')
                radio.flush()
                time.sleep(.15) # Allow the unfinished bytes to reach the module UART.
                radio.close()
                deadline = time.monotonic()+10
                while usb.request('bluetooth status')['connected']:
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Bluetooth disconnect was not detected')
                    time.sleep(.1)
                if usb.request('config show')['draft_active']:
                    raise RuntimeError('Owner disconnect retained its draft')
                owned = False
                radio.open()
                bt = transport.Client(radio)
                if bt.request('ping',timeout=8).get('message') != 'PONG':
                    raise RuntimeError('Bluetooth reconnect/ID reset failed')
                report['session_checks'] = ['shared protocol', 'independent IDs', 'exact retry', 'foreign mutation rejected',
                                            'USB observer disconnect', 'partial-command radio disconnect', 'reconnect']
                report['radio_verified'] = True
            finally:
                if owned and radio.is_open:
                    bt.request('config discard')
        report['final_status'] = usb.request('bluetooth status')
    print('MAIN USB/Bluetooth protocol, ownership and reconnect PASSED; no settings saved.', flush=True)


def wait_line(port, prefix, timeout=3):
    deadline = time.monotonic() + timeout
    pending = bytearray()
    while time.monotonic() < deadline:
        raw = port.read_until(b'\n', 2048)
        if not raw:
            continue
        pending.extend(raw)
        if len(pending) > 2048:
            raise RuntimeError('Oversized diagnostic reply')
        if not pending.endswith(b'\n'):
            continue
        line = pending.decode('ascii', errors='backslashreplace').strip()
        pending.clear()
        print(line, flush=True)
        if line.startswith(prefix):
            return line
    raise RuntimeError(f'Timed out waiting for {prefix!r}; select/upload BLUETOOTH_TEST first.')


def request(port, text, prefix, timeout=3):
    port.write((text + '\n').encode('ascii'))
    return wait_line(port, prefix, timeout)


def query(port, text):
    line = request(port, 'at ' + text, 'AT_RESULT ')
    match = re.fullmatch(r'AT_RESULT ok=([01]) command=(AT(?:\+[A-Z]+)?) rx_bytes=(\d+)', line)
    if not match or match[2] != text:
        raise RuntimeError('Malformed or mismatched AT result: ' + line)
    return {'ok': match[1] == '1', 'result': line}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM9', help='Board USB serial port')
    parser.add_argument('--radio-port', help='Optional paired Bluetooth SPP COM port, distinct from USB')
    parser.add_argument('--main', action='store_true', help='Test the shared MAIN protocol, ownership and reconnect (RAM-only draft)')
    parser.add_argument('--at-gate', choices=('low','high'), default='low', help='PWRC level during queries; high permits testing alternative EN routing')
    parser.add_argument('--output', type=Path, help='JSON report (default: .pio/bluetooth_<id>.json)')
    args = parser.parse_args()
    if args.radio_port and args.radio_port.casefold() == args.port.casefold():
        parser.error('--radio-port must be distinct from the USB port')
    if args.main and not args.radio_port:
        parser.error('--main requires --radio-port')
    report = {'usb_port': args.port, 'at_gate': args.at_gate, 'uart_verified': False, 'radio_verified': False, 'attempts': [], 'queries': {}}
    output = args.output or Path(__file__).resolve().parents[1] / '.pio' / f'bluetooth_{uuid.uuid4().hex}.json'
    if output.exists():
        parser.error('Report already exists: '+str(output))
    try:
        if args.main:
            check_main(args.port,args.radio_port,report)
            return 0
        with serial.Serial(args.port, 115200, timeout=.1, write_timeout=2) as board:
            report['initial_status'] = request(board, 'status', 'BT_TEST status ')
            request(board, 'gate ' + args.at_gate, 'AT_GATE ')
            for baud in (230400, 600000, 9600, 115200, 19200, 38400, 57600, 4800):
                request(board, f'baud {baud}', 'LOCAL_BAUD ')
                answer = query(board, 'AT')
                report['attempts'].append({'baud': baud, **answer})
                if answer['ok']:
                    report['uart_verified'] = True
                    report['baud'] = baud
                    break
            if not report['uart_verified']:
                # Leave the diagnostic at its documented default after a failed scan.
                request(board, 'baud 9600', 'LOCAL_BAUD ')
                report['final_status'] = request(board, 'status', 'BT_TEST status ')
                raise RuntimeError('No matching AT reply at the tested rates. Check UART wiring, power and soldering; radio connection is unverified.')
            for command in ('AT+VERSION', 'AT+NAME', 'AT+NAMB', 'AT+BAUD', 'AT+STAT', 'AT+BTMODE'):
                report['queries'][command] = query(board, command)
            if args.radio_port:
                # This tests the wireless route, not a USB loopback or UART AT acknowledgment.
                with serial.Serial(args.radio_port, 115200, timeout=.1, write_timeout=2) as radio:
                    request(radio, 'ping', 'PONG', timeout=8)
                    token = 'BT_TEST_' + uuid.uuid4().hex
                    response = request(radio, 'echo ' + token, 'ECHO ', timeout=8)
                    if response != 'ECHO ' + token:
                        raise RuntimeError('Bluetooth echo payload mismatch')
                    report['radio_verified'] = True
                    report['radio_port'] = args.radio_port
            report['final_status'] = request(board, 'status', 'BT_TEST status ')
            print('UART/module probe PASSED. ' + ('Wireless ping/echo PASSED.' if report['radio_verified'] else
                  'Wireless ping/echo NOT YET VERIFIED: connect a phone and send ping, then echo <text>.'))
    except (OSError, RuntimeError, ValueError, serial.SerialException) as error:
        report['error'] = str(error)
        print('FAILED: ' + str(error), flush=True)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as target:
            json.dump(report, target, indent=2)
        print(f'Report: {output}')
    return 1 if 'error' in report else 0


if __name__ == '__main__':
    raise SystemExit(main())
