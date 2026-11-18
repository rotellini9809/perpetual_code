"""Clients regression scenarios; run through tools/run_tests.py."""
import importlib.util
import json
from pathlib import Path
import contextlib
import io
import tempfile
import sys
import threading
import time
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from device_client import Client, CommandError, ConnectionLost, DeviceConnection, ProtocolError

def check_replay_pacing():
    ROOT = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('pacing_replay', ROOT / 'tools/replay/replay_sensors_usb.py')
    replay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(replay)


    class Clock:
        value = 10.0

        def now(self):
            return self.value

        def sleep(self, seconds):
            self.value += seconds


    clock = Clock()
    pacer = replay.ReplayPacer(200, clock.now, clock.sleep)
    sent, late = pacer.wait(0)
    assert late == 0
    clock.value += 0.001
    assert pacer.replied(sent) == 1000
    sent, late = pacer.wait(5)
    assert abs(sent-10.005) < 1e-10 and late == 0
    clock.value += 0.020
    assert pacer.replied(sent) == 20000
    _, late = pacer.wait(10)
    assert late == 15000  # Late work does not shift this or subsequent deadlines.
    sent, late = pacer.wait(30)
    assert abs(sent-10.030) < 1e-10 and late == 0
    report = pacer.report(30)
    assert report['samples'] == 4 and report['sends_late_by_period'] == 1 and report['max_send_late_us'] == 15000


    class BufferedLink:
        def __init__(self, extra=b'', size=0):
            self.data, self.extra, self.size, self.reads = b'', extra, size, 0

        @property
        def in_waiting(self):
            return len(self.data)

        def write(self, wire):
            identity = int(wire.split()[0][1:])
            self.data = json.dumps(dict(id=identity, type='reply', command='ping', ok=True, message='x'*self.size)).encode()+b'\n'+self.extra
            return len(wire)

        def read(self, count):
            self.reads += 1
            result, self.data = self.data[:count], self.data[count:]
            return result


    link = BufferedLink(size=500)
    assert replay.transport.Client(link).request('ping')['ok'] and link.reads == 1
    for link in (BufferedLink(extra=b'{}\n'), BufferedLink(size=1100)):
        try:
            replay.transport.Client(link).request('ping')
            raise AssertionError('Invalid framing was accepted')
        except RuntimeError:
            pass


    class FlowControl:
        def __init__(self):
            self.pending = iter((48, 20, 16))
            self.calls = 0

        def request(self, command):
            assert command == 'hil status'
            self.calls += 1
            return dict(recording_error='None', pending_samples=next(self.pending))


    flow = FlowControl()
    replay.wait_for_queue(flow, 64)
    assert flow.calls == 3


    class Closing:
        def __init__(self, stuck=False, error='None'):
            self.calls, self.stuck, self.error = 0, stuck, error

        def request(self, command):
            assert command == 'hil status'
            self.calls += 1
            return dict(recording_sequence='7', recording_busy=self.stuck or self.calls < 3,
                        saved_samples=11 if self.calls >= 3 else 0, recording_error=self.error)


    closing = Closing()
    final = replay.wait_for_recording(closing, dict(recording_sequence='7', recording_busy=True,
                                                   cancelled_pending=2), clock=clock.now, sleep=clock.sleep)
    assert closing.calls == 3 and final['saved_samples'] == 11 and final['cancelled_pending'] == 2
    assert not final['recording_busy']
    failed = Closing(error='Sample')
    assert replay.wait_for_recording(failed,dict(recording_sequence='7',recording_busy=True),
                                    clock=clock.now,sleep=clock.sleep)['recording_error'] == 'Sample'
    try:
        replay.wait_for_recording(Closing(stuck=True),dict(recording_sequence='7',recording_busy=True),
                                  timeout=0.01,clock=clock.now,sleep=clock.sleep)
        raise AssertionError('Stalled close accepted as complete')
    except TimeoutError:
        pass
    print('Replay pacing tests passed: absolute deadlines, lateness, bulk USB replies, framing and queue flow control.')

def check_simulation_client():
    ROOT = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('standalone_client', ROOT/'tools/replay/run_simulation_usb.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class Client:
        def __init__(self, missed=0, stuck=False, closing=0, corrupt=0):
            self.calls = []
            self.missed, self.stuck = missed, stuck
            self.closing = closing
            self.corrupt = corrupt
        def request(self, command, timeout=5):
            self.calls.append(command)
            if command == 'flights recover':
                return dict(ready=True,corrupt_flights=self.corrupt)
            if command.startswith('sim begin'):
                return dict(active=True, t_ms=0, phase=0, source='hoppino',duration_ms=32995,total_samples=6600)
            if command == 'sim status':
                busy = self.closing > 0
                self.closing -= int(busy)
                return dict(active=self.stuck, phase=3, t_ms=32995, samples=6600-self.missed,source='hoppino',duration_ms=32995,total_samples=6600,
                            output_mode='VIRTUAL', reason='completed', error='None', invalid_frames=0,
                            recording_sequence='9', recording_busy=busy, saved_samples=0 if busy else 6600-self.missed,
                            dropped_samples=0, recording_error='None')
            if command == 'sim timing':
                return dict(cycle=[6600-self.missed, 100000, 400], missed_ticks=self.missed, over_budget=0, max_lateness_us=12)
            if command == 'sim events':
                return dict(phases=[[i,i*5000] for i in range(4)], starts=[[10,'pyro',1,13000]])
            return {}

    with tempfile.TemporaryDirectory(prefix='standalone_client_') as folder:
        path = Path(folder)/'report.json'
        client = Client()
        with contextlib.redirect_stdout(io.StringIO()):
            report = module.run(client,path,True,sleep=lambda _: None)
        assert json.loads(path.read_text()) == report
        assert client.calls == ['flights recover','flights timing reset','sim begin record','sim status','sim timing','sim events','flights timing']
        closing = Client(closing=3)
        with contextlib.redirect_stdout(io.StringIO()):
            report = module.run(closing,Path(folder)/'closing.json',True,sleep=lambda _:None)
        assert closing.calls.count('sim status')==4 and report['status']['saved_samples']==6600
        assert closing.calls.index('flights timing') > max(i for i,c in enumerate(closing.calls) if c=='sim status')
        flagged=Client(corrupt=2)
        warning=io.StringIO()
        with contextlib.redirect_stdout(warning):
            module.run(flagged,Path(folder)/'corrupt_retained.json',True,sleep=lambda _:None)
        assert '2 corrupt flight(s) retained' in warning.getvalue()
        assert not any('delete' in command for command in flagged.calls)
        try:
            module.run(client,path)
            raise AssertionError('Overwrite accepted')
        except FileExistsError:
            pass
        for record in (False,True):
            client = Client(missed=1)
            failed = Path(folder)/f'failed_{record}.json'
            with contextlib.redirect_stdout(io.StringIO()):
                try:
                    module.run(client,failed,record,sleep=lambda _:None)
                    raise AssertionError('Missed deadline accepted')
                except RuntimeError:
                    assert failed.exists()
            assert all('sample' not in command for command in client.calls)
        client = Client(stuck=True)
        times = iter([0,0,70])
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                module.run(client,Path(folder)/'timeout.json',clock=lambda:next(times),sleep=lambda _:None)
                raise AssertionError('Timeout accepted')
            except TimeoutError:
                assert client.calls[-1] == 'sim stop'
    print('Standalone client passed: control-only commands, report validation, missed deadlines, timeout stop and exclusive files.')

def check_bluetooth_client():
    ROOT = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('bt_client', ROOT / 'tools/run_bluetooth_test.py')
    client = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(client)


    class Port:
        def __init__(self, radio=False, responds=True, bad_echo=False):
            self.radio, self.responds, self.bad_echo = radio, responds, bad_echo
            self.pending = bytearray()
            self.commands = []
            self.baud = 9600

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def write(self, data):
            line = data.decode('ascii').strip()
            self.commands.append(line)
            if self.radio:
                reply = 'PONG' if line == 'ping' else 'ECHO ' + ('wrong' if self.bad_echo else line[5:])
            elif line == 'status':
                reply = f'BT_TEST status baud={self.baud} rx_bytes=0'
            elif line.startswith('baud '):
                self.baud = int(line[5:])
                reply = f'LOCAL_BAUD {self.baud}'
            elif line in ('gate high','gate low'):
                reply = 'AT_GATE ' + line[5:]
            elif line.startswith('at '):
                reply = f'AT_RESULT ok={int(self.responds and self.baud == 115200)} command={line[3:]} rx_bytes=4'
            else:
                raise AssertionError(line)
            self.pending.extend((reply+'\n').encode('ascii'))
            return len(data)

        def read_until(self, *_):
            # A USB read can stop at timeout before the newline. Never parse fragments
            # as whole AT results, or a truncated success could be accepted.
            count = min(7, len(self.pending))
            chunk = bytes(self.pending[:count]); del self.pending[:count]
            return chunk


    def run(folder, responds=True, radio=False, bad_echo=False):
        board = Port(responds=responds)
        wireless = Port(radio=True, bad_echo=bad_echo)
        report = folder / 'report.json'
        args = ['run_bluetooth_test.py', '--port', 'USB_TEST', '--output', str(report)]
        if radio:
            args += ['--radio-port', 'RADIO_TEST']
        def opened(name, *a, **kw):
            return board if name == 'USB_TEST' else wireless
        with patch.object(sys, 'argv', args), patch.object(client.serial, 'Serial', side_effect=opened), contextlib.redirect_stdout(io.StringIO()):
            result = client.main()
        return result, json.loads(report.read_text()), board, wireless


    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        result, report, board, _ = run(folder)
        assert result == 0 and report['uart_verified'] and not report['radio_verified'] and report['baud'] == 115200
        assert set(report['queries']) == {'AT+VERSION', 'AT+NAME', 'AT+NAMB', 'AT+BAUD', 'AT+STAT', 'AT+BTMODE'}
        (folder/'report.json').unlink()
        result, report, board, _ = run(folder, responds=False)
        assert result == 1 and not report['uart_verified'] and not report['radio_verified']
        assert board.baud == 9600 and 'No matching AT reply' in report['error']
        (folder/'report.json').unlink()
        result, report, board, radio = run(folder, radio=True)
        assert result == 0 and report['radio_verified'] and radio.commands[0] == 'ping'
        (folder/'report.json').unlink()
        result, report, _, _ = run(folder, radio=True, bad_echo=True)
        assert result == 1 and report['uart_verified'] and not report['radio_verified'] and 'payload mismatch' in report['error']

    print('Bluetooth client passed: fragmented replies, local baud discovery/restoration, honest UART/radio distinction and verified echo payload.')

def check_shared_connection():
    class Port:
        def __init__(self, handler=None, fragment=4096):
            self.handler, self.fragment = handler, fragment
            self.data, self.commands, self.identities = bytearray(), [], []
            self.closed, self.gate, self.fail_write = False, None, False

        @property
        def in_waiting(self):
            return 0 if self.gate and not self.gate.is_set() else len(self.data)

        def write(self, wire):
            if self.fail_write:
                raise OSError('Port unplugged')
            identity, command = wire.decode('ascii').rstrip('\n').split(' ',1)
            identity = int(identity[1:]); self.commands.append(command); self.identities.append(identity)
            values = {'info':dict(device='Perpetual',protocol_version=1),
                      'control status':dict(transport='bluetooth',owner='none'), 'ping':dict(message='PONG')}
            response = dict(id=identity,type='reply',command=command.split()[0],ok=True,**values.get(command,{}))
            if self.handler:
                response = self.handler(command,identity,response)
            if response is not None:
                self.data.extend(response if isinstance(response,bytes) else json.dumps(response).encode()+b'\n')
            return len(wire)

        def read(self, count):
            if self.gate and not self.gate.is_set():
                time.sleep(.001); return b''
            if not self.data:
                time.sleep(.001)
            n=min(count,self.fragment,len(self.data)); data=bytes(self.data[:n]); del self.data[:n]
            return data

        def close(self):
            self.closed=True

    def wait_for(test):
        until=time.monotonic()+2
        while not test():
            assert time.monotonic()<until, 'Worker did not reach the expected state'
            time.sleep(.002)

    def fails(error_type, operation):
        try:
            operation()
        except error_type as error:
            return error
        raise AssertionError(f'Expected {error_type.__name__}')

    port=Port(fragment=1); client=Client(port)
    assert client.request('ping')['message']=='PONG' and port.identities==[1]
    for bad in ('', ' ', '@1 ping', 'ping\nconfig apply', 'ping\r', 'ping\0', 'n\xe8', 'x'*118):
        fails(ValueError,lambda:client.request(bad))
    for timeout in (0,-1,float('nan'),float('inf'),True):
        fails(ValueError,lambda:client.request('ping',timeout))
    assert port.identities==[1]
    client.request_id=0xFFFFFFFF
    fails(ConnectionLost,lambda:client.request('ping')); assert port.identities==[1]

    def reject(command,identity,response):
        return dict(id=identity,type='error',command='config',ok=False,code='CONTROL_BUSY',owner='usb') if command=='config begin' else response
    port=Port(reject); client=Client(port)
    error=fails(CommandError,lambda:client.request('config begin'))
    assert error.code=='CONTROL_BUSY' and error.response['owner']=='usb'
    assert client.request('ping')['message']=='PONG' and client.last_reply is not None

    for raw in (b'[]\n', b'{bad}\n', b'{"id":1,"ok":NaN}\n', b'x'*1025+b'\n', b'{}\n{}\n'):
        port=Port(lambda *_, raw=raw:raw); client=Client(port)
        fails(ProtocolError,lambda:client.request('ping'))
        fails(ConnectionLost,lambda:client.request('ping')); assert port.commands==['ping']
    for patch_values in ({'id':True},{'id':1.0},{'id':2},{'type':'event'},{'ok':1},{'command':'info'},
                         {'ok':False,'type':'error','code':''},{'ok':False,'type':'error','code':'ID_CONFLICT'},
                         {'ok':False,'type':'error','code':'STALE_REQUEST'}):
        port=Port(lambda command,identity,response:dict(response,**patch_values)); client=Client(port)
        fails(ProtocolError,lambda:client.request('ping'))
        assert port.commands==['ping']
    port=Port(lambda *_:None); client=Client(port)
    fails(TimeoutError,lambda:client.request('config apply',.02))
    fails(ConnectionLost,lambda:client.request('status')); assert port.commands==['config apply']
    port=Port(); port.fail_write=True; client=Client(port)
    fails(OSError,lambda:client.request('ping'))
    fails(ConnectionLost,lambda:client.request('ping'))
    port=Port(); original=port.write
    port.write=lambda data:original(data)-1
    client=Client(port); fails(ConnectionLost,lambda:client.request('ping'))
    fails(ConnectionLost,lambda:client.request('ping')); assert len(port.commands)==1

    # A reply arriving after its deadline is not accepted as success.
    ticks=iter((0,0,.2)); port=Port(); client=Client(port,clock=lambda:next(ticks))
    fails(TimeoutError,lambda:client.request('ping',.1))

    ports=[]
    def open_port(_):
        port=Port(); ports.append(port); return port
    connection=DeviceConnection('SPP_TEST',opener=open_port,settle=0,heartbeat_interval=.03,reply_timeout=.1)
    with connection:
        assert connection.snapshot()['transport']=='bluetooth'
        port=ports[-1]; gate=threading.Event()
        def slow(command,identity,response):
            if command=='slow': port.gate=gate
            return response
        port.handler=slow
        first=connection.submit('slow',timeout=.5)
        wait_for(lambda:'slow' in port.commands)
        callback_done=threading.Event()
        def callback(_):
            fails(RuntimeError,lambda:connection.request('ping'))
            callback_done.set()
        first.add_done_callback(callback)
        cancelled=connection.submit('cancelled'); assert cancelled.cancel()
        last=connection.submit('status')
        time.sleep(.05)
        assert port.commands==['info','control status','slow'] and not last.done()
        assert connection.snapshot()['state']=='waiting'
        gate.set(); first.result(1); last.result(1)
        assert callback_done.wait(1)
        assert port.commands[:4]==['info','control status','slow','status']
        port.handler=reject
        fails(CommandError,lambda:connection.request('config begin'))
        assert connection.snapshot()['state']=='connected'
        assert connection.request('ping')['message']=='PONG'
        wait_for(lambda:port.commands.count('ping')>=2)
        assert connection.snapshot()['reply_age_s']<.15
    assert port.closed and connection.snapshot()['state']=='disconnected'
    with connection:
        assert ports[-1].identities[:2]==[1,2] # Explicit reconnect starts a new session.
    assert all(p.closed for p in ports)

    port=Port(lambda command,identity,response:None if command=='lost' else response)
    connection=DeviceConnection('USB_TEST',opener=lambda _:port,settle=0)
    connection.connect().result(1)
    lost=connection.submit('lost',timeout=.02); queued=connection.submit('never_sent')
    fails(TimeoutError,lambda:lost.result(1)); fails(ConnectionLost,lambda:queued.result(1))
    wait_for(lambda:port.closed)
    assert connection.snapshot()['state']=='unresponsive' and port.commands==['info','control status','lost']
    fails(ConnectionLost,lambda:connection.submit('ping'))
    connection.disconnect()

    # A full queue is bounded; disconnect completes all queued Futures without sending them.
    port=Port(); connection=DeviceConnection('USB_TEST',opener=lambda _:port,settle=0)
    connection.connect().result(1); port.gate=threading.Event()
    running=connection.submit('blocked'); wait_for(lambda:'blocked' in port.commands)
    queued=[connection.submit('queued') for _ in range(32)]
    fails(RuntimeError,lambda:connection.submit('overflow'))
    closing=connection.disconnect(wait=False); closing.result(1)
    fails(ConnectionLost,lambda:running.result(1))
    for future in queued: fails(ConnectionLost,lambda:future.result(1))
    assert port.commands==['info','control status','blocked'] and port.closed

    for key,value in (('device','Other'),('protocol_version',2),('protocol_version',True)):
        port=Port(lambda command,identity,response:dict(response,**{key:value}) if command=='info' else response)
        connection=DeviceConnection('OTHER',opener=lambda _:port,settle=0)
        fails(ProtocolError,lambda:connection.connect().result(1)); wait_for(lambda:port.closed)
        assert connection.snapshot()['state']=='error' and port.commands==['info']
        connection.disconnect()
    port=Port(lambda command,identity,response:dict(response,message='wrong') if command=='ping' else response)
    connection=DeviceConnection('SPP_TEST',opener=lambda _:port,settle=0,heartbeat_interval=.02)
    connection.connect().result(1); wait_for(lambda:port.closed)
    assert connection.snapshot()['state']=='unresponsive' and port.commands==['info','control status','ping']
    connection.disconnect()

    # Opening a Bluetooth port can be delayed by the OS: cancelling never sends a command later.
    gate=threading.Event(); port=Port()
    def delayed_open(_): gate.wait(1); return port
    connection=DeviceConnection('SPP_TEST',opener=delayed_open,settle=0)
    connecting=connection.connect(); closing=connection.disconnect(wait=False)
    gate.set(); closing.result(1); fails(ConnectionLost,lambda:connecting.result(1))
    assert port.closed and not port.commands
    def denied(_): raise OSError('Port busy')
    connection=DeviceConnection('BUSY',opener=denied,settle=0)
    fails(OSError,lambda:connection.connect().result(1))
    wait_for(lambda:connection.snapshot()['state']=='error')
    connection.disconnect()
    print('Shared connection passed: strict protocol, no retry/reuse after failure, handshake, FIFO/cancellation, idle heartbeat, long operations, reconnect and queued shutdown.')


def check_build_cache():
    """A header/flag change must affect the executable; failed builds cannot be reused."""
    import subprocess
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT / 'tools'))
    import host_build

    with tempfile.TemporaryDirectory(prefix='host cache ') as directory:
        root = Path(directory)
        (root / 'include').mkdir()
        header = root / 'include/calibration values.h'
        header.write_text('#define VALUE 1\n')
        source = root / 'check.cpp'
        code = '#include "calibration values.h"\n#include <cstdio>\n#ifndef EXTRA\n#define EXTRA 0\n#endif\nint main() { std::printf("%d", VALUE + EXTRA); }\n'
        source.write_text(code)

        def build(expected, changed):
            builder = host_build.HostBuild(root=root)
            executable = builder.build('check', ['check.cpp'])
            assert subprocess.check_output([str(executable)], text=True) == str(expected)
            assert builder.compiled == builder.linked == int(changed)

        build(1, True)
        build(1, False)
        header.write_text('#define VALUE 2\n')
        build(2, True)
        with patch.object(host_build, 'FLAGS', (*host_build.FLAGS, '-DEXTRA=10')):
            build(12, True)
            build(12, False)
        source.write_text('invalid C++\n')
        try:
            build(2, True)
            raise AssertionError('Invalid C++ was reused from cache')
        except RuntimeError:
            pass
        source.write_text(code)
        build(2, True)
    print('Host cache passed: skipped unchanged build, header/flag invalidation, paths with spaces and failed-build recovery.')


def check_test_selection():
    import contextlib
    import io
    from unittest.mock import patch
    import run_tests

    def plan(*arguments):
        output=io.StringIO()
        with patch.object(sys,'argv',['run_tests.py',*arguments,'--plan']), \
             patch.object(run_tests,'changed_files',return_value={'tools/run_tests.py'}), \
             patch.object(run_tests,'HostBuild',side_effect=AssertionError('A plan must not build')), \
             contextlib.redirect_stdout(output):
            assert run_tests.main()==0
        return output.getvalue().split('Plan (',1)[1].split(': ',1)[1].strip().split(', ')

    quick=plan('--quick')
    assert set(quick)==set(run_tests.CPP)|{'clients','replay'}
    assert plan('--suite','flight','flight')==['flight']
    assert plan('--area','arm_button')==['desktop/arm_button']
    release=plan('--full','--desktop')
    assert set(run_tests.RECORDED)<=set(release)
    assert all('desktop/'+area in release for area in run_tests.desktop_areas())
    for arguments in (('--quick','--full'),('--quick','--desktop'),('--quick','--suite','flight'),('--area','unknown')):
        with patch.object(sys,'argv',['run_tests.py',*arguments,'--plan']), contextlib.redirect_stderr(io.StringIO()):
            try:
                run_tests.main()
            except SystemExit as error:
                assert error.code==2
            else:
                raise AssertionError('Conflicting or unknown selection accepted')
    print('Test selection passed: quick/focused/release plans, deduplication and no build in plan mode.')


if __name__ == "__main__":
    check_replay_pacing()
    check_simulation_client()
    check_bluetooth_client()
    check_shared_connection()
    check_build_cache()
    check_test_selection()
