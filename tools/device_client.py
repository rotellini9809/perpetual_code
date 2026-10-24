"""Shared Perpetual USB/SPP protocol, connection worker and small CLI monitor."""
from concurrent.futures import Future
import json
import math
import queue
import threading
import time


class CommandError(RuntimeError):
    """The device replied with a normal command rejection; the link is healthy."""
    def __init__(self, command, response):
        self.command, self.response, self.code = command, response, response['code']
        super().__init__(f'{command}: {self.code}')


class ConnectionLost(RuntimeError):
    pass


class ProtocolError(ConnectionLost):
    pass


def positive_seconds(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError('Timeout/interval must be finite and positive')
    return float(value)


def validate_command(command):
    if not isinstance(command, str) or not command.strip(' \t') or command.lstrip(' \t').startswith('@'):
        raise ValueError('Supply one command without a request ID')
    if any((ord(c) < 32 and c != '\t') or ord(c) > 126 for c in command):
        raise ValueError('Command must be ASCII without line endings or control characters')
    if len('@4294967295 '+command) > 128:
        raise ValueError('Request exceeds board line limit (including reserved uint32 ID)')


def reject_json_constant(_):
    raise ValueError('Non-finite JSON')


class Client:
    """Synchronous protocol on a borrowed link with bounded reads/writes.

    Serializes callers and never retries. A timeout, IO or framing failure makes
    this instance unusable. CommandError retains structured firmware error data.
    DeviceConnection owns real ports; host fixtures may supply another byte link.
    """
    def __init__(self, link, *, clock=time.monotonic, stop=None):
        self.link, self.request_id = link, 0
        self.last_reply = None
        self._clock, self._stop = clock, stop
        self._lock, self._broken = threading.Lock(), False

    def request(self, command, timeout=5):
        validate_command(command)
        timeout = positive_seconds(timeout)
        with self._lock:
            if self._broken:
                raise ConnectionLost('Session is unusable; reconnect and query state before any retry')
            if self.request_id == 0xFFFFFFFF:
                raise ConnectionLost('Request IDs exhausted; reconnect before sending another command')
            wire = f'@{self.request_id+1} {command}\n'.encode('ascii')
            if len(wire)-1 > 128:
                raise ValueError('Request exceeds board line limit')
            self.request_id += 1
            deadline = self._clock()+timeout
            try:
                if self._stop and self._stop.is_set():
                    raise ConnectionLost('Connection closed')
                if self.link.write(wire) != len(wire):
                    raise ConnectionLost('Incomplete command write; outcome unknown, not retrying')
                raw = bytearray()
                while self._clock() < deadline:
                    if self._stop and self._stop.is_set():
                        raise ConnectionLost('Connection closed; request outcome may be unknown')
                    available = getattr(self.link, 'in_waiting', 0)
                    raw.extend(self.link.read(max(1, min(1025-len(raw), available))))
                    if self._stop and self._stop.is_set():
                        raise ConnectionLost('Connection closed; request outcome may be unknown')
                    if self._clock() >= deadline:
                        break
                    if len(raw) > 1024:
                        raise ProtocolError('Oversized reply')
                    if b'\n' not in raw:
                        continue
                    if raw[-1:] != b'\n' or raw.count(b'\n') != 1:
                        raise ProtocolError('Unexpected data after reply; reconnect')
                    # Also detect already-buffered extra lines with fragmented reads.
                    if getattr(self.link, 'in_waiting', 0):
                        raise ProtocolError('Unexpected buffered data after reply; reconnect')
                    try:
                        response = json.loads(raw.decode('utf-8'), parse_constant=reject_json_constant)
                    except (ValueError, UnicodeError) as error:
                        raise ProtocolError('Malformed JSON reply') from error
                    expected = command.split()[0]
                    if not isinstance(response, dict) or type(response.get('id')) is not int or response['id'] != self.request_id:
                        raise ProtocolError('Unexpected reply ID; reconnect, do not replay requests automatically')
                    if type(response.get('ok')) is not bool or response.get('type') != ('reply' if response['ok'] else 'error'):
                        raise ProtocolError('Invalid reply envelope')
                    if response.get('command', expected if not response['ok'] else None) != expected:
                        raise ProtocolError('Unexpected reply command')
                    if not response['ok'] and (not isinstance(response.get('code'), str) or not response['code']):
                        raise ProtocolError('Invalid error reply')
                    if not response['ok'] and response['code'] in ('ID_CONFLICT', 'STALE_REQUEST'):
                        raise ProtocolError('Device request-ID session disagrees; reconnect and query state')
                    self.last_reply = self._clock()
                    if not response['ok']:
                        raise CommandError(command, response)
                    return response
                raise TimeoutError(f'No reply for {command}; outcome unknown, request was not retried')
            except CommandError:
                raise
            except BaseException:
                self._broken = True
                raise


def open_serial(port):
    """Same PC port settings for USB and outgoing SPP; module UART stays 9600."""
    import serial
    link = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=2)
    try:
        link.port = port
        link.dtr = False
        link.open()
        link.dtr = True
        return link
    except BaseException:
        link.close()
        raise


class DeviceConnection:
    """One worker owns the port, bounded FIFO and idle heartbeat.

    connect()/submit() return Futures for a future GUI. request() and the context
    manager are blocking conveniences for scripts. Reconnect is explicit and
    never resubmits commands. Future callbacks can run on the worker; marshal to UI.
    """
    def __init__(self, port, *, opener=open_serial, heartbeat_interval=2, reply_timeout=3, settle=.15):
        self.port, self._opener = port, opener
        self._interval, self._timeout = positive_seconds(heartbeat_interval), positive_seconds(reply_timeout)
        if not isinstance(settle, (int, float)) or not math.isfinite(settle) or settle < 0:
            raise ValueError('Port settling delay must be finite and nonnegative')
        self._settle = settle
        self._lock = threading.Lock()
        self._thread = None
        self._state, self._error, self._pending = 'disconnected', None, None
        self._last_reply, self._info, self._transport = None, None, None
        self._stop = threading.Event()
        self._jobs = queue.Queue(maxsize=32)
        self._ready, self._closed = Future(), Future()
        self._closed.set_result(None)

    def snapshot(self):
        with self._lock:
            age = None if self._last_reply is None else max(0, time.monotonic()-self._last_reply)
            state = self._state
            if state == 'connected' and age is not None and age > self._interval+self._timeout:
                state = 'unresponsive'
            return dict(state=state, port=self.port, transport=self._transport, pending=self._pending,
                        reply_age_s=age, error=self._error, info=dict(self._info) if self._info else None)

    def connect(self):
        with self._lock:
            if self._thread is not None or not self._closed.done():
                raise RuntimeError('Disconnect this session before connecting again')
            self._stop.clear()
            self._jobs = queue.Queue(maxsize=32)
            self._ready, self._closed = Future(), Future()
            self._state, self._error, self._pending = 'connecting', None, None
            self._last_reply, self._info, self._transport = None, None, None
            self._thread = threading.Thread(target=self._run, name='Perpetual connection', daemon=True)
            self._thread.start()
            ready = self._ready
        return ready

    def submit(self, command, timeout=5):
        validate_command(command)
        timeout = positive_seconds(timeout)
        future = Future()
        with self._lock:
            if self._state not in ('connected', 'waiting') or self._stop.is_set():
                raise ConnectionLost('Connection is not ready')
            try:
                self._jobs.put_nowait((future, command, timeout))
            except queue.Full as error:
                raise RuntimeError('Command queue is full') from error
        return future

    def request(self, command, timeout=5):
        if threading.current_thread() is self._thread:
            raise RuntimeError('Use submit(), not blocking request(), in a Future callback')
        return self.submit(command, timeout).result()

    def disconnect(self, wait=True):
        with self._lock:
            thread = self._thread
            closed = self._closed
            if thread is None:
                self._state = 'disconnected'
                return closed
            self._state = 'disconnecting'
            self._stop.set()
            try:
                self._jobs.put_nowait(None)
            except queue.Full:
                pass
        if wait and thread is not threading.current_thread():
            thread.join(3)
            if thread.is_alive():
                raise TimeoutError('Port worker is still closing; wait for disconnect completion')
        return closed

    def _confirmed(self, client):
        with self._lock:
            self._last_reply, self._pending = client.last_reply, None
            if not self._stop.is_set():
                self._state = 'connected'

    def _run(self):
        link, current, established = None, None, False
        try:
            if not self._ready.set_running_or_notify_cancel():
                self._stop.set()
                return
            link = self._opener(self.port)
            if self._stop.wait(self._settle):
                raise ConnectionLost('Connection cancelled')
            client = Client(link, stop=self._stop)
            info = client.request('info', timeout=8)
            if info.get('device') != 'Perpetual' or type(info.get('protocol_version')) is not int or info['protocol_version'] != 1:
                raise ProtocolError('Unexpected device or unsupported protocol version')
            control = client.request('control status', timeout=self._timeout)
            if control.get('transport') not in ('usb', 'bluetooth'):
                raise ProtocolError('Missing transport identity')
            with self._lock:
                self._info, self._transport = info, control['transport']
            self._confirmed(client)
            established = True
            self._ready.set_result(dict(info))
            while not self._stop.is_set():
                due = max(0, client.last_reply+self._interval-time.monotonic())
                try:
                    job = self._jobs.get(timeout=due)
                except queue.Empty:
                    response = client.request('ping', timeout=self._timeout)
                    if response.get('message') != 'PONG':
                        raise ProtocolError('Invalid heartbeat reply')
                    self._confirmed(client)
                    continue
                if job is None:
                    break
                current, command, timeout = job
                if not current.set_running_or_notify_cancel():
                    current = None
                    continue
                with self._lock:
                    self._state, self._pending = 'waiting', command
                try:
                    response = client.request(command, timeout)
                except CommandError as error:
                    self._confirmed(client)
                    current.set_exception(error)
                else:
                    self._confirmed(client)
                    current.set_result(response)
                current = None
        except Exception as error:
            with self._lock:
                self._state = 'unresponsive' if established else 'error'
                self._error, self._pending = str(error), None
            if current is not None and not current.done():
                current.set_exception(error)
            if not self._ready.done():
                self._ready.set_exception(error)
        finally:
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass
            # Change state before draining so concurrent submitters cannot add work.
            with self._lock:
                if self._stop.is_set():
                    self._state = 'disconnected'
                elif self._state not in ('error', 'unresponsive'):
                    self._state = 'unresponsive'
                self._pending = None
            while True:
                try:
                    job = self._jobs.get_nowait()
                except queue.Empty:
                    break
                if job is not None and job[0].set_running_or_notify_cancel():
                    job[0].set_exception(ConnectionLost('Connection closed before this command was sent'))
            with self._lock:
                self._thread = None
            self._closed.set_result(None)

    def __enter__(self):
        try:
            self.connect().result()
        except BaseException:
            self.disconnect()
            raise
        return self

    def __exit__(self, *_):
        self.disconnect()


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='USB or outgoing Bluetooth SPP port')
    parser.add_argument('--command', default='status', help='One command after handshake (default: status)')
    parser.add_argument('--watch', nargs='?', const=math.inf, type=float, default=0, metavar='SECONDS', help='Observe heartbeat state until Ctrl+C, or for SECONDS')
    args = parser.parse_args()
    if math.isnan(args.watch) or args.watch < 0:
        parser.error('--watch must be nonnegative')
    validate_command(args.command)
    with DeviceConnection(args.port) as device:
        print(json.dumps(device.snapshot()), flush=True)
        print(json.dumps(device.request(args.command)), flush=True)
        until = time.monotonic()+args.watch
        while time.monotonic() < until:
            time.sleep(.5)
            snapshot = device.snapshot()
            print(json.dumps(snapshot), flush=True)
            if snapshot['state'] in ('error', 'unresponsive'):
                raise ConnectionLost(snapshot['error'] or 'No recent device reply')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        raise SystemExit(str(error)) from error
