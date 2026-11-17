"""Local ST-Link/CAN diagnostic interface. Python standard library only.

OpenOCD owns the probe and exposes RTT channel 0 as a bidirectional TCP stream.
This client never reads/writes target memory or halts/resets/flashes the board.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import secrets
import socket
import struct
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from . import protocol as p
except ImportError:
    import protocol as p


def elf_address(elf, nm):
    output = subprocess.check_output([nm, '-n', str(elf)], text=True)
    found = [int(row.split()[0], 16) for row in output.splitlines()
             if len(row.split()) == 3 and row.split()[2] == '_SEGGER_RTT']
    if len(found) != 1 or not 0x20000000 <= found[0] < 0x20020000:
        raise ValueError('ELF has no valid _SEGGER_RTT SRAM symbol')
    return found[0]


class Board:
    """An event stream over RTT channel 0; never reads target memory directly."""
    def __init__(self, port=9090):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=3)
        self.sock.settimeout(.2)
        self.condition = threading.Condition()
        self.lock = threading.RLock()
        self.events = collections.deque(maxlen=1000)
        self.replies = {}
        self.latest = None
        self.error = None
        self.closed = False
        self.token = secrets.randbelow(0x40000000) + 1
        self.received_at = time.monotonic()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        try:
            hello = self.request('hello')
            if hello.get('protocol') != 2:
                raise RuntimeError('RTT protocol mismatch: flash the matching firmware')
            self.request('status')
        except Exception:
            self.close()
            raise

    def _read(self):
        buffer = bytearray()
        try:
            while not self.closed:
                try:
                    data = self.sock.recv(8192)
                except socket.timeout:
                    continue
                if not data:
                    raise RuntimeError('RTT disconnected; reconnect without blindly retrying commands')
                buffer.extend(data)
                if len(buffer) > 65536:
                    raise RuntimeError('Invalid RTT record length')
                while b'\n' in buffer:
                    raw, _, remaining = buffer.partition(b'\n')
                    buffer = bytearray(remaining)
                    record = json.loads(raw)
                    with self.condition:
                        self.received_at = time.monotonic()
                        kind = record.get('type')
                        if kind == 'event':
                            words = record['words']
                            if len(words) != 10 or any(type(v) is not int or not 0 <= v <= 0xffffffff for v in words):
                                raise ValueError('Malformed RTT CAN event')
                            self.events.append(p.decode_event(words))
                        elif kind in ('reply', 'status', 'hello'):
                            if kind == 'status':
                                if record.get('protocol') != 2 or len(record['bus']) != 2 or any(len(b) != 16 for b in record['bus']):
                                    raise ValueError('Malformed RTT status')
                                if self.latest and record['uptime_ms'] < self.latest['uptime_ms']:
                                    self.events.clear()
                                self.latest = record
                            self.replies[record['token']] = record
                            # Bound unsolicited replies from manual terminal users.
                            while len(self.replies) > 32:
                                self.replies.pop(next(iter(self.replies)))
                        self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.condition.notify_all()

    def request(self, command, *args):
        with self.lock:
            self.token = (self.token + 1) & 0xffffffff or 1
            token = self.token
            text = ' '.join(map(str, (command, token, *args))) + '\n'
            if len(text) > 255 or any(c in text[:-1] for c in '\r\n\0'):
                raise ValueError('Invalid RTT command')
            with self.condition:
                if self.error:
                    raise RuntimeError(self.error)
                self.sock.sendall(text.encode('ascii'))
                deadline = time.monotonic() + 3
                while token not in self.replies:
                    if self.error:
                        raise RuntimeError(self.error)
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('No RTT reply; do not retry a motor command blindly')
                    self.condition.wait(remaining)
                result = self.replies.pop(token)
                code = result.get('result', 0)
                if code:
                    raise RuntimeError(p.RESULTS[code] if code < len(p.RESULTS) else 'unknown firmware error')
                return result

    def state(self):
        with self.condition:
            if self.error:
                raise RuntimeError(self.error)
            words = self.latest
            buses = [dict(zip(p.BUS_FIELDS, b)) for b in words['bus']]
            for bus in buses:
                esr = bus['esr']
                bus.update(tec=(esr >> 16) & 255, rec=(esr >> 24) & 255,
                           lec=(esr >> 4) & 7, bus_off=bool(esr & 4), passive=bool(esr & 2))
            return dict(demo=False, transport='RTT', uptime_ms=words['uptime_ms'], loops=words['loops'],
                        status_age_s=round(time.monotonic()-self.received_at, 2),
                        ack=words['token'], result='RTT connected', buses=buses,
                        events=list(self.events), lost=words['rtt_dropped'])

    def action(self, body):
        kind = body['action']
        channel = p.number(body.get('channel', 1))
        if channel not in (1, 2):
            raise ValueError('CAN channel must be 1 or 2')
        with self.lock:
            if kind == 'status':
                response = self.request('status')
            elif kind == 'configure':
                bitrate, mode = p.number(body['bitrate']), p.number(body['mode'])
                if bitrate not in (125000, 200000, 250000, 500000, 1000000) or mode not in (0, 1, 2):
                    raise ValueError('Unsupported bitrate or mode')
                response = self.request('config', channel, bitrate, mode)
            elif kind == 'read':
                p.read_register(body['motor_id'], body['register'])
                response = self.request('read', channel, p.number(body['motor_id']), p.number(body['register']))
            else:
                if kind == 'raw':
                    if body.get('acknowledge') is not True:
                        raise ValueError('Confirm that raw frames can move or reconfigure the motor')
                    can_id, payload, flags = p.frame(body['id'], body['data'], body.get('flags', 0))
                elif kind == 'disable':
                    can_id, payload, flags = p.frame(body['id'], bytes([255]*7+[253]))
                else:
                    raise ValueError('Unknown action')
                response = self.request('send', channel, can_id, flags, payload.hex() or '-')
            if kind != 'status':
                self.request('status')
            return dict(token=response['token'], result='RTT command accepted; inspect CAN completion and reply separately')

    def close(self):
        self.closed = True
        self.sock.close()
        if threading.current_thread() is not self.reader:
            self.reader.join(timeout=1)


class DemoBoard:
    """Deterministic UI fixture, completely disconnected from any probe."""
    def state(self):
        bus = dict.fromkeys(p.BUS_FIELDS, 0)
        bus.update(bitrate=1000000, ready=1, mode=0, tec=0, rec=0, lec=0, bus_off=False, passive=False)
        return dict(demo=True, uptime_ms=1200, loops=600, ack=0, result='demo: no hardware',
                    buses=[bus, bus.copy()], events=[], lost=0)

    def action(self, body):
        return dict(result='DEMO: action simulated; no hardware connected')


def clean_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean_json(item) for item in value]
    return value


def make_handler(board, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def reply(self, status, data, content_type='application/json'):
            raw = json.dumps(clean_json(data), allow_nan=False).encode() if content_type == 'application/json' else data
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def authorized(self):
            return self.headers.get('X-Diagnosis-Token') == token

        def do_GET(self):
            if self.path == '/':
                html = Path(__file__).with_name('index.html').read_bytes().replace(b'__TOKEN__', token.encode())
                self.reply(200, html, 'text/html; charset=utf-8')
            elif self.path == '/api/state' and self.authorized():
                try:
                    self.reply(200, board.state())
                except Exception as exc:
                    self.reply(503, dict(error=str(exc)))
            else:
                self.reply(404, dict(error='Not found'))

        def do_POST(self):
            # Token and exact local Origin prevent unrelated websites from
            # commanding the board through a browser's loopback connection.
            origin = self.headers.get('Origin')
            expected = f'http://127.0.0.1:{self.server.server_port}'
            if self.path != '/api/action' or not self.authorized() or origin not in (None, expected):
                return self.reply(403, dict(error='Unauthorized request'))
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request length')
                body = json.loads(self.rfile.read(length))
                self.reply(200, board.action(body))
            except Exception as exc:
                self.reply(400, dict(error=str(exc)))
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rtt-port', type=int, default=9090)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--demo', action='store_true')
    args = parser.parse_args()
    live_board = None
    if args.demo:
        board = DemoBoard()
    else:
        board = live_board = Board(args.rtt_port)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(board, secrets.token_hex(24)))
    print(f'Diagnosis: http://127.0.0.1:{server.server_port}' + (' [DEMO]' if args.demo else ''), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if live_board:
            live_board.close()


if __name__ == '__main__':
    main()
