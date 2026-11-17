"""Local ST-Link/CAN diagnostic interface. Python standard library only.

OpenOCD owns the probe. This client only reads/writes the live SRAM ABI; it
never halts, resets, flashes, resumes, or writes peripheral registers.
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


class TclMemory:
    def __init__(self, port, address):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=3)
        self.address = address

    def command(self, command):
        self.sock.sendall(command.encode('ascii') + b'\x1a')
        data = bytearray()
        while not data.endswith(b'\x1a'):
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError('OpenOCD disconnected; restart the interface after reconnecting')
            data.extend(chunk)
        response = data[:-1].decode()
        if response.startswith('ERROR:'):
            raise RuntimeError(response[6:])
        return response

    def read(self, word, count):
        addr = self.address + word * 4
        response = self.command(f'if {{[catch {{read_memory {addr} 32 {count}}} result]}} {{set result "ERROR:$result"}}; set result')
        values = [int(v, 0) for v in response.split()]
        if len(values) != count:
            raise RuntimeError('Incomplete OpenOCD memory read')
        return values

    def write(self, word, values):
        data = ' '.join(str(int(v)) for v in values)
        self.command(f'if {{[catch {{write_memory {self.address + word * 4} 32 {{{data}}}}} result]}} {{set result "ERROR:$result"}}; set result')

    def close(self):
        self.sock.close()


def elf_address(elf, nm):
    output = subprocess.check_output([nm, '-n', str(elf)], text=True)
    found = [int(row.split()[0], 16) for row in output.splitlines()
             if len(row.split()) == 3 and row.split()[2] == 'g_diagnosis']
    if len(found) != 1 or not 0x20000000 <= found[0] < 0x20020000 - p.WORDS * 4:
        raise ValueError('ELF has no valid g_diagnosis SRAM symbol')
    return found[0]


class Board:
    def __init__(self, memory):
        self.memory = memory
        self.cursor = 0
        self.previous_ms = None
        self.events = collections.deque(maxlen=1000)
        self.lost = 0
        self.lock = threading.RLock()
        self.snapshot()

    def snapshot(self):
        for _ in range(30):
            words = self.memory.read(0, 48)
            if words[:3] != [p.MAGIC, p.ABI, p.WORDS * 4]:
                raise RuntimeError('Firmware ABI mismatch: flash the matching Diagnosis ELF first')
            if words[3] & 1:
                time.sleep(0.001)
                continue
            if self.memory.read(3, 1)[0] == words[3]:
                return words
        raise RuntimeError('No stable snapshot; target may be halted or SWD too slow')

    def state(self):
        with self.lock:
            words = self.snapshot()
            if self.previous_ms is not None and (words[4] < self.previous_ms or words[6] < self.cursor):
                self.events.clear()
                self.cursor = 0
                self.lost = 0
            self.previous_ms = words[4]
            head = words[6]
            self.lost += max(0, head - self.cursor - 128)
            start = max(self.cursor + 1, head - 127)
            if start <= head:
                # Copy the ring once. A second head read proves no slot in the
                # selected range was overwritten while SWD copied its payload.
                ring = self.memory.read(64, 1280)
                after = self.memory.read(6, 1)[0]
                for seq in range(start, head + 1):
                    slot = (seq - 1) % 128 * 10
                    event = ring[slot:slot + 10]
                    # The oldest slot can already be invalidated by the next
                    # write before event_head advances. Recheck its commit word.
                    at_risk = after - seq == 127
                    valid_commit = not at_risk or self.memory.read(64 + slot, 1)[0] == seq
                    if after < head or after - seq >= 128 or event[0] != seq or not valid_commit:
                        self.lost += 1
                    else:
                        self.events.append(p.decode_event(event))
                self.cursor = head
            buses = [dict(zip(p.BUS_FIELDS, words[16+i*16:32+i*16])) for i in range(2)]
            for bus in buses:
                esr = bus['esr']
                bus.update(tec=(esr >> 16) & 255, rec=(esr >> 24) & 255,
                           lec=(esr >> 4) & 7, bus_off=bool(esr & 4), passive=bool(esr & 2))
            return dict(demo=False, uptime_ms=words[4], loops=words[5], ack=words[7],
                        result=p.RESULTS[words[8]] if words[8] < len(p.RESULTS) else 'unknown',
                        buses=buses, events=list(self.events), lost=self.lost)

    def request(self, op, channel, arg0=0, arg1=0, can_id=0, flags=0, payload=b''):
        if channel not in (1, 2):
            raise ValueError('CAN channel must be 1 or 2')
        with self.lock:
            words = self.snapshot()
            # Reconnects never overwrite a command already committed by another
            # host. Exactly one host owns this mailbox at a time.
            old = self.memory.read(48, 1)[0]
            if old != words[7]:
                raise RuntimeError('Previous request still pending; is the target running?')
            seq = (old + 1) & 0xffffffff or 1
            low, high = struct.unpack('<II', payload.ljust(8, b'\0'))
            self.memory.write(49, [op, channel, arg0, arg1, can_id, flags, len(payload), low, high] + [0] * 6)
            self.memory.write(48, [seq])
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                words = self.snapshot()
                if words[7] == seq:
                    if words[8]:
                        raise RuntimeError(p.RESULTS[words[8]])
                    return dict(token=seq, result='accepted; inspect TX_OK / TX_ERROR and motor reply separately')
                time.sleep(.01)
            raise TimeoutError('No firmware acknowledgement; do not retry blindly. Read status first.')

    def action(self, body):
        kind = body['action']
        channel = p.number(body.get('channel', 1))
        if kind == 'configure':
            bitrate, mode = p.number(body['bitrate']), p.number(body['mode'])
            if bitrate not in (125000, 200000, 250000, 500000, 1000000) or mode not in (0, 1, 2):
                raise ValueError('Unsupported bitrate or mode')
            return self.request(1, channel, bitrate, mode)
        if kind == 'read':
            can_id, payload, flags = p.read_register(body['motor_id'], body['register'])
        elif kind == 'raw':
            if body.get('acknowledge') is not True:
                raise ValueError('Confirm that raw frames can move or reconfigure the motor')
            can_id, payload, flags = p.frame(body['id'], body['data'], body.get('flags', 0))
        elif kind == 'disable':
            # Use an explicitly supplied mode-specific command ID; do not guess.
            can_id, payload, flags = p.frame(body['id'], bytes([255]*7 + [253]))
        else:
            raise ValueError('Unknown action')
        return self.request(2, channel, can_id=can_id, flags=flags, payload=payload)


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
    parser.add_argument('--elf', type=Path)
    parser.add_argument('--nm', default='arm-none-eabi-nm')
    parser.add_argument('--tcl-port', type=int, default=6666)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--demo', action='store_true')
    args = parser.parse_args()
    memory = None
    if args.demo:
        board = DemoBoard()
    else:
        if not args.elf:
            parser.error('--elf is required unless --demo is selected')
        memory = TclMemory(args.tcl_port, elf_address(args.elf, args.nm))
        board = Board(memory)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(board, secrets.token_hex(24)))
    print(f'Diagnosis: http://127.0.0.1:{server.server_port}' + (' [DEMO]' if args.demo else ''), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if memory:
            memory.close()


if __name__ == '__main__':
    main()
