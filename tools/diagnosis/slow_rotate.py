"""Run the confirmed CAN1 J3507 at +0.1 rad/s over RTT until Ctrl-C.

Reads back IDs, mode and feedback ranges before enabling. Mode changes are
volatile; no parameter-save command is sent. One process owns RTT channel 0.
"""
import os
import signal
import struct
import time
from threading import Event

from interface import Board
from protocol import decode_feedback

SPEED_RAD_S = 0.1


class RotationSession:
    def __init__(self, board):
        self.board = board
        self.limits = None
        self.feedback = None
        self.feedback_at = 0.0
        self.cursor = 0

    def last_sequence(self):
        with self.board.condition:
            return max((e['seq'] for e in self.board.events), default=0)

    def wait(self, predicate, timeout=.5):
        deadline = time.monotonic() + timeout
        with self.board.condition:
            while True:
                if self.board.error:
                    raise RuntimeError(self.board.error)
                match = next((e for e in reversed(self.board.events) if predicate(e)), None)
                if match:
                    return match
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('No fresh CAN completion/reply within 0.5 s')
                self.board.condition.wait(remaining)

    def send(self, can_id, payload):
        reply = self.board.request('send', 1, can_id, 0, payload.hex())
        event = self.wait(lambda e: e['token'] == reply['token'] and
                          e['kind'] in ('TX_OK', 'TX_ERROR', 'TX_TIMEOUT'))
        if event['kind'] != 'TX_OK':
            raise RuntimeError('CAN transmission failed: ' + event['kind'])

    def register(self, rid, floating=False):
        before = self.last_sequence()
        self.board.request('read', 1, 1, rid)
        event = self.wait(lambda e: e['seq'] > before and e['channel'] == 1 and
                          e['id'] == '0x000' and e.get('register_candidate', {}).get('motor_id') == 1 and
                          e['register_candidate']['register'] == rid)
        return event['register_candidate']['float32' if floating else 'uint32']

    def collect_feedback(self):
        with self.board.condition:
            events = [e for e in self.board.events if e['seq'] > self.cursor]
            if events:
                self.cursor = events[-1]['seq']
        for event in events:
            if (event['kind'] != 'RX' or event['channel'] != 1 or event['id'] != '0x000' or
                    event['flags'] or event['length'] != 8 or 'register_candidate' in event):
                continue
            data = bytes.fromhex(event['data'])
            if data[0] & 15 != 1:
                continue
            self.feedback = decode_feedback(data, self.limits)
            self.feedback_at = time.monotonic()
        return self.feedback


def run(board, stopped):
    session = RotationSession(board)
    velocity_mode = False
    try:
        board.request('config', 1, 1000000, 1)
        if session.register(8) != 1 or session.register(7) != 0:
            raise RuntimeError('Motor/feedback ID differs from the confirmed J3507')
        previous_mode = session.register(10)
        session.limits = dict(zip(('p_max', 'v_max', 't_max'),
                                  (session.register(rid, True) for rid in (21, 22, 23))))
        # Also validates that the actual ranges are finite and positive.
        decode_feedback(bytes(8), session.limits)
        print(f'PID={os.getpid()} previous_mode={previous_mode} limits={session.limits}', flush=True)
        session.send(1, bytes([255] * 7 + [253]))
        session.send(0x7ff, bytes.fromhex('01 00 55 0A 03 00 00 00'))
        if session.register(10) != 3:
            raise RuntimeError('Velocity mode readback failed; motor not enabled')
        velocity_mode = True
        session.send(0x201, struct.pack('<fI', 0.0, 0))
        session.cursor = session.last_sequence()
        if stopped.is_set():
            return
        session.send(1, bytes([255] * 7 + [252]))
        enabled_at = time.monotonic()
        next_report = enabled_at
        print(f'Enable sent; target +{SPEED_RAD_S} rad/s. Ctrl-C sends zero speed and disable.', flush=True)
        while not stopped.is_set():
            session.send(0x201, struct.pack('<fI', SPEED_RAD_S, 0))
            stopped.wait(.02)
            now = time.monotonic()
            feedback = session.collect_feedback()
            if now - max(session.feedback_at, enabled_at) > .5:
                raise RuntimeError('Motor feedback stale for more than 0.5 s')
            if feedback:
                if feedback['state'] != 1:
                    raise RuntimeError(f"Motor is not enabled/healthy: state={feedback['state']}")
                if abs(feedback['velocity_rad_s']) > 1.0:
                    raise RuntimeError('Measured speed exceeds the low-speed test bound of 1 rad/s')
                if now >= next_report:
                    print(f"position={feedback['position_rad']:.4f} rad "
                          f"velocity={feedback['velocity_rad_s']:.4f} rad/s "
                          f"torque={feedback['torque_nm']:.4f} Nm state={feedback['state']}", flush=True)
                    next_report = now + 2
    finally:
        # Attempt disable independently even if the zero-speed frame fails.
        if velocity_mode:
            try:
                session.send(0x201, struct.pack('<fI', 0.0, 0))
            except Exception as exc:
                print(f'ZERO_SPEED_FAILED: {exc}', flush=True)
        try:
            session.send(1, bytes([255] * 7 + [253]))
            print('DISABLE_TX_OK', flush=True)
            if session.limits:
                before = session.last_sequence()
                session.send(0x7ff, bytes.fromhex('01 00 CC 00 00 00 00 00'))
                session.wait(lambda e: e['seq'] > before and e['kind'] == 'RX' and
                             e['channel'] == 1 and e['id'] == '0x000' and e['length'] == 8 and
                             'register_candidate' not in e)
                feedback = session.collect_feedback()
                if not feedback or feedback['state'] != 0:
                    raise RuntimeError('Disabled state not confirmed')
                print('DISABLED_FEEDBACK_CONFIRMED', feedback, flush=True)
        except Exception as exc:
            print(f'STOP_NOT_CONFIRMED: {exc}', flush=True)


def main():
    stopped = Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    board = Board()
    try:
        run(board, stopped)
    finally:
        board.close()


if __name__ == '__main__':
    main()
