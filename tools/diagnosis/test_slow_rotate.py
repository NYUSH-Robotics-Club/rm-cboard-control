import collections
import struct
import threading
import unittest
from unittest.mock import patch

import protocol
import slow_rotate


class BoardFixture:
    def __init__(self, stopped, mode_readback=True):
        self.condition = threading.Condition()
        self.events = collections.deque(maxlen=1000)
        self.error = None
        self.token = 0
        self.mode = 1
        self.enabled = 0
        self.sent = []
        self.stopped = stopped
        self.mode_readback = mode_readback
        self.rotations = 0

    def event(self, kind, can_id, data, token=0):
        low, high = struct.unpack('<II', data.ljust(8, b'\0'))
        self.events.append(protocol.decode_event([len(self.events)+1, 10, kind, 1, can_id, 0,
                                                 len(data), low, high, token]))

    def request(self, command, *args):
        self.token += 1
        if command == 'read':
            rid = args[2]
            values = {7:0, 8:1, 10:self.mode, 21:12.566, 22:50., 23:5.}
            value = values[rid]
            payload = struct.pack('<HBB', 1, 0x33, rid) + struct.pack('<f' if rid >= 21 else '<I', value)
            self.event(1, 0, payload)
        elif command == 'send':
            can_id, payload = args[1], bytes.fromhex(args[3])
            self.sent.append((can_id, payload))
            self.event(3, can_id, payload, self.token)
            if can_id == 0x7ff and payload[2] == 0x55:
                if self.mode_readback: self.mode = payload[4]
            elif can_id == 1:
                self.enabled = int(payload[-1] == 0xfc)
            if can_id == 0x201 and struct.unpack('<f', payload[:4])[0] > 0:
                self.rotations += 1
                if self.rotations == 2: self.stopped.set()
            if can_id in (1, 0x201) or can_id == 0x7ff and payload[2] == 0xcc:
                self.event(1, 0, bytes([self.enabled << 4 | 1, 0x80, 0, 0x80, 8, 0, 30, 25]))
        return dict(token=self.token)


class RotationTests(unittest.TestCase):
    @patch('builtins.print')
    def test_zero_before_enable_and_stop_on_exit(self, _print):
        stopped = threading.Event()
        board = BoardFixture(stopped)
        slow_rotate.run(board, stopped)
        enable_at = board.sent.index((1, bytes([255]*7+[252])))
        self.assertIn((0x201, bytes(8)), board.sent[:enable_at])
        self.assertEqual(board.sent[-3][0], 0x201)
        self.assertEqual(board.sent[-3][1], bytes(8))
        self.assertEqual(board.sent[-2], (1, bytes([255]*7+[253])))
        self.assertEqual(board.enabled, 0)
        moving = [struct.unpack('<f', payload[:4])[0] for can_id, payload in board.sent
                  if can_id == 0x201 and payload != bytes(8)]
        self.assertEqual(len(moving), 2)
        self.assertTrue(all(abs(v-.1) < 1e-6 for v in moving))

    @patch('builtins.print')
    def test_failed_mode_readback_never_enables(self, _print):
        stopped = threading.Event()
        board = BoardFixture(stopped, mode_readback=False)
        with self.assertRaises(RuntimeError): slow_rotate.run(board, stopped)
        self.assertNotIn((1, bytes([255]*7+[252])), board.sent)
        self.assertEqual(board.enabled, 0)


if __name__ == '__main__': unittest.main()
