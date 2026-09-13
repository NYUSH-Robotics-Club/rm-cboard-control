"""Test snapshot validity, units and the running-target read-only connection path."""
import contextlib
import io
from pathlib import Path
import socket
import struct
import tempfile
import unittest
from unittest.mock import patch

import gimbal_monitor as monitor


def packet(sequence=2, tick=100, flags=31, unit=1, status=0, current=-1234):
    header = monitor.HEADER.pack(sequence, monitor.MAGIC, 1, monitor.SIZE, 7, tick, 4, 1, 1)
    yaw = monitor.AXIS.pack(5, flags, (tick-3) & 0xffffffff, unit, status, current, -1200,
                            8194.0, 8200.0, -6.0, -5.0, -6.0, .004, 2)
    pitch = monitor.AXIS.pack(8, 31, tick-1 if tick else 0, 2, 0, 600, 500,
                              1900, 1971, 0, 10, 0, .004, 1900)
    return header+yaw+pitch


class SnapshotTests(unittest.TestCase):
    def test_signed_feedback_and_continuous_position(self):
        sample = monitor.decode_snapshot(packet(), 2, 105)
        self.assertEqual(sample['yaw']['current_actual_raw'], -1200)
        self.assertEqual(sample['yaw']['current_target_raw'], -1234)
        self.assertEqual(sample['yaw']['position_actual_ticks'], 8194)
        self.assertEqual(sample['yaw']['encoder_raw'], 2)
        self.assertEqual(sample['yaw']['feedback_age_ms'], 8)
        self.assertEqual(sample['snapshot_age_ms'], 5)

    def test_voltage_and_unknown_never_imply_target_current(self):
        for unit in (0, 2, 99):
            sample = monitor.decode_snapshot(packet(unit=unit), 2, 105)
            self.assertIsNone(sample['yaw']['current_target_raw'])
        self.assertIsNone(sample['pitch']['current_target_raw'])
        self.assertEqual(sample['pitch']['command_raw'], 600)

    def test_failed_service_request_is_not_current_target(self):
        sample = monitor.decode_snapshot(packet(status=-4), 2, 105)
        self.assertIsNone(sample['yaw']['current_target_raw'])
        self.assertEqual(sample['yaw']['command_raw'], -1234)

    def test_seqlock_rejects_torn_and_busy(self):
        self.assertIsNone(monitor.decode_snapshot(packet(), 4, 105))
        self.assertIsNone(monitor.decode_snapshot(packet(sequence=3), 3, 105))
        self.assertIsNone(monitor.decode_snapshot(bytes(monitor.SIZE), 0, 0))

    def test_incompatible_and_nonfinite_data_fail(self):
        bad = bytearray(packet())
        struct.pack_into('<I', bad, 8, 99)
        with self.assertRaises(monitor.fw.Failure):
            monitor.decode_snapshot(bad, 2, 105)
        bad = bytearray(packet())
        struct.pack_into('<f', bad, 36+28, float('nan'))
        with self.assertRaises(monitor.fw.Failure):
            monitor.decode_snapshot(bad, 2, 105)
        with self.assertRaises(monitor.fw.Failure):
            monitor.decode_snapshot(b'bad', 2, 105)

    def test_time_wrap_and_missing_feedback(self):
        sample = monitor.decode_snapshot(packet(tick=0xfffffff0), 2, 5)
        self.assertEqual(sample['snapshot_age_ms'], 21)
        self.assertEqual(sample['yaw']['feedback_age_ms'], 24)
        sample = monitor.decode_snapshot(packet(flags=1), 2, 105)
        self.assertIsNone(sample['yaw']['feedback_age_ms'])

    def test_display_inactive_targets_and_stale_data(self):
        sample = monitor.decode_snapshot(packet(flags=7), 2, 1000)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            monitor.display(sample, 20, 3, Path('record.csv'))
        value = out.getvalue()
        self.assertIn('快照已过期', value)
        self.assertIn('目标无效', value)
        self.assertIn('未闭合位置环', value)
        self.assertIn('电压命令请求 600', value)
        self.assertEqual(monitor.flatten(sample)['yaw_speed_actual_rpm'], -6)

    def test_rpc_handles_fragmented_reply(self):
        class FakeSocket:
            def __init__(self):
                self.sent = b''
                self.parts = iter([b'0xff ', b'0x12', b'\x1a'])
            def sendall(self, data):
                self.sent = data
            def recv(self, count):
                return next(self.parts)
        sock = FakeSocket()
        self.assertEqual(monitor.TclClient(sock).words(0x20000000, 2), [255, 18])
        self.assertEqual(sock.sent, b'read_memory 0x20000000 32 2\x1a')

    def test_rpc_read_error_and_disconnect(self):
        class Broken:
            def sendall(self, data): pass
            def recv(self, count): return b''
        with self.assertRaises(monitor.fw.Failure):
            monitor.TclClient(Broken()).words(0, 1)
        with patch.object(monitor.TclClient, 'call', return_value='target not examined'):
            with self.assertRaises(monitor.fw.Failure):
                monitor.TclClient(None).words(0, 1)

    def test_readonly_verification_and_mismatch(self):
        class FakeTarget:
            state = 'running'
            def __init__(self, directory, data):
                self.commands = []
                self.directory, self.data = directory, data
            def words(self, address, count):
                assert (address, count) == (0xe0042000, 1)
                return [0x413]
            def call(self, command):
                self.commands.append(command)
                if command == 'stm32f4x.cpu curstate': return self.state
                if command.startswith('read_memory'): return '1024'
                if command.startswith('dump_image'):
                    (self.directory/'flash.bin').write_bytes(self.data)
                    return ''
                raise AssertionError(command)
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            target = FakeTarget(directory, b'image')
            monitor.verify_image_readonly(target, b'image', directory)
            self.assertTrue(any(c.startswith('dump_image ') for c in target.commands))
            for forbidden in ('verify_image', 'checksum', 'halt', 'reset', 'resume', 'write_memory', 'mww'):
                self.assertFalse(any(forbidden in c for c in target.commands))
            with self.assertRaises(monitor.fw.Failure):
                monitor.verify_image_readonly(target, b'wrong', directory)
            target.state = 'halted'
            target.commands = []
            with self.assertRaises(monitor.fw.Failure):
                monitor.verify_image_readonly(target, b'image', directory)
            self.assertEqual(target.commands, ['stm32f4x.cpu curstate'])

    def test_server_has_no_work_ram_or_debugger_port(self):
        command = monitor.openocd_command('openocd', 'ABC123', 4567)
        options = ' '.join(command)
        self.assertIn('-event examine-end {} -work-area-size 0', options)
        self.assertIn('gdb_port disabled', options)
        self.assertIn('bindto 127.0.0.1', options)
        self.assertNotIn('flash_checked', options)
        with self.assertRaises(monitor.fw.Failure):
            monitor.openocd_command('openocd', 'x; reset', 4567)

    def test_argument_validation(self):
        for args in (['--hz', 'nan'], ['--hz', '0'], ['--duration', '-1']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                monitor.main(args)


if __name__ == '__main__':
    unittest.main()
