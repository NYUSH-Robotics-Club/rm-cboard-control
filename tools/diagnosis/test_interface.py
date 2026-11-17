import json
import struct
import shutil
import subprocess
import socket
import tempfile
import time
from pathlib import Path
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import interface as ui
import protocol as p


class Memory:
    def __init__(self):
        self.words = [0] * p.WORDS
        self.words[:3] = [p.MAGIC, 1, p.WORDS*4]
        self.writes = []

    def read(self, word, count):
        return self.words[word:word+count]

    def write(self, word, values):
        self.writes.append((word, list(values)))
        self.words[word:word+len(values)] = values
        if word == 48:
            self.words[7] = values[0]  # Firmware acknowledges after commit.


class ProtocolTests(unittest.TestCase):
    def test_read_register_wire_format(self):
        self.assertEqual(p.read_register('0x123', '0x08'), (0x7ff, bytes.fromhex('23 01 33 08'), 0))

    def test_frame_boundaries(self):
        self.assertEqual(p.frame('0x7ff', '')[1], b'')
        self.assertEqual(p.frame('0x1fffffff', '00 '*8, 1)[0], 0x1fffffff)
        for args in [(0x800, ''), (-1, ''), (1, '00 '*9), (1, 'xx'), (1, '', -1)]:
            with self.assertRaises(ValueError): p.frame(*args)

    def test_commit_last_and_no_pending_overwrite(self):
        mem = Memory()
        board = ui.Board(mem)
        result = board.action(dict(action='read', channel=2, motor_id=1, register=8))
        self.assertEqual(result['token'], 1)
        self.assertEqual(mem.writes[0][0], 49)
        self.assertEqual(mem.writes[1], (48, [1]))
        self.assertEqual(mem.words[49:58], [2, 2, 0, 0, 0x7ff, 0, 4, 0x08330001, 0])
        mem.words[48] = 10
        with self.assertRaises(RuntimeError): board.request(2, 1)
        self.assertEqual(len(mem.writes), 2)

    def test_fail_closed_raw_and_bad_abi(self):
        mem = Memory()
        board = ui.Board(mem)
        with self.assertRaises(ValueError): board.action(dict(action='raw', id=1, data='FF'))
        self.assertEqual(mem.writes, [])
        mem.words[1] = 999
        with self.assertRaises(RuntimeError): board.state()

    def test_ring_overwrite_and_reset(self):
        mem = Memory()
        board = ui.Board(mem)
        mem.words[6] = 140
        for seq in range(13, 141):
            offset = 64 + (seq-1) % 128 * 10
            mem.words[offset:offset+10] = [seq, 20, 1, 1, 0, 0, 8, 0x08330001, 4, 0]
        state = board.state()
        self.assertEqual(state['lost'], 12)
        self.assertEqual(len(state['events']), 128)
        self.assertEqual(state['events'][-1]['register_candidate']['register'], 8)
        mem.words[6] = 0
        self.assertEqual(board.state()['events'], [])

    def test_register_reply_not_implicit_feedback(self):
        event=p.decode_event([1, 4, 1, 1, 0, 0, 8, 0x08330001, 1, 0])
        self.assertEqual(event['register_candidate']['motor_id'], 1)
        self.assertNotIn('position_rad', event)
        limits=dict(p_max=12.5, v_max=30, t_max=10)
        feedback=p.decode_feedback(bytes.fromhex('10 80 00 80 08 00 20 21'), limits)
        self.assertAlmostEqual(feedback['position_rad'], 0, delta=.001)
        self.assertEqual(feedback['state'], 1)
        with self.assertRaises(ValueError): p.decode_feedback(bytes(8), dict(limits, p_max=float('nan')))

    def test_uncommitted_overwrite_of_oldest_record_is_rejected(self):
        mem=Memory()
        mem.words[6]=128
        for seq in range(1,129):
            offset=64+(seq-1)*10
            mem.words[offset:offset+10]=[seq,1,1,1,1,0,0,0,0,0]
        original=mem.read
        def read(word,count):
            result=original(word,count)
            if word==64 and count==1280:
                mem.words[64]=0  # Writer started slot 1 but has not advanced head.
            return result
        mem.read=read
        state=ui.Board(mem).state()
        self.assertEqual(state['lost'],1)
        self.assertEqual(len(state['events']),127)
        self.assertEqual(state['events'][0]['seq'],2)

    def test_nonfinite_register_is_valid_json(self):
        self.assertEqual(json.dumps(ui.clean_json(dict(value=float('nan'))), allow_nan=False), '{"value": null}')


class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=ThreadingHTTPServer(('127.0.0.1', 0), ui.make_handler(ui.DemoBoard(), 'secret'))
        cls.thread=threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url=f'http://127.0.0.1:{cls.server.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown(); cls.server.server_close(); cls.thread.join()

    def test_page_and_authenticated_state(self):
        with urlopen(self.url) as response:
            self.assertIn(b'CAN Diagnosis', response.read())
        req=Request(self.url+'/api/state', headers={'X-Diagnosis-Token':'secret'})
        with urlopen(req) as response: self.assertTrue(json.load(response)['demo'])

    def test_cross_origin_and_missing_token_rejected(self):
        for headers in ({}, {'X-Diagnosis-Token':'secret', 'Origin':'https://example.com'}):
            req=Request(self.url+'/api/action', data=b'{"action":"disable"}', headers=headers)
            with self.assertRaises(HTTPError) as error: urlopen(req)
            self.assertEqual(error.exception.code, 403)
            error.exception.close()


@unittest.skipUnless(shutil.which('openocd'), 'OpenOCD not installed')
class FlashTests(unittest.TestCase):
    def test_real_openocd_tcl_transport_without_probe(self):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            port=listener.getsockname()[1]
        # Substitute only SRAM commands; no interface, target or init command
        # is configured. This verifies Jim Tcl syntax and TCP framing itself.
        script='''
set memory(0) 1145651527
set memory(1) 1
proc read_memory {address width count} {
    set result {}
    for {set n 0} {$n < $count} {incr n} {
        set offset [expr {($address - 0x20000000) / 4 + $n}]
        if {![info exists ::memory($offset)]} {error "unmapped memory"}
        lappend result $::memory($offset)
    }
    return $result
}
proc write_memory {address width values} {
    set offset [expr {($address - 0x20000000) / 4}]
    foreach value $values {set ::memory($offset) $value; incr offset}
}
'''
        with tempfile.TemporaryFile() as log:
            process=subprocess.Popen(['openocd','-c','noinit','-c',f'bindto 127.0.0.1; tcl_port {port}; telnet_port disabled; gdb_port disabled', '-c',script], stdout=log, stderr=log)
            memory=None
            try:
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    try:
                        memory=ui.TclMemory(port, 0x20000000)
                        break
                    except OSError: time.sleep(.03)
                self.assertIsNotNone(memory, 'Mock OpenOCD Tcl server did not start')
                self.assertEqual(memory.read(0,2), [p.MAGIC,1])
                memory.write(20, [1,0xffffffff,0x08330001])
                self.assertEqual(memory.read(20,3), [1,0xffffffff,0x08330001])
                with self.assertRaises(RuntimeError): memory.read(50,1)
            finally:
                if memory: memory.close()
                process.terminate()
                process.wait(timeout=5)

    def test_checked_flash_failure_paths_without_hardware(self):
        root=Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            elf=Path(directory)/'fixture.elf'
            elf.write_bytes(b'test fixture, never programmed')
            for scenario in ['success', 'init-failure', 'read-failure', 'wrong-id', 'wrong-size',
                             'changed-id', 'reset-failure', 'write-failure', 'verify-failure']:
                command=['openocd', '-c', 'noinit', '-c', f'set SCENARIO {scenario}',
                         '-f', str(root/'tools/openocd/test_flash_mock.tcl'),
                         '-f', str(root/'tools/openocd/flash_checked.tcl'),
                         '-c', f'set FW_ELF {{{elf}}}; set FW_RUN_AFTER 1; fw_program']
                result=subprocess.run(command, capture_output=True, text=True, timeout=10)
                output=result.stdout+result.stderr
                if scenario=='success':
                    self.assertEqual(result.returncode, 0, output)
                    self.assertIn('FIRMWARE_FLASH_OK', output)
                else:
                    self.assertNotEqual(result.returncode, 0, output)
                    self.assertNotIn('FIRMWARE_FLASH_OK', output)
                    self.assertNotIn('MOCK_RESET_run', output)
                    if scenario not in ('write-failure', 'verify-failure'):
                        self.assertNotIn('MOCK_WRITE', output)


if __name__ == '__main__': unittest.main()
