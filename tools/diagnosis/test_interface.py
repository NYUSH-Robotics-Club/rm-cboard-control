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


class RttFixture:
    def __init__(self):
        self.listener=socket.socket()
        self.listener.bind(('127.0.0.1',0))
        self.listener.listen(1)
        self.port=self.listener.getsockname()[1]
        self.commands=[]
        self.connection=None
        self.thread=threading.Thread(target=self.serve,daemon=True)
        self.thread.start()

    def serve(self):
        try:
            self.connection,_=self.listener.accept()
            buffer=bytearray()
            while data:=self.connection.recv(8192):
                buffer.extend(data)
                while b'\n' in buffer:
                    raw,_,rest=buffer.partition(b'\n');buffer=bytearray(rest)
                    fields=raw.decode().split();self.commands.append(fields)
                    command,token=fields[0],int(fields[1])
                    response=dict(type='reply',token=token,result=0)
                    if command=='hello': response=dict(type='hello',token=token,protocol=2)
                    if command=='status': response=dict(type='status',token=token,protocol=2,uptime_ms=100,loops=50,event_head=1,rtt_dropped=0,bus=[[1000000,0,1]+[0]*13 for _ in range(2)])
                    records=[response]
                    if command=='read':
                        low=int(fields[3])|(0x33<<16)|(int(fields[4])<<24)
                        records.append(dict(type='event',words=[1,100,1,int(fields[2]),0,0,8,low,0,token]))
                    encoded=(''.join(json.dumps(r)+'\n' for r in records)).encode()
                    self.connection.sendall(encoded[:7])
                    self.connection.sendall(encoded[7:])
        except OSError:
            pass

    def close(self):
        if self.connection:
            try:self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            self.connection.close()
        self.listener.close();self.thread.join(timeout=1)


class ProtocolTests(unittest.TestCase):
    def test_read_register_wire_format(self):
        self.assertEqual(p.read_register('0x123','0x07'),(0x7ff,bytes.fromhex('23 01 33 07 00 00 00 00'),0))

    def test_frame_boundaries(self):
        self.assertEqual(p.frame('0x7ff','')[1],b'')
        for args in [(0x800,''),(-1,''),(1,'00 '*9),(1,'xx'),(1,'',-1)]:
            with self.assertRaises(ValueError):p.frame(*args)

    def test_feedback_requires_verified_ranges(self):
        limits=dict(p_max=12.566,v_max=50,t_max=5)
        value=p.decode_feedback(bytes.fromhex('10 80 00 80 08 00 20 21'),limits)
        self.assertAlmostEqual(value['position_rad'],0,delta=.001)
        with self.assertRaises(ValueError):p.decode_feedback(bytes(8),dict(limits,p_max=float('nan')))

    def test_nonfinite_register_is_valid_json(self):
        self.assertEqual(json.dumps(ui.clean_json(dict(value=float('nan'))),allow_nan=False),'{"value": null}')


class RttTests(unittest.TestCase):
    def setUp(self):
        self.fixture=RttFixture();self.board=ui.Board(self.fixture.port)

    def tearDown(self):
        self.board.close();self.fixture.close()

    def test_fragmented_bidirectional_stream_and_cached_state(self):
        self.assertEqual(self.board.state()['transport'],'RTT')
        count=len(self.fixture.commands)
        for _ in range(10):self.board.state()
        self.assertEqual(len(self.fixture.commands),count) # HTTP status never probes SRAM.
        self.board.action(dict(action='read',channel=1,motor_id=1,register=7))
        event=self.board.state()['events'][-1]
        self.assertEqual(event['register_candidate']['register'],7)
        self.assertEqual(event['register_candidate']['motor_id'],1)
        self.assertEqual([c[0] for c in self.fixture.commands],['hello','status','read','status'])

    def test_raw_confirmation_and_command_validation(self):
        count=len(self.fixture.commands)
        with self.assertRaises(ValueError):self.board.action(dict(action='raw',id=1,data='FF'))
        with self.assertRaises(ValueError):self.board.action(dict(action='configure',channel=1,bitrate=2000000,mode=1))
        self.assertEqual(len(self.fixture.commands),count)
        self.board.action(dict(action='raw',channel=1,id='0x123',data='01 02',acknowledge=True))
        self.assertEqual(self.fixture.commands[-2][2:],['1','291','0','0102'])

    def test_disconnect_fails_without_resending(self):
        self.fixture.close();time.sleep(.05)
        with self.assertRaises((RuntimeError,OSError)):self.board.action(dict(action='read',channel=1,motor_id=1,register=7))


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
