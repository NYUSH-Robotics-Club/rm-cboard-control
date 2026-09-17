"""Check real C frames against Python and browser decoders and canvas rendering.

Fixtures replace time, RTT and application sources only; no probe is opened.
"""
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from scripts.rtt_common.telemetry import FrameParser, crc16_firmware
from scripts.rtt_common.monitor_writer import MONITOR_COLUMNS, _frame_row_values

ROOT = Path(__file__).resolve().parents[1]


class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cls.directory = Path(cls.temp.name)
        includes = ['config', 'config/robots', 'core/common', 'core/contracts', 'core/motor',
                    'application/dashboard', 'application/gimbal', 'application/chassis',
                    'modules/algorithm', 'modules/message_center', 'services/motor',
                    'bsp/time', 'bsp/critical', 'third_party/SEGGER/RTT', 'third_party/SEGGER/Config']
        exe = cls.directory / 'fixture'
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-DROBOT_TYPE_infantry_standard', *['-I' + p for p in includes],
                        'application/dashboard/dashboard.c', 'modules/message_center/message_center.c',
                        'bsp/critical/bsp_critical.c', 'tests/host/test_dashboard.c',
                        '-lm', '-o', str(exe)], cwd=ROOT, check=True)
        cls.raw = subprocess.check_output([str(exe)])
        cls.frames = FrameParser().feed(cls.raw, 12345)
        cls.html = (ROOT / 'scripts/dashboard/rtt_web_viewer.html').read_text()

    def test_c_producer_units_validity_and_order(self):
        self.assertEqual(len(self.frames), 4)
        first, stale, position, dropped = self.frames
        self.assertEqual((first.version, first.payload_size), (8, 300))
        self.assertEqual(first.can_link_bitmap, 3)
        self.assertEqual(first.chassis_motor_ids, [2, 1, 4, 3])
        self.assertEqual(first.motor_rpm, [20, 10, 40, 30])
        self.assertEqual(first.motor_target_rpm, [100, -200, 300, -400])
        self.assertEqual(first.gimbal_yaw_actual_deg, 720)
        self.assertEqual(first.gimbal_yaw_actual_deg_s, 12)
        self.assertEqual(first.gimbal_yaw_target_deg_s, 24)
        self.assertTrue(math.isnan(first.gimbal_yaw_target_deg))
        self.assertEqual(first.gimbal_pitch_target_deg, 45)
        self.assertAlmostEqual(first.imu_gyro_deg_s[2], 57.29578, places=4)
        self.assertEqual(first.rc_rocker_r_x, 111)
        self.assertEqual(stale.can_link_bitmap, 0)
        self.assertEqual(stale.status_flags, 0)
        self.assertTrue(math.isnan(stale.gimbal_yaw_actual_deg))
        self.assertEqual(position.gimbal_yaw_target_deg, 360)
        self.assertEqual(dropped.telemetry_drop_count, 1)

    def test_split_stream_crc_and_legacy(self):
        for chunk_size in (1, 17, 309, 512):
            parser = FrameParser()
            frames = []
            for offset in range(0, len(self.raw), chunk_size):
                frames.extend(parser.feed(self.raw[offset:offset + chunk_size], 10))
            self.assertEqual([f.seq for f in frames], [0, 1, 2, 4])
            self.assertEqual(parser.crc_error_count, 0)
        parser = FrameParser()
        parser.feed(self.raw[:310], 1)
        corrupt = bytearray(self.raw[:310]); corrupt[100] ^= 1
        frames = parser.feed(corrupt + self.raw[:310], 2)
        self.assertEqual(len(frames), 1)
        self.assertEqual(parser.crc_error_count, 1)
        # Legacy v7 has an identical prefix, but no appended source metadata.
        legacy = bytearray(self.raw[:296]); legacy[2:4] = bytes([7, 32])
        legacy.extend(struct.pack('<H', crc16_firmware(legacy)))
        frame = FrameParser().feed(legacy, 3)[0]
        self.assertEqual(frame.version, 7)
        self.assertEqual(frame.gimbal_pitch_encoder_raw, 2048)
        self.assertEqual(frame.capabilities, 0)

    def test_saved_log_retains_validity_and_continuous_error(self):
        for frame in self.frames:
            self.assertEqual(len(MONITOR_COLUMNS), len(_frame_row_values(frame)))
        row = dict(zip(MONITOR_COLUMNS, _frame_row_values(self.frames[2])))
        self.assertEqual(row['gimbal_yaw_error_deg'], -360)
        self.assertEqual(row['chassis_motor_id_2'], 4)
        self.assertEqual(row['capabilities'], 31)

    @unittest.skipUnless(shutil.which('node'), 'Node required for browser code regression')
    def test_browser_decoder_and_canvas_use_real_source(self):
        # Execute actual page functions, not a second implementation of the ABI.
        decoder = self.html.split('              const seq = dv.getUint32(4, true);', 1)[1]
        decoder = 'const seq = dv.getUint32(4, true);' + decoder.split('              if ((store.lastSeq', 1)[0]
        draw = self.html.split('      function drawInteractiveLines(', 1)[1].split('      function LineChartPanel(', 1)[0]
        draw = 'function drawInteractiveLines(' + draw
        code = '''const assert = require('node:assert/strict');
const fs = require('node:fs');
function decode(bytes) {
 const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
 const version = dv.getUint8(2);
 const expectedPayloadSize = bytes.length - 10;
 const payloadSizeHeader = dv.getUint8(3);
''' + decoder + '''
 assert.equal(offset, bytes.length - 2);
 return latest;
}
const all = fs.readFileSync(process.argv[2]);
const first = decode(all.subarray(0,310));
assert.equal(first.capabilities,31);
assert.deepEqual(first.chassis_motor_ids,[2,1,4,3]);
assert.equal(first.gimbal_yaw_actual_deg,720);
assert.equal(first.gimbal_yaw_actual_deg_s,12);
assert.equal(first.gimbal_yaw_target_deg_s,24);
assert.equal(first.gimbal_pitch_encoder_raw,2048);
assert.equal(first.can_link_bitmap,3);
assert.equal(first.rc_rocker_r_x,111);
assert.equal(first.imu_angle_deg[1],720);
assert(Number.isNaN(first.gimbal_yaw_target_deg));
assert(Number.isNaN(decode(all.subarray(310,620)).gimbal_yaw_actual_deg));
const window = {devicePixelRatio:2};
const document = {documentElement:{}};
const getComputedStyle = () => ({getPropertyValue:()=> '#888'});
const formatWindowLabel = s => s.toFixed(1) + 's';
const calls = [];
const ctx = new Proxy({}, {get(o,k) {
 if (k in o) return o[k];
 if(k === 'measureText') return () => ({width:12});
 return (...args) => {for(const v of args) if(typeof v === 'number') assert(Number.isFinite(v)); calls.push([k,o.strokeStyle,...args]);};
}});
const canvas = {clientWidth:600,clientHeight:250,getContext:()=>ctx};
''' + draw + '''
drawInteractiveLines(canvas,[0,20,40],[{color:'red',values:[1,2,NaN]}],null,1000);
assert(calls.some(c=>c[0]==='stroke' && c[1]==='red'), 'valid segment before trailing NaN was lost');
assert.equal(calls.filter(c=>c[0]==='lineTo' && c[1]==='red').length,1);
calls.length=0;
drawInteractiveLines(canvas,[0,20,40,400],[{color:'red',values:[1,NaN,3,4]}],null,1000);
assert.equal(calls.filter(c=>c[0]==='lineTo' && c[1]==='red').length,0,'must not connect invalid samples or transport gaps');
calls.length=0;
drawInteractiveLines(canvas,[0],[{color:'red',values:[NaN]}],null,1000);
assert(calls.some(c=>c[0]==='fillText' && c[2].includes('No valid data')));
'''
        app = self.html.split('      function App() {', 1)[1].split('        return (\n          <div className="app">', 1)[0]
        code += """
const location = {href:'http://localhost:9091/',protocol:'http:'};
const localStorage = {getItem:()=>null};
const MIN_HISTORY_WINDOW_SEC=1, DEFAULT_HISTORY_WINDOW_SEC=20;
let stateIndex=0;
const useState = initial => [stateIndex++ === 3 ? true : (typeof initial === 'function' ? initial() : initial), ()=>{}];
const useEffect = ()=>{};
const useMemo = callback=>callback();
let latestForRender=first;
let age=0;
const useRef = value=> {
 if(value && value.frames === 0) { value.latest=latestForRender; value.lastFrameAt=Date.now()-age; }
 return {current:value};
};
function renderFields() {
stateIndex=0;
""" + app + """
return {can:canStatus(0), basicItems, remoteItems, imuItems};
}
let fields=renderFields();
assert.equal(fields.can,'ON');
assert.equal(fields.imuItems.find(i=>i.name==='gimbal_yaw_target_deg').value,'N/A');
assert.equal(fields.remoteItems.find(i=>i.name==='rc_rocker_r_x').value,111);
assert(fields.remoteItems.find(i=>i.name==='emergency_stop(flag2)').value.includes('N/A'));
age=1500; assert.equal(renderFields().can,'STALE');
age=0; latestForRender={...first,dashboard_version:7}; assert.equal(renderFields().can,'N/A');
latestForRender=decode(all.subarray(310,620));
assert.equal(renderFields().can,'OFF');
assert(renderFields().remoteItems.find(i=>i.name==='rc_rocker_r_x').value.includes('N/A'));
"""
        script = self.directory / 'browser-test.cjs'; script.write_text(code)
        binary = self.directory / 'frames.bin'; binary.write_bytes(self.raw)
        subprocess.run(['node', str(script), str(binary)], check=True)

    def test_bridge_injects_selected_websocket_port(self):
        from scripts.dashboard.rtt_ws_bridge import RTTWebSocketBridge, parse_args
        for options, expected in (([], 8080), (['--ws-port', '8765'], 8765), (['--http-port', '9091'], 9091)):
            with patch('sys.argv', ['bridge', '--no-monitor-save', *options]):
                args = parse_args()
            bridge = RTTWebSocketBridge(args)
            self.assertIn(('window.RTT_CONFIG=' + json.dumps({'wsPort': expected})).encode(), bridge.viewer_html)


if __name__ == '__main__':
    unittest.main()
