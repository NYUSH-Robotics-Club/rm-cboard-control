"""Check real C frames against Python and browser decoders and canvas rendering.

Fixtures replace time, RTT and application sources only; no probe is opened.
"""
import json
import csv
import io
import base64
import math
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from scripts.rtt_common.telemetry import FrameParser, crc16_firmware
from scripts.rtt_common.monitor_writer import MONITOR_COLUMNS, MonitorWriter, _frame_row_values, _age_ms

ROOT = Path(__file__).resolve().parents[1]


class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cls.directory = Path(cls.temp.name)
        includes = ['config', 'config/robots', 'core/common', 'core/contracts', 'core/motor',
                    'application/dashboard', 'application/gimbal', 'application/chassis',
                    'modules/algorithm', 'modules/message_center', 'modules/motor', 'services/motor',
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
        self.assertEqual(len(self.frames), 6)
        first, stale, position, dropped, disabled, negative = self.frames
        self.assertEqual((first.version, first.payload_size), (11, 552))
        self.assertEqual(first.can_link_bitmap, 3)
        self.assertEqual(first.chassis_motor_ids, [2, 1, 4, 3])
        self.assertEqual(first.motor_rpm, [20, 10, 40, 30])
        self.assertEqual(first.motor_target_rpm, [100, -200, 300, -400])
        self.assertEqual(first.chassis_pid_output, [101, -202, 303, -404])
        self.assertEqual(first.chassis_current_actual_raw, [200, 100, 400, 300])
        self.assertEqual(first.gimbal_yaw_actual_deg, 720)
        self.assertEqual(first.gimbal_yaw_actual_deg_s, 12)
        self.assertEqual(first.gimbal_yaw_target_deg_s, 24)
        self.assertTrue(math.isnan(first.gimbal_yaw_target_deg))
        self.assertEqual(first.gimbal_cmd_yaw_deg, 360)
        self.assertEqual(first.gimbal_pitch_target_deg, 45)
        self.assertAlmostEqual(first.imu_gyro_deg_s[2], 57.29578, places=4)
        self.assertEqual(first.rc_rocker_r_x, 111)
        self.assertEqual(stale.can_link_bitmap, 0)
        self.assertEqual(stale.status_flags, 0)
        self.assertTrue(math.isnan(stale.gimbal_yaw_actual_deg))
        self.assertTrue(math.isnan(stale.gimbal_cmd_yaw_deg))
        self.assertEqual(position.gimbal_yaw_target_deg, 360)
        self.assertEqual(dropped.telemetry_drop_count, 1)
        for field in ('gimbal_cmd_yaw_deg', 'gimbal_yaw_target_deg', 'gimbal_yaw_target_deg_s'):
            self.assertTrue(math.isnan(getattr(disabled, field)))
        self.assertEqual(negative.gimbal_cmd_yaw_deg, -540)
        self.assertEqual(negative.gimbal_yaw_target_deg_s, -24)
        self.assertTrue(math.isnan(negative.gimbal_yaw_target_deg))

    def test_split_stream_crc_and_legacy(self):
        for chunk_size in (1, 17, 309, 512):
            parser = FrameParser()
            frames = []
            for offset in range(0, len(self.raw), chunk_size):
                frames.extend(parser.feed(self.raw[offset:offset + chunk_size], 10))
            self.assertEqual([f.seq for f in frames], [0, 1, 2, 4, 5, 6])
            self.assertEqual(parser.crc_error_count, 0)
        parser = FrameParser()
        parser.feed(self.raw[:562], 1)
        corrupt = bytearray(self.raw[:562]); corrupt[100] ^= 1
        frames = parser.feed(corrupt + self.raw[:562], 2)
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
        for frame, reference in ((self.frames[0], 360), (self.frames[5], -540)):
            row = dict(zip(MONITOR_COLUMNS, _frame_row_values(frame)))
            self.assertEqual(row['gimbal_cmd_yaw_deg'], reference)
            self.assertTrue(math.isnan(row['gimbal_yaw_target_deg']))
            self.assertTrue(math.isnan(row['gimbal_yaw_error_deg']))

    def test_yaw_diagnostics_are_synchronous_and_saved(self):
        first, stale, _, _, disabled, _ = self.frames
        # The latest RC is +111, but this control callback consumed -660.
        self.assertEqual((first.rc_rocker_r_x, first.yaw_rc_ch0, first.yaw_route_rate), (111, -660, 1))
        self.assertEqual((first.yaw_rc_sequence, first.yaw_route_sequence, first.yaw_callback_count), (17, 122, 123))
        self.assertEqual((first.yaw_sample_ms, first.yaw_callback_dt_ms), (1000, 4))
        self.assertAlmostEqual(first.yaw_pid_dt_s, 0.004)
        self.assertEqual((first.yaw_pid_pout, first.yaw_pid_iout, first.yaw_pid_dout, first.yaw_pid_output), (240, -2, 3, 230))
        self.assertEqual((first.yaw_command_raw, first.yaw_current_actual_raw, first.yaw_command_status), (321, -1234, 0))
        self.assertEqual((first.yaw_pid_kp, first.yaw_pid_ki, first.yaw_pid_kd), (120, 1, 0.5))
        self.assertEqual((first.yaw_pid_output_max, first.yaw_pid_integral_max), (6000, 200))
        self.assertEqual(stale.yaw_diag_valid, 0)
        for name in ('yaw_rc_ch0', 'yaw_sample_ms', 'yaw_command_raw', 'yaw_pid_output', 'yaw_pid_kp'):
            self.assertTrue(math.isnan(getattr(stale, name)))
        for name in ('yaw_pid_dt_s', 'yaw_pid_output', 'yaw_mode'):
            self.assertTrue(math.isnan(getattr(disabled, name)))
        self.assertEqual(disabled.yaw_pid_kp, 120)

        path = self.directory / 'monitor.txt'
        writer = MonitorWriter(path)
        try:
            writer.write_frames(self.frames)
        finally:
            writer.close()
        with path.open() as handle:
            rows = list(csv.DictReader(handle, delimiter='\t'))
        self.assertEqual(len(rows), 6)
        self.assertNotIn(None, rows[0])
        self.assertEqual(len(MONITOR_COLUMNS), len(set(MONITOR_COLUMNS)))
        self.assertEqual(rows[0]['yaw_rc_ch0'], '-660')
        self.assertEqual(rows[0]['yaw_command_raw'], '321')
        self.assertEqual(rows[0]['yaw_current_actual_raw'], '-1234')
        self.assertEqual([rows[0][k] for k in ('yaw_rc_age_ms', 'yaw_command_age_ms', 'yaw_feedback_age_ms')], ['10', '2', '4'])
        self.assertEqual(rows[1]['yaw_command_raw'], 'nan')
        self.assertEqual(_age_ms(3, 0xFFFFFFFE), 5)

    def test_v8_and_missing_command_source_do_not_fabricate_inputs(self):
        legacy = bytearray(self.raw[:308])
        legacy[2:4] = bytes([8, 300 & 255])
        legacy.extend(struct.pack('<H', crc16_firmware(legacy)))
        frame = FrameParser().feed(legacy, 3)[0]
        self.assertEqual((frame.version, frame.payload_size, frame.capabilities), (8, 300, 31))
        self.assertEqual(frame.yaw_diag_valid, 0)
        self.assertTrue(math.isnan(frame.yaw_rc_ch0))
        self.assertTrue(math.isnan(frame.yaw_command_raw))
        row = dict(zip(MONITOR_COLUMNS, _frame_row_values(frame)))
        self.assertTrue(math.isnan(row['yaw_rc_age_ms']))
        self.assertEqual(row['rc_rocker_r_x'], 111)

        no_source = bytearray(self.raw[:562])
        struct.pack_into('<I', no_source, 8 + 300 + 4 * 4, 0)  # yaw_trace_flags
        no_source[-2:] = struct.pack('<H', crc16_firmware(no_source[:-2]))
        frame = FrameParser().feed(no_source, 4)[0]
        self.assertEqual(frame.yaw_diag_valid, 1)
        for name in ('yaw_rc_ch0', 'yaw_rc_sequence', 'yaw_route_ms'):
            self.assertTrue(math.isnan(getattr(frame, name)))
        self.assertEqual(frame.yaw_route_rate, 1)  # Startup/non-RC commands can still have a rate.

    @unittest.skipUnless(shutil.which('node'), 'Node required for browser code regression')
    def test_browser_decoder_and_canvas_use_real_source(self):
        # Execute actual page functions, not a second implementation of the ABI.
        decoder = self.html.split('              const seq = dv.getUint32(4, true);', 1)[1]
        decoder = 'const seq = dv.getUint32(4, true);' + decoder.split('              if ((store.lastSeq', 1)[0]
        draw = self.html.split('      function drawInteractiveLines(', 1)[1].split('      function LineChartPanel(', 1)[0]
        draw = 'function drawInteractiveLines(' + draw
        helpers = 'function yawTargetStatus(' + self.html.split('      function yawTargetStatus(', 1)[1].split('      function drawInteractiveLines(', 1)[0]
        series_store = 'function clearDashboardSeries(' + self.html.split('      function clearDashboardSeries(', 1)[1].split('      function formatWindowLabel(', 1)[0]
        # Use the real JSX series options so this catches accidental removal of foreground/dash.
        speed_series = self.html.split('title="Gimbal Yaw Speed (deg/s)"', 1)[1].split('series={', 1)[1].split(']}', 1)[0] + ']'
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
const first = decode(all.subarray(0,562));
assert.equal(first.capabilities,31);
assert.deepEqual(first.chassis_motor_ids,[2,1,4,3]);
assert.equal(first.gimbal_yaw_actual_deg,720);
assert.equal(first.gimbal_yaw_actual_deg_s,12);
assert.equal(first.gimbal_yaw_target_deg_s,24);
assert.equal(first.gimbal_cmd_yaw_deg,360);
assert.equal(first.gimbal_pitch_encoder_raw,2048);
assert.equal(first.can_link_bitmap,3);
assert.equal(first.rc_rocker_r_x,111);
assert.equal(first.yaw_rc_ch0,-660);
assert.equal(first.yaw_route_rate,1);
assert.equal(first.yaw_route_sequence,122);
assert.equal(first.yaw_callback_count,123);
assert.equal(first.yaw_current_actual_raw,-1234);
assert.equal(first.yaw_pid_kp,120);
assert.equal(first.imu_angle_deg[1],720);
assert(Number.isNaN(first.gimbal_yaw_target_deg));
assert(Number.isNaN(decode(all.subarray(562,1124)).gimbal_yaw_actual_deg));
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
''' + helpers + draw + '''
drawInteractiveLines(canvas,[0,20,40],[{color:'red',values:[1,2,NaN]}],null,1000);
assert(calls.some(c=>c[0]==='stroke' && c[1]==='red'), 'valid segment before trailing NaN was lost');
assert.equal(calls.filter(c=>c[0]==='lineTo' && c[1]==='red').length,1);
calls.length=0;
drawInteractiveLines(canvas,[0,20,40,400],[{color:'red',values:[1,NaN,3,4]}],null,1000);
assert.equal(calls.filter(c=>c[0]==='lineTo' && c[1]==='red').length,0,'must not connect invalid samples or transport gaps');
calls.length=0;
drawInteractiveLines(canvas,[0],[{color:'red',values:[NaN]}],null,1000);
assert(calls.some(c=>c[0]==='fillText' && c[2].includes('No valid data')));
const s = {yawSpeed:{target:[0,0],actual:[0,0]}};
const yawReadout = yawSpeedReadout({...first,gimbal_yaw_target_deg_s:0},true,1000,1001);
assert.equal(yawReadout.targetText,'0.00 °/s');
const speedSeries = ''' + speed_series + ''';
calls.length=0;
drawInteractiveLines(canvas,[0,20],speedSeries,null,1000);
const painted = calls.filter(c=>c[0]==='stroke' && speedSeries.some(s=>s.color===c[1]));
assert.deepEqual(painted.map(c=>c[1]),[speedSeries[1].color,speedSeries[0].color], 'target must remain visible above coincident actual');
assert(calls.some(c=>c[0]==='setLineDash' && c[1]===speedSeries[0].color && JSON.stringify(c[2])==='[6,4]'));
assert.deepEqual(calls.filter(c=>c[0]==='setLineDash').at(-1)[2],[], 'hover and axes must not inherit target dash');
assert.equal(latestSeriesValue([0]),'0.00');
assert.equal(latestSeriesValue([0,NaN]),'N/A');
assert(yawTargetStatus(first).includes('角度参考未参与闭环'));
assert(yawTargetStatus({...first,yaw_flags:31}).includes('位置环已启用'));
assert(yawTargetStatus({...first,gimbal_enabled:0}).includes('控制未启用'));
assert(yawTargetStatus({...first,gimbal_startup_ready:0}).includes('等待启动对齐'));
assert(yawTargetStatus({...first,yaw_flags:7}).includes('控制目标无效'));
'''
        app = self.html.split('      function App() {', 1)[1].split('        return (\n          <div className="app">', 1)[0]
        code += """
const location = {href:'http://localhost:9091/',protocol:'http:'};
const localStorage = {getItem:()=>null};
const MIN_HISTORY_WINDOW_SEC=1, DEFAULT_HISTORY_WINDOW_SEC=20;
let stateIndex=0;
const useState = initial => [stateIndex++ === 3 ? true : (typeof initial === 'function' ? initial() : initial), ()=>{}];
let hookEffects=[];
const useEffect = callback=>hookEffects.push(callback);
const useMemo = callback=>callback();
let latestForRender=first;
let age=0;
let actualStore;
const useRef = value=> {
 if(value && value.frames === 0) { actualStore=value; value.latest=latestForRender; value.lastFrameAt=Date.now()-age; }
 return {current:value};
};
function renderFields() {
stateIndex=0;
hookEffects=[];
""" + app + """
return {can:canStatus(0), basicItems, remoteItems, imuItems, yawReadout};
}
let fields=renderFields();
assert.equal(fields.can,'ON');
assert.equal(fields.imuItems.find(i=>i.name==='gimbal_yaw_target_deg').value,'N/A');
assert.equal(fields.remoteItems.find(i=>i.name==='rc_rocker_r_x').value,111);
assert(fields.remoteItems.find(i=>i.name==='emergency_stop(flag2)').value.includes('N/A'));
age=1500; assert.equal(renderFields().can,'STALE');
age=0; latestForRender={...first,dashboard_version:7}; assert.equal(renderFields().can,'N/A');
latestForRender=decode(all.subarray(562,1124));
assert.equal(renderFields().can,'OFF');
assert(renderFields().remoteItems.find(i=>i.name==='rc_rocker_r_x').value.includes('N/A'));
"""
        code += 'const HARD_MAX_POINTS = 12000;\n' + series_store + """
const position=decode(all.subarray(1124,1686));
const disabled=decode(all.subarray(2248,2810));
const negative=decode(all.subarray(2810,3372));
assert(Number.isNaN(disabled.gimbal_cmd_yaw_deg));
assert.equal(negative.gimbal_cmd_yaw_deg,-540);
assert.equal(negative.gimbal_yaw_target_deg_s,-24);
clearDashboardSeries(actualStore);
for (const [index,frame] of [first,position,disabled,negative,{...first,dashboard_version:7}].entries()) {
 pushDashboardSample(actualStore,{...frame,timestamp_ms:index*20});
}
assert.deepEqual(actualStore.yawAngle.reference,[360,NaN,NaN,-540,NaN]);
assert.deepEqual(actualStore.yawAngle.target,[NaN,360,NaN,NaN,NaN]);
trimDashboardStore(actualStore,40);
assert.deepEqual(actualStore.yawAngle.reference,[NaN,-540,NaN]);
assert.equal(actualStore.yawAngle.actual.length,actualStore.timeline.length);
clearDashboardSeries(actualStore);
assert.equal(actualStore.yawAngle.reference.length,0);
assert.equal(actualStore.timeline.length,0);
latestForRender=disabled;
assert.equal(renderFields().yawReadout.targetText,'—');
assert(renderFields().yawReadout.reason.includes('控制未启用'));
latestForRender={...first,gimbal_yaw_target_deg_s:0};
assert.equal(renderFields().yawReadout.targetText,'0.00 °/s');
latestForRender=negative;
assert.equal(renderFields().yawReadout.targetText,'-24.00 °/s');
latestForRender={...first,gimbal_yaw_target_deg_s:NaN};
assert(renderFields().yawReadout.reason.includes('固件未提供'));
latestForRender={...first,status_flags:first.status_flags & ~8};
assert(renderFields().yawReadout.reason.includes('快照过期'));
assert.equal(renderFields().yawReadout.targetText,'—');
latestForRender=first; age=1500;
assert(renderFields().yawReadout.reason.includes('未更新'));
assert.equal(renderFields().yawReadout.actualText,'—');
assert(yawSpeedReadout(first,false,1000,1001).reason.includes('连接已断开'));
assert(yawSpeedReadout(null,true,0,1001).reason.includes('等待首帧'));
assert(yawSpeedReadout(first,true,0,1001).reason.includes('等待本次连接首帧'));
assert.equal(yawSpeedReadout({...first,dashboard_version:7},true,1000,1001).targetText,'24.00 °/s');
"""
        # Run the actual WebSocket effect and onmessage path, including split frames and restart.
        from scripts.dashboard.rtt_ws_bridge import RTTWebSocketBridge, STREAM_DASHBOARD
        packet_constants = 'const PACKET_MAGIC' + self.html.split('      const PACKET_MAGIC', 1)[1].split('      const HARD_MAX_POINTS', 1)[0]
        frame_constants = 'const DASHBOARD_MAGIC' + self.html.split('      const DASHBOARD_MAGIC', 1)[1].split('      // Match firmware', 1)[0]
        buffer_helpers = 'function crc16Firmware(' + self.html.split('      function crc16Firmware(', 1)[1].split('      function clearDashboardSeries(', 1)[0]
        packets = [base64.b64encode(RTTWebSocketBridge._make_packet(STREAM_DASHBOARD, part)).decode()
                   for part in (self.raw[2248:2810], self.raw[:17], self.raw[17:562], self.raw[2810:3372])]
        code += packet_constants + frame_constants + buffer_helpers + """
let liveSocket;
class WebSocket {
 constructor() { liveSocket=this; }
 close() {}
}
const requestAnimationFrame = callback=>callback();
(async () => {
 latestForRender=null; age=0; renderFields();
 const liveStore=actualStore;
 const cleanup=hookEffects[1]();
 liveSocket.onopen();
 assert.equal(liveStore.lastFrameAt,0);
 const packets = """ + json.dumps(packets) + """;
 async function receive(index) {
  const bytes=Buffer.from(packets[index],'base64');
  await liveSocket.onmessage({data:bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)});
  return yawSpeedReadout(liveStore.latest,true,liveStore.lastFrameAt,Date.now());
 }
 assert((await receive(0)).reason.includes('控制未启用'));
 assert.equal((await receive(1)).targetText,'—'); // incomplete frame cannot replace latest
 assert.equal((await receive(2)).targetText,'24.00 °/s');
 assert.equal(liveStore.frames,2);
 assert.equal(liveStore.crcError,0);
 assert.equal((await receive(3)).targetText,'-24.00 °/s');
 assert.equal(liveStore.yawSpeed.target.at(-1),-24);
 assert.equal(liveStore.latest.seq,6);
 cleanup();
})().catch(error=>{console.error(error);process.exitCode=1;});
"""
        script = self.directory / 'browser-test.cjs'; script.write_text(code)
        binary = self.directory / 'frames.bin'; binary.write_bytes(self.raw)
        subprocess.run(['node', str(script), str(binary)], check=True)

    def test_bridge_injects_selected_websocket_port(self):
        from scripts.dashboard.rtt_ws_bridge import RTTWebSocketBridge, parse_args
        for options, expected in (([], None), (['--ws-port', '8765'], 8765), (['--http-port', '9091'], None)):
            with patch('sys.argv', ['bridge', '--no-monitor-save', *options]):
                args = parse_args()
            bridge = RTTWebSocketBridge(args)
            self.assertIn(('window.RTT_CONFIG=' + json.dumps({'wsPort': expected})).encode(), bridge.viewer_html)

    def test_browser_keeps_forwarded_port_for_shared_listener(self):
        from scripts.dashboard.rtt_ws_bridge import RTTWebSocketBridge, parse_args
        if shutil.which('node') is None:
            self.skipTest('node is required for browser URL checks')
        # Execute the page's URL construction with the actual server-injected config.
        prelude = self.html.split('      function App() {', 1)[1].split('        const [wsUrl,', 1)[0]
        for options, href, expected in (
            ([], 'http://localhost:18080/?forwarded=1', 'ws://localhost:18080/'),
            ([], 'https://example.test/', 'wss://example.test/'),
            (['--http-port', '9091'], 'http://localhost:19091/', 'ws://localhost:19091/'),
            (['--ws-port', '8765'], 'http://localhost:18080/', 'ws://localhost:8765/'),
        ):
            with self.subTest(href=href, options=options), patch('sys.argv', ['bridge', '--no-monitor-save', *options]):
                bridge = RTTWebSocketBridge(parse_args())
                config = bridge.viewer_html.decode().split('window.RTT_CONFIG=', 1)[1].split(';</script>', 1)[0]
                code = ('const location = new URL(' + json.dumps(href) + ');'
                        + 'const window = {RTT_CONFIG:' + config + '};' + prelude
                        + 'require("assert").equal(wsDefault,' + json.dumps(expected) + ');')
                subprocess.run(['node', '-e', code], check=True)

    def test_http_refresh_reads_saved_page_on_both_listeners(self):
        from scripts.dashboard.rtt_ws_bridge import RTTWebSocketBridge, parse_args
        from websockets.datastructures import Headers
        from websockets.http11 import Request
        viewer = self.directory / 'refresh.html'
        viewer.write_text('<head></head>before')
        with patch('sys.argv', ['bridge', '--no-monitor-save', '--viewer-file', str(viewer)]):
            bridge = RTTWebSocketBridge(parse_args())
        request = Request('/?refresh=1', Headers())
        self.assertIn(b'before', bridge._serve_http_request(request).body)
        # Capture the actual split-listener handler without opening network/probe connections.
        with patch('scripts.dashboard.rtt_ws_bridge.http.server.ThreadingHTTPServer') as server, \
                patch('scripts.dashboard.rtt_ws_bridge.threading.Thread'):
            bridge._start_http_server()
            handler_class = server.call_args.args[1]
        handler = object.__new__(handler_class)
        handler.path = '/index.html?refresh=2'
        handler.send_response = lambda status: self.assertEqual(status, 200)
        headers = {}
        handler.send_header = lambda key, value: headers.update({key: value})
        handler.end_headers = lambda: None
        for version in ('after', 'saved-again'):
            viewer.write_text('<head></head>' + version)
            response = bridge._serve_http_request(request)
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
            self.assertIn(version.encode(), response.body)
            self.assertIn(b'window.RTT_CONFIG=', response.body)
            handler.wfile = io.BytesIO()
            handler.do_GET()
            self.assertEqual(handler.wfile.getvalue(), response.body)
            self.assertEqual(headers['Cache-Control'], 'no-store')
        viewer.unlink()
        self.assertEqual(bridge._serve_http_request(request).status_code, 503)
        with patch.object(handler, 'send_error') as error:
            handler.do_GET()
            self.assertEqual(error.call_args.args[0], 503)


if __name__ == '__main__':
    unittest.main()
