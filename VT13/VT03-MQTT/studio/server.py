"""Isolated, loopback-only WebXR UI studio. No actuator/output adapters."""
from __future__ import annotations
import argparse
from collections import deque
import copy
import json
import math
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit
from vehicle import DirectionCalibration, DualInput
from repository_calibration import DEFAULT_PROFILE, TriggerCalibration, page_assets, profile_name

ROOT = Path(__file__).resolve().parent
TRACKING_REASONS = (
    'no-source', 'session-not-visible', 'no-frame', 'bare-hand', 'no-grip-space',
    'no-trigger-mapping', 'emulated-pose', 'no-grip-pose', 'input-source', 'tracked-source',
)
DEFAULT = {"version": 1, "widgets": [
    {"id": "title", "type": "label", "text": "双手柄 · VR 工作台", "x": 30, "y": 22, "w": 900, "h": 64, "color": "#19283f", "binding": "none"},
    {"id": "left", "type": "status", "text": "左手柄", "x": 30, "y": 108, "w": 440, "h": 160, "color": "#163c50", "binding": "left"},
    {"id": "right", "type": "status", "text": "右手柄", "x": 490, "y": 108, "w": 440, "h": 160, "color": "#34365b", "binding": "right"},
    {"id": "note", "type": "label", "text": "右扳机＋位移：平移；左扳机＋左右位移：旋转", "x": 30, "y": 292, "w": 900, "h": 88, "color": "#27344a", "binding": "none"},
    {"id": "demo", "type": "button", "text": "射线指向这里，扣扳机测试 UI", "x": 30, "y": 406, "w": 900, "h": 94, "color": "#295b75", "binding": "none"},
]}

def number(value, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError("numeric value out of range")
    return value

def validate_layout(data):
    if not isinstance(data, dict) or data.get("version") != 1 or not isinstance(data.get("widgets"), list) or not 1 <= len(data["widgets"]) <= 30:
        raise ValueError("layout must contain 1..30 widgets")
    widgets, ids = [], set()
    for w in data["widgets"]:
        if not isinstance(w, dict): raise ValueError("invalid widget")
        wid = w.get("id")
        if not isinstance(wid, str) or not 1 <= len(wid) <= 48 or not all(c.isascii() and (c.isalnum() or c in "_-") for c in wid) or wid in ids:
            raise ValueError("widget id invalid/duplicate")
        ids.add(wid)
        if w.get("type") not in ("label", "status", "button") or w.get("binding") not in ("none", "left", "right"):
            raise ValueError("widget type/binding invalid")
        if not isinstance(w.get("text"), str) or len(w["text"]) > 180: raise ValueError("text too long")
        color = w.get("color", "")
        if not isinstance(color, str) or len(color) != 7 or color[0] != "#" or any(c not in "0123456789abcdefABCDEF" for c in color[1:]): raise ValueError("color invalid")
        x, y, width, height = [number(w.get(k), 0, 960) for k in ("x", "y", "w", "h")]
        if width < 60 or height < 40 or x + width > 960 or y + height > 600: raise ValueError("widget outside canvas")
        widgets.append({k: w[k] for k in ("id", "type", "text", "x", "y", "w", "h", "color", "binding")})
    return {"version": 1, "widgets": widgets}

def validate_frame(frame):
    if not isinstance(frame, dict): raise ValueError("frame required")
    seq = frame.get("seq")
    if type(seq) is not int or not 0 <= seq < 2**53: raise ValueError("invalid seq")
    if frame.get("space") != "webxr-local-floor-rh-m-xyzw": raise ValueError("unsupported reference space")
    visibility = frame.get("visibility")
    if visibility not in ("visible", "visible-blurred", "hidden"): raise ValueError("visibility invalid")
    result = {"seq": seq, "space": frame["space"], "visibility": visibility, "hands": {}}
    result['rtt_ms']=number(frame.get('rtt_ms',0),0,10000)
    tracking = frame.get('tracking')
    result['tracking'] = {
        side: tracking[side]
        for side in ('left', 'right')
        if isinstance(tracking, dict) and tracking.get(side) in TRACKING_REASONS
    }
    hands = frame.get("hands")
    if not isinstance(hands, dict) or set(hands) != {"left", "right"}: raise ValueError("both hand slots required")
    for side in ("left", "right"):
        hand = hands[side]
        if not isinstance(hand, dict) or type(hand.get("tracked")) is not bool: raise ValueError("tracked must be boolean")
        if hand["tracked"] and visibility != "visible": raise ValueError("tracking not valid while hidden/blurred")
        if not hand["tracked"]:
            result["hands"][side] = {"tracked": False, "pose": None, "trigger": 0, "grip": 0, "buttons": [], "axes": []}
            continue
        pose = hand.get("pose")
        if not isinstance(pose, dict): raise ValueError("pose required")
        p, q = pose.get("position"), pose.get("orientation")
        if not isinstance(p, list) or len(p) != 3 or not isinstance(q, list) or len(q) != 4: raise ValueError("pose dimensions invalid")
        p = [number(n, -1000, 1000) for n in p]
        q = [number(n, -1.001, 1.001) for n in q]
        if abs(sum(n*n for n in q) - 1) > .03: raise ValueError("quaternion must be unit length")
        buttons, axes = hand.get("buttons", []), hand.get("axes", [])
        if not isinstance(buttons, list) or len(buttons) > 16 or not isinstance(axes, list) or len(axes) > 16: raise ValueError("gamepad too large")
        result["hands"][side] = {"tracked": True, "pose": {"position": p, "orientation": q}, "trigger": number(hand.get("trigger"), 0, 1), "grip": number(hand.get("grip"), 0, 1), "buttons": [number(n, 0, 1) for n in buttons], "axes": [number(n, -1, 1) for n in axes]}
    head=frame.get('head')
    result['head']=None
    if head is not None:
        if not isinstance(head,dict) or not isinstance(head.get('position'),list) or len(head['position'])!=3 or not isinstance(head.get('orientation'),list) or len(head['orientation'])!=4:raise ValueError('head pose invalid')
        p=[number(v,-1000,1000) for v in head['position']];q=[number(v,-1.001,1.001) for v in head['orientation']]
        if abs(sum(v*v for v in q)-1)>.03:raise ValueError('head quaternion invalid')
        result['head']={'position':p,'orientation':q}
    return result

class State:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.layout_path = self.directory / "layout.json"
        self.layout = validate_layout(json.loads(self.layout_path.read_text("utf-8"))) if self.layout_path.exists() else copy.deepcopy(DEFAULT)
        self.revision = 1
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.session = None
        self.seq = -1
        self.frame = None
        self.last = 0
        self.accepted = 0
        self.events = 0
        self.last_event = None
        self.control=DualInput()
        self.calibration = DirectionCalibration()
        self.video=None
        self.publisher=None
        self.publisher_received=0
        self.xr_entry = None
        self.repo_draft = None
        self.repo_points = deque(maxlen=120)
        self.repo_profile_directory = self.directory / 'webxr-calibrations'

    def claim(self):
        with self.lock:
            if self.session and time.monotonic() - self.last < 3:
                raise ValueError("another headset session is active; exit VR there and wait 3 seconds")
            self.session, self.seq, self.frame, self.last = secrets.token_urlsafe(24), -1, None, time.monotonic()
            self.control.disarm()
            self.calibration = DirectionCalibration()
            self.repo_draft = None
            self.repo_points.clear()
            return self.session

    def accept(self, session, frame):
        value = validate_frame(frame)
        with self.lock:
            if not self.session or session != self.session: raise ValueError("session expired")
            if value["seq"] <= self.seq: raise ValueError("non-monotonic frame")
            if self.repo_draft and time.monotonic() - self.last > .12:
                self.repo_draft.released = False
            self.seq, self.frame, self.last = value["seq"], value, time.monotonic()
            self.accepted += 1
            self.update_repository_calibration(value)

    def update_repository_calibration(self, frame):
        draft = self.repo_draft
        hand = frame['hands']['right']
        if not hand['tracked'] or frame['visibility'] != 'visible':
            self.repo_points.clear()
            if draft:
                draft.released = False
            return
        x, y, z = hand['pose']['position']
        self.repo_points.append({'x': x, 'y': y, 'z': -z, 'recv_unix': time.time(),
                                 'flag': hand['trigger'] >= .65})
        if not draft or not draft.feed(hand['pose']['position'], hand['trigger']):
            return
        try:
            draft.persist(self.repo_profile_directory)
        except OSError:
            draft.error = '保存失败，请松开右扳机再按一次'
            return
        self.control.disarm()
        self.calibration.basis = draft.basis
        self.calibration.stage = 'ready'

    def tracking_snapshot(self):
        with self.lock:
            age = (time.monotonic() - self.last) if self.last else None
            fresh = bool(self.session and self.frame and age is not None and age < .3)
            frame = copy.deepcopy(self.frame) if fresh else None
            return {"fresh": fresh, "age_ms": round(age * 1000) if age is not None else None, "frame": frame, "accepted_frames": self.accepted, "ui_events": self.events, "last_ui_event": self.last_event, "robot_output": "DISCONNECTED_BY_DESIGN"}

    def snapshot(self):
        with self.lock:
            value=self.tracking_snapshot()
            value['control']={**self.control.update(value),'armed':self.control.armed,'basis':self.control.basis}
            publisher=copy.deepcopy(self.publisher)
            if publisher and time.monotonic()-self.publisher_received>.5:
                publisher.update(running=False,connected=False,error='发布器状态已过期')
            value['publisher']=publisher
            value['calibration'] = self.calibration.status()
            if self.repo_draft:
                value['calibration']['prompt'] = self.repo_draft.prompt
            value['xr_entry'] = self.xr_entry
            value['robot_output']='MQTT_WORKER' if self.publisher else 'NOT_CONNECTED'
            value['video']=self.video.snapshot() if self.video else {'fresh':False,'error':'视频接收未启动'}
            return value

class Handler(BaseHTTPRequestHandler):
    server_version = "IsolatedXR/1"
    def log_message(self, fmt, *args):
        if not self.path.startswith('/api/'):
            super().log_message(fmt, *args)

    def send(self, code, data, kind="application/json; charset=utf-8"):
        raw = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if kind.startswith("application/json") else data
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        # The pinned A-Frame bundle uses generated functions internally. Permit
        # those only on the VR document; editor/API responses retain strict CSP.
        script_policy = "'self' 'unsafe-eval'" if urlsplit(self.path).path == "/vr" else "'self'"
        self.send_header("Content-Security-Policy", f"default-src 'self'; script-src {script_policy}; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; worker-src 'self' blob:; object-src 'none'; frame-ancestors 'none'")
        self.send_header("Permissions-Policy", "xr-spatial-tracking=(self), camera=(), microphone=(), geolocation=()")
        self.end_headers()
        self.wfile.write(raw)

    def allowed(self, mutation=False):
        host = self.headers.get("Host", "")
        port = self.server.server_port
        if host not in (f"127.0.0.1:{port}", f"localhost:{port}"): return False
        origin = self.headers.get("Origin")
        if origin and origin != f"http://{host}": return False
        if self.headers.get("Sec-Fetch-Site") == "cross-site": return False
        if mutation and (self.headers.get("X-Studio-Token") != self.server.state.token or self.headers.get_content_type() != "application/json"): return False
        return True

    def do_GET(self):
        if not self.allowed(): return self.send(403, {"error": "local origin only"})
        path = urlsplit(self.path).path
        state = self.server.state
        if path in ('/calibration-ui', '/calibration-ui.js'):
            try:
                html, script = page_assets()
            except (OSError, ValueError) as exc:
                return self.send(500, {'error': str(exc)})
            if path.endswith('.js'):
                return self.send(200, script.encode(), 'text/javascript; charset=utf-8')
            return self.send(200, html.encode(), 'text/html; charset=utf-8')
        if path == '/snapshot':
            with state.lock:
                live = bool(state.frame and state.frame['hands']['right']['tracked']
                            and time.monotonic() - state.last < .12)
                draft = state.repo_draft
                return self.send(200, {'type': 'snapshot', 'points': list(state.repo_points) if live else [],
                                       'gate_open': live, 'save_state': draft.stage if draft else None,
                                       'calibration': draft.data if draft else None,
                                       'prompt': draft.prompt if draft else None})
        if path == '/calibrations':
            profiles = [{'name': file.stem} for file in sorted(state.repo_profile_directory.glob('*.json'))]
            return self.send(200, {'active': DEFAULT_PROFILE, 'profiles': profiles})
        if path == '/calibration':
            try:
                name = profile_name(parse_qs(urlsplit(self.path).query).get('profile', [DEFAULT_PROFILE])[0])
                data = json.loads((state.repo_profile_directory / (name + '.json')).read_text('utf-8'))
            except FileNotFoundError:
                data = None
            except (OSError, ValueError) as exc:
                return self.send(400, {'error': str(exc)})
            return self.send(200, data)
        if path == "/api/bootstrap": return self.send(200, {"token": state.token, "mode": "dual-vr-studio", "server_pid":os.getpid()})
        if path == "/api/status": return self.send(200, state.snapshot())
        if path == '/api/video/frame.jpg':
            jpeg=state.video.latest() if state.video else None
            return self.send(200,jpeg,'image/jpeg') if jpeg else self.send(204,b'','application/octet-stream')
        if path == "/api/layout":
            with state.lock: return self.send(200, {"revision": state.revision, "layout": state.layout})
        files = {"/": ("index.html", "text/html; charset=utf-8"), "/vr": ("vr.html", "text/html; charset=utf-8"), "/style.css": ("style.css", "text/css; charset=utf-8"), "/shared.js": ("shared.js", "text/javascript; charset=utf-8"), "/editor.js": ("editor.js", "text/javascript; charset=utf-8"), "/vr.js": ("vr.js", "text/javascript; charset=utf-8"), "/vendor/aframe-1.7.1.min.js": ("vendor/aframe-1.7.1.min.js", "text/javascript; charset=utf-8")}
        if path not in files: return self.send(404, {"error": "not found"})
        name, kind = files[path]
        self.send(200, (ROOT / "www" / name).read_bytes(), kind)

    def do_POST(self):
        if not self.allowed(True): return self.send(403, {"error": "origin/token/content-type rejected"})
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 65536: raise ValueError("body size invalid")
            data = json.loads(self.rfile.read(size), parse_constant=lambda _: (_ for _ in ()).throw(ValueError("non-finite JSON")))
            if not isinstance(data, dict): raise ValueError("object required")
            state, path = self.server.state, urlsplit(self.path).path
            with state.lock:
                if path == '/api/repository-calibration':
                    if data.get('action') != 'begin':
                        raise ValueError('unknown repository calibration action')
                    if not state.session:
                        raise ValueError('请先进入 Quest 的沉浸式 VR')
                    if 'session' in data and data['session'] != state.session:
                        raise ValueError('VR 会话已更换')
                    name = profile_name(data.get('profile', DEFAULT_PROFILE))
                    state.control.disarm()
                    state.calibration = DirectionCalibration()
                    state.calibration.stage = 'repository'
                    state.repo_draft = TriggerCalibration(name)
                    state.repo_points.clear()
                    return self.send(200, {'state': state.repo_draft.stage})
                if path == '/api/xr-entry':
                    stage, message = data.get('stage'), data.get('message')
                    if stage not in ('unsupported', 'loading', 'requested', 'waiting', 'entered', 'error') or not isinstance(message, str):
                        raise ValueError('invalid XR entry status')
                    state.xr_entry = {'stage': stage, 'message': message[:240], 'time': time.time()}
                    return self.send(200, {'ok': True})
                if path == "/api/session":
                    return self.send(200, {"session": state.claim()})
                if path == "/api/frame":
                    state.accept(data.get("session"), data.get("frame"))
                    return self.send(200, {"ok": True})
                if path == "/api/close":
                    if data.get("session") != state.session: raise ValueError("session mismatch")
                    state.session, state.frame = None, None
                    state.control.disarm()
                    state.calibration = DirectionCalibration()
                    state.repo_draft = None
                    state.repo_points.clear()
                    return self.send(200, {"ok": True})
                if path == '/api/control':
                    if 'session' in data and data['session'] != state.session:
                        raise ValueError('VR 会话已切换，请刷新页面')
                    if data.get('action') == 'arm':
                        if state.calibration.stage != 'ready':
                            raise ValueError('请先完成本次 VR 会话的校准')
                        publisher = state.publisher or {}
                        if not publisher.get('connected') or time.monotonic() - state.publisher_received > .5:
                            raise ValueError('MQTT 发布器未连接')
                        state.control.arm(state.tracking_snapshot(), state.calibration.basis)
                    elif data.get('action')=='disarm':state.control.disarm()
                    else:raise ValueError('unknown control action')
                    return self.send(200,{'armed':state.control.armed})
                if path == '/api/calibration':
                    if 'session' in data and data['session'] != state.session:
                        raise ValueError('VR 会话已切换，请刷新页面')
                    action = data.get('action')
                    if action == 'start':
                        if not state.session:
                            raise ValueError('请先戴上头显并进入 VR')
                        state.control.disarm()
                        state.repo_draft = None
                        state.calibration.start()
                    elif action == 'capture':
                        state.calibration.capture(state.tracking_snapshot())
                    elif action == 'invalidate':
                        state.control.disarm()
                        state.calibration = DirectionCalibration()
                        state.repo_draft = None
                        state.repo_points.clear()
                    else:
                        raise ValueError('unknown calibration action')
                    return self.send(200, state.calibration.status())
                if path == '/api/publisher-status':
                    # Status only; command axes are never accepted from this API.
                    state.publisher={k:data.get(k) for k in ('running','connected','sequence','sent','error','output','time','publish_ok')}
                    state.publisher_received=time.monotonic()
                    return self.send(200,{'ok':True})
                if path == "/api/layout":
                    if data.get("revision") != state.revision:
                        return self.send(409, {"error": "layout changed elsewhere; reload before saving"})
                    layout = validate_layout(data.get("layout"))
                    # Atomic replacement affects only this app's own layout.
                    tmp = state.directory / "layout.pending"
                    tmp.write_text(json.dumps(layout, ensure_ascii=False, indent=2), "utf-8")
                    tmp.replace(state.layout_path)
                    state.layout, state.revision = layout, state.revision + 1
                    return self.send(200, {"revision": state.revision})
                if path == "/api/ui-event":
                    if data.get("session") != state.session or not state.snapshot()["fresh"]: raise ValueError("active tracking session required")
                    wid, hand = data.get("widget"), data.get("hand")
                    if hand not in ("left", "right") or not state.frame["hands"][hand]["tracked"]: raise ValueError("tracked hand required")
                    if not any(w["id"] == wid and w["type"] == "button" for w in state.layout["widgets"]): raise ValueError("unknown UI button")
                    state.events += 1
                    state.last_event = {"widget": wid, "hand": hand, "count": state.events}
                    return self.send(200, {"ok": True, "count": state.events})
                return self.send(404, {"error": "not found"})
        except (ValueError, TypeError, KeyError) as exc:
            self.send(400, {"error": str(exc)})

def make_server(port=18780, directory=None):
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    server.state = State(directory or ROOT / "data")
    return server

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=18780)
    parser.add_argument('--video',action='store_true')
    args = parser.parse_args()
    http = make_server(args.port)
    if args.video:
        from video import VideoReceiver
        http.state.video=VideoReceiver();http.state.video.start()
    print(f"PC editor: http://127.0.0.1:{args.port}/", flush=True)
    print(f"Quest browser: http://127.0.0.1:{args.port}/vr", flush=True)
    print("MQTT output requires the separate, explicit vehicle publisher. No serial/SSH command output.", flush=True)
    try: http.serve_forever()
    except KeyboardInterrupt: pass
    finally: http.server_close()
