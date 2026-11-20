import copy
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import DEFAULT, State, make_server, validate_frame, validate_layout

def frame(seq=0):
    hand = {"tracked": True, "pose": {"position": [.1, 1.2, -.3], "orientation": [0, 0, 0, 1]}, "trigger": .5, "grip": .2, "buttons": [.5, .2], "axes": [0, 0]}
    return {"seq": seq, "space": "webxr-local-floor-rh-m-xyzw", "visibility": "visible", "hands": {"left": copy.deepcopy(hand), "right": copy.deepcopy(hand)}}

class ValidationTests(unittest.TestCase):
    def test_layout_bounds_and_duplicate_ids(self):
        self.assertEqual(validate_layout(DEFAULT), DEFAULT)
        for mutate in (lambda d: d["widgets"][0].update(x=950), lambda d: d["widgets"].append(copy.deepcopy(d["widgets"][0])), lambda d: d["widgets"][0].update(color="<script>")):
            bad=copy.deepcopy(DEFAULT); mutate(bad)
            with self.assertRaises(ValueError): validate_layout(bad)

    def test_atomic_two_hands_and_hidden_rejected(self):
        value=validate_frame(frame())
        self.assertEqual(set(value["hands"]), {"left", "right"})
        bad=frame(); bad["visibility"]="hidden"
        with self.assertRaises(ValueError): validate_frame(bad)
        bad=frame(); del bad["hands"]["left"]
        with self.assertRaises(ValueError): validate_frame(bad)

    def test_nonfinite_and_bad_quaternion_rejected(self):
        for p in (float("nan"), float("inf"), True):
            bad=frame(); bad["hands"]["left"]["pose"]["position"][0]=p
            with self.assertRaises(ValueError): validate_frame(bad)
        bad=frame(); bad["hands"]["left"]["pose"]["orientation"]=[0,0,0,0]
        with self.assertRaises(ValueError): validate_frame(bad)

    def test_invalid_tracking_removes_pose(self):
        value=frame(); value["hands"]["left"]["tracked"]=False
        clean=validate_frame(value)["hands"]["left"]
        self.assertIsNone(clean["pose"]); self.assertEqual(clean["trigger"],0)

    def test_session_sequence_expiry_and_exclusion(self):
        with tempfile.TemporaryDirectory() as directory:
            state=State(directory); session=state.claim(); state.accept(session,frame())
            self.assertTrue(state.snapshot()["fresh"])
            with self.assertRaises(ValueError): state.claim()
            with self.assertRaises(ValueError): state.accept(session,frame())
            with self.assertRaises(ValueError): state.accept("other",frame(1))
            state.last-=.31
            self.assertFalse(state.snapshot()["fresh"]); self.assertIsNone(state.snapshot()["frame"])
            state.last-=3; next_session=state.claim()
            with self.assertRaises(ValueError): state.accept(session,frame(1))
            state.accept(next_session,frame())

class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(); cls.http=make_server(0,cls.tmp.name)
        cls.thread=threading.Thread(target=cls.http.serve_forever,daemon=True); cls.thread.start()
        cls.port=cls.http.server_port
    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown(); cls.http.server_close(); cls.tmp.cleanup()
    def request(self,path,method="GET",body=None,headers=None):
        conn=http.client.HTTPConnection("127.0.0.1",self.port,timeout=3)
        conn.request(method,path,json.dumps(body) if body is not None else None,headers or {})
        r=conn.getresponse(); code=r.status; raw=r.read(); conn.close()
        return code,raw
    def headers(self):
        return {"Content-Type":"application/json","X-Studio-Token":self.http.state.token,"Origin":f"http://127.0.0.1:{self.port}"}
    def test_static_allowlist(self):
        self.assertEqual(self.request("/")[0],200)
        for path in ("/../server.py","/%2e%2e/server.py","/data/layout.json","/api/drive"):
            self.assertEqual(self.request(path)[0],404)
    def test_cross_origin_host_and_missing_token_rejected(self):
        self.assertEqual(self.request("/api/bootstrap",headers={"Host":"evil.example"})[0],403)
        self.assertEqual(self.request("/api/bootstrap",headers={"Origin":"https://evil.example"})[0],403)
        self.assertEqual(self.request("/api/session","POST",{})[0],403)
        headers=self.headers();headers["Origin"]="https://evil.example"
        self.assertEqual(self.request("/api/session","POST",{},headers)[0],403)
    def test_save_conflict_and_owned_directory(self):
        before=self.http.state.revision
        self.assertEqual(self.request("/api/layout","POST",{"revision":before,"layout":DEFAULT},self.headers())[0],200)
        self.assertEqual(self.request("/api/layout","POST",{"revision":before,"layout":DEFAULT},self.headers())[0],409)
        self.assertEqual(set(p.name for p in Path(self.tmp.name).iterdir()),{"layout.json"})
    def test_ui_event_requires_tracked_hand_and_never_drives(self):
        code,raw=self.request("/api/session","POST",{},self.headers());self.assertEqual(code,200)
        session=json.loads(raw)["session"]
        code,_=self.request("/api/frame","POST",{"session":session,"frame":frame()},self.headers());self.assertEqual(code,200)
        code,_=self.request("/api/ui-event","POST",{"session":session,"widget":"demo","hand":"left"},self.headers());self.assertEqual(code,200)
        code,raw=self.request("/api/status");status=json.loads(raw)
        self.assertEqual(status["ui_events"],1);self.assertEqual(status["robot_output"],"NOT_CONNECTED")
        self.assertEqual(self.request("/api/drive","POST",{},self.headers())[0],404)
        self.assertEqual(self.request("/api/close","POST",{"session":session},self.headers())[0],200)
        self.assertFalse(json.loads(self.request("/api/status")[1])["fresh"])

    def test_calibration_interlock_and_mapping(self):
        def post(path, **data):
            code, raw = self.request(path, 'POST', data, self.headers())
            return code, json.loads(raw)

        code, response = post('/api/session')
        self.assertEqual(code, 200)
        session = response['session']
        try:
            def sample(seq, right_position, left_trigger=0, right_trigger=0):
                value = frame(seq)
                value['head'] = {'position': [0, 1.6, 0], 'orientation': [0, 0, 0, 1]}
                value['hands']['right']['pose']['position'] = right_position
                value['hands']['left']['trigger'] = left_trigger
                value['hands']['right']['trigger'] = right_trigger
                self.assertEqual(post('/api/frame', session=session, frame=value)[0], 200)

            self.assertEqual(post('/api/control', action='arm')[0], 400)
            self.assertEqual(post('/api/calibration', action='start')[1]['stage'], 'center')
            sample(0, [0, 1, 0], left_trigger=.9)
            self.assertEqual(post('/api/calibration', action='capture')[0], 400)
            sample(1, [0, 1, 0])
            self.assertEqual(post('/api/calibration', action='capture')[1]['stage'], 'forward')
            sample(2, [0, 1, -.03])
            self.assertEqual(post('/api/calibration', action='capture')[0], 400)
            sample(3, [0, 1, -.2])
            self.assertEqual(post('/api/calibration', action='capture')[1]['stage'], 'right')
            sample(4, [-.2, 1, 0])
            self.assertEqual(post('/api/calibration', action='capture')[0], 400)
            sample(5, [.2, 1, 0])
            self.assertEqual(post('/api/calibration', action='capture')[1]['stage'], 'neutral')
            sample(6, [0, 1, 0])
            self.assertEqual(post('/api/calibration', action='capture')[1]['stage'], 'ready')
            post('/api/publisher-status', connected=True)
            self.assertEqual(post('/api/control', action='arm')[0], 200)
            json.loads(self.request('/api/status')[1])  # Observe released hands.
            sample(7, [0, 1, -.15], right_trigger=.9)
            status = json.loads(self.request('/api/status')[1])
            self.assertEqual(status['control']['forward'], 100)
            self.assertEqual(post('/api/calibration', action='start')[1]['stage'], 'center')
            status = json.loads(self.request('/api/status')[1])
            self.assertFalse(status['control']['armed'])
            self.assertEqual(status['control']['forward'], 0)
        finally:
            post('/api/close', session=session)
            self.http.state.publisher = None

    def test_repository_calibration_saves_on_right_trigger(self):
        def post(path, **data):
            code, raw = self.request(path, 'POST', data, self.headers())
            return code, json.loads(raw)

        _, result = post('/api/session')
        session = result['session']
        sequence = 0
        try:
            self.assertEqual(self.request('/calibration-ui')[0], 200)
            self.assertEqual(self.request('/calibration-ui.js')[0], 200)
            self.assertEqual(post('/api/repository-calibration', action='begin')[0], 200)
            self.assertEqual(post('/api/control', action='arm')[0], 400)

            def send(position, trigger):
                nonlocal sequence
                value = frame(sequence)
                sequence += 1
                value['hands']['right'].update(trigger=trigger)
                value['hands']['right']['pose']['position'] = position
                self.assertEqual(post('/api/frame', session=session, frame=value)[0], 200)

            neutral = [0, 1, 0]
            for position in (neutral, [.2, 1, 0], neutral, [0, 1, -.2], neutral):
                send(position, 0)
                send(position, .9)
                send(position, .9)  # Holding the trigger cannot advance another step.
            snapshot = json.loads(self.request('/snapshot')[1])
            self.assertEqual(snapshot['save_state'], 'save')
            self.assertIsNone(json.loads(self.request('/calibration?profile=quest_webxr')[1]))
            send(neutral, 0)
            send(neutral, .9)
            state = json.loads(self.request('/api/status')[1])
            self.assertFalse(state['control']['armed'])
            self.assertEqual(state['calibration']['stage'], 'ready')
            self.assertEqual(state['calibration']['basis']['forward'], [0, -1])
            saved = json.loads(self.request('/calibration?profile=quest_webxr')[1])
            self.assertEqual(saved['coordinate_space'], 'webxr-local-floor-via-unity')
        finally:
            post('/api/close', session=session)
            test_profile = Path(self.tmp.name) / 'webxr-calibrations/quest_webxr.json'
            if test_profile.exists():
                test_profile.unlink()
                test_profile.parent.rmdir()

if __name__=="__main__": unittest.main(verbosity=2)
