"""Reuse the repository's Quest source; no APK install/configuration or app restarts."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
import zmq
import numpy as np

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent/'vendor/quest-trajectory-recorder'
sys.path.insert(0,str(REPO/'src'))
from portable_quest_source import DirectQuestTargetSource
from quest_trajectory_recorder.teleop_frame import load_quest_calibration,calibration_from_dict

sys.path.insert(0, str(ROOT.parent/'pc'))
from vr_config import SETTINGS
ADB = Path(SETTINGS['adb_path'])
SERIAL = SETTINGS['quest_serial']
PORTS = [8087,8095,8100,8105,8110,8125,8126,8127,10505,15001]


def adb_command(*args,timeout=8):
    return subprocess.run([str(ADB),*(['-s', SERIAL] if SERIAL else []),*args],capture_output=True,text=True,
                          encoding='utf-8',errors='replace',timeout=timeout,
                          creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)


def prepare_usb():
    state = adb_command('get-state')
    if state.returncode or state.stdout.strip()!='device':
        return dict(ready=False,message='Quest 未授权或未连接；请在头显允许 USB 调试。')
    mappings = adb_command('reverse','--list').stdout
    errors = []
    for port in PORTS:
        if f'tcp:{port} tcp:{port}' in mappings:
            continue
        result = adb_command('reverse','--no-rebind',f'tcp:{port}',f'tcp:{port}')
        if result.returncode:
            errors.append(result.stderr.strip())
    pid = adb_command('shell','pidof','com.Xigbee.FrankaBot')
    return dict(ready=not errors,app_running=bool(pid.stdout.strip()),errors=errors,
                message='USB 已连接；请在头显打开已有 FrankaBot。' if not pid.stdout.strip() else 'USB 已连接，FrankaBot 正在运行。')


class QuestInput:
    def __init__(self,remote_port=8125,pause_port=8100,drain=True,calibration_path=None):
        self.profile=Path(calibration_path) if calibration_path is not None else ROOT/'repository_calibrations/quest_lab_current.json'
        self.calibration=load_quest_calibration(self.profile)
        if self.calibration is None:
            raise ValueError('有效校准文件不存在：'+str(self.profile))
        self.calibration_sha256=hashlib.sha256(self.profile.read_bytes()).hexdigest()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.packet = None
        self.gate_open = False
        self.reason = '等待 Quest 输入'
        self.count = 0
        self.pending_calibration=None
        self.invalid = 0
        self.diagnostics={}
        self.remote_port,self.pause_port,self.drain = remote_port,pause_port,drain
        self.session_id = uuid.uuid4().hex
        self.thread = threading.Thread(target=self._run,daemon=True,name='existing-quest-source')

    def start(self):
        self.thread.start()

    def _run(self):
        source,extras = None,[]
        context = zmq.Context()
        try:
            profile = self.profile
            source = DirectQuestTargetSource(context=context,host='127.0.0.1',remote_port=self.remote_port,
                pause_port=self.pause_port,calibration=self.calibration,no_gate=False,
                trajectory_gate_pause='High',allow_initial_high=False,gripper_mode='hold',
                initial_gripper=-1.,session_id=self.session_id,calibration_id=profile.stem,
                calibration_sha256=self.calibration_sha256,tracking_loss_grace_ms=0.)
            if self.drain:
                for port in [8087,8095,8105,8110,8126,8127,10505,15001]:
                    socket = context.socket(zmq.PULL)
                    socket.setsockopt(zmq.LINGER,0)
                    socket.setsockopt(zmq.CONFLATE,1)
                    socket.bind(f'tcp://127.0.0.1:{port}')
                    extras.append(socket)
            while not self.stop_event.is_set():
                with self.lock:
                    pending=self.pending_calibration
                    self.pending_calibration=None
                if pending is not None:
                    source.calibration=calibration_from_dict(pending)
                    source.digest=hashlib.sha256((ROOT/'repository_calibrations/quest_lab_current.json').read_bytes()).hexdigest()
                    source.calibration_id='vr_calibration'
                    source.armed=False;source.gate_open=False
                target = source.poll(20)
                for socket in extras:
                    try:
                        socket.recv(zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                with self.lock:
                    self.gate_open = source.gate_open
                    if target is not None:
                        self.packet = json.loads(target.to_json())
                        self.count += 1
                        self.reason = '已收到有效手柄位姿' if target.tracking_valid else '手柄追踪无效'
                    self.invalid = source.invalid_remote_count
                    self.diagnostics=dict(raw_pose_count=source.raw_count,pause_count=source.pause_count,
                        last_pause=source.last_pause,last_pause_received_monotonic_ns=source.last_pause_received_ns,
                        transport_events=list(source.transport_events))
        except Exception as exc:
            with self.lock:
                self.reason='输入端口错误：'+str(exc)
                self.gate_open=False
        finally:
            if source is not None:
                source.close()
            for socket in extras:
                socket.close(0)
            context.term()

    def snapshot(self):
        with self.lock:
            packet = self.packet.copy() if self.packet is not None else None
            return dict(packet=packet,gate_open=self.gate_open,count=self.count,invalid=self.invalid,reason=self.reason,
                        diagnostics=dict(self.diagnostics),thread_alive=self.thread.is_alive(),
                        calibration_path=str(self.profile),calibration_sha256=self.calibration_sha256)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)

    def calibrate_forward(self,start,end):
        start,end=np.asarray(start,dtype=float),np.asarray(end,dtype=float)
        if start.shape!=(3,) or end.shape!=(3,) or not np.all(np.isfinite([start,end])):
            raise ValueError('校准点无效')
        forward=end-start;forward[1]=0
        if np.linalg.norm(forward)<.05:
            raise ValueError('请沿身体前方水平移动至少 5 cm 后再确认')
        forward/=np.linalg.norm(forward)
        right=np.array([forward[2],0,-forward[0]])
        profile=dict(version=5,state='ready',origin=dict(zip('xyz',start.tolist())),
                     right=dict(zip('xyz',right.tolist())),forward=dict(zip('xyz',forward.tolist())),
                     up=dict(x=0,y=1,z=0),method='native_two_point_forward',saved_unix=time.time())
        calibration_from_dict(profile)
        (ROOT/'repository_calibrations/quest_lab_current.json').write_text(json.dumps(profile,ensure_ascii=False,indent=2),encoding='utf-8')
        with self.lock:
            self.pending_calibration=profile
        return profile
