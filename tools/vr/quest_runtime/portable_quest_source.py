"""Windows adapter for the repository's Quest wire protocol.

Uses its receiver parser and coordinate functions directly. The upstream target
wrapper imports embodied_ops -> fcntl at package initialization, so only that
wrapper is replaced here. No upstream code or Quest file is modified.
"""
from dataclasses import dataclass
import json
import math
import time
import numpy as np
import zmq
from zmq.utils.monitor import recv_monitor_message
from quest_trajectory_recorder.receiver import parse_remote_text
from quest_trajectory_recorder.teleop_frame import quest_pos_to_teleop,quest_rotation_to_teleop_matrix


@dataclass
class NativeTarget:
    payload: dict

    @property
    def tracking_valid(self):
        return self.payload['tracking_valid']

    def to_json(self):
        return json.dumps(self.payload,allow_nan=False)


class DirectQuestTargetSource:
    def __init__(self,*,context,host,remote_port,pause_port,calibration,no_gate=False,
                 trajectory_gate_pause='High',allow_initial_high=False,gripper_mode='hold',
                 initial_gripper=-1.,session_id='',calibration_id=None,calibration_sha256=None,
                 tracking_loss_grace_ms=0):
        if no_gate or trajectory_gate_pause!='High' or allow_initial_high or gripper_mode!='hold':
            raise ValueError('Native adapter requires Low -> High clutch and hold-trigger semantics')
        self.context=context;self.calibration=calibration
        self.session_id=session_id;self.calibration_id=calibration_id;self.digest=calibration_sha256
        self.gate_open=False;self.armed=False;self.gripper=initial_gripper
        self.invalid_remote_count=0;self.raw_count=0
        self.pause_count=0;self.last_pause=None;self.transport_events=[];self.monitors=[]
        self.last_pause_received_ns=0
        self.poller=zmq.Poller()
        sockets=[]
        try:
            for port in [remote_port,pause_port]:
                sock=context.socket(zmq.PULL)
                sock.setsockopt(zmq.LINGER,0);sock.setsockopt(zmq.CONFLATE,1)
                self.monitors.append(sock.get_monitor_socket())
                sock.bind(f'tcp://{host}:{port}')
                self.poller.register(sock,zmq.POLLIN);sockets.append(sock)
        except Exception:
            for sock in sockets:
                sock.close(0)
            raise
        self.remote,self.pause=sockets

    def poll(self,timeout_ms):
        for index,monitor in enumerate(self.monitors):
            while True:
                try:
                    event=recv_monitor_message(monitor,flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                self.transport_events.append(dict(channel='pose' if index==0 else 'gate',
                    event=zmq.Event(event['event']).name,value=int(event['value'])))
                self.transport_events=self.transport_events[-12:]
        ready=dict(self.poller.poll(timeout_ms))
        if self.pause in ready:
            state=self.pause.recv().decode('utf-8',errors='replace').strip('\x00\r\n\t :')
            self.pause_count+=1;self.last_pause=state
            self.last_pause_received_ns=time.monotonic_ns()
            if state=='Low':
                self.armed=True;self.gate_open=False
            elif state=='High':
                self.gate_open=self.armed
            else:
                self.gate_open=False
        if self.remote not in ready:
            return None
        stamp=time.monotonic_ns();self.raw_count+=1
        try:
            remote=parse_remote_text(self.remote.recv().decode('utf-8'))
            if remote is None:
                raise ValueError('malformed pose')
            position=np.asarray(remote['position']);quaternion=np.asarray(remote['rotation'])
            if (not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)) or
                np.linalg.norm(position)<1e-8 or np.linalg.norm(quaternion)<1e-6):
                raise ValueError('invalid tracking')
            if self.gate_open:
                self.gripper=1. if remote['flag'] else -1.
            p=quest_pos_to_teleop(position,self.calibration)
            R=quest_rotation_to_teleop_matrix(quaternion,self.calibration)
            return NativeTarget(dict(schema='embodied.teleop_target/v1',seq=self.raw_count,frame_id=self.raw_count,
                session_id=self.session_id,source='quest',position=p,rotation=R,gripper=self.gripper,
                gate_open=self.gate_open,tracking_valid=True,host_received_monotonic_ns=stamp,
                source_metadata=dict(raw_position=remote['position'],raw_rotation=remote['rotation'],flag=remote['flag'],
                                     calibration_id=self.calibration_id,calibration_sha256=self.digest,
                                     adapter='Windows native wrapper; upstream parser and coordinate math')))
        except (ValueError,TypeError,KeyError) as exc:
            self.invalid_remote_count+=1
            return NativeTarget(dict(schema='embodied.teleop_target/v1',seq=self.raw_count,frame_id=self.raw_count,
                session_id=self.session_id,tracking_valid=False,gate_open=False,host_received_monotonic_ns=stamp,
                source_metadata=dict(reason=str(exc))))

    def close(self):
        for monitor in self.monitors:
            monitor.close(0)
        for sock in [self.remote,self.pause]:
            sock.disable_monitor()
            self.poller.unregister(sock);sock.close(0)
