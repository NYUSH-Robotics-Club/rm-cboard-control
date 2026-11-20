"""Run the repository's original calibration HTML/receiver on Windows.

The repository files remain unchanged. Only its atomic JSON writer import is
replaced in memory: importing embodied_ops itself would load POSIX-only fcntl.
This program owns the raw Quest ports; close it before starting run_vr.py.
"""
import ast
import json
import os
import socket
import webbrowser
from pathlib import Path
import subprocess
import sys
import tempfile
import types

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parent/'vendor/quest-trajectory-recorder'
sys.path.insert(0,str(REPO/'src'))
CALIBRATION=ROOT/'repository_calibrations'/'quest_lab_current.json'


def atomic_write_json(path,data):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    encoded=json.dumps(data,indent=2,sort_keys=True,allow_nan=False).encode()+b'\n'
    with tempfile.NamedTemporaryFile(dir=path.parent,prefix='.calibration-',suffix='.tmp',delete=False) as f:
        temporary=Path(f.name)
        f.write(encoded);f.flush();os.fsync(f.fileno())
    os.replace(temporary,path)


def load_original_web():
    name='quest_trajectory_recorder.live3d_web'
    if name in sys.modules:
        return sys.modules[name]
    path=REPO/'src/quest_trajectory_recorder/live3d_web.py'
    tree=ast.parse(path.read_text(encoding='utf-8'),filename=str(path))
    replaced=0
    for index,node in enumerate(tree.body):
        if isinstance(node,ast.ImportFrom) and node.module=='embodied_ops.teleop':
            if len(node.names)!=1 or node.names[0].name!='atomic_write_json':
                raise RuntimeError('Upstream writer import changed; review Windows adapter.')
            tree.body[index]=ast.copy_location(ast.Pass(),node);replaced+=1
    if replaced!=1:
        raise RuntimeError('Expected exactly one POSIX-dependent writer import.')
    module=types.ModuleType(name)
    module.__file__=str(path)
    module.__package__='quest_trajectory_recorder'
    module.atomic_write_json=atomic_write_json
    sys.modules[name]=module
    try:
        exec(compile(tree,str(path),'exec'),module.__dict__)
    except Exception:
        sys.modules.pop(name,None)
        raise
    return module


def main():
    for port in (8766,18872):
        with socket.socket() as probe:
            probe.settimeout(.2)
            if port==18872:
                try:
                    probe.bind(('127.0.0.1',port));occupied=False
                except OSError:
                    occupied=True
            else:
                occupied=probe.connect_ex(('127.0.0.1',port))==0
        if not occupied:continue
        if port==8766:
            webbrowser.open('http://127.0.0.1:8766/')
        else:
            from tkinter import messagebox
            messagebox.showinfo('请先关闭机械臂控制','请先关闭 MuJoCo 的 VR 操作台，再打开校准入口。校准与控制共用手柄端口。')
        return 0
    load_original_web()
    from vr_input import prepare_usb
    try:
        print(json.dumps(prepare_usb(),ensure_ascii=False),flush=True)
    except Exception as exc:
        print('Calibration page will wait for Quest: '+str(exc),flush=True)
    from quest_trajectory_recorder.live3d import main as original_main
    return original_main(['--open-browser','--web-host','127.0.0.1','--web-port','8766',
        '--calibration-out',str(CALIBRATION),'--out-dir',str(ROOT/'repository_calibration_sessions'),
        '--no-record','--no-gate'])


if __name__=='__main__':
    raise SystemExit(main())
