"""Explicit WebXR -> official MQTT publisher; no SSH/serial drive commands."""
import importlib.util
import json
from pathlib import Path
import sys
import time
from urllib.request import Request,urlopen

ROOT=Path(__file__).resolve().parent
LEGACY=ROOT.parent/'firmware/tools/vr/pc/mqtt'
sys.path.insert(0,str(LEGACY))
# Reuse tested official MQTT transport and exclusive publisher lock, not its
# FrankaBot listener. All old files stay unchanged.
from bridge import Transport,publisher_lock
spec=importlib.util.spec_from_file_location('dual_wire',ROOT.parent/'firmware/tools/vr/pc/frankabot_vehicle.py')
wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(wire)
URL='http://127.0.0.1:18780'
STOP=ROOT/'data/vehicle-stop.request'

def api(path,data=None,token=None):
    headers={'Content-Type':'application/json'}
    if token:headers['X-Studio-Token']=token
    req=Request(URL+path,data=json.dumps(data).encode() if data is not None else None,headers=headers)
    with urlopen(req,timeout=.6) as response:return json.load(response)

def run():
    token=api('/api/bootstrap')['token']
    api('/api/control',{'action':'disarm'},token)
    transport=Transport('127.0.0.1',3333,'1')
    sequence=int(time.time_ns()//1000000)&65535;generation=-1;primed=False
    STOP.unlink(missing_ok=True)
    try:
        while not STOP.exists():
            tick=time.monotonic();output=(False,0,0,0);error=''
            connected=transport.connected.is_set()
            try:
                # Read endpoints still work after a server restart, so a failed
                # status POST alone cannot detect it before publishing. Check
                # the server epoch first and always recover in neutral.
                current_token=api('/api/bootstrap')['token']
                if current_token!=token:
                    token=current_token;primed=False;generation=-1
                    api('/api/control',{'action':'disarm'},token)
                if generation!=transport.generation or not connected:
                    api('/api/control',{'action':'disarm'},token)
                    generation=transport.generation;primed=False
                status=api('/api/status')
                c=status['control']
                if connected and primed and status['fresh'] and c['armed']:
                    output=(bool(c['enabled']),int(c['forward']),int(c['right']),int(c['rotation']))
            except Exception as exc:
                error=str(exc);primed=False
                # UI server failure cannot retain a command. Recovery requires
                # an explicit arm again; never reuse an old successful sample.
                try:
                    # A UI-service restart changes its CSRF token. Reacquire
                    # ownership only in the disabled state, then require arm.
                    token=api('/api/bootstrap')['token']
                    api('/api/control',{'action':'disarm'},token)
                except Exception:pass
            sequence=(sequence+1)&65535
            success=transport.send(wire.pack_payload(sequence,*output[:3],rotation=output[3]))
            primed=primed or success
            record={'running':True,'connected':connected,'sequence':sequence,'sent':transport.published,'error':error or transport.error,'output':output if success else None,'time':time.time(),'publish_ok':success}
            try:api('/api/publisher-status',record,token)
            except Exception:
                primed=False
                try:
                    token=api('/api/bootstrap')['token']
                    api('/api/control',{'action':'disarm'},token)
                except Exception:pass
            time.sleep(max(0,.05-(time.monotonic()-tick)))
    finally:
        transport.send(wire.pack_payload((sequence+1)&65535,False,0,0,rotation=0))
        transport.close()
        try:
            api('/api/control',{'action':'disarm'},token)
            api('/api/publisher-status',{'running':False,'connected':False,'output':[False,0,0,0],'time':time.time()},token)
        except Exception:pass

if __name__=='__main__':
    with publisher_lock():run()
