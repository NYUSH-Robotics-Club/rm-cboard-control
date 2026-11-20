"""Exercise real receiver: VRD3 rotation only enters RC channel 4."""
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from frankabot_vehicle import pack, crc

def main():
    def wire(seq, enabled=False, forward=0, right=0, rotation=0, diagnostic=False):
        return pack(seq,enabled,forward,right,diagnostic,command=0x0311,rotation=rotation)
    events=[(0,wire(1,rotation=0,diagnostic=True),(1,0,0,0,0,0)),
            (1,wire(2,True,rotation=100),(2,0,0,0,0,0)),
            (2,wire(3),(3,1,0,0,0,0)),
            (3,wire(4,True,30,-20,-100),(4,2,30,-20,-100,0)),
            (4,wire(5,True,rotation=100),(5,3,0,0,100,0)),
            (5,pack(6,True,40,-10,command=0x0311),(6,4,40,-10,0,0))]
    # Correct outer CRC but invalid reserved byte/range/disabled rotation.
    def corrupt(seq, offset, value):
        frame=bytearray(wire(seq));frame[7+offset:7+offset+len(value)]=value
        frame[-2:]=struct.pack('<H',crc(frame[:-2],0xFFFF,0x8408));return bytes(frame)
    events += [(6,corrupt(7,14,b'\x01'),(6,4,40,-10,0,0)),
               (7,corrupt(8,12,struct.pack('<h',101)),(6,4,40,-10,0,0)),
               (8,corrupt(9,12,struct.pack('<h',50)),(6,4,40,-10,0,0)),
               (300,wire(10,True,rotation=50),(7,4,40,-10,0,0)),
               (301,wire(11),(8,5,0,0,0,0)),
               (302,wire(12,True,rotation=-50),(9,6,0,0,-50,0)),
               (303,None,(9,6,66,0,0,0)),
               (304,wire(13,True,rotation=50),(10,6,66,0,0,0))]
    cc=shutil.which(os.environ.get('CC','cc'))
    if not cc: raise RuntimeError('host C compiler required')
    with tempfile.TemporaryDirectory() as directory:
        binary=Path(directory)/'receiver';fixture=Path(directory)/'frames.bin'
        includes=['modules/remote','core/contracts','bsp/remote','bsp/time','bsp/critical','modules/message_center']
        subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror',*['-I'+str(REPO/p) for p in includes],str(REPO/'modules/remote/remote_control.c'),str(ROOT/'tests/receiver_harness.c'),'-o',str(binary)],check=True)
        with fixture.open('wb') as f:
            for stamp,frame,_ in events:
                f.write(struct.pack('<IH',stamp,65535 if frame is None else len(frame)))
                if frame is not None:f.write(frame)
        actual=[tuple(map(int,row.split())) for row in subprocess.check_output([str(binary),str(fixture)],text=True).splitlines()]
        assert actual==[expected for _,_,expected in events],actual
    print('DUAL_RECEIVER_OK: 14 checks; rotation ch4 only, yaw ch0 always zero, VRD2 compatible, DBUS priority preserved')
if __name__=='__main__': main()
