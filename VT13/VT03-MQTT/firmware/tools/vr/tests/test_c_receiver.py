"""Cross-language test: Python wire frames into the repository's actual C parser."""
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from frankabot_vehicle import pack as pack_frame


def run(command):
    def pack(*args, **kwargs):
        return pack_frame(*args, **kwargs, command=command)
    compiler = shutil.which(os.environ.get("CC", "cc"))
    if not compiler:
        raise SystemExit("A host C compiler is required; no hardware is accessed.")
    events = []
    def add(t, frame, expected): events.append((t, frame, expected))
    add(0, pack(1,False,0,diagnostic=True), (1,0,0,0))
    add(1, pack(2,True,50), (2,0,0,0))
    add(2, pack(3,False,0), (3,1,0,0))
    add(3, pack(4,True,50,-25), (4,2,50,-25))
    add(4, pack(4,True,50,-25), (5,2,50,-25))
    add(5, pack(3,True,-50), (6,2,50,-25))
    corrupt=bytearray(pack(5,True,1)); corrupt[-1]^=1
    add(6,bytes(corrupt),(6,2,50,-25))
    frame=pack(5,True,-100,100)
    # Interleave the VT03's native 21-byte RC stream with custom frames.
    add(7,b"\xa9\x53" + bytes(19) + b"noise"+frame[:11],(6,2,50,-25))
    add(8,frame[11:],(7,3,-100,100))
    add(300,pack(6,True,30),(8,3,-100,100))
    add(301,pack(7,False,0),(9,4,0,0))
    add(302,pack(8,True,30),(10,5,30,0))
    add(303,None,(10,5,66,0))
    add(304,pack(9,True,70),(11,5,66,0))
    add(505,pack(10,True,70),(12,5,66,0))
    add(506,pack(11,False,0),(13,6,0,0))
    add(507,pack(12,True,70),(14,7,70,0))
    with tempfile.TemporaryDirectory(prefix="vr-receiver-test-") as temp:
        binary=Path(temp)/"receiver"
        fixture=Path(temp)/"frames.bin"
        include=["modules/remote","core/contracts","bsp/remote","bsp/time","bsp/critical","modules/message_center"]
        subprocess.run([compiler,"-std=c11","-Wall","-Wextra","-Werror",*["-I"+str(REPO/p) for p in include],
                        str(REPO/"modules/remote/remote_control.c"), str(ROOT/"tests/receiver_harness.c"),"-o",str(binary)],check=True)
        with fixture.open("wb") as file:
            for stamp,frame,_ in events:
                file.write(struct.pack("<IH",stamp,65535 if frame is None else len(frame)))
                if frame is not None:file.write(frame)
        output=subprocess.check_output([str(binary),str(fixture)],text=True)
        actual=[tuple(map(int,line.split())) for line in output.splitlines()]
        assert [row[:4] for row in actual]==[expected for _,_,expected in events], (actual,events)
        assert all(row[4:] == (0, 0) for row in actual), "legacy path must never command chassis rotation or gimbal yaw"
    print(f"C_RECEIVER_INTEROP_OK: command={command:#06x}; 17 checks (CRC, split/native RC/noise, directions, sequence, timeout/rearm, DBUS priority)")


def main():
    run(0x0302)
    run(0x0311)


if __name__=="__main__": main()
