"""Read-only check of the original DJI serial-device discovery library."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vr_config import STOCK_ROOT
STOCK = STOCK_ROOT
HELPER = STOCK / "RoboMasterClient_Data/StreamingAssets/Tools/RMClientDaemon"


def inspect():
    from serial.tools import list_ports
    dll_path = HELPER / "DeviceInfo.dll"
    library = ctypes.CDLL(str(dll_path))
    detected = {}
    for label, name in (("custom_controller", "GetCCSSerialPort"),
                        ("old_video", "GetVideoSerialPort")):
        function = getattr(library, name)
        function.argtypes = []
        function.restype = ctypes.c_int
        detected[label] = function()
    return {
        "stock_discovery_dll": str(dll_path),
        "sha256": hashlib.sha256(dll_path.read_bytes()).hexdigest(),
        "detected": detected,
        "ports": [{"port": p.device, "description": p.description, "hwid": p.hwid}
                  for p in list_ports.comports()],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-controller", type=int)
    args = parser.parse_args()
    result = inspect()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return int(args.require_controller is not None and
               result["detected"]["custom_controller"] != args.require_controller)


if __name__ == "__main__":
    raise SystemExit(main())
