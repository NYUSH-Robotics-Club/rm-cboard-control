"""Run existing calibrated Quest control through a virtual COM pair only."""
import argparse
import ctypes
import sys
from pathlib import Path

from discovery import inspect
from vr_config import SETTINGS, PROFILE, RUNTIME_ROOT

ROOT = Path(__file__).resolve().parents[1]


def acquire_instance():
    from ctypes import wintypes
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.CreateMutexW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR]
    kernel.CreateMutexW.restype = wintypes.HANDLE
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.CloseHandle.restype = wintypes.BOOL
    handle = kernel.CreateMutexW(None, False, "Local\\FrankaBotVehicleVirtualSerial")
    error = ctypes.get_last_error()
    if not handle:
        raise ctypes.WinError(error)
    if error == 183:
        kernel.CloseHandle(handle)
        return None, None
    return kernel, handle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--writer", default=SETTINGS["writer_port"])
    parser.add_argument("--client-port", type=int, default=SETTINGS["player_port"])
    parser.add_argument("--serial", default=SETTINGS["quest_serial"])
    parser.add_argument("--neutral-only", action="store_true")
    parser.add_argument("--seconds", type=float)
    args = parser.parse_args()
    result = inspect()
    if result["detected"]["custom_controller"] != args.client_port:
        parser.exit(2, "Original client discovery does not select COM%d; no VR commands sent.\n" % args.client_port)
    if args.writer.upper() in {"COM%d" % args.client_port,
                              "COM%d" % result["detected"]["old_video"]}:
        parser.error("VR must open the other end of the pair, not the client or VT12 COM port")
    kernel, instance = acquire_instance()
    if instance is None:
        print("VR virtual-serial listener is already running; no second instance started.")
        return 0
    sys.path.insert(0, str(ROOT))
    from frankabot_vehicle import main as run
    sys.argv = ["frankabot_vehicle.py", "--port", args.writer, "--existing-usb",
                "--serial", args.serial, "--calibration",
                str(PROFILE),
                "--status-file", str(RUNTIME_ROOT / "virtual-status.json")]
    if args.neutral_only:
        sys.argv += ["--neutral-only"]
    if args.seconds is not None:
        sys.argv += ["--seconds", str(args.seconds)]
    print("Transport: Quest -> %s -> virtual pair -> stock COM%d -> VT12 -> RF -> VT02 -> UART1" %
          (args.writer, args.client_port), flush=True)
    try:
        return run()
    finally:
        kernel.CloseHandle(instance)


if __name__ == "__main__":
    raise SystemExit(main())
