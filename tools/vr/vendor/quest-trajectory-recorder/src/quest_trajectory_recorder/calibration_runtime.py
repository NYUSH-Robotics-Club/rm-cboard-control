"""Package-native Quest calibration runtime with hot ADB attachment."""

from __future__ import annotations

import argparse
import threading

from .calibration_profiles import profile_path
from .device_cli import main as device_main
from .live3d import DEFAULT_CALIBRATION_WEB_PORT
from .live3d import main as live3d_main
from .quest_ports import adb_connected


def _attach_quest(stop: threading.Event) -> None:
    announced_wait = False
    while not stop.is_set():
        if not adb_connected():
            if not announced_wait:
                print("Calibration page is ready; waiting for authorized Quest ADB.", flush=True)
                announced_wait = True
            stop.wait(1.0)
            continue
        announced_wait = False
        try:
            if device_main(["prepare", "--wait-seconds", "0"]) == 0:
                print("Quest attached to the live calibration page.", flush=True)
            else:
                print("Quest preparation failed; calibration remains available.", flush=True)
        except (OSError, RuntimeError):
            print("Quest preparation failed; calibration remains available.", flush=True)
        while adb_connected() and not stop.wait(1.0):
            pass
        if not stop.is_set():
            print("Quest disconnected; waiting to restore the calibration link.", flush=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="lab")
    parser.add_argument("--web-port", type=int, default=DEFAULT_CALIBRATION_WEB_PORT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    # Calibration is always authored in user storage. Legacy checkout profiles are
    # read-only migration inputs and must never become an editor destination.
    calibration = profile_path(args.profile, legacy_dir=None)
    stop = threading.Event()
    attachment = threading.Thread(
        target=_attach_quest,
        args=(stop,),
        name="quest-calibration-adb",
        daemon=True,
    )
    attachment.start()
    try:
        return live3d_main(
            [
                "--open-browser",
                "--web-port",
                str(args.web_port),
                "--calibration-out",
                str(calibration),
            ]
        )
    finally:
        stop.set()
        attachment.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
