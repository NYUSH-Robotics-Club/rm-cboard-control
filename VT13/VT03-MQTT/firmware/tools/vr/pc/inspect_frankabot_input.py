"""Read the existing FrankaBot input without starting simulation or drive output."""

import argparse
import json
from pathlib import Path
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / "quest_runtime"))
from vr_input import QuestInput, adb_command, prepare_usb
import vr_input


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--serial", help="ADB serial when using a different Quest.")
    parser.add_argument("--prepare-usb", action="store_true",
                        help="Restore missing ADB reverse mappings; do not start the APK.")
    args = parser.parse_args()
    if args.serial:
        vr_input.SERIAL = args.serial
    if not 0 < args.seconds <= 120:
        parser.error("--seconds must be in (0, 120]")
    state = adb_command("get-state")
    if state.returncode or state.stdout.strip() != "device":
        print(json.dumps({"ready": False, "reason": "Quest is not visible to ADB",
                          "adb": state.stderr.strip()}, ensure_ascii=False))
        return 2
    if args.prepare_usb:
        ready = prepare_usb()
        if not ready["ready"]:
            print(json.dumps(ready, ensure_ascii=False))
            return 2
    source = QuestInput()
    source.start()
    deadline = time.monotonic() + args.seconds
    last = 0
    try:
        while time.monotonic() < deadline:
            time.sleep(0.2)
            snap = source.snapshot()
            if snap["count"] != last and snap["packet"]:
                last = snap["count"]
                print(json.dumps(snap["packet"], ensure_ascii=False), flush=True)
            if not snap["thread_alive"]:
                break
        summary = source.snapshot()
        summary.pop("packet")
        print(json.dumps(summary, ensure_ascii=False))
        return 0 if summary["count"] > summary["invalid"] else 3
    finally:
        source.stop_event.set()
        source.thread.join(timeout=2)


if __name__ == "__main__":
    raise SystemExit(main())
