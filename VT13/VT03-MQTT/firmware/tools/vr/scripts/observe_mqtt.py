"""Record PC status and read-only SWD observations; never publishes a command."""
import argparse
import json
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc/mqtt"))
sys.path.insert(0, str(ROOT / "pc"))
from board_status import query_board
from vr_config import RUNTIME_ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=90)
    parser.add_argument("--output", type=Path, default=RUNTIME_ROOT / "mqtt-observations.json")
    args = parser.parse_args()
    stop = threading.Event()
    boards, errors, samples = [], [], []

    def read_board():
        while not stop.is_set():
            try:
                data = {**query_board(), "host_received_time": time.time()}
                boards.append(data)
                p = RUNTIME_ROOT / "mqtt-board-status.json"
                temp = p.with_suffix(".pending")
                temp.write_text(json.dumps(data), encoding="utf-8")
                try: temp.replace(p)
                except OSError: pass
            except Exception as exc:
                errors.append(dict(time=time.time(), error=str(exc)))
            stop.wait(.6)

    thread = threading.Thread(target=read_board, daemon=True)
    thread.start()
    deadline = time.monotonic() + args.seconds
    fields = ["host_time", "sequence", "output", "samples", "tracking_valid", "gate_fresh",
              "trigger", "last_publish_ok", "frames_sent", "calibration_sha256"]
    last_sequence = None
    while time.monotonic() < deadline:
        try:
            data = json.loads((RUNTIME_ROOT / "mqtt-status.json").read_text(encoding="utf-8"))
            if data.get("sequence") != last_sequence:
                samples.append({key: data.get(key) for key in fields})
                last_sequence = data.get("sequence")
        except (OSError, ValueError):
            pass
        time.sleep(.025)
    stop.set()
    thread.join(timeout=20)
    args.output.write_text(json.dumps(dict(pc=samples, board=boards, errors=errors), indent=2), encoding="utf-8")
    print(json.dumps(dict(pc_samples=len(samples), board_samples=len(boards), errors=len(errors))))


if __name__ == "__main__": main()
