"""Explicit USB preparation; starts FrankaBot only when --launch is requested."""
import argparse, json, subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "quest_runtime"))
import vr_input
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--serial")
parser.add_argument("--launch", action="store_true")
args=parser.parse_args()
if args.serial: vr_input.SERIAL=args.serial
state=vr_input.prepare_usb()
print(json.dumps(state, ensure_ascii=False, indent=2))
if not state["ready"]: raise SystemExit(2)
if args.launch and not state.get("app_running"):
    result=vr_input.adb_command("shell", "am", "start", "-n",
        "com.Xigbee.FrankaBot/com.unity3d.player.UnityPlayerActivity", timeout=12)
    print(result.stdout, result.stderr)
    raise SystemExit(result.returncode)
