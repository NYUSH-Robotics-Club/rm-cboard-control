"""SSH diagnostic wrapper; controller values never travel through this module."""
import json
from pathlib import Path
import shlex
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vr_config import SETTINGS


def query_board():
    repository = SETTINGS.get("jetson_vr_repository", "/home/nyu/rm-cboard-control-vr")
    elf = SETTINGS.get("jetson_mqtt_elf", repository + "/build/mqtt-vt03/infantry_standard/NYUSH_Infantry.elf")
    command = "python3 " + shlex.quote(repository + "/tools/vr/scripts/read_board_status.py") + " --elf " + shlex.quote(elf)
    result = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                             SETTINGS["diagnostic_ssh_host"], command], capture_output=True,
                            text=True, timeout=20, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout).strip()[-800:])
    return json.loads(result.stdout)


if __name__ == "__main__":
    print(json.dumps(query_board(), ensure_ascii=False, indent=2))
