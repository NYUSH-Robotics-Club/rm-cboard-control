"""Bundle paths and local settings; config.local.json is never committed."""
import json
from pathlib import Path
VR_ROOT = Path(__file__).resolve().parents[1]
_file = VR_ROOT / "config.local.json"
if not _file.exists():
    _file = VR_ROOT / "config.example.json"
SETTINGS = json.loads(_file.read_text(encoding="utf-8-sig"))
STOCK_ROOT = Path(SETTINGS["stock_client_root"])
RUNTIME_ROOT = VR_ROOT / "runtime"
RUNTIME_ROOT.mkdir(exist_ok=True)
PROFILE = VR_ROOT / "quest_runtime/repository_calibrations/quest_lab_current.json"
