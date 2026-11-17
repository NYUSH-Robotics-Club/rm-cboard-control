import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_runtime"))
import vr_input
from run_repository_calibration import main
# Active ADB reverse mappings were configured by this session. Start listeners
# without blocking on optional mappings before the read-only calibration opens.
vr_input.prepare_usb = lambda: {"ready": True, "transport": "existing ADB reverse"}
raise SystemExit(main())
