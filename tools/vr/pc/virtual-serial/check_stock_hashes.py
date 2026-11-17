"""Verify the selected original client files have not changed during setup."""
import hashlib
import json
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vr_config import STOCK_ROOT
manifest = Path(__file__).with_name("stock-hashes-before.json")
expected = json.loads(manifest.read_text(encoding="utf-8"))
changed = [name for name, digest in expected.items()
           if not (STOCK_ROOT / name).is_file() or hashlib.sha256((STOCK_ROOT / name).read_bytes()).hexdigest() != digest]
print(json.dumps({"files_checked": len(expected), "unchanged": not changed, "changed": changed}))
raise SystemExit(bool(changed))
