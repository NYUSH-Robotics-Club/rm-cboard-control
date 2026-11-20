"""Disposable test data; never uses the real editor layout."""
import sys
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import make_server
with tempfile.TemporaryDirectory(prefix="isolated-xr-browser-test-") as directory:
    http = make_server(18781, directory)
    print("TEST_READY", flush=True)
    try: http.serve_forever()
    except KeyboardInterrupt: pass
    finally: http.server_close()
