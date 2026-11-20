"""Agent-friendly lifecycle for the Quest calibration page only."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import uuid
import webbrowser
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator
from urllib.error import URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

from .device_cli import status_payload as device_status_payload

STATE_SCHEMA = "quest.calibration_session/v1"
LEGACY_STATE_SCHEMA = "quest.agent_session/v1"
DEFAULT_PORT = 8766


def runtime_dir() -> Path:
    configured = os.environ.get("QUEST_TELEOP_RUNTIME_DIR")
    if configured:
        return Path(configured).expanduser().resolve()
    return Path(tempfile.gettempdir()) / f"quest-teleop-{os.getuid()}"


def _state_path() -> Path:
    return runtime_dir() / "session.json"


def _log_path() -> Path:
    return runtime_dir() / "session.log"


@contextmanager
def _lock() -> Iterator[None]:
    runtime_dir().mkdir(parents=True, exist_ok=True)
    with (runtime_dir() / "session.lock").open("a+", encoding="utf-8") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def _read_state() -> dict[str, Any] | None:
    try:
        value = json.loads(_state_path().read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(value, dict):
        return None
    if value.get("schema_version") not in {STATE_SCHEMA, LEGACY_STATE_SCHEMA}:
        return None
    return value if value.get("kind") == "calibration" else None


def _write_state(state: dict[str, Any]) -> None:
    runtime_dir().mkdir(parents=True, exist_ok=True)
    temporary = runtime_dir() / f"session.{os.getpid()}.tmp"
    temporary.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(_state_path())


def _process(pid: int) -> tuple[bool, str]:
    result = subprocess.run(
        ["ps", "-p", str(pid), "-o", "stat=", "-o", "command="],
        check=False,
        capture_output=True,
        text=True,
        timeout=1.0,
    )
    line = result.stdout.strip()
    if result.returncode != 0 or not line:
        return False, ""
    state, _, command = line.partition(" ")
    return "Z" not in state, command.strip()


def _managed(state: dict[str, Any]) -> bool:
    try:
        alive, command = _process(int(state["pid"]))
    except (KeyError, TypeError, ValueError, OSError, subprocess.SubprocessError):
        return False
    return alive and any(
        marker in command
        for marker in (
            "run_calibration.sh",
            "quest_trajectory_recorder.calibration_runtime",
            "quest_trajectory_recorder.live3d",
        )
    )


def _ready(url: str, timeout: float = 0.25) -> bool:
    try:
        with urlopen(url, timeout=timeout) as response:  # noqa: S310 - fixed local URL
            return response.status == 200
    except (OSError, TimeoutError, URLError, ValueError):
        return False


def _terminate(state: dict[str, Any], timeout: float) -> bool:
    if not _managed(state):
        return True
    pid = int(state["pid"])
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _process(pid)[0]:
            return True
        time.sleep(0.1)
    if _managed(state):
        os.killpg(pid, signal.SIGKILL)
    return not _process(pid)[0]


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def start(args: argparse.Namespace) -> int:
    port = int(os.environ.get("CALIBRATION_WEB_PORT", DEFAULT_PORT))
    # A running source owns both the raw input and its persistent editor. Reuse
    # that editor instead of launching another raw receiver on the same ports.
    url = f"http://127.0.0.1:{port}/"
    try:
        with urlopen(url + "editor/status", timeout=0.5) as response:
            editor = json.load(response)
    except (OSError, ValueError):
        editor = None
    if isinstance(editor, dict) and editor.get("schema_version") == "quest.calibration_editor/v1":
        request = Request(
            url + "editor/command",
            method="POST",
            headers={"Content-Type": "application/json"},
            data=json.dumps({"action": "begin", "request_id": str(uuid.uuid4())}).encode(),
        )
        try:
            with urlopen(request, timeout=3) as response:
                result = json.load(response)
            if not result.get("applied"):
                raise ValueError(result.get("message", "Source did not accept calibration"))
        except (OSError, ValueError) as exc:
            print(f"START FAILED: {exc}")
            return 1
        url += "?" + urlencode({"profile": args.profile})
        webbrowser.open(url)
        print(f"READY source-owned editor; saving as profile={args.profile}\nURL {url}")
        return 0
    command = [
        sys.executable,
        "-m",
        "quest_trajectory_recorder.calibration_runtime",
        "--profile",
        args.profile,
        "--web-port",
        str(port),
    ]
    requested = {
        "schema_version": STATE_SCHEMA,
        "session_id": str(uuid.uuid4()),
        "kind": "calibration",
        "label": "Quest calibration",
        "profile": args.profile,
        "service_url": f"http://127.0.0.1:{port}/",
        "log_path": str(_log_path()),
    }
    with _lock():
        existing = _read_state()
        if existing is not None and _managed(existing):
            if existing.get("profile") == args.profile and _ready(existing["service_url"]):
                print(
                    f"ALREADY RUNNING Quest calibration pid={existing['pid']} "
                    f"url={existing['service_url']}"
                )
                return 0
            print(f"START FAILED: calibration is already running (pid {existing.get('pid')})")
            return 1
        with _log_path().open("w", encoding="utf-8") as log:
            process = subprocess.Popen(  # noqa: S603 - fixed local launcher
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        requested.update({"pid": process.pid, "status": "starting", "started_at": _utc_now()})
        _write_state(requested)
    deadline = time.monotonic() + args.wait_seconds
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        if _ready(requested["service_url"]):
            requested.update({"status": "ready", "ready_at": _utc_now()})
            with _lock():
                _write_state(requested)
            print(f"READY Quest calibration pid={process.pid} profile={args.profile}")
            print(f"URL {requested['service_url']}")
            return 0
        time.sleep(0.1)
    _terminate(requested, 8.0)
    requested.update({"status": "failed", "stopped_at": _utc_now()})
    with _lock():
        _write_state(requested)
    print("START FAILED: calibration page did not become ready")
    return 1


def status(args: argparse.Namespace) -> int:
    state = _read_state()
    if state is None or not _managed(state):
        payload = {"schema_version": STATE_SCHEMA, "state": "stopped", "running": False}
        url = f"http://127.0.0.1:{int(os.environ.get('CALIBRATION_WEB_PORT', DEFAULT_PORT))}/"
        try:
            with urlopen(url + "editor/status", timeout=args.probe_timeout) as response:
                editor = json.load(response)
            if editor.get("schema_version") == "quest.calibration_editor/v1":
                payload.update(
                    {
                        "running": True,
                        "state": "ready",
                        "owner": "source",
                        "pid": None,
                        "service_url": url,
                        "calibration_ready": True,
                        "profile": editor["profile"],
                        "editor": editor,
                    }
                )
        except (OSError, ValueError, AttributeError):
            pass
    else:
        page_ready = _ready(state["service_url"], args.probe_timeout)
        try:
            quest = device_status_payload()
        except (OSError, subprocess.SubprocessError):
            quest = None
        ready = page_ready and bool(quest and quest.get("ready"))
        payload = {
            "schema_version": STATE_SCHEMA,
            "state": "ready" if ready else "degraded",
            "running": True,
            "profile": state.get("profile"),
            "pid": state.get("pid"),
            "service_url": state.get("service_url"),
            "calibration_ready": page_ready,
            "quest": quest,
        }
    if args.as_json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    elif payload["running"]:
        print(f"{payload['state'].upper()} Quest calibration pid={payload['pid']}")
        print(f"URL {payload['service_url']}")
    else:
        print("STOPPED no managed Quest calibration")
    return 0 if payload["state"] == "ready" else 1


def stop(args: argparse.Namespace) -> int:
    state = _read_state()
    if state is None or not _managed(state):
        print("STOPPED no managed Quest calibration")
        return 0
    if not _terminate(state, args.wait_seconds):
        print(f"STOP FAILED: Quest calibration pid={state['pid']} is still alive")
        return 1
    state.update({"status": "stopped", "stopped_at": _utc_now(), "schema_version": STATE_SCHEMA})
    with _lock():
        _write_state(state)
    print(f"STOPPED Quest calibration pid={state['pid']}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    start_parser = subparsers.add_parser("start")
    start_parser.add_argument("--profile", default="lab")
    start_parser.add_argument("--wait-seconds", type=float, default=180.0)
    status_parser = subparsers.add_parser("status")
    status_parser.add_argument("--json", action="store_true", dest="as_json")
    status_parser.add_argument("--probe-timeout", type=float, default=0.6)
    stop_parser = subparsers.add_parser("stop")
    stop_parser.add_argument("--wait-seconds", type=float, default=12.0)
    logs_parser = subparsers.add_parser("logs")
    logs_parser.add_argument("--lines", type=int, default=80)
    args = parser.parse_args(argv)
    if args.command == "start":
        return start(args)
    if args.command == "status":
        return status(args)
    if args.command == "stop":
        return stop(args)
    try:
        lines = _log_path().read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        print(f"No calibration log at {_log_path()}")
        return 1
    print("\n".join(lines[-args.lines :]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
