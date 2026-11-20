"""One raw input owner, with an optional live calibration editor.

All changes are applied on the source ingest thread. HTTP workers enqueue bounded
requests; they never touch a raw socket or apply a profile behind the source's back.
"""

from __future__ import annotations

import hashlib
import json
import queue
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from embodied_ops.teleop import atomic_write_json

from .calibration_profiles import sanitize_profile
from .live_state import LiveState
from .receiver import iso_now
from .teleop_frame import calibration_from_dict, load_quest_calibration


@dataclass
class PendingRequest:
    value: dict[str, Any]
    deadline: float
    done: threading.Event = field(default_factory=threading.Event)
    result: dict[str, Any] | None = None


class CalibrationSession:
    def __init__(self, path: Path, *, url: str, storage_dir: Path | None = None) -> None:
        self.path = path
        self.storage_dir = storage_dir or path.parent
        self.calibration = load_quest_calibration(path)
        self.digest = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        self.state = "teleop" if self.calibration is not None else "calibrating"
        self.last_action: str | None = None
        self.revision = str(uuid.uuid4())
        self.url = url
        self.live = LiveState(max_points=5000)
        self.saw_pause = False
        self.last_pose_at: float | None = None
        self.lock = threading.RLock()
        self.requests: queue.Queue[PendingRequest] = queue.Queue(maxsize=16)
        self.responses: dict[str, dict[str, Any]] = {}

    @property
    def enabled(self) -> bool:
        return self.state == "teleop" and self.calibration is not None

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "schema_version": "quest.calibration_editor/v1",
                "state": self.state,
                "last_action": self.last_action,
                "active": self.state == "calibrating",
                "revision": self.revision,
                "profile": self.path.stem,
                "url": self.url,
                "calibration_sha256": self.digest,
                "tracking_valid": self.last_pose_at is not None
                and time.monotonic() - self.last_pose_at <= 0.5,
            }

    def metadata(self) -> dict[str, Any]:
        with self.lock:
            return {
                "calibration_valid": self.state != "calibrating" and self.calibration is not None,
                "calibration_revision": self.revision,
                "calibration_editor": self.snapshot(),
                "effective_calibration": asdict(self.calibration) if self.calibration else None,
            }

    def pause(self, paused: bool) -> None:
        if self.state == "awaiting_b":
            if paused:
                self.saw_pause = True
            elif self.saw_pause:
                self.state = "teleop"
                self._announce("resume")

    def observe(self, remote: dict[str, Any] | None, seq: int, now: float) -> None:
        self.last_pose_at = now if remote is not None else None
        if self.state != "calibrating":
            return  # The idle editor retains its picture, but receives no pose updates.
        online = remote is not None
        if self.live.gate_open != online:
            self.live.gate_open = online
            self.live.set_status()
        if remote is None:
            return
        position, rotation = remote["position"], remote["rotation"]
        self.live.add_pose(
            {
                "seq": seq,
                "recv_unix": time.time(),
                "recv_iso": iso_now(),
                "kind": remote["kind"],
                **dict(zip(("x", "y", "z"), position)),
                **dict(zip(("qx", "qy", "qz", "qw"), rotation)),
                "flag": remote.get("flag", False),
            }
        )

    def _announce(self, action: str) -> None:
        state = self.snapshot()
        self.live.broadcast({"type": "editor", "editor": state})
        print(
            json.dumps({"event": "calibration", "time": iso_now(), "action": action, **state}),
            flush=True,
        )

    def command(self, request: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            request_id = request.get("request_id")
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
                return {"accepted": False, "applied": False, "message": "Request ID required"}
            if request_id in self.responses:
                return self.responses[request_id]
            try:
                action = request.get("action")
                message = ""
                if action == "status":
                    return {"accepted": True, "applied": True, "editor": self.snapshot()}
                if action == "begin":
                    if self.state != "calibrating":
                        self.state = "calibrating"
                        self.last_action = "begin"
                        self.revision = str(uuid.uuid4())
                        self.live.reset_points()
                        self.live.gate_open = False
                        self.last_pose_at = None
                        self._announce("begin")
                    message = "Calibration ready"
                elif action in {"finish", "cancel"}:
                    if self.state != "calibrating" or request.get("revision") != self.revision:
                        raise ValueError("This editor session has changed; reload before saving")
                    if action == "finish":
                        data = request.get("calibration")
                        if not isinstance(data, dict):
                            raise ValueError("A complete calibration profile is required")
                        data = dict(data)
                        data.pop("rotation", None)
                        calibration = calibration_from_dict(data)
                        name = sanitize_profile(request.get("profile"), self.path.stem)
                        data["profile"] = name
                        path = self.storage_dir / f"{name}.json"
                        encoded = (
                            json.dumps(data, indent=2, sort_keys=True, allow_nan=False).encode()
                            + b"\n"
                        )
                        digest = hashlib.sha256(encoded).hexdigest()
                        atomic_write_json(path, data)
                        self.calibration, self.path, self.digest = calibration, path, digest
                    if self.calibration is None:
                        raise ValueError("No saved calibration to return to")
                    self.state = "awaiting_b"
                    self.last_action = action
                    self.revision = str(uuid.uuid4())
                    self.saw_pause = False
                    self.live.gate_open = False
                    self.live.set_status()
                    self._announce(action)
                    message = "Saved and applied. Return to your controls, then pause/resume B."
                    if action == "cancel":
                        message = "Previous profile retained. Pause/resume B to continue."
                else:
                    raise ValueError("Unknown calibration action")
                result = {
                    "accepted": True,
                    "applied": True,
                    "message": message,
                    "editor": self.snapshot(),
                }
            except (OSError, ValueError, TypeError) as exc:
                print(
                    json.dumps(
                        {
                            "event": "calibration_request_failed",
                            "time": iso_now(),
                            "action": request.get("action"),
                            "request_id": request_id,
                            "revision": self.revision,
                            "message": str(exc),
                        }
                    ),
                    flush=True,
                )
                result = {
                    "accepted": False,
                    "applied": False,
                    "message": str(exc),
                    "editor": self.snapshot(),
                }
            if len(self.responses) >= 128:
                self.responses.pop(next(iter(self.responses)))
            self.responses[request_id] = result
            return result

    def submit(self, value: dict[str, Any]) -> dict[str, Any]:
        pending = PendingRequest(value, time.monotonic() + 2.0)
        try:
            self.requests.put_nowait(pending)
        except queue.Full:
            return {"accepted": False, "applied": False, "message": "Source busy; retry"}
        if not pending.done.wait(2.5):
            return {
                "accepted": False,
                "applied": False,
                "message": "Source did not acknowledge; check status before retrying",
            }
        assert pending.result is not None
        return pending.result

    def drain(self) -> None:
        for _ in range(16):
            try:
                pending = self.requests.get_nowait()
            except queue.Empty:
                return
            pending.result = (
                self.command(pending.value)
                if time.monotonic() < pending.deadline
                else {
                    "accepted": False,
                    "applied": False,
                    "message": "Request expired; not applied",
                }
            )
            pending.done.set()
