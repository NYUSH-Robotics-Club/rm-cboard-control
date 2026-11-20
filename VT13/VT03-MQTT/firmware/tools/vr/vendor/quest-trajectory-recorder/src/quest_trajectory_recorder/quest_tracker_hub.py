"""Publish calibrated Quest controller targets for any simulator backend."""

from __future__ import annotations

import argparse
import hashlib
import math
import queue
import signal
import subprocess
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import zmq
from embodied_ops.teleop import TeleopSourceStatus
from embodied_ops.teleop.zmq_transport import (
    DEFAULT_TARGET_ENDPOINT,
    DEFAULT_TARGET_TOPIC,
    TeleopTargetPublisher,
)

from .calibration_profiles import calibration_dir
from .calibration_session import CalibrationSession
from .live3d_web import ReusableThreadingHTTPServer, make_handler
from .quest_ports import (
    DEFAULT_GRIPPER_PORT,
    adb_connected,
    adb_reverse_ports,
    focus_frankabot,
    quest_activity_resumed,
    quest_device_info,
    setup_adb_reverse,
)
from .quest_target_source import DirectQuestTargetSource
from .receiver import DEFAULT_PORTS
from .teleop_frame import load_quest_calibration

_DISCONNECTED_DEVICE = {
    "adb_connected": False,
    "model": None,
    "serial": None,
    "app_resumed": False,
}
_ADB_DISCONNECT_CONFIRMATIONS = 2


@dataclass(frozen=True, slots=True)
class _AdbHealthUpdate:
    device: dict[str, Any]
    events: tuple[str, ...]


class _AdbHealthMonitor:
    """Run slow ADB recovery checks away from the target-ingest thread."""

    def __init__(
        self,
        *,
        required_ports: list[int],
        check_sec: float,
        initial_device: dict[str, Any],
        connected_fn: Callable[[], bool] = adb_connected,
        reverse_ports_fn: Callable[[], set[int]] = adb_reverse_ports,
        setup_reverse_fn: Callable[[list[int]], None] = setup_adb_reverse,
        device_info_fn: Callable[[], dict[str, Any]] = quest_device_info,
    ) -> None:
        if check_sec <= 0:
            raise ValueError("ADB health-check interval must be positive")
        self.required_ports = tuple(int(port) for port in required_ports)
        self.check_sec = float(check_sec)
        self._connected_fn = connected_fn
        self._reverse_ports_fn = reverse_ports_fn
        self._setup_reverse_fn = setup_reverse_fn
        self._device_info_fn = device_info_fn
        self._previous_connected = bool(initial_device.get("adb_connected"))
        self._last_device = dict(initial_device)
        self._consecutive_disconnects = 0
        self._updates: queue.SimpleQueue[_AdbHealthUpdate] = queue.SimpleQueue()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="quest-adb-health",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        # ADB is external to this process and may itself be wedged during USB
        # teardown. The worker is a daemon and owns no ZMQ sockets, so cleanup
        # must not wait forever for an uninterruptible platform-tools call.
        self._thread.join(timeout=2.0)

    def take_updates(self) -> list[_AdbHealthUpdate]:
        updates: list[_AdbHealthUpdate] = []
        while True:
            try:
                updates.append(self._updates.get_nowait())
            except queue.Empty:
                return updates

    def _check_once(self) -> _AdbHealthUpdate:
        events: list[str] = []
        connected = self._connected_fn()
        device = dict(_DISCONNECTED_DEVICE)
        if connected:
            try:
                missing_ports = sorted(set(self.required_ports) - self._reverse_ports_fn())
                if missing_ports:
                    self._setup_reverse_fn(missing_ports)
                    events.append(f"ADB reverse mappings restored: {missing_ports}")
                device = dict(self._device_info_fn())
                if device.get("adb_connected") is False:
                    connected = False
            except (OSError, RuntimeError, subprocess.SubprocessError):
                connected = False
        if connected:
            self._consecutive_disconnects = 0
        else:
            self._consecutive_disconnects += 1
            if (
                self._previous_connected
                and self._consecutive_disconnects < _ADB_DISCONNECT_CONFIRMATIONS
            ):
                return _AdbHealthUpdate(device=dict(self._last_device), events=())
        if connected != self._previous_connected:
            events.append(f"ADB device {'connected' if connected else 'disconnected'}.")
        self._previous_connected = connected
        self._last_device = dict(device)
        return _AdbHealthUpdate(device=dict(device), events=tuple(events))

    def _run(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            self._updates.put(self._check_once())
            remaining = max(0.0, self.check_sec - (time.monotonic() - started))
            if self._stop.wait(remaining):
                return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Quest raw ZMQ -> simulator-neutral TeleopTarget publisher."
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Raw Quest ZMQ bind host. The safe ADB-reverse default is loopback; opt into 0.0.0.0 for LAN.",
    )
    parser.add_argument("--remote-port", type=int, default=DEFAULT_PORTS["remote"])
    parser.add_argument("--pause-port", type=int, default=DEFAULT_PORTS["pause"])
    parser.add_argument("--resolution-port", type=int, default=DEFAULT_PORTS["resolution"])
    parser.add_argument("--gripper-port", type=int, default=DEFAULT_GRIPPER_PORT)
    parser.add_argument("--adb-reverse", action="store_true")
    parser.add_argument("--calibration", type=str, default="calibrations/quest_teleop_frame.json")
    parser.add_argument("--no-gate", action="store_true")
    parser.add_argument("--trajectory-gate-pause", choices=("High", "Low"), default="High")
    parser.add_argument("--allow-initial-high", action="store_true")
    parser.add_argument("--gripper-mode", choices=("toggle", "hold"), default="toggle")
    parser.add_argument(
        "--initial-gripper",
        choices=("open", "closed"),
        default="open",
        help="Initial canonical gripper state before the first controller toggle.",
    )
    parser.add_argument(
        "--tracking-loss-grace-ms",
        type=float,
        default=120.0,
        help="Ignore isolated invalid Quest pose placeholders for this long before holding.",
    )
    parser.add_argument("--target-bind", default=DEFAULT_TARGET_ENDPOINT)
    parser.add_argument("--source-control-bind", default="tcp://127.0.0.1:8133")
    parser.add_argument("--web-host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8766)
    parser.add_argument("--print-every", type=int, default=30)
    parser.add_argument("--session-id", default="")
    parser.add_argument("--status-every-sec", type=float, default=1.0)
    parser.add_argument(
        "--adb-check-sec",
        type=float,
        default=5.0,
        help="When using ADB reverse, restore port mappings after a USB reconnect; 0 disables.",
    )
    return parser.parse_args()


def _source_state(
    *,
    adb_online: bool | None,
    has_received_pose: bool,
    raw_online: bool,
    tracking_valid: bool,
    gate_open: bool,
    pause_state: str | None,
) -> str:
    """Classify operator state without mistaking an intentional pause for dropout."""

    if adb_online is False and not raw_online:
        return "adb_disconnected"
    if pause_state is not None and not gate_open:
        return "ready"
    if not has_received_pose:
        return "waiting_for_controller"
    if not raw_online:
        return "controller_offline"
    if not tracking_valid:
        return "tracking_invalid"
    return "streaming" if gate_open else "ready"


def _prepare_initial_adb(
    required_ports: list[int],
    *,
    connected_fn: Callable[[], bool] = adb_connected,
    setup_reverse_fn: Callable[[list[int]], None] = setup_adb_reverse,
    activity_resumed_fn: Callable[[], bool] = quest_activity_resumed,
    focus_fn: Callable[[], None] = focus_frankabot,
    device_info_fn: Callable[[], dict[str, Any]] = quest_device_info,
) -> dict[str, Any]:
    """Prepare an available Quest without making a USB flap fatal to the source."""

    if not connected_fn():
        print(
            "ADB device is not connected yet; the hub will restore reverse ports after reconnect.",
            flush=True,
        )
        return dict(_DISCONNECTED_DEVICE)
    try:
        setup_reverse_fn(required_ports)
        if not activity_resumed_fn():
            focus_fn()
        return dict(device_info_fn())
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(
            f"Initial ADB preparation was interrupted ({exc}); "
            "the hub will stay safe and retry after reconnect.",
            flush=True,
        )
        return dict(_DISCONNECTED_DEVICE)


def main() -> int:
    args = parse_args()
    required_ports = [
        args.remote_port,
        args.pause_port,
        args.resolution_port,
        args.gripper_port,
    ]
    if any(port < 1 or port > 65535 for port in required_ports):
        raise ValueError("Quest ports must be integers from 1 to 65535")
    if len(set(required_ports)) != len(required_ports):
        raise ValueError("Quest raw-data ports must be distinct")
    if args.print_every < 0:
        raise ValueError("--print-every must be non-negative")
    if args.status_every_sec < 0 or args.adb_check_sec < 0:
        raise ValueError("status and ADB check intervals must be non-negative")
    if not math.isfinite(args.tracking_loss_grace_ms) or args.tracking_loss_grace_ms < 0:
        raise ValueError("--tracking-loss-grace-ms must be finite and non-negative")
    device = (
        _prepare_initial_adb(required_ports)
        if args.adb_reverse
        else {
            "adb_connected": None,
            "model": None,
            "serial": None,
            "app_resumed": None,
        }
    )
    calibration_path = None if not args.calibration else Path(args.calibration)
    calibration = load_quest_calibration(calibration_path)
    calibration_sha256 = (
        hashlib.sha256(calibration_path.read_bytes()).hexdigest()
        if calibration_path is not None and calibration_path.is_file()
        else None
    )
    calibration_id = calibration_path.stem if calibration_path is not None else None
    session_id = args.session_id or str(uuid.uuid4())
    if calibration is None:
        print(
            f"Warning: no calibration loaded from {args.calibration}; publishing fallback teleop axes.",
            flush=True,
        )
    else:
        print(f"Loaded calibration: {args.calibration}", flush=True)

    context = zmq.Context()
    publisher = TeleopTargetPublisher(context, args.target_bind)
    if calibration_path is None:
        raise ValueError("A calibration profile path is required")
    editor = CalibrationSession(
        calibration_path,
        url=f"http://{args.web_host}:{args.web_port}/",
        storage_dir=calibration_dir(),
    )
    control_socket = context.socket(zmq.REP)
    control_socket.setsockopt(zmq.LINGER, 0)
    control_socket.setsockopt(zmq.MAXMSGSIZE, 65536)
    control_socket.bind(args.source_control_bind)
    server = ReusableThreadingHTTPServer(
        (args.web_host, args.web_port),
        make_handler(editor.live, calibration_path, editor=editor),
    )
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"Calibration editor: {editor.url}", flush=True)
    source = DirectQuestTargetSource(
        context=context,
        host=args.host,
        remote_port=args.remote_port,
        pause_port=args.pause_port,
        calibration=calibration,
        no_gate=args.no_gate,
        trajectory_gate_pause=args.trajectory_gate_pause,
        allow_initial_high=args.allow_initial_high,
        gripper_mode=args.gripper_mode,
        initial_gripper=1.0 if args.initial_gripper == "closed" else -1.0,
        session_id=session_id,
        calibration_id=calibration_id,
        calibration_sha256=calibration_sha256,
        tracking_loss_grace_ms=args.tracking_loss_grace_ms,
        editor=editor,
    )
    stop = False
    last_status_at = 0.0
    adb_monitor = (
        _AdbHealthMonitor(
            required_ports=required_ports,
            check_sec=args.adb_check_sec,
            initial_device=device,
        )
        if args.adb_reverse and args.adb_check_sec > 0
        else None
    )

    def handle_signal(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    if adb_monitor is not None:
        adb_monitor.start()
    print(
        f"Publishing TeleopTarget on {args.target_bind} "
        f"topic={DEFAULT_TARGET_TOPIC.decode('ascii')!r}",
        flush=True,
    )
    print(f"Session: {session_id}", flush=True)
    try:
        while not stop:
            now = time.monotonic()
            if adb_monitor is not None:
                for update in adb_monitor.take_updates():
                    device = update.device
                    for event in update.events:
                        print(event, flush=True)
            editor.drain()
            if editor.last_pose_at is not None and now - editor.last_pose_at > 0.5:
                editor.observe(None, source.raw_remote_count, now)
            try:
                request = control_socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except (ValueError, UnicodeDecodeError):
                control_socket.send_json(
                    {"accepted": False, "applied": False, "message": "Invalid source request"}
                )
            else:
                result = (
                    editor.command(request)
                    if isinstance(request, dict)
                    else {
                        "accepted": False,
                        "applied": False,
                        "message": "Expected an object",
                    }
                )
                control_socket.send_json(result)
            if not editor.enabled:
                source.gate_open = False
            target = source.poll(50)
            for event in source.take_events():
                print(event, flush=True)
            if target is None and source.latest_target is not None:
                latest = source.latest_target
                if (
                    latest.gate_open != source.gate_open
                    or latest.pause_state != source.pause_state
                    or latest.gripper != source.gripper
                ):
                    latest = replace(
                        latest,
                        gate_open=source.gate_open,
                        gripper=source.gripper,
                        host_published_unix_ns=time.time_ns(),
                        source_metadata={
                            **latest.source_metadata,
                            "pause_state": source.pause_state,
                        },
                    )
                    source.latest_target = latest
                    target = latest
            if target is not None:
                target = replace(
                    target,
                    host_published_unix_ns=time.time_ns(),
                    gate_open=target.gate_open and editor.enabled,
                    source_metadata={**target.source_metadata, **editor.metadata()},
                )
                source.latest_target = target
                publisher.publish(target)
            if (
                target is not None
                and target.tracking_valid
                and args.print_every
                and target.remote_count % args.print_every == 0
            ):
                x, y, z = target.position
                print(
                    f"target seq={target.seq} gate={target.gate_open} grip={target.gripper:+.0f} "
                    f"pos=({x:+.3f},{y:+.3f},{z:+.3f})",
                    flush=True,
                )

            now = time.monotonic()
            if args.status_every_sec > 0 and now - last_status_at >= args.status_every_sec:
                target_age_sec = (
                    None
                    if source.latest_target_at is None
                    else max(0.0, time.time() - source.latest_target_at)
                )
                raw_age_sec = (
                    None if source.latest_raw_at is None else max(0.0, now - source.latest_raw_at)
                )
                valid_age_sec = (
                    None
                    if source.latest_valid_at is None
                    else max(0.0, now - source.latest_valid_at)
                )
                raw_online = raw_age_sec is not None and raw_age_sec <= 0.5
                tracking_valid = bool(
                    raw_online
                    and source.latest_raw_valid
                    and valid_age_sec is not None
                    and valid_age_sec <= 0.5
                )
                state = _source_state(
                    adb_online=device.get("adb_connected"),
                    has_received_pose=source.latest_raw_at is not None,
                    raw_online=raw_online,
                    tracking_valid=tracking_valid,
                    gate_open=source.gate_open,
                    pause_state=source.pause_state,
                )
                status = TeleopSourceStatus(
                    source="quest",
                    session_id=session_id,
                    state=state,
                    target_seq=None if source.latest_target is None else source.latest_target.seq,
                    target_age_ms=None if target_age_sec is None else target_age_sec * 1000.0,
                    gate_open=source.gate_open,
                    control_ready=tracking_valid and source.gate_open and editor.enabled,
                    stream_online=raw_online,
                    tracking_valid=tracking_valid,
                    raw_age_ms=None if raw_age_sec is None else raw_age_sec * 1000.0,
                    valid_age_ms=(None if valid_age_sec is None else valid_age_sec * 1000.0),
                    pause_state=source.pause_state,
                    source_metadata={
                        "valid_remote_count": source.remote_count,
                        "raw_remote_count": source.raw_remote_count,
                        "invalid_remote_count": source.invalid_remote_count,
                        "tracking_loss_count": source.tracking_loss_count,
                        "consecutive_invalid_count": source.consecutive_invalid_count,
                        "tracking_loss_grace_ms": source.tracking_loss_grace_ms,
                        "last_invalid_reason": source.last_invalid_reason,
                        "calibration_id": editor.path.stem,
                        "calibration_sha256": editor.digest,
                        **device,
                        **editor.metadata(),
                    },
                )
                publisher.publish_status(status)
                last_status_at = now
    finally:
        if adb_monitor is not None:
            adb_monitor.close()
        source.close()
        control_socket.close(0)
        server.shutdown()
        server.server_close()
        publisher.close()
        context.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
