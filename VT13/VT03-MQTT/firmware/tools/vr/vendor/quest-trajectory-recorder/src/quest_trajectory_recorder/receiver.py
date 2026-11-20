#!/usr/bin/env python3
"""Lightweight receiver for the controller-tracking Quest APK.

The Quest app uses ZMQ PUSH sockets. This script binds matching ZMQ PULL
sockets and records the raw frames plus parsed right-hand keypoints.
"""

from __future__ import annotations

import argparse
import base64
import collections
import csv
import datetime as dt
import json
import signal
import time
from pathlib import Path
from typing import Any

import zmq

DEFAULT_PORTS = {
    "keypoints": 8087,
    "remote": 8125,
    "resolution": 8095,
    "pause": 8100,
}


def iso_now() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="milliseconds")


def parse_keypoint_text(text: str) -> dict[str, Any] | None:
    clean = text.strip("\x00\r\n\t :")
    if not (clean.startswith("absolute:") or clean.startswith("relative:")):
        return None

    kind, body = clean.split(":", 1)
    body = body.strip(" :")
    points: list[list[float]] = []
    for vector_str in body.split("|"):
        vector_str = vector_str.strip(" :")
        if not vector_str:
            continue
        parts = [p for p in vector_str.split(",") if p]
        if len(parts) < 3:
            continue
        points.append([float(parts[0]), float(parts[1]), float(parts[2])])

    return {
        "kind": kind,
        "num_points": len(points),
        "points": points,
    }


def parse_remote_text(text: str) -> dict[str, Any] | None:
    clean = text.strip("\x00\r\n\t :")
    parts = clean.split("|")
    if len(parts) < 4 or parts[0] not in {"absolute", "relative"}:
        return None

    def parse_vector(value: str, expected: int) -> list[float]:
        vector = [float(part) for part in value.split(",") if part]
        if len(vector) != expected:
            raise ValueError(f"expected {expected} floats, got {len(vector)}")
        return vector

    points: list[list[float]] = []
    for value in parts[4:]:
        if value:
            points.append(parse_vector(value, 3))

    return {
        "kind": parts[0],
        "position": parse_vector(parts[1], 3),
        "rotation": parse_vector(parts[2], 4),
        "flag": parts[3].strip().lower() == "true",
        "num_points": len(points),
        "points": points,
    }


def decode_frame(payload: bytes, channel: str) -> dict[str, Any]:
    record: dict[str, Any] = {
        "channel": channel,
        "bytes": len(payload),
        "base64": base64.b64encode(payload).decode("ascii"),
    }
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError:
        record["text"] = payload.decode("utf-8", errors="replace")
        return record

    record["text"] = text.strip()
    try:
        parsed = parse_keypoint_text(text) if channel == "keypoints" else None
    except (TypeError, ValueError) as exc:
        parsed = None
        record["parse_error"] = f"keypoints: {exc}"
    if parsed is not None:
        record["parsed"] = parsed
    try:
        remote = parse_remote_text(text) if channel == "remote" else None
    except (TypeError, ValueError) as exc:
        remote = None
        record["parse_error"] = f"remote: {exc}"
    if remote is not None:
        record["remote"] = remote
    return record


def make_socket(
    context: zmq.Context[zmq.Socket], host: str, port: int, socket_type: int
) -> zmq.Socket:
    socket = context.socket(socket_type)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVHWM, 10000)
    socket.bind(f"tcp://{host}:{port}")
    return socket


def write_keypoint_rows(
    writer: csv.DictWriter, base: dict[str, Any], parsed: dict[str, Any]
) -> None:
    for joint_index, point in enumerate(parsed["points"]):
        writer.writerow(
            {
                **base,
                "kind": parsed["kind"],
                "num_points": parsed["num_points"],
                "joint_index": joint_index,
                "x": point[0],
                "y": point[1],
                "z": point[2],
            }
        )


def write_remote_row(
    writer: csv.DictWriter, base: dict[str, Any], remote: dict[str, Any], text: str
) -> None:
    row: dict[str, Any] = {
        **base,
        "kind": remote["kind"],
        "pos_x": remote["position"][0],
        "pos_y": remote["position"][1],
        "pos_z": remote["position"][2],
        "quat_x": remote["rotation"][0],
        "quat_y": remote["rotation"][1],
        "quat_z": remote["rotation"][2],
        "quat_w": remote["rotation"][3],
        "flag": remote["flag"],
        "num_points": remote["num_points"],
        "raw_text": text,
    }
    for point_index, point in enumerate(remote["points"][:3]):
        row[f"point{point_index}_x"] = point[0]
        row[f"point{point_index}_y"] = point[1]
        row[f"point{point_index}_z"] = point[2]
    writer.writerow(row)


def summarize(seq: int, channel: str, decoded: dict[str, Any]) -> str:
    text = decoded.get("text", "")
    parsed = decoded.get("parsed")
    if parsed:
        wrist = parsed["points"][0] if parsed["points"] else None
        return f"#{seq} {channel}: {parsed['kind']} {parsed['num_points']} pts" + (
            f" wrist=({wrist[0]:.4f},{wrist[1]:.4f},{wrist[2]:.4f})" if wrist else ""
        )
    remote = decoded.get("remote")
    if remote:
        pos = remote["position"]
        return (
            f"#{seq} {channel}: {remote['kind']} pos=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f})"
            f" points={remote['num_points']} flag={remote['flag']}"
        )
    if text:
        return f"#{seq} {channel}: {text!r}"
    return f"#{seq} {channel}: {decoded['bytes']} raw bytes"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive Quest controller ZMQ frames.")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="ZMQ bind host. The safe ADB-reverse default is loopback; opt into 0.0.0.0 for LAN.",
    )
    parser.add_argument("--keypoint-port", type=int, default=DEFAULT_PORTS["keypoints"])
    parser.add_argument("--remote-port", type=int, default=DEFAULT_PORTS["remote"])
    parser.add_argument("--resolution-port", type=int, default=DEFAULT_PORTS["resolution"])
    parser.add_argument("--pause-port", type=int, default=DEFAULT_PORTS["pause"])
    parser.add_argument(
        "--keypoint-socket",
        choices=("pull", "rep"),
        default="pull",
        help="Use rep if the APK logs 'Message send failed or would block' and expects an ACK.",
    )
    parser.add_argument("--ack-text", default="ok", help="Reply text for REP keypoint sockets.")
    parser.add_argument(
        "--remote-socket",
        choices=("pull", "rep"),
        default="pull",
        help="Socket type for the ControllerTracking remote stream.",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("captures"))
    parser.add_argument("--session", default=dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--heartbeat-sec", type=float, default=5.0)
    parser.add_argument(
        "--event-print-interval-sec",
        type=float,
        default=2.0,
        help="Aggregate non-keypoint console output; use 0 to print every event.",
    )
    parser.add_argument(
        "--stop-on-pause",
        choices=("High", "Low"),
        help="Stop after this pause state persists and no trajectory frames arrive.",
    )
    parser.add_argument(
        "--stop-pause-count",
        type=int,
        default=20,
        help="Consecutive matching pause frames required before auto-stop.",
    )
    parser.add_argument(
        "--stop-no-data-sec",
        type=float,
        default=0.5,
        help="Require this many seconds without keypoint/remote frames before auto-stop.",
    )
    parser.add_argument(
        "--stop-idle-sec",
        type=float,
        default=0.0,
        help="Stop after this many seconds without trajectory frames after recording starts; 0 disables.",
    )
    parser.add_argument(
        "--trajectory-gate-pause",
        choices=("High", "Low"),
        help="Only write trajectory rows while the latest pause channel state equals this value.",
    )
    parser.add_argument(
        "--gate-requires-prior-pause",
        choices=("High", "Low"),
        help="Before opening the trajectory gate, require this pause state once. Useful to discard stale queued frames.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.out_dir / f"{args.session}_raw.jsonl"
    keypoints_path = args.out_dir / f"{args.session}_keypoints.csv"
    remote_path = args.out_dir / f"{args.session}_remote.csv"
    events_path = args.out_dir / f"{args.session}_events.csv"

    context: zmq.Context[zmq.Socket] = zmq.Context()
    poller = zmq.Poller()
    sockets: dict[zmq.Socket, tuple[str, int, str]] = {}
    for channel, port in {
        "keypoints": args.keypoint_port,
        "remote": args.remote_port,
        "resolution": args.resolution_port,
        "pause": args.pause_port,
    }.items():
        if channel == "keypoints":
            mode = args.keypoint_socket
        elif channel == "remote":
            mode = args.remote_socket
        else:
            mode = "pull"
        socket_type = zmq.REP if mode == "rep" else zmq.PULL
        socket = make_socket(context, args.host, port, socket_type)
        sockets[socket] = (channel, port, mode)
        poller.register(socket, zmq.POLLIN)
        print(f"ZMQ {mode.upper()} listening: {channel} tcp://{args.host}:{port}", flush=True)

    stop = False

    def handle_signal(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    keypoint_fields = [
        "recv_unix",
        "recv_iso",
        "seq",
        "channel",
        "port",
        "kind",
        "num_points",
        "joint_index",
        "x",
        "y",
        "z",
    ]
    event_fields = ["recv_unix", "recv_iso", "seq", "channel", "port", "text", "bytes"]
    remote_fields = [
        "recv_unix",
        "recv_iso",
        "seq",
        "channel",
        "port",
        "kind",
        "pos_x",
        "pos_y",
        "pos_z",
        "quat_x",
        "quat_y",
        "quat_z",
        "quat_w",
        "flag",
        "num_points",
        "point0_x",
        "point0_y",
        "point0_z",
        "point1_x",
        "point1_y",
        "point1_z",
        "point2_x",
        "point2_y",
        "point2_z",
        "raw_text",
    ]
    new_keypoints = not keypoints_path.exists() or keypoints_path.stat().st_size == 0
    new_remote = not remote_path.exists() or remote_path.stat().st_size == 0
    new_events = not events_path.exists() or events_path.stat().st_size == 0

    print(f"Raw JSONL: {raw_path.resolve()}", flush=True)
    print(f"Keypoint CSV: {keypoints_path.resolve()}", flush=True)
    print(f"Remote CSV: {remote_path.resolve()}", flush=True)
    print(f"Event CSV: {events_path.resolve()}", flush=True)
    print("Waiting for Quest frames...", flush=True)

    seq = 0
    last_message = time.time()
    pending_events: collections.Counter[tuple[str, str]] = collections.Counter()
    last_event_print = time.time()
    seen_trajectory = False
    last_trajectory_time = 0.0
    consecutive_stop_pause = 0
    pause_state: str | None = None
    gate_prereq_seen = args.gate_requires_prior_pause is None
    trajectory_gate_open = args.trajectory_gate_pause is None
    discarded_trajectory = 0

    def flush_event_summary(force: bool = False) -> None:
        nonlocal last_event_print
        if not pending_events:
            return
        now = time.time()
        if (
            not force
            and args.event_print_interval_sec > 0
            and now - last_event_print < args.event_print_interval_sec
        ):
            return
        summary = ", ".join(
            f"{channel} {text!r} x{count}"
            for (channel, text), count in pending_events.most_common(6)
        )
        print(f"{iso_now()} events: {summary}", flush=True)
        pending_events.clear()
        last_event_print = now

    def update_trajectory_gate(recv_iso: str, text: str) -> None:
        nonlocal pause_state, gate_prereq_seen, trajectory_gate_open
        pause_state = text
        if args.gate_requires_prior_pause and text == args.gate_requires_prior_pause:
            gate_prereq_seen = True
        if not args.trajectory_gate_pause:
            trajectory_gate_open = True
            return
        new_state = gate_prereq_seen and text == args.trajectory_gate_pause
        if new_state != trajectory_gate_open:
            flush_event_summary(force=True)
            state = "open" if new_state else "closed"
            print(
                f"{recv_iso} trajectory gate {state}: pause={text!r} "
                f"required={args.trajectory_gate_pause!r} prereq_seen={gate_prereq_seen}",
                flush=True,
            )
        trajectory_gate_open = new_state

    try:
        with (
            raw_path.open("a", encoding="utf-8") as raw_file,
            keypoints_path.open("a", newline="", encoding="utf-8") as keypoints_file,
            remote_path.open("a", newline="", encoding="utf-8") as remote_file,
            events_path.open("a", newline="", encoding="utf-8") as events_file,
        ):
            keypoint_writer = csv.DictWriter(keypoints_file, fieldnames=keypoint_fields)
            remote_writer = csv.DictWriter(remote_file, fieldnames=remote_fields)
            event_writer = csv.DictWriter(events_file, fieldnames=event_fields)
            if new_keypoints:
                keypoint_writer.writeheader()
            if new_remote:
                remote_writer.writeheader()
            if new_events:
                event_writer.writeheader()

            while not stop:
                ready = dict(poller.poll(timeout=250))
                now = time.time()
                if not ready:
                    if (
                        seen_trajectory
                        and args.stop_idle_sec > 0
                        and now - last_trajectory_time >= args.stop_idle_sec
                    ):
                        flush_event_summary(force=True)
                        print(
                            f"{iso_now()} auto-stop: no trajectory frames for {now - last_trajectory_time:.2f}s",
                            flush=True,
                        )
                        stop = True
                        continue
                    flush_event_summary()
                    if args.heartbeat_sec > 0 and now - last_message >= args.heartbeat_sec:
                        print(f"{iso_now()} still waiting...", flush=True)
                        last_message = now
                    continue

                for socket in ready:
                    channel, port, mode = sockets[socket]
                    payload = socket.recv()
                    if mode == "rep":
                        socket.send_string(args.ack_text)
                    seq += 1
                    recv_unix = time.time()
                    recv_iso = iso_now()
                    decoded = decode_frame(payload, channel)
                    record = {
                        "recv_unix": recv_unix,
                        "recv_iso": recv_iso,
                        "seq": seq,
                        "port": port,
                        **decoded,
                    }
                    raw_file.write(
                        json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
                    )
                    raw_file.flush()

                    event_base = {
                        "recv_unix": recv_unix,
                        "recv_iso": recv_iso,
                        "seq": seq,
                        "channel": channel,
                        "port": port,
                    }
                    if channel == "pause" and isinstance(decoded.get("text"), str):
                        update_trajectory_gate(recv_iso, decoded["text"])

                    parsed = decoded.get("parsed")
                    remote = decoded.get("remote")
                    trajectory_written = False
                    if parsed and trajectory_gate_open:
                        flush_event_summary(force=True)
                        write_keypoint_rows(keypoint_writer, event_base, parsed)
                        keypoints_file.flush()
                        print(f"{recv_iso} {summarize(seq, channel, decoded)}", flush=True)
                        trajectory_written = True
                    elif remote and trajectory_gate_open:
                        write_remote_row(remote_writer, event_base, remote, decoded.get("text", ""))
                        remote_file.flush()
                        pending_events[(channel, f"{remote['kind']} frames")] += 1
                        flush_event_summary()
                        trajectory_written = True
                    elif parsed or remote:
                        discarded_trajectory += 1
                        pending_events[(channel, "discarded trajectory before gate")] += 1
                        flush_event_summary()
                    else:
                        event_writer.writerow(
                            {
                                **event_base,
                                "text": decoded.get("text", ""),
                                "bytes": decoded["bytes"],
                            }
                        )
                        events_file.flush()
                        if args.event_print_interval_sec == 0:
                            print(f"{recv_iso} {summarize(seq, channel, decoded)}", flush=True)
                        else:
                            pending_events[
                                (channel, decoded.get("text", f"{decoded['bytes']} raw bytes"))
                            ] += 1
                            flush_event_summary()

                    last_message = recv_unix
                    if trajectory_written:
                        seen_trajectory = True
                        last_trajectory_time = recv_unix
                        consecutive_stop_pause = 0
                    elif (
                        args.stop_on_pause
                        and channel == "pause"
                        and decoded.get("text") == args.stop_on_pause
                        and seen_trajectory
                    ):
                        consecutive_stop_pause += 1
                        no_data_for = recv_unix - last_trajectory_time
                        if (
                            consecutive_stop_pause >= args.stop_pause_count
                            and no_data_for >= args.stop_no_data_sec
                        ):
                            flush_event_summary(force=True)
                            print(
                                f"{recv_iso} auto-stop: pause={args.stop_on_pause!r} "
                                f"count={consecutive_stop_pause} no_data_for={no_data_for:.2f}s",
                                flush=True,
                            )
                            stop = True
                            break
                    elif channel == "pause" and args.stop_on_pause:
                        consecutive_stop_pause = 0
    finally:
        flush_event_summary(force=True)
        if discarded_trajectory:
            print(
                f"{iso_now()} discarded trajectory frames before gate: {discarded_trajectory}",
                flush=True,
            )
        for socket in sockets:
            poller.unregister(socket)
            socket.close(0)
        context.term()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
