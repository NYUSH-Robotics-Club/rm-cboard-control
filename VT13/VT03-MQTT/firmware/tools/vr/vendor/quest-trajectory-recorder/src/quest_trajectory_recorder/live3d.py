"""Live 3D browser view for Quest controller trajectory frames."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import signal
import threading
import time
import webbrowser
from pathlib import Path

import zmq

from .calibration_profiles import DEFAULT_CALIBRATION_PATH
from .live3d_web import ReusableThreadingHTTPServer, make_handler
from .live_state import EVENT_FIELDS, REMOTE_FIELDS, LiveState, is_origin
from .quest_ports import DEFAULT_GRIPPER_PORT, setup_adb_reverse
from .receiver import (
    DEFAULT_PORTS,
    iso_now,
    make_socket,
    parse_remote_text,
    write_remote_row,
)

DEFAULT_CALIBRATION_WEB_PORT = 8766


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live 3D browser visualizer for Quest controller trajectories."
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Raw Quest ZMQ bind host. The safe ADB-reverse default is loopback; opt into 0.0.0.0 for LAN.",
    )
    parser.add_argument("--remote-port", type=int, default=DEFAULT_PORTS["remote"])
    parser.add_argument("--resolution-port", type=int, default=DEFAULT_PORTS["resolution"])
    parser.add_argument("--pause-port", type=int, default=DEFAULT_PORTS["pause"])
    parser.add_argument("--gripper-port", type=int, default=DEFAULT_GRIPPER_PORT)
    parser.add_argument("--web-host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=DEFAULT_CALIBRATION_WEB_PORT)
    parser.add_argument("--out-dir", type=Path, default=Path("captures"))
    parser.add_argument(
        "--calibration-out",
        type=Path,
        default=DEFAULT_CALIBRATION_PATH,
        help="Path where browser teleop-frame calibration is saved for downstream teleop scripts.",
    )
    parser.add_argument(
        "--session",
        default=dt.datetime.now().astimezone().strftime("live_%Y%m%d_%H%M%S"),
    )
    parser.add_argument(
        "--max-points", type=int, default=5000, help="Max points kept in browser memory."
    )
    parser.add_argument(
        "--print-every", type=int, default=60, help="Print every N accepted poses; 0 disables."
    )
    parser.add_argument("--trajectory-gate-pause", choices=("High", "Low"), default="High")
    parser.add_argument("--gate-requires-prior-pause", choices=("High", "Low"), default="Low")
    parser.add_argument(
        "--no-gate",
        action="store_true",
        help="Accept remote frames immediately, ignoring pause state.",
    )
    parser.add_argument(
        "--keep-origin",
        "--keep-leading-origin",
        action="store_true",
        help="Keep exact 0,0,0 placeholder frames. By default they are dropped anywhere in the stream.",
    )
    parser.add_argument(
        "--max-step-m",
        type=float,
        default=0.20,
        help="Reset the visible path when a single accepted step exceeds this distance; 0 disables.",
    )
    parser.add_argument(
        "--no-record", action="store_true", help="Do not write captures/*.csv files."
    )
    parser.add_argument(
        "--adb-reverse",
        action="store_true",
        help="Run adb reverse for the Quest ports before listening.",
    )
    parser.add_argument(
        "--open-browser", action="store_true", help="Open the live viewer in the default browser."
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    ports = [
        args.remote_port,
        args.resolution_port,
        args.pause_port,
        args.gripper_port,
        args.web_port,
    ]
    if any(port < 1 or port > 65535 for port in ports):
        raise ValueError("Quest and Web ports must be from 1 to 65535")
    if len(set(ports)) != len(ports):
        raise ValueError("Quest raw-data and Web ports must be distinct")
    if args.max_points < 10 or args.print_every < 0 or args.max_step_m < 0:
        raise ValueError("max points/step and print interval must be non-negative and valid")
    if args.adb_reverse:
        setup_adb_reverse(
            [args.remote_port, args.resolution_port, args.pause_port, args.gripper_port]
        )

    state = LiveState(max_points=max(10, args.max_points))
    state.gate_open = bool(args.no_gate)
    state.gate_prereq_seen = bool(args.no_gate or not args.gate_requires_prior_pause)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    remote_path = args.out_dir / f"{args.session}_remote.csv"
    events_path = args.out_dir / f"{args.session}_events.csv"

    args.calibration_out.parent.mkdir(parents=True, exist_ok=True)
    server = ReusableThreadingHTTPServer(
        (args.web_host, args.web_port), make_handler(state, args.calibration_out)
    )
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    url = f"http://{args.web_host}:{args.web_port}/"
    print(f"Live 3D viewer: {url}", flush=True)
    if args.open_browser:
        webbrowser.open(url)

    context: zmq.Context[zmq.Socket] = zmq.Context()
    poller = zmq.Poller()
    sockets: dict[zmq.Socket, tuple[str, int]] = {}
    for channel, port in {
        "remote": args.remote_port,
        "resolution": args.resolution_port,
        "pause": args.pause_port,
        "gripper": args.gripper_port,
    }.items():
        socket = make_socket(context, args.host, port, zmq.PULL)
        poller.register(socket, zmq.POLLIN)
        sockets[socket] = (channel, port)
        print(f"ZMQ PULL listening: {channel} tcp://{args.host}:{port}", flush=True)

    stop = False

    def handle_signal(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print("Waiting for Quest frames. Press Ctrl+C to stop.", flush=True)
    print(
        "Tip: keep Quest IP at 127.0.0.1; press B red -> green to open the trajectory gate.",
        flush=True,
    )

    remote_file = None
    event_file = None
    try:
        if not args.no_record:
            remote_file = remote_path.open("w", newline="", encoding="utf-8")
            event_file = events_path.open("w", newline="", encoding="utf-8")
            remote_writer = csv.DictWriter(remote_file, fieldnames=REMOTE_FIELDS)
            event_writer = csv.DictWriter(event_file, fieldnames=EVENT_FIELDS)
            remote_writer.writeheader()
            event_writer.writeheader()
            print(f"Remote CSV: {remote_path.resolve()}", flush=True)
            print(f"Event CSV: {events_path.resolve()}", flush=True)
        else:
            remote_writer = None
            event_writer = None

        accepted = 0
        last_position: list[float] | None = None
        while not stop:
            ready = dict(poller.poll(timeout=250))
            for socket in ready:
                channel, port = sockets[socket]
                payload = socket.recv()
                state.seq += 1
                recv_unix = time.time()
                recv_iso = iso_now()
                text = payload.decode("utf-8", errors="replace").strip()

                if channel == "pause":
                    previous_gate = state.gate_open
                    state.pause_state = text
                    if text == args.gate_requires_prior_pause:
                        state.gate_prereq_seen = True
                    if args.no_gate:
                        state.gate_open = True
                    else:
                        state.gate_open = (
                            state.gate_prereq_seen and text == args.trajectory_gate_pause
                        )
                    if state.gate_open and not previous_gate:
                        state.reset_points()
                        last_position = None
                        accepted = 0
                        print(f"{recv_iso} trajectory gate opened", flush=True)
                    elif previous_gate and not state.gate_open:
                        print(f"{recv_iso} trajectory gate closed", flush=True)
                    state.set_status(pause_state=text)
                elif channel == "resolution":
                    state.set_status(resolution_state=text)
                elif channel == "gripper":
                    state.set_status(gripper_state=text)
                    print(f"{recv_iso} gripper event: {text!r}", flush=True)
                elif channel == "remote":
                    state.total_received += 1
                    try:
                        remote = parse_remote_text(text)
                    except (TypeError, ValueError) as exc:
                        print(f"{recv_iso} parse error: {exc}: {text!r}", flush=True)
                        continue
                    if not remote or not state.gate_open:
                        continue
                    position = [float(value) for value in remote["position"]]
                    if not args.keep_origin and is_origin(position):
                        print(f"{recv_iso} dropped exact origin placeholder frame", flush=True)
                        continue
                    if last_position is not None and args.max_step_m > 0:
                        step = (
                            sum((a - b) * (a - b) for a, b in zip(position, last_position)) ** 0.5
                        )
                        if step > args.max_step_m:
                            state.reset_points()
                            accepted = 0
                            print(
                                f"{recv_iso} trajectory path reset: step={step:.3f}m > {args.max_step_m:.3f}m",
                                flush=True,
                            )

                    event_base = {
                        "recv_unix": recv_unix,
                        "recv_iso": recv_iso,
                        "seq": state.seq,
                        "channel": channel,
                        "port": port,
                    }
                    if remote_writer and remote_file:
                        write_remote_row(remote_writer, event_base, remote, text)
                        remote_file.flush()

                    accepted += 1
                    point = {
                        "seq": state.seq,
                        "recv_unix": recv_unix,
                        "recv_iso": recv_iso,
                        "kind": remote["kind"],
                        "x": position[0],
                        "y": position[1],
                        "z": position[2],
                        "qx": remote["rotation"][0],
                        "qy": remote["rotation"][1],
                        "qz": remote["rotation"][2],
                        "qw": remote["rotation"][3],
                        "flag": remote["flag"],
                    }
                    state.add_pose(point)
                    last_position = position
                    if args.print_every and accepted % args.print_every == 0:
                        print(
                            f"{recv_iso} accepted={accepted} pos=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f})",
                            flush=True,
                        )

                if channel != "remote" and event_writer and event_file:
                    event_writer.writerow(
                        {
                            "recv_unix": recv_unix,
                            "recv_iso": recv_iso,
                            "seq": state.seq,
                            "channel": channel,
                            "port": port,
                            "text": text,
                            "bytes": len(payload),
                        }
                    )
                    event_file.flush()
    finally:
        server.shutdown()
        server.server_close()
        for socket in sockets:
            poller.unregister(socket)
            socket.close(0)
        context.term()
        if remote_file:
            remote_file.close()
        if event_file:
            event_file.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
