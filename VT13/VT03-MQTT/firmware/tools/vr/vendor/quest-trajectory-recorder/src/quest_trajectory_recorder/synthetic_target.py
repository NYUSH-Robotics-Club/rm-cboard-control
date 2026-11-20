"""Deterministic TeleopTarget generator for backend bring-up without controllers."""

from __future__ import annotations

import argparse
import math
import signal
import time
import uuid

import zmq
from embodied_ops.teleop import TeleopSourceStatus
from embodied_ops.teleop.zmq_transport import (
    DEFAULT_TARGET_ENDPOINT,
    TeleopTargetPublisher,
)

from .teleop_target import TeleopTarget


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default=DEFAULT_TARGET_ENDPOINT)
    parser.add_argument("--rate-hz", type=float, default=72.0)
    parser.add_argument(
        "--duration-sec", type=float, default=0.0, help="Zero runs until interrupted."
    )
    parser.add_argument("--pattern", choices=("hold", "circle", "axes"), default="circle")
    parser.add_argument("--amplitude-m", type=float, default=0.04)
    parser.add_argument("--period-sec", type=float, default=8.0)
    parser.add_argument("--gate-open", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gripper", choices=("open", "closed"), default="open")
    return parser.parse_args(argv)


def _position(pattern: str, elapsed: float, amplitude: float, period: float) -> list[float]:
    if pattern == "hold":
        return [0.0, 0.0, 0.0]
    phase = (elapsed % period) / period
    if pattern == "circle":
        angle = 2.0 * math.pi * phase
        return [
            amplitude * math.cos(angle),
            amplitude * math.sin(angle),
            0.5 * amplitude * math.sin(angle / 2.0),
        ]
    segment = int(phase * 6.0) % 6
    local = phase * 6.0 - segment
    axis = segment // 2
    sign = 1.0 if segment % 2 == 0 else -1.0
    result = [0.0, 0.0, 0.0]
    result[axis] = sign * amplitude * math.sin(math.pi * local)
    return result


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.rate_hz <= 0 or args.period_sec <= 0 or args.amplitude_m < 0:
        raise ValueError("rate and period must be positive; amplitude must be non-negative")
    if args.duration_sec < 0:
        raise ValueError("--duration-sec must be non-negative")
    context = zmq.Context()
    publisher = TeleopTargetPublisher(context, args.bind)
    stop = False

    def handle_signal(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    session_id = f"synthetic-{uuid.uuid4()}"
    started = time.monotonic()
    deadline = started
    seq = 0
    print(
        f"Synthetic TeleopTarget: {args.bind} pattern={args.pattern} rate={args.rate_hz:g}Hz",
        flush=True,
    )
    time.sleep(0.2)
    try:
        while not stop:
            now = time.monotonic()
            elapsed = now - started
            if args.duration_sec > 0 and elapsed >= args.duration_sec:
                break
            seq += 1
            position = _position(args.pattern, elapsed, args.amplitude_m, args.period_sec)
            now_ns = time.time_ns()
            target = TeleopTarget(
                seq=seq,
                timestamp=now_ns / 1e9,
                position=position,
                rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                gripper=1.0 if args.gripper == "closed" else -1.0,
                gate_open=args.gate_open,
                source="synthetic",
                session_id=session_id,
                frame_id=seq,
                host_received_monotonic_ns=time.monotonic_ns(),
                host_published_unix_ns=now_ns,
                tracking_valid=True,
                source_metadata={
                    "controller_id": "synthetic",
                    "raw_position": [
                        0.25 + position[0],
                        1.1 + position[2],
                        -0.2 + position[1],
                    ],
                    "raw_rotation": [0.0, 0.0, 0.0, 1.0],
                    "flag": False,
                    "pause_state": "High" if args.gate_open else "Low",
                    "remote_count": seq,
                    "calibration_id": "synthetic_identity",
                },
            )
            publisher.publish(target)
            if seq == 1 or seq % max(1, round(args.rate_hz)) == 0:
                publisher.publish_status(
                    TeleopSourceStatus(
                        source="synthetic",
                        session_id=session_id,
                        state="streaming" if args.gate_open else "ready",
                        target_seq=seq,
                        target_age_ms=0.0,
                        gate_open=args.gate_open,
                        control_ready=args.gate_open,
                        stream_online=True,
                        tracking_valid=True,
                        pause_state="High" if args.gate_open else "Low",
                        timestamp_unix_ns=now_ns,
                    )
                )
            deadline += 1.0 / args.rate_hz
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()
    finally:
        publisher.close()
        context.term()
    print(f"Synthetic target stopped after {seq} frames.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
