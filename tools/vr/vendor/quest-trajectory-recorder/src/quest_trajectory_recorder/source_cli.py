"""Prepare Quest and publish calibrated controller targets over canonical ZMQ."""

from __future__ import annotations

import argparse
import json
import os
import sys

from .calibration_cli import main as calibration_main
from .calibration_profiles import calibration_health, profile_path
from .device_cli import main as device_main


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="lab")
    parser.add_argument("--target-bind", default="tcp://127.0.0.1:8130")
    parser.add_argument("--initial-gripper", choices=("open", "closed"), default="open")
    parser.add_argument("--adb-wait-seconds", type=float, default=120.0)
    parser.add_argument("--no-prepare", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.adb_wait_seconds < 0:
        raise ValueError("--adb-wait-seconds must be non-negative")
    calibration = profile_path(args.profile, must_exist=True)
    try:
        health = calibration_health(json.loads(calibration.read_text(encoding="utf-8")))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not read calibration profile {calibration}: {exc}") from exc
    if not health["valid"]:
        raise ValueError(
            f"invalid calibration profile {calibration}: {'; '.join(health['issues'])}"
        )
    # Calibration and streaming consume the same raw APK ports. Both belong to
    # this source adapter, so consumers never need to coordinate that handoff.
    if calibration_main(["stop"]) != 0:
        return 1
    if not args.no_prepare:
        status = device_main(["prepare", "--wait-seconds", str(args.adb_wait_seconds)])
        if status != 0:
            return status
    command = [
        sys.executable,
        "-m",
        "quest_trajectory_recorder.quest_tracker_hub",
        "--adb-reverse",
        "--calibration",
        str(calibration),
        "--target-bind",
        args.target_bind,
        "--initial-gripper",
        args.initial_gripper,
    ]
    os.execv(sys.executable, command)
    return 1  # pragma: no cover - execv only returns on failure


if __name__ == "__main__":
    raise SystemExit(main())
