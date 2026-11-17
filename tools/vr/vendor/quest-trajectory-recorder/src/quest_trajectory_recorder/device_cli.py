"""Small, scriptable ADB control surface for the Quest teleoperation device."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from typing import Any

from .quest_ports import (
    QUEST_REVERSE_PORTS,
    adb_connected,
    adb_reverse_ports,
    focus_frankabot,
    keep_quest_awake,
    quest_activity_resumed,
    quest_device_info,
    setup_adb_reverse,
)


def status_payload() -> dict[str, Any]:
    """Return stable, machine-readable device and port readiness state."""
    device = quest_device_info()
    mapped = adb_reverse_ports() if device["adb_connected"] else set()
    missing = sorted(set(QUEST_REVERSE_PORTS) - mapped)
    return {
        "schema_version": "quest.adb_status/v1",
        "ready": bool(device["adb_connected"] and device["app_resumed"] and not missing),
        "adb_connected": device["adb_connected"],
        "app_resumed": device["app_resumed"],
        "model": device["model"],
        "serial": device["serial"],
        "mapped_ports": sorted(mapped),
        "missing_ports": missing,
    }


def _print_status(payload: dict[str, Any], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return
    adb_label = "connected" if payload["adb_connected"] else "offline"
    app_label = "active" if payload["app_resumed"] else "not active"
    ports_label = "ready" if not payload["missing_ports"] else f"missing {payload['missing_ports']}"
    identity = " / ".join(value for value in (payload["model"], payload["serial"]) if value)
    print(f"Quest:     {adb_label}{f' ({identity})' if identity else ''}")
    print(f"FrankaBot: {app_label}")
    print(f"ADB ports: {ports_label}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    status = subparsers.add_parser("status", help="Show Quest, app, and reverse-port state")
    status.add_argument("--json", action="store_true", dest="as_json")

    prepare = subparsers.add_parser(
        "prepare",
        help="Wait for ADB, wake Quest, restore ports, and focus FrankaBot without restarting it",
    )
    prepare.add_argument("--wait-seconds", type=float, default=120.0)
    subparsers.add_parser("focus", help="Bring FrankaBot forward without restarting it")
    subparsers.add_parser("restart", help="Explicitly restart FrankaBot and bring it forward")
    return parser.parse_args(argv)


def _wait_for_adb(timeout: float) -> bool:
    if adb_connected():
        return True
    print("Waiting for Quest ADB (wake the headset and authorize USB debugging)...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.25)
        if adb_connected():
            return True
    return False


def _wait_for_frankabot(timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if quest_activity_resumed(assume_connected=True):
            return True
        time.sleep(0.1)
    return False


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.command == "status":
        payload = status_payload()
        _print_status(payload, as_json=args.as_json)
        return 0 if payload["ready"] else 1

    if args.command == "prepare" and args.wait_seconds < 0:
        print("--wait-seconds must be non-negative.")
        return 2
    wait_seconds = args.wait_seconds if args.command == "prepare" else 0.0
    if not _wait_for_adb(wait_seconds):
        print("Quest is not connected through ADB.")
        return 1

    try:
        if args.command == "prepare":
            keep_quest_awake()
            setup_adb_reverse(list(QUEST_REVERSE_PORTS))
            focus_frankabot()
            if not _wait_for_frankabot():
                print("Quest is connected, but FrankaBot did not acquire XR foreground.")
                return 1
            print("Quest is awake; reverse ports and FrankaBot XR foreground are ready.")
        elif args.command == "focus":
            focus_frankabot()
            print("FrankaBot was focused without a restart.")
        else:
            focus_frankabot(restart=True)
            print("FrankaBot was explicitly restarted and focused.")
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"Quest preparation failed: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
