"""Read-only Quest/APK/ADB/calibration readiness report."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any

from .calibration_profiles import DEFAULT_CALIBRATION_PATH, calibration_health
from .quest_ports import QUEST_PACKAGE, quest_activity_resumed

REQUIRED_REVERSE_PORTS = (8095, 8100, 8125, 8127)


def _run(*arguments: str) -> subprocess.CompletedProcess[str] | None:
    try:
        return subprocess.run(
            list(arguments),
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


def parse_reverse_ports(output: str) -> set[int]:
    ports: set[int] = set()
    for line in output.splitlines():
        for token in line.split():
            if token.startswith("tcp:"):
                try:
                    ports.add(int(token.removeprefix("tcp:")))
                except ValueError:
                    continue
    return ports


def _result(name: str, status: str, detail: str, **extra: Any) -> dict[str, Any]:
    return {"name": name, "status": status, "detail": detail, **extra}


def build_report(calibration_path: Path) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    adb_version = _run("adb", "version")
    adb_available = bool(adb_version and adb_version.returncode == 0)
    checks.append(
        _result(
            "adb_tool",
            "pass" if adb_available else "fail",
            (adb_version.stdout.splitlines()[0] if adb_available else "adb is unavailable"),
        )
    )
    state = _run("adb", "get-state") if adb_available else None
    connected = bool(state and state.returncode == 0 and state.stdout.strip() == "device")
    checks.append(
        _result(
            "quest_usb",
            "pass" if connected else "fail",
            "authorized Quest/Android device" if connected else "no authorized ADB device",
        )
    )

    model = serial = None
    if connected:
        model_result = _run("adb", "shell", "getprop", "ro.product.model")
        serial_result = _run("adb", "get-serialno")
        model = None if model_result is None else model_result.stdout.strip() or None
        serial = None if serial_result is None else serial_result.stdout.strip() or None

        package = _run("adb", "shell", "dumpsys", "package", QUEST_PACKAGE)
        package_text = "" if package is None else package.stdout
        installed = "versionName=" in package_text
        version_line = next(
            (line.strip() for line in package_text.splitlines() if "versionName=" in line),
            "version unavailable",
        )
        checks.append(
            _result(
                "frankabot_apk",
                "pass" if installed else "fail",
                version_line if installed else f"{QUEST_PACKAGE} is not installed",
            )
        )

        focused = quest_activity_resumed(assume_connected=True)
        checks.append(
            _result(
                "frankabot_activity",
                "pass" if focused else "warn",
                "Unity activity is resumed" if focused else "APK is not currently resumed",
            )
        )

        reverse = _run("adb", "reverse", "--list")
        mapped = parse_reverse_ports("" if reverse is None else reverse.stdout)
        missing = sorted(set(REQUIRED_REVERSE_PORTS) - mapped)
        checks.append(
            _result(
                "adb_reverse",
                "pass" if not missing else "warn",
                "required raw-stream ports mapped"
                if not missing
                else f"missing reverse ports: {missing}; rerun scripts/start_frankabot.sh --no-install",
                mapped_ports=sorted(mapped),
            )
        )

    calibration = None
    if calibration_path.is_file():
        try:
            value = json.loads(calibration_path.read_text(encoding="utf-8"))
            calibration = calibration_health(value)
        except (OSError, json.JSONDecodeError) as exc:
            calibration = {"valid": False, "issues": [str(exc)]}
        checks.append(
            _result(
                "calibration",
                "pass" if calibration["valid"] else "fail",
                str(calibration_path) if calibration["valid"] else "; ".join(calibration["issues"]),
                health=calibration,
            )
        )
    else:
        checks.append(
            _result(
                "calibration",
                "warn",
                f"not created yet: {calibration_path} (controller required)",
            )
        )

    return {
        "schema_version": "quest.device_doctor/v1",
        "ready_without_controller": not any(row["status"] == "fail" for row in checks),
        "controller_pose_verified": False,
        "controller_pose_note": "Run the tracker hub with a hand controller to verify pose/buttons/calibration.",
        "device": {"model": model, "serial": serial},
        "calibration_path": str(calibration_path),
        "checks": checks,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument("--json", action="store_true", dest="as_json")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_report(args.calibration)
    if args.as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for check in report["checks"]:
            print(f"[{check['status'].upper():4}] {check['name']}: {check['detail']}")
        print("[INFO] controller_pose: deferred until a hand controller is available")
    return 0 if report["ready_without_controller"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
