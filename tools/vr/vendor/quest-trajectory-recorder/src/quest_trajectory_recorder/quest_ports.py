"""Quest app port constants and ADB helpers."""

from __future__ import annotations

import subprocess
from typing import Any

DEFAULT_GRIPPER_PORT = 8127
QUEST_REVERSE_PORTS = (8087, 8095, 8100, 8105, 8110, 8125, 8126, 8127, 10505, 15001)
QUEST_PACKAGE = "com.Xigbee.FrankaBot"
QUEST_ACTIVITY = "com.unity3d.player.UnityPlayerActivity"
QUEST_VR_CATEGORY = "com.oculus.intent.category.VR"
QUEST_BLOCKING_PACKAGES = (
    "com.oculus.firsttimenux",
    "com.oculus.panelapp.library",
    "com.oculus.store",
)
RESUMED_ACTIVITY_MARKERS = (
    "topResumedActivity",
    "mResumedActivity",
    "ResumedActivity:",
    "Resumed:",
)


def setup_adb_reverse(ports: list[int]) -> None:
    """Forward Quest localhost traffic back to this host over USB ADB."""
    for port in ports:
        subprocess.run(
            ["adb", "reverse", "--remove", f"tcp:{port}"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        subprocess.run(
            ["adb", "reverse", f"tcp:{port}", f"tcp:{port}"],
            check=True,
            stdout=subprocess.DEVNULL,
        )


def adb_reverse_ports() -> set[int]:
    """Return currently configured device-side ADB reverse TCP ports."""
    try:
        result = subprocess.run(
            ["adb", "reverse", "--list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return set()
    ports: set[int] = set()
    for token in result.stdout.split():
        if not token.startswith("tcp:"):
            continue
        try:
            ports.add(int(token.removeprefix("tcp:")))
        except ValueError:
            continue
    return ports


def adb_connected() -> bool:
    """Return whether one authorized Android / Quest device is reachable."""
    try:
        result = subprocess.run(
            ["adb", "get-state"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and result.stdout.strip() == "device"


def quest_activity_resumed(*, assume_connected: bool = False) -> bool:
    """Return whether the controller-tracking Unity activity owns XR focus."""
    if not assume_connected and not adb_connected():
        return False
    try:
        result = subprocess.run(
            ["adb", "shell", "dumpsys", "activity", "activities"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and activity_dump_reports_resumed(result.stdout)


def activity_dump_reports_resumed(output: str) -> bool:
    """Handle the resumed-activity labels used across Quest OS releases."""
    return any(
        QUEST_PACKAGE in line and any(marker in line for marker in RESUMED_ACTIVITY_MARKERS)
        for line in output.splitlines()
    )


def keep_quest_awake() -> None:
    """Best-effort wake and power policy used by explicit session preparation."""
    if not adb_connected():
        raise RuntimeError("Quest is not connected through ADB")
    for command in (
        ["adb", "shell", "input", "keyevent", "KEYCODE_WAKEUP"],
        ["adb", "shell", "svc", "power", "stayon", "true"],
        ["adb", "shell", "am", "broadcast", "-a", "com.oculus.vrpowermanager.prox_close"],
    ):
        subprocess.run(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
        )


def focus_frankabot(*, close_panels: bool = True, restart: bool = False) -> None:
    """Bring FrankaBot forward, restarting it only when explicitly requested."""
    if not adb_connected():
        raise RuntimeError("Quest is not connected through ADB")
    if close_panels:
        for package in QUEST_BLOCKING_PACKAGES:
            subprocess.run(
                ["adb", "shell", "am", "force-stop", package],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5.0,
            )
    command = ["adb", "shell", "am", "start"]
    if restart:
        command.append("-S")
    command.extend(
        [
            "-a",
            "android.intent.action.MAIN",
            "-c",
            QUEST_VR_CATEGORY,
            "-n",
            f"{QUEST_PACKAGE}/{QUEST_ACTIVITY}",
            "--es",
            "unity",
            "-force-gles",
        ]
    )
    subprocess.run(
        command,
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=8.0,
    )


def quest_device_info() -> dict[str, Any]:
    """Small read-only device diagnostic used in hub status heartbeats."""
    if not adb_connected():
        return {
            "adb_connected": False,
            "model": None,
            "serial": None,
            "app_resumed": False,
        }

    def value(*args: str) -> str | None:
        try:
            result = subprocess.run(
                ["adb", *args],
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return None
        text = result.stdout.strip()
        return text or None

    return {
        "adb_connected": True,
        "model": value("shell", "getprop", "ro.product.model"),
        "serial": value("get-serialno"),
        "app_resumed": quest_activity_resumed(assume_connected=True),
    }
