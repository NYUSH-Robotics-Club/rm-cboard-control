"""Quest source adapter for the hardware-neutral embodied-ops target contract."""

from __future__ import annotations

import math
import time
from typing import Any

from embodied_ops.teleop import TARGET_SCHEMA, TeleopTarget

from .teleop_frame import (
    QuestCalibration,
    quest_pos_to_teleop,
    quest_rotation_to_teleop_matrix,
)

TELEOP_TARGET_SCHEMA = TARGET_SCHEMA


def valid_remote(remote: dict[str, Any] | None) -> bool:
    if not remote:
        return False
    try:
        position = [float(value) for value in remote["position"]]
        rotation = [float(value) for value in remote["rotation"]]
    except (KeyError, TypeError, ValueError):
        return False
    return (
        len(position) == 3
        and len(rotation) == 4
        and all(math.isfinite(value) for value in (*position, *rotation))
        and any(abs(value) > 1e-8 for value in position)
        and math.sqrt(sum(value * value for value in rotation)) > 1e-6
    )


def target_from_remote(
    remote: dict[str, Any],
    *,
    calibration: QuestCalibration | None,
    seq: int,
    gripper: float,
    gate_open: bool,
    pause_state: str | None,
    remote_count: int,
    source: str = "quest",
    session_id: str = "unspecified",
    received_monotonic_ns: int | None = None,
    calibration_id: str | None = None,
    calibration_sha256: str | None = None,
) -> TeleopTarget:
    """Calibrate a Quest-specific raw frame into the shared Cartesian target."""

    now_ns = time.time_ns()
    return TeleopTarget(
        seq=seq,
        timestamp=now_ns / 1_000_000_000.0,
        position=quest_pos_to_teleop(remote["position"], calibration),
        rotation=quest_rotation_to_teleop_matrix(remote["rotation"], calibration),
        gripper=gripper,
        gate_open=gate_open,
        source=source,
        session_id=session_id,
        frame_id=remote_count,
        host_received_monotonic_ns=(
            time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns
        ),
        host_published_unix_ns=now_ns,
        tracking_valid=True,
        source_metadata={
            "controller_id": "right",
            "raw_position": [float(value) for value in remote["position"]],
            "raw_rotation": [float(value) for value in remote["rotation"]],
            "flag": bool(remote.get("flag")),
            "pause_state": pause_state,
            "remote_count": remote_count,
            "calibration_id": calibration_id,
            "calibration_sha256": calibration_sha256,
        },
    )
