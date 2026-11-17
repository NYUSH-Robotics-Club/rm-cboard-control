"""Calibration profile storage shared by the UI and installed Quest tools."""

from __future__ import annotations

import math
import os
import re
from pathlib import Path
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
LEGACY_CALIBRATION_DIR = PACKAGE_ROOT / "calibrations"
PROFILE_RE = re.compile(r"^[A-Za-z0-9_.-]+$")


def calibration_dir() -> Path:
    """Return the user-owned profile directory, independent of any checkout."""

    configured = os.environ.get("QUEST_CALIBRATION_DIR")
    if configured:
        return Path(configured).expanduser().resolve()
    return Path.home() / ".config" / "quest-trajectory-recorder" / "calibrations"


DEFAULT_CALIBRATION_PATH = calibration_dir() / "quest_teleop_frame.json"


def sanitize_profile(raw: str | None, default: str) -> str:
    profile = (raw or default).strip()
    profile = profile.removesuffix(".json")
    if not profile or "/" in profile or "\\" in profile or not PROFILE_RE.fullmatch(profile):
        raise ValueError("profile must contain only letters, numbers, '_', '-', or '.'")
    return profile


def calibration_file(calibration_dir: Path, profile: str) -> Path:
    return calibration_dir / f"{profile}.json"


def profile_path(
    raw_profile: str,
    *,
    must_exist: bool = False,
    legacy_dir: Path | None = LEGACY_CALIBRATION_DIR,
) -> Path:
    """Resolve a named profile without exposing a repository path to consumers.

    New profiles live in the user configuration directory. A legacy checkout is
    read only as a compatibility source so existing local calibrations continue
    to work while they are migrated.
    """

    profile = sanitize_profile(raw_profile, "quest_teleop_frame")
    preferred = calibration_file(calibration_dir(), profile)
    candidates = [preferred]
    if legacy_dir is not None:
        legacy = calibration_file(legacy_dir, profile)
        if legacy != preferred:
            candidates.append(legacy)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    if must_exist:
        checked = ", ".join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(
            f"calibration profile {profile!r} was not found (checked {checked})"
        )
    return preferred


def calibration_complete(data: Any) -> bool:
    return bool(calibration_health(data)["valid"])


def calibration_health(data: Any) -> dict[str, Any]:
    """Validate geometry and report metrics suitable for a UI or doctor command."""

    issues: list[str] = []
    vectors: dict[str, list[float]] = {}
    if not isinstance(data, dict):
        return {"valid": False, "issues": ["profile must be a JSON object"]}
    for name in ("origin", "right", "forward", "up"):
        value = data.get(name)
        if not isinstance(value, dict):
            issues.append(f"{name} must be an xyz object")
            continue
        try:
            vector = [float(value[axis]) for axis in ("x", "y", "z")]
        except (KeyError, TypeError, ValueError):
            issues.append(f"{name} must contain finite numeric x/y/z")
            continue
        if not all(math.isfinite(item) for item in vector):
            issues.append(f"{name} must contain finite numeric x/y/z")
            continue
        vectors[name] = vector

    norms: dict[str, float] = {}
    normalized: dict[str, list[float]] = {}
    for name in ("right", "forward", "up"):
        if name not in vectors:
            continue
        norm = math.sqrt(sum(item * item for item in vectors[name]))
        norms[name] = norm
        if norm <= 1e-6:
            issues.append(f"{name} axis is degenerate")
        else:
            normalized[name] = [item / norm for item in vectors[name]]

    dot_products: dict[str, float] = {}
    for left, right in (("right", "forward"), ("right", "up"), ("forward", "up")):
        if left in normalized and right in normalized:
            dot = sum(a * b for a, b in zip(normalized[left], normalized[right]))
            dot_products[f"{left}_{right}"] = dot
            if abs(dot) > 0.05:
                issues.append(f"{left}/{right} axes are not orthogonal (dot={dot:.4f})")

    determinant = None
    if all(name in normalized for name in ("right", "forward", "up")):
        r, f, u = (normalized[name] for name in ("right", "forward", "up"))
        determinant = (
            r[0] * (f[1] * u[2] - f[2] * u[1])
            - r[1] * (f[0] * u[2] - f[2] * u[0])
            + r[2] * (f[0] * u[1] - f[1] * u[0])
        )
        # Quest / Unity world coordinates are left-handed.  A position frame
        # captured as physical right, forward, and up is therefore commonly a
        # reflection (det=-1), which is valid for mapping displacement vectors.
        # Rotation conversion handles that reflection with a basis conjugation
        # rather than treating the position basis itself as a rotation.
        if abs(determinant) < 0.95:
            issues.append(f"axes must form an orthonormal coordinate frame (det={determinant:.4f})")

    return {
        "valid": not issues,
        "issues": issues,
        "axis_norms": norms,
        "axis_dot_products": dot_products,
        "determinant": determinant,
        "handedness": (None if determinant is None else "right" if determinant > 0 else "left"),
    }
