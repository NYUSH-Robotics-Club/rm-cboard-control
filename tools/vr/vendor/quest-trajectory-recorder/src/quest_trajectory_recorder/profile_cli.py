"""Manage named Quest calibration profiles independently of a source checkout."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from .calibration_profiles import (
    LEGACY_CALIBRATION_DIR,
    calibration_dir,
    calibration_health,
    profile_path,
    sanitize_profile,
)


def _valid_profile(path: Path) -> bool:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return bool(calibration_health(data)["valid"])


def migrate_profile(raw_profile: str) -> Path:
    """Copy one valid legacy profile into user storage without overwriting."""

    profile = sanitize_profile(raw_profile, "quest_teleop_frame")
    destination = profile_path(profile, legacy_dir=None)
    if destination.is_file():
        if not _valid_profile(destination):
            raise ValueError(f"existing destination profile is invalid: {destination}")
        return destination
    source = LEGACY_CALIBRATION_DIR / f"{profile}.json"
    if not source.is_file():
        raise FileNotFoundError(f"legacy calibration profile does not exist: {source}")
    if not _valid_profile(source):
        raise ValueError(f"legacy calibration profile is invalid: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return destination


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    path_parser = subparsers.add_parser("path", help="Print the resolved profile path")
    path_parser.add_argument("profile")
    path_parser.add_argument("--for-write", action="store_true")
    subparsers.add_parser("list", help="List valid named profiles")
    migrate = subparsers.add_parser(
        "migrate", help="Copy a legacy checkout profile to user storage"
    )
    migrate.add_argument("profile")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.command == "path":
        path = profile_path(
            args.profile,
            must_exist=not args.for_write,
            legacy_dir=None if args.for_write else LEGACY_CALIBRATION_DIR,
        )
        if args.for_write:
            path.parent.mkdir(parents=True, exist_ok=True)
        print(path)
        return 0
    if args.command == "migrate":
        print(migrate_profile(args.profile))
        return 0

    found: dict[str, Path] = {}
    for directory in (calibration_dir(), LEGACY_CALIBRATION_DIR):
        if not directory.is_dir():
            continue
        for path in sorted(directory.glob("*.json")):
            if _valid_profile(path):
                found.setdefault(path.stem, path)
    for name, path in sorted(found.items()):
        location = "user" if path.parent == calibration_dir() else "legacy"
        print(f"{name}\t{location}\t{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
