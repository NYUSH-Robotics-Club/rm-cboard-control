#!/usr/bin/env python3
"""Remove obvious placeholder and discontinuity artifacts from a remote trajectory CSV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def position(row: dict[str, str]) -> tuple[float, float, float]:
    return tuple(float(row[f"pos_{axis}"]) for axis in "xyz")  # type: ignore[return-value]


def is_origin(pos: tuple[float, float, float]) -> bool:
    return all(abs(value) < 1e-8 for value in pos)


def path_length(points: list[tuple[float, float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def default_output_path(path: Path) -> Path:
    return path.with_name(f"{path.stem}_cleaned{path.suffix}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="Input *_remote.csv file.")
    parser.add_argument(
        "--out", type=Path, help="Output CSV path. Defaults to *_cleaned.csv next to input."
    )
    parser.add_argument(
        "--keep-origin", action="store_true", help="Keep exact 0,0,0 placeholder rows."
    )
    parser.add_argument(
        "--max-step-m",
        type=float,
        default=0.20,
        help="Drop rows whose step from the last kept row exceeds this distance; 0 disables.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    src = args.csv.resolve()
    dst = (args.out or default_output_path(src)).resolve()
    dst.parent.mkdir(parents=True, exist_ok=True)

    kept_rows: list[dict[str, str]] = []
    kept_points: list[tuple[float, float, float]] = []
    dropped_origin = 0
    dropped_jump = 0
    jump_examples: list[
        tuple[int, int, float, tuple[float, float, float], tuple[float, float, float]]
    ] = []

    with src.open(newline="", encoding="utf-8") as input_file:
        reader = csv.DictReader(input_file)
        if not reader.fieldnames:
            raise SystemExit(f"No CSV header found in {src}")
        for row_index, row in enumerate(reader, start=1):
            try:
                pos = position(row)
            except (KeyError, ValueError) as exc:
                raise SystemExit(f"Invalid position at row {row_index}: {exc}") from exc

            if not args.keep_origin and is_origin(pos):
                dropped_origin += 1
                continue

            if kept_points and args.max_step_m > 0:
                step = math.dist(kept_points[-1], pos)
                if step > args.max_step_m:
                    dropped_jump += 1
                    if len(jump_examples) < 8:
                        jump_examples.append(
                            (row_index, int(float(row.get("seq", "0"))), step, kept_points[-1], pos)
                        )
                    continue

            kept_rows.append(row)
            kept_points.append(pos)

    with dst.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=reader.fieldnames)
        writer.writeheader()
        writer.writerows(kept_rows)

    print(f"Input: {src}")
    print(f"Output: {dst}")
    print(f"Rows kept: {len(kept_rows)}")
    print(f"Dropped origin placeholders: {dropped_origin}")
    print(f"Dropped jump rows: {dropped_jump} (max_step_m={args.max_step_m:.3f})")
    print(f"Cleaned path length: {path_length(kept_points):.6f} m")
    for row_index, seq, step, prev, pos in jump_examples:
        print(
            f"  dropped row={row_index} seq={seq} step={step:.3f} m "
            f"from=({prev[0]:.3f},{prev[1]:.3f},{prev[2]:.3f}) "
            f"to=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
