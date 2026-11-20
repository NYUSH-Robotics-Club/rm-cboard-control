#!/usr/bin/env python3
"""Print simple sanity checks for the controller-tracking Quest APK CSV captures."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

AXES = ("x", "y", "z")
POS_KEYS = ("pos_x", "pos_y", "pos_z")
QUAT_KEYS = ("quat_x", "quat_y", "quat_z", "quat_w")


def parse_float(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pos = tuple(parse_float(row.get(k, "0")) for k in POS_KEYS)
            quat = tuple(parse_float(row.get(k, "0")) for k in QUAT_KEYS)
            points = []
            for index in range(3):
                keys = (f"point{index}_x", f"point{index}_y", f"point{index}_z")
                if all(row.get(key, "") != "" for key in keys):
                    points.append(tuple(parse_float(row.get(key, "0")) for key in keys))
            rows.append(
                {
                    "seq": int(parse_float(row.get("seq", "0"))),
                    "t": parse_float(row.get("recv_unix", "0")),
                    "pos": pos,
                    "quat": quat,
                    "points": points,
                }
            )
    return rows


def is_origin(pos: tuple[float, float, float], eps: float = 1e-8) -> bool:
    return abs(pos[0]) < eps and abs(pos[1]) < eps and abs(pos[2]) < eps


def norm3(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def sub3(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def fmt_m(value: float) -> str:
    return f"{value:.6f} m"


def fmt_s(value: float) -> str:
    return f"{value:.6f} s"


def mean(values: list[float]) -> float:
    return statistics.mean(values) if values else 0.0


def pstdev(values: list[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def quat_norm(q: tuple[float, float, float, float]) -> float:
    return math.sqrt(sum(x * x for x in q))


def quat_angle_deg(
    q0: tuple[float, float, float, float], q1: tuple[float, float, float, float]
) -> float:
    n0 = quat_norm(q0)
    n1 = quat_norm(q1)
    if n0 == 0 or n1 == 0:
        return 0.0
    dot = abs(sum(a * b for a, b in zip(q0, q1)) / (n0 * n1))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def normalize3(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = norm3(v)
    if length <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def dot3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def quat_axes_xyzw(q: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = q
    n = quat_norm(q)
    if n <= 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        normalize3((1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy))),
        normalize3((2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx))),
        normalize3((2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy))),
    )


def print_auxiliary_axis_report(rows: list[dict[str, object]]) -> None:
    samples: list[
        tuple[
            tuple[float, float, float],
            list[tuple[float, float, float]],
            tuple[float, float, float, float],
        ]
    ] = []
    for row in rows:
        pos = row["pos"]
        quat = row["quat"]
        points = row.get("points", [])
        if (
            isinstance(pos, tuple)
            and isinstance(quat, tuple)
            and isinstance(points, list)
            and len(points) >= 3
        ):
            samples.append((pos, points, quat))  # type: ignore[arg-type]

    if not samples:
        return

    labels = ("quat +X", "quat +Y", "quat +Z")
    print("Auxiliary axis points:")
    for point_index in range(3):
        lengths: list[float] = []
        signed_matches: list[tuple[int, float]] = []
        for pos, points, quat in samples:
            axis = sub3(points[point_index], pos)
            lengths.append(norm3(axis))
            unit_axis = normalize3(axis)
            q_axes = quat_axes_xyzw(quat)
            dots = [dot3(unit_axis, q_axis) for q_axis in q_axes]
            best = max(range(3), key=lambda index: abs(dots[index]))
            signed_matches.append((best, dots[best]))

        counts = {
            index: sum(1 for best, _ in signed_matches if best == index) for index in range(3)
        }
        best_index = max(counts, key=counts.get)
        best_dots = [dot for best, dot in signed_matches if best == best_index]
        sign = "+" if mean(best_dots) >= 0 else "-"
        print(
            f"  point{point_index}: length_mean={mean(lengths):.6f} m, "
            f"length_std={pstdev(lengths):.6f} m, "
            f"best_match={sign}{labels[best_index].replace('quat +', '')} "
            f"(mean_dot={mean(best_dots):.3f}, samples={counts[best_index]}/{len(samples)})"
        )


def print_static_segment(name: str, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    positions = [r["pos"] for r in rows]
    assert all(isinstance(p, tuple) for p in positions)
    xs = [p[0] for p in positions]  # type: ignore[index]
    ys = [p[1] for p in positions]  # type: ignore[index]
    zs = [p[2] for p in positions]  # type: ignore[index]
    center = (mean(xs), mean(ys), mean(zs))
    radii = [norm3(sub3(p, center)) for p in positions]  # type: ignore[arg-type]
    print(f"{name} static-window sanity ({len(rows)} samples; only valid if you held still):")
    print(f"  centroid: x={center[0]:.6f}, y={center[1]:.6f}, z={center[2]:.6f}")
    print(f"  std:      x={pstdev(xs):.6f}, y={pstdev(ys):.6f}, z={pstdev(zs):.6f}")
    print(
        f"  jitter:   rms={math.sqrt(mean([r * r for r in radii])):.6f} m, max={max(radii):.6f} m"
    )


def analyze(path: Path, drop_leading_origin: bool, static_samples: int) -> None:
    rows = load_rows(path)
    if not rows:
        raise SystemExit(f"No rows found in {path}")

    leading_origin = 0
    for row in rows:
        if is_origin(row["pos"]):  # type: ignore[arg-type]
            leading_origin += 1
        else:
            break
    all_origin = sum(1 for row in rows if is_origin(row["pos"]))  # type: ignore[arg-type]

    work_rows = rows[leading_origin:] if drop_leading_origin else rows
    if not work_rows:
        work_rows = rows

    positions = [r["pos"] for r in work_rows]
    quats = [r["quat"] for r in work_rows]
    times = [r["t"] for r in work_rows]
    assert all(isinstance(p, tuple) for p in positions)
    assert all(isinstance(q, tuple) for q in quats)

    xs = [p[0] for p in positions]  # type: ignore[index]
    ys = [p[1] for p in positions]  # type: ignore[index]
    zs = [p[2] for p in positions]  # type: ignore[index]
    ranges = [max(vals) - min(vals) for vals in (xs, ys, zs)]

    path_len = 0.0
    steps: list[
        tuple[float, float, int, int, tuple[float, float, float], tuple[float, float, float]]
    ] = []
    for row_a, row_b, pos_a, pos_b in zip(work_rows, work_rows[1:], positions, positions[1:]):
        step = norm3(sub3(pos_b, pos_a))  # type: ignore[arg-type]
        path_len += step
        steps.append(
            (
                step,
                float(row_b["t"]) - float(row_a["t"]),
                int(row_a["seq"]),
                int(row_b["seq"]),
                pos_a,  # type: ignore[arg-type]
                pos_b,  # type: ignore[arg-type]
            )
        )

    displacement = sub3(positions[-1], positions[0])  # type: ignore[arg-type]
    straight = norm3(displacement)
    duration = max(0.0, float(times[-1]) - float(times[0])) if len(times) > 1 else 0.0
    fps = (len(times) - 1) / duration if duration > 0 else 0.0

    print(f"CSV: {path}")
    print(f"Rows: {len(rows)} total, {len(work_rows)} analyzed")
    print(f"Origin placeholder rows: {all_origin} total, {leading_origin} leading")
    if drop_leading_origin and leading_origin:
        print("Dropped leading origin rows for motion statistics.")
    print()

    print("Timing:")
    print(f"  receive span: {fmt_s(duration)}")
    print(f"  receive rate estimate: {fps:.1f} Hz")
    if fps > 240:
        print(
            "  warning: receive timestamps look buffered/bursty; use them for ordering, not timing accuracy."
        )
    print()

    print("Position ranges:")
    for axis, vals, rng in zip(AXES, (xs, ys, zs), ranges):
        print(
            f"  {axis}: min={fmt_m(min(vals))}, max={fmt_m(max(vals))}, range={fmt_m(rng)}, std={fmt_m(pstdev(vals))}"
        )

    dominant = max(range(3), key=lambda i: ranges[i])
    second = sorted(ranges, reverse=True)[1] if len(ranges) > 1 else 0.0
    ratio = ranges[dominant] / second if second > 0 else float("inf")
    print(
        f"  dominant varying stream axis: +-{AXES[dominant].upper()} (range {fmt_m(ranges[dominant])}, ratio vs next {ratio:.2f}x)"
    )
    print()

    print("Path:")
    print(f"  start: x={positions[0][0]:.6f}, y={positions[0][1]:.6f}, z={positions[0][2]:.6f}")  # type: ignore[index]
    print(f"  end:   x={positions[-1][0]:.6f}, y={positions[-1][1]:.6f}, z={positions[-1][2]:.6f}")  # type: ignore[index]
    print(
        f"  displacement: dx={displacement[0]:.6f}, dy={displacement[1]:.6f}, dz={displacement[2]:.6f}, norm={fmt_m(straight)}"
    )
    print(f"  integrated path length: {fmt_m(path_len)}")
    jump_candidates = [step for step in steps if step[0] > 0.10]
    if jump_candidates:
        print(f"  jump candidates >0.100 m: {len(jump_candidates)}")
        for step, step_dt, seq_a, seq_b, pos_a, pos_b in sorted(jump_candidates, reverse=True)[:8]:
            print(
                f"    seq {seq_a}->{seq_b}: step={step:.3f} m, dt={step_dt:.4f}s, "
                f"from=({pos_a[0]:.3f},{pos_a[1]:.3f},{pos_a[2]:.3f}) "
                f"to=({pos_b[0]:.3f},{pos_b[1]:.3f},{pos_b[2]:.3f})"
            )
    print()

    quat_norms = [quat_norm(q) for q in quats]  # type: ignore[arg-type]
    rotation_from_start = [quat_angle_deg(quats[0], q) for q in quats]  # type: ignore[arg-type]
    print("Orientation:")
    print(
        f"  quaternion norm: min={min(quat_norms):.6f}, max={max(quat_norms):.6f}, mean={mean(quat_norms):.6f}"
    )
    print(f"  max rotation from first sample: {max(rotation_from_start):.2f} deg")
    print()

    print_auxiliary_axis_report(work_rows)
    print()

    if static_samples > 0:
        count = min(static_samples, len(work_rows))
        print_static_segment("First", work_rows[:count])
        print_static_segment("Last", work_rows[-count:])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv", type=Path, help="A *_remote.csv capture from quest_trajectory_recorder.receiver"
    )
    parser.add_argument(
        "--drop-leading-origin",
        action="store_true",
        help="Ignore initial rows where position is exactly 0,0,0.",
    )
    parser.add_argument(
        "--static-samples",
        type=int,
        default=50,
        help="Number of first/last samples to use for static-window jitter checks.",
    )
    args = parser.parse_args()
    analyze(args.csv, args.drop_leading_origin, max(0, args.static_samples))


if __name__ == "__main__":
    main()
