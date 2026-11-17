#!/usr/bin/env python3
"""Render a dependency-free 3D perspective SVG trajectory preview."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import shutil
import subprocess
from pathlib import Path

CAPTURE_DIR = Path("captures")
PLOT_DIR = Path("plots")


def latest_remote_csv() -> Path:
    candidates = [p for p in CAPTURE_DIR.glob("*_remote.csv") if p.stat().st_size > 200]
    if not candidates:
        raise SystemExit(f"No usable *_remote.csv found in {CAPTURE_DIR}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot Quest controller trajectory as a 3D SVG.")
    parser.add_argument(
        "csv_path", nargs="?", type=Path, help="Path to *_remote.csv. Defaults to latest."
    )
    parser.add_argument("--out", type=Path, help="Output SVG path.")
    parser.add_argument(
        "--png", action="store_true", help="Also convert SVG to PNG with macOS sips."
    )
    parser.add_argument("--max-points", type=int, default=2500)
    parser.add_argument("--azimuth", type=float, default=42.0, help="Camera azimuth in degrees.")
    parser.add_argument(
        "--elevation", type=float, default=24.0, help="Camera elevation in degrees."
    )
    parser.add_argument(
        "--keep-origin",
        "--keep-leading-origin",
        action="store_true",
        help="Do not drop exact 0,0,0 rows. By default these are treated as tracking placeholders.",
    )
    return parser.parse_args()


def is_origin(row: dict[str, float | str]) -> bool:
    return (
        abs(float(row["x"])) < 1e-8 and abs(float(row["y"])) < 1e-8 and abs(float(row["z"])) < 1e-8
    )


def load_positions(
    path: Path, max_points: int, drop_origin: bool
) -> tuple[list[dict[str, float | str]], int, int]:
    rows: list[dict[str, float | str]] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                rows.append(
                    {
                        "recv_unix": float(row.get("recv_unix") or len(rows)),
                        "recv_iso": row.get("recv_iso", ""),
                        "x": float(row["pos_x"]),
                        "y": float(row["pos_y"]),
                        "z": float(row["pos_z"]),
                    }
                )
            except (KeyError, ValueError):
                continue
    total = len(rows)
    dropped = 0
    if drop_origin:
        kept = [row for row in rows if not is_origin(row)]
        if kept:
            dropped = len(rows) - len(kept)
            rows = kept
    if max_points > 0 and len(rows) > max_points:
        stride = math.ceil(len(rows) / max_points)
        rows = rows[::stride]
    return rows, total, dropped


def esc(text: object) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def path_length(rows: list[dict[str, float | str]]) -> float:
    length = 0.0
    for a, b in zip(rows, rows[1:]):
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        dz = float(b["z"]) - float(a["z"])
        length += math.sqrt(dx * dx + dy * dy + dz * dz)
    return length


class Projector:
    def __init__(
        self, rows: list[dict[str, float | str]], azimuth: float, elevation: float
    ) -> None:
        self.az = math.radians(azimuth)
        self.el = math.radians(elevation)
        self.cx = (min(float(r["x"]) for r in rows) + max(float(r["x"]) for r in rows)) / 2
        self.cy = (min(float(r["y"]) for r in rows) + max(float(r["y"]) for r in rows)) / 2
        self.cz = (min(float(r["z"]) for r in rows) + max(float(r["z"]) for r in rows)) / 2

    def raw_project(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        x -= self.cx
        y -= self.cy
        z -= self.cz
        xr = math.cos(self.az) * x - math.sin(self.az) * z
        zr = math.sin(self.az) * x + math.cos(self.az) * z
        yr = math.cos(self.el) * y - math.sin(self.el) * zr
        depth = math.sin(self.el) * y + math.cos(self.el) * zr
        return xr, -yr, depth


def render_svg(
    rows: list[dict[str, float | str]],
    total_rows: int,
    dropped_leading_origin: int,
    source: Path,
    azimuth: float,
    elevation: float,
) -> str:
    if not rows:
        raise SystemExit("No valid trajectory rows found.")

    width, height = 1200, 860
    margin = 90
    projector = Projector(rows, azimuth, elevation)

    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]
    zs = [float(r["z"]) for r in rows]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    xpad = max((xmax - xmin) * 0.08, 0.05)
    ypad = max((ymax - ymin) * 0.08, 0.05)
    zpad = max((zmax - zmin) * 0.08, 0.05)
    xmin, xmax = xmin - xpad, xmax + xpad
    ymin, ymax = ymin - ypad, ymax + ypad
    zmin, zmax = zmin - zpad, zmax + zpad

    # Include trajectory, box corners, and floor grid in the scale calculation.
    points_3d: list[tuple[float, float, float]] = [
        (float(r["x"]), float(r["y"]), float(r["z"])) for r in rows
    ]
    for x in (xmin, xmax):
        for y in (ymin, ymax):
            for z in (zmin, zmax):
                points_3d.append((x, y, z))
    for i in range(6):
        t = i / 5
        points_3d.extend(
            [
                (xmin + (xmax - xmin) * t, ymin, zmin),
                (xmin + (xmax - xmin) * t, ymin, zmax),
                (xmin, ymin, zmin + (zmax - zmin) * t),
                (xmax, ymin, zmin + (zmax - zmin) * t),
            ]
        )

    raw = [projector.raw_project(*p) for p in points_3d]
    umin, umax = min(p[0] for p in raw), max(p[0] for p in raw)
    vmin, vmax = min(p[1] for p in raw), max(p[1] for p in raw)
    scale = min(
        (width - 2 * margin) / max(umax - umin, 1e-9), (height - 190) / max(vmax - vmin, 1e-9)
    )
    ox = width / 2 - (umin + umax) * scale / 2
    oy = 500 - (vmin + vmax) * scale / 2

    def project(x: float, y: float, z: float) -> tuple[float, float, float]:
        u, v, d = projector.raw_project(x, y, z)
        return ox + u * scale, oy + v * scale, d

    def line(a: tuple[float, float, float], b: tuple[float, float, float], cls: str) -> str:
        ax, ay, _ = project(*a)
        bx, by, _ = project(*b)
        return f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{bx:.1f}" y2="{by:.1f}" class="{cls}"/>'

    floor_lines = []
    for i in range(6):
        t = i / 5
        floor_lines.append(
            line(
                (xmin + (xmax - xmin) * t, ymin, zmin),
                (xmin + (xmax - xmin) * t, ymin, zmax),
                "grid",
            )
        )
        floor_lines.append(
            line(
                (xmin, ymin, zmin + (zmax - zmin) * t),
                (xmax, ymin, zmin + (zmax - zmin) * t),
                "grid",
            )
        )

    box_edges = [
        ((xmin, ymin, zmin), (xmax, ymin, zmin)),
        ((xmax, ymin, zmin), (xmax, ymin, zmax)),
        ((xmax, ymin, zmax), (xmin, ymin, zmax)),
        ((xmin, ymin, zmax), (xmin, ymin, zmin)),
        ((xmin, ymax, zmin), (xmax, ymax, zmin)),
        ((xmax, ymax, zmin), (xmax, ymax, zmax)),
        ((xmax, ymax, zmax), (xmin, ymax, zmax)),
        ((xmin, ymax, zmax), (xmin, ymax, zmin)),
        ((xmin, ymin, zmin), (xmin, ymax, zmin)),
        ((xmax, ymin, zmin), (xmax, ymax, zmin)),
        ((xmax, ymin, zmax), (xmax, ymax, zmax)),
        ((xmin, ymin, zmax), (xmin, ymax, zmax)),
    ]
    box_svg = [line(a, b, "box") for a, b in box_edges]

    projected_rows = [
        (project(float(r["x"]), float(r["y"]), float(r["z"])), i) for i, r in enumerate(rows)
    ]
    segments = []
    for (a, ia), (b, ib) in zip(projected_rows, projected_rows[1:]):
        t = ib / max(len(rows) - 1, 1)
        hue = 196 - 160 * t
        color = f"hsl({hue:.1f}, 78%, 34%)"
        width_px = 3.0 + 1.2 * t
        segments.append(
            f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" '
            f'stroke="{color}" stroke-width="{width_px:.2f}" class="traj-seg"/>'
        )

    sx, sy, _ = projected_rows[0][0]
    ex, ey, _ = projected_rows[-1][0]
    axis_origin = (xmin, ymin, zmin)
    axes = [
        (axis_origin, (xmax, ymin, zmin), "axis-x", "X"),
        (axis_origin, (xmin, ymax, zmin), "axis-y", "Y"),
        (axis_origin, (xmin, ymin, zmax), "axis-z", "Z"),
    ]
    axis_svg = []
    for a, b, cls, label in axes:
        axis_svg.append(line(a, b, cls))
        lx, ly, _ = project(*b)
        axis_svg.append(
            f'<text x="{lx + 8:.1f}" y="{ly - 8:.1f}" class="{cls} label">{label}</text>'
        )

    duration = float(rows[-1]["recv_unix"]) - float(rows[0]["recv_unix"])
    generated = dt.datetime.now().astimezone().isoformat(timespec="seconds")

    drop_note = f" | dropped_origin={dropped_leading_origin}" if dropped_leading_origin else ""
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <style>
    .bg {{ fill: #f8f4eb; }}
    .grid {{ stroke: #d9d0bf; stroke-width: 1.1; }}
    .box {{ stroke: #9a8f7c; stroke-width: 1.2; fill: none; opacity: 0.7; }}
    .axis-x {{ stroke: #c94033; fill: #c94033; stroke-width: 3.5; font: 700 18px Menlo, monospace; }}
    .axis-y {{ stroke: #208f55; fill: #208f55; stroke-width: 3.5; font: 700 18px Menlo, monospace; }}
    .axis-z {{ stroke: #2a66b7; fill: #2a66b7; stroke-width: 3.5; font: 700 18px Menlo, monospace; }}
    .traj-seg {{ stroke-linecap: round; }}
    .start {{ fill: #24b36b; stroke: #0c5c35; stroke-width: 3; }}
    .end {{ fill: #e24c3f; stroke: #7a1d15; stroke-width: 3; }}
    .title {{ font: 700 32px Georgia, serif; fill: #1b1b17; }}
    .sub {{ font: 15px Menlo, monospace; fill: #444139; }}
    .chip {{ fill: #fffdf7; stroke: #222; stroke-width: 1.5; }}
  </style>
  <rect x="0" y="0" width="{width}" height="{height}" class="bg"/>
  <text x="54" y="58" class="title">Quest Controller Trajectory — 3D View</text>
  <text x="54" y="89" class="sub">source={esc(source.name)} | rows={total_rows} | plotted={len(rows)}{esc(drop_note)} | receive_span={duration:.3f}s | path_length~{path_length(rows):.3f}m | az={azimuth:.1f} | elev={elevation:.1f} | generated={esc(generated)}</text>
  <text x="54" y="116" class="sub">green=start, red=end, color gradient=sample order, axes: X red / Y green / Z blue; receive_span is local arrival time</text>
  <rect x="42" y="142" width="1116" height="666" rx="26" class="chip"/>
  {"".join(floor_lines)}
  {"".join(box_svg)}
  {"".join(axis_svg)}
  {"".join(segments)}
  <circle cx="{sx:.1f}" cy="{sy:.1f}" r="8" class="start"/>
  <circle cx="{ex:.1f}" cy="{ey:.1f}" r="9" class="end"/>
</svg>
"""


def main() -> int:
    args = parse_args()
    csv_path = (args.csv_path or latest_remote_csv()).resolve()
    rows, total_rows, dropped = load_positions(csv_path, args.max_points, not args.keep_origin)
    out_path = (args.out or (PLOT_DIR / f"{csv_path.stem}_3d.svg")).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        render_svg(rows, total_rows, dropped, csv_path, args.azimuth, args.elevation),
        encoding="utf-8",
    )
    print(out_path)
    if args.png:
        sips = shutil.which("sips")
        if not sips:
            print("SVG was written, but macOS sips was not found; skipping PNG conversion.")
            return 0
        png_path = out_path.with_suffix(".png")
        subprocess.run(
            [sips, "-s", "format", "png", str(out_path), "--out", str(png_path)],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        print(png_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
