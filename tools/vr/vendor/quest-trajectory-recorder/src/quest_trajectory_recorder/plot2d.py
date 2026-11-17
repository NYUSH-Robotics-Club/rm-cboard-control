#!/usr/bin/env python3
"""Render a dependency-free SVG trajectory preview from a remote.csv capture."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import shutil
import subprocess
from pathlib import Path
from typing import Iterable

CAPTURE_DIR = Path("captures")
PLOT_DIR = Path("plots")


def latest_remote_csv() -> Path:
    candidates = [p for p in CAPTURE_DIR.glob("*_remote.csv") if p.stat().st_size > 200]
    if not candidates:
        raise SystemExit(f"No usable *_remote.csv found in {CAPTURE_DIR}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot Quest controller trajectory as SVG.")
    parser.add_argument(
        "csv_path", nargs="?", type=Path, help="Path to *_remote.csv. Defaults to latest."
    )
    parser.add_argument("--out", type=Path, help="Output SVG path.")
    parser.add_argument(
        "--png", action="store_true", help="Also convert the SVG to PNG with macOS sips."
    )
    parser.add_argument(
        "--max-points", type=int, default=3000, help="Downsample long captures for SVG size."
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


def path_length(points: list[dict[str, float | str]]) -> float:
    total = 0.0
    for a, b in zip(points, points[1:]):
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        dz = float(b["z"]) - float(a["z"])
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def bounds(values: Iterable[float]) -> tuple[float, float]:
    vals = list(values)
    lo, hi = min(vals), max(vals)
    if math.isclose(lo, hi):
        lo -= 0.5
        hi += 0.5
    pad = (hi - lo) * 0.08
    return lo - pad, hi + pad


def esc(text: object) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def map_points(
    rows: list[dict[str, float | str]],
    x_key: str,
    y_key: str,
    panel: tuple[int, int, int, int],
) -> tuple[str, tuple[float, float], tuple[float, float]]:
    px, py, pw, ph = panel
    xs = [float(r[x_key]) for r in rows]
    ys = [float(r[y_key]) for r in rows]
    x0, x1 = bounds(xs)
    y0, y1 = bounds(ys)

    def project(x: float, y: float) -> tuple[float, float]:
        sx = px + (x - x0) / (x1 - x0) * pw
        sy = py + ph - (y - y0) / (y1 - y0) * ph
        return sx, sy

    pts = [project(float(r[x_key]), float(r[y_key])) for r in rows]
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in pts), pts[0], pts[-1]


def panel_svg(
    rows: list[dict[str, float | str]],
    title: str,
    x_key: str,
    y_key: str,
    panel: tuple[int, int, int, int],
    x_label: str,
    y_label: str,
) -> str:
    px, py, pw, ph = panel
    points, start, end = map_points(rows, x_key, y_key, panel)
    grid = []
    for i in range(6):
        x = px + pw * i / 5
        y = py + ph * i / 5
        grid.append(f'<line x1="{x:.1f}" y1="{py}" x2="{x:.1f}" y2="{py + ph}" class="grid"/>')
        grid.append(f'<line x1="{px}" y1="{y:.1f}" x2="{px + pw}" y2="{y:.1f}" class="grid"/>')
    return f"""
    <g>
      <rect x="{px}" y="{py}" width="{pw}" height="{ph}" rx="18" class="panel"/>
      {"".join(grid)}
      <polyline points="{points}" class="trajectory"/>
      <circle cx="{start[0]:.1f}" cy="{start[1]:.1f}" r="6" class="start"/>
      <circle cx="{end[0]:.1f}" cy="{end[1]:.1f}" r="7" class="end"/>
      <text x="{px + 18}" y="{py + 30}" class="panel-title">{esc(title)}</text>
      <text x="{px + pw - 62}" y="{py + ph - 12}" class="axis">{esc(x_label)}</text>
      <text x="{px + 12}" y="{py + 54}" class="axis">{esc(y_label)}</text>
    </g>
    """


def render_svg(
    rows: list[dict[str, float | str]], total_rows: int, dropped_leading_origin: int, source: Path
) -> str:
    if not rows:
        raise SystemExit("No valid trajectory rows found.")

    first, last = rows[0], rows[-1]
    duration = float(last["recv_unix"]) - float(first["recv_unix"])
    length = path_length(rows)
    generated = dt.datetime.now().astimezone().isoformat(timespec="seconds")
    title = source.name

    width, height = 1200, 860
    drop_note = f" | dropped_origin={dropped_leading_origin}" if dropped_leading_origin else ""
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <style>
    .bg {{ fill: #f8f4eb; }}
    .panel {{ fill: #fffdf7; stroke: #222; stroke-width: 1.5; }}
    .grid {{ stroke: #ded7c7; stroke-width: 1; }}
    .trajectory {{ fill: none; stroke: #0f6b7d; stroke-width: 3.5; stroke-linecap: round; stroke-linejoin: round; }}
    .start {{ fill: #24b36b; stroke: #0c5c35; stroke-width: 2; }}
    .end {{ fill: #e24c3f; stroke: #7a1d15; stroke-width: 2; }}
    .title {{ font: 700 30px Georgia, serif; fill: #1b1b17; }}
    .sub {{ font: 15px Menlo, monospace; fill: #444139; }}
    .panel-title {{ font: 700 18px Georgia, serif; fill: #1b1b17; }}
    .axis {{ font: 13px Menlo, monospace; fill: #6a6255; }}
    .legend {{ font: 14px Menlo, monospace; fill: #332f28; }}
  </style>
  <rect x="0" y="0" width="{width}" height="{height}" class="bg"/>
  <text x="50" y="58" class="title">Quest Controller Trajectory</text>
  <text x="50" y="88" class="sub">source={esc(title)} | rows={total_rows} | plotted={len(rows)}{esc(drop_note)} | receive_span={duration:.2f}s | path_length~{length:.3f}m | generated={esc(generated)}</text>
  <text x="50" y="114" class="legend">green=start, red=end, line=tracked position; receive_span is local arrival time, not guaranteed device time</text>
  {panel_svg(rows, "Top View (X-Z)", "x", "z", (50, 150, 520, 300), "X", "Z")}
  {panel_svg(rows, "Side View (X-Y)", "x", "y", (630, 150, 520, 300), "X", "Y")}
  {panel_svg(rows, "Front View (Z-Y)", "z", "y", (50, 500, 520, 300), "Z", "Y")}
  {panel_svg(rows, "Height vs Time", "recv_unix", "y", (630, 500, 520, 300), "time", "Y")}
</svg>
"""


def main() -> int:
    args = parse_args()
    csv_path = args.csv_path or latest_remote_csv()
    csv_path = csv_path.resolve()
    rows, total_rows, dropped = load_positions(csv_path, args.max_points, not args.keep_origin)
    out_path = args.out or (PLOT_DIR / f"{csv_path.stem}.svg")
    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(render_svg(rows, total_rows, dropped, csv_path), encoding="utf-8")
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
