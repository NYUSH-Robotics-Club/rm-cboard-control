#!/usr/bin/env python3
"""Read-only local viewer for image frames and unverified detector candidates."""

import argparse
import csv
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


PAGE = Path(__file__).with_name("vision_replay_web.html")


def load_session(images_dir, predictions_path):
    images = sorted(p for p in images_dir.iterdir()
                    if p.is_file() and p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        raise ValueError(f"No images in {images_dir}")
    by_name = {p.name: i for i, p in enumerate(images)}
    frames = [{"name": p.name, "boxes": []} for p in images]
    with predictions_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"image", "confidence", "x1", "y1", "x2", "y2"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"Missing CSV columns: {', '.join(sorted(required - set(reader.fieldnames or [])))}")
        for line, row in enumerate(reader, 2):
            index = by_name.get(Path(row["image"]).name)
            if index is None or not row["confidence"]:
                continue
            try:
                values = [float(row[key]) for key in ("confidence", "x1", "y1", "x2", "y2")]
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid prediction at CSV line {line}") from exc
            confidence, x1, y1, x2, y2 = values
            if not all(math.isfinite(v) for v in values) or not (
                0 <= confidence <= 1 and 0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1
            ):
                raise ValueError(f"Out-of-range prediction at CSV line {line}")
            frames[index]["boxes"].append({
                "confidence": confidence,
                "rect": [x1, y1, x2, y2],
                "class": row.get("label_class") or "?",
            })
    return images, {"source": "offline replay", "verified": False,
                    "board_connected": False, "command_sent": False, "frames": frames}


def make_handler(images, session):
    session_bytes = json.dumps(session, ensure_ascii=False).encode("utf-8")
    page_bytes = PAGE.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                content, mime = page_bytes, "text/html; charset=utf-8"
            elif self.path == "/api/session":
                content, mime = session_bytes, "application/json; charset=utf-8"
            elif self.path.startswith("/frames/"):
                index_text = self.path.removeprefix("/frames/")
                if not index_text.isdecimal() or not 0 <= int(index_text) < len(images):
                    self.send_error(404)
                    return
                path = images[int(index_text)]
                content = path.read_bytes()
                mime = "image/png" if path.suffix.lower() == ".png" else "image/jpeg"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(content)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("build/vision-labeling/demo"))
    parser.add_argument("--predictions", type=Path,
                        default=Path("build/vision-labeling/nyu-predictions.csv"))
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    try:
        images, session = load_session(args.images, args.predictions)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(images, session))
    print(f"Offline vision replay: http://127.0.0.1:{args.port} ({len(images)} frames)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
