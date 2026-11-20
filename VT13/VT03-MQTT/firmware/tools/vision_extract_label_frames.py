#!/usr/bin/env python3
"""Extract evenly spaced video frames for manual LabelRoboMaster annotation."""

import argparse
import csv
from pathlib import Path

import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interval-s", type=float, default=2.0)
    parser.add_argument("--max-frames", type=int, default=30)
    args = parser.parse_args()
    if args.interval_s <= 0 or args.max_frames <= 0:
        parser.error("interval and max-frames must be positive")
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        parser.error(f"cannot open video: {args.video}")
    fps = capture.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        parser.error("video has no usable frame rate")
    stride = max(1, round(fps * args.interval_s))
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = args.output / "manifest.csv"
    if manifest.exists():
        parser.error(f"output already has a manifest: {manifest}")
    saved = 0
    frame_number = 0
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("image", "source_video", "time_s", "label_status"))
        while saved < args.max_frames:
            ok, frame = capture.read()
            if not ok:
                break
            if frame_number % stride == 0:
                image = f"{args.video.stem}_f{frame_number:06d}.jpg"
                path = args.output / image
                if path.exists():
                    parser.error(f"refusing to overwrite {path}")
                if not cv2.imwrite(str(path), frame):
                    raise OSError(f"could not save {path}")
                writer.writerow((image, str(args.video), round(frame_number / fps, 3), "unlabelled"))
                saved += 1
            frame_number += 1
    capture.release()
    print(f"extracted {saved} unlabelled frames to {args.output}")


if __name__ == "__main__":
    main()
