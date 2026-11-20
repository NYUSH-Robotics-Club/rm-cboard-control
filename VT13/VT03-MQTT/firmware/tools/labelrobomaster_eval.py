#!/usr/bin/env python3
"""Match normalized detection boxes to LabelRoboMaster four-corner labels.

Input CSV columns: split,image,confidence,x1,y1,x2,y2,label_class.
Alternatively replace label_class with color,name,armor_type from nyu-vision.
Coordinates are normalized to [0,1]. Include one row with only split and image
for a frame without detections. Label files have the same stem as image and
live under --labels; each line is class plus four normalized (x,y) corners.
"""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


COLOR_OFFSET = {"blue": 0, "red": 9, "extinguish": 18, "purple": 27}
NAME_OFFSET = {"sentry": 0, "one": 1, "two": 2, "three": 3, "four": 4,
               "five": 5, "outpost": 6, "base": 7}


def prediction_class(row):
    if row.get("label_class", "").strip():
        return int(row["label_class"])
    color, name, armor_type = row["color"], row["name"], row["armor_type"]
    if color not in COLOR_OFFSET or name not in NAME_OFFSET or armor_type not in {"small", "big"}:
        raise ValueError("invalid nyu-vision color, name or armor_type")
    return COLOR_OFFSET[color] + (8 if name == "base" and armor_type == "big"
                                  else NAME_OFFSET[name])


def box_from_label(path, missing_empty):
    result = []
    if not path.exists():
        if missing_empty:
            return result
        raise FileNotFoundError(f"missing label file: {path}")
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        cells = line.split()
        if len(cells) != 9:
            raise ValueError(f"{path}:{number}: expected class and four corners")
        class_id = int(cells[0])
        coords = [float(value) for value in cells[1:]]
        if not 0 <= class_id <= 35 or not all(math.isfinite(v) and 0 <= v <= 1 for v in coords):
            raise ValueError(f"{path}:{number}: invalid class or normalized corner")
        xs, ys = coords[0::2], coords[1::2]
        result.append((class_id, (min(xs), min(ys), max(xs), max(ys))))
    return result


def iou(a, b):
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = width * height
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def read_predictions(path):
    frames = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"split", "image", "confidence", "x1", "y1", "x2", "y2"}
        if not required <= set(reader.fieldnames or []):
            raise ValueError(f"CSV requires {','.join(sorted(required))}")
        if "label_class" not in reader.fieldnames and not {"color", "name", "armor_type"} <= set(reader.fieldnames):
            raise ValueError("CSV needs label_class or color,name,armor_type")
        for line, row in enumerate(reader, 2):
            split, image = row["split"], row["image"]
            if split not in {"train", "holdout"} or not image or Path(image).name != image:
                raise ValueError(f"line {line}: invalid split or image filename")
            key = (split, image)
            frames[key]  # Retain frames without detections.
            if not row["confidence"].strip():
                continue
            confidence = float(row["confidence"])
            coords = tuple(float(row[key]) for key in ("x1", "y1", "x2", "y2"))
            class_id = prediction_class(row)
            if not (math.isfinite(confidence) and 0 <= confidence <= 1 and
                    0 <= class_id <= 35 and all(math.isfinite(v) and 0 <= v <= 1 for v in coords) and
                    coords[0] < coords[2] and coords[1] < coords[3]):
                raise ValueError(f"line {line}: invalid normalized detection")
            frames[key].append((confidence, class_id, coords))
    if not frames:
        raise ValueError("no frames")
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("predictions", type=Path)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--missing-empty", action="store_true",
                        help="treat missing txt files as reviewed negative images")
    args = parser.parse_args()
    if not 0 < args.iou <= 1:
        parser.error("--iou must be in (0,1]")
    frames = read_predictions(args.predictions)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("split", "confidence", "truth"))
        for (split, image), predictions in sorted(frames.items()):
            labels = box_from_label(args.labels / (Path(image).stem + ".txt"), args.missing_empty)
            used = set()
            for confidence, class_id, box in sorted(predictions, reverse=True):
                choices = [(iou(box, truth_box), index) for index, (truth_class, truth_box)
                           in enumerate(labels) if index not in used and truth_class == class_id]
                match = max(choices, default=(0.0, -1))
                true_positive = match[0] >= args.iou
                if true_positive:
                    used.add(match[1])
                writer.writerow((split, confidence, int(true_positive)))
            for index in range(len(labels)):
                if index not in used:
                    writer.writerow((split, "", 1))


if __name__ == "__main__":
    main()
