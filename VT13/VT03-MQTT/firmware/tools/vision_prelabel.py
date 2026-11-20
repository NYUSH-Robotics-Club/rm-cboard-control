#!/usr/bin/env python3
"""Generate unverified LabelRoboMaster suggestions with its ONNX model.

Suggestions use .suggested.txt and must be reviewed before being renamed to
the image's matching .txt ground-truth label file.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


def sigmoid(value):
    value = float(value)
    if value >= 0:
        return 1.0 / (1.0 + math.exp(-value))
    positive = math.exp(value)
    return positive / (1.0 + positive)


def overlap(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    return min(ax2, bx2) > max(ax1, bx1) and min(ay2, by2) > max(ay1, by1)


def infer(net, image, confidence_floor):
    height, width = image.shape[:2]
    scale = 640.0 / max(height, width)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    padded = np.full((640, 640, 3), 127, dtype=np.uint8)
    padded[:resized.shape[0], :resized.shape[1]] = resized
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    net.setInput(cv2.dnn.blobFromImage(rgb) / 255.0)
    output = net.forward().reshape(-1, 22)
    candidates = []
    for row in output:
        confidence = sigmoid(row[8])
        if confidence < confidence_floor:
            continue
        points = [(float(row[2 * i]) / scale, float(row[2 * i + 1]) / scale)
                  for i in range(4)]
        coords = [coordinate for point in points for coordinate in point]
        if not all(math.isfinite(value) for value in coords):
            continue
        box = (min(x for x, _ in points), min(y for _, y in points),
               max(x for x, _ in points), max(y for _, y in points))
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        class_id = int(np.argmax(row[9:13])) * 9 + int(np.argmax(row[13:22]))
        candidates.append((confidence, class_id, points, box))
    selected = []
    for candidate in sorted(candidates, key=lambda value: value[0], reverse=True):
        if not any(overlap(candidate[3], accepted[3]) for accepted in selected):
            selected.append(candidate)
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confidence", type=float, default=0.5)
    args = parser.parse_args()
    if not 0 < args.confidence < 1:
        parser.error("confidence must be between 0 and 1")
    images = sorted(path for path in args.images.iterdir()
                    if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        parser.error("image directory is empty")
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "predictions.csv").exists():
        parser.error("output already contains predictions.csv")
    net = cv2.dnn.readNetFromONNX(str(args.model))
    total = 0
    with (args.output / "predictions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("split", "image", "confidence", "x1", "y1", "x2", "y2", "label_class"))
        for path in images:
            image = cv2.imread(str(path))
            if image is None:
                raise ValueError(f"cannot read {path}")
            height, width = image.shape[:2]
            detections = infer(net, image, args.confidence)
            suggestion = args.output / (path.stem + ".suggested.txt")
            with suggestion.open("w", encoding="utf-8") as labels:
                for confidence, class_id, points, box in detections:
                    coords = [value for px, py in points
                              for value in (max(0.0, min(1.0, px / width)),
                                            max(0.0, min(1.0, py / height)))]
                    labels.write(str(class_id) + " " + " ".join(f"{value:.6f}" for value in coords) + "\n")
                    writer.writerow(("", path.name, f"{confidence:.6f}",
                                     f"{max(0.0, box[0] / width):.6f}",
                                     f"{max(0.0, box[1] / height):.6f}",
                                     f"{min(1.0, box[2] / width):.6f}",
                                     f"{min(1.0, box[3] / height):.6f}", class_id))
                    total += 1
            if not detections:
                writer.writerow(("", path.name, "", "", "", "", "", ""))
    metadata = {"model": str(args.model), "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
                "images": len(images), "suggestions": total, "reviewed": False}
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"generated {total} unverified suggestions across {len(images)} images")


if __name__ == "__main__":
    main()
