#!/usr/bin/env python3
"""Export nyu-vision YOLOv5 ONNX detections before min_confidence filtering.

This host-side replay mirrors the current YOLOV5 preprocessing, score gate,
NMS, ROI fallback and class mapping. It does not include traditional corner
refinement or camera timing; compare against the target Jetson build before
using the numbers as release evidence.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def sigmoid(value):
    value = float(value)
    if value >= 0:
        return 1.0 / (1.0 + math.exp(-value))
    positive = math.exp(value)
    return positive / (1.0 + positive)


def detect(session, image, score_floor, offset_x=0, offset_y=0):
    height, width = image.shape[:2]
    scale = min(640.0 / height, 640.0 / width)
    resized = cv2.resize(image, (int(width * scale), int(height * scale)))
    canvas = np.zeros((640, 640, 3), dtype=np.uint8)
    canvas[:resized.shape[0], :resized.shape[1]] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    input_tensor = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))[None]
    output = session.run(None, {session.get_inputs()[0].name: input_tensor})[0].reshape(-1, 22)
    boxes = []
    scores = []
    classes = []
    for row in output:
        score = float(row[8])
        if not 0.0 <= score <= 1.0:
            score = sigmoid(score)
        if score < score_floor:
            continue
        color = int(np.argmax(row[9:13]))
        number = int(np.argmax(row[13:22]))
        if number == 8:  # nyu-vision maps this model class to not_armor.
            continue
        points = [(float(row[2 * i]) / scale + offset_x,
                   float(row[2 * i + 1]) / scale + offset_y) for i in range(4)]
        x1, y1 = min(p[0] for p in points), min(p[1] for p in points)
        x2, y2 = max(p[0] for p in points), max(p[1] for p in points)
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
            continue
        boxes.append([int(x1), int(y1), max(1, int(x2 - x1)), max(1, int(y2 - y1))])
        scores.append(score)
        # The runtime treats model color index 3 as extinguished.
        classes.append((min(color, 2) * 9) + number)
    indices = cv2.dnn.NMSBoxes(boxes, scores, score_floor, 0.3)
    if len(indices) == 0:
        return []
    return [(scores[int(i)], classes[int(i)], boxes[int(i)])
            for i in np.asarray(indices).reshape(-1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("nyu-vision/configs/odin.yaml"))
    parser.add_argument("--model", type=Path, required=True,
                        help="ONNX export corresponding to the configured detector")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "holdout", "unassigned"), default="unassigned")
    args = parser.parse_args()
    try:
        import onnxruntime as ort
    except ImportError as error:
        parser.error(f"onnxruntime is required: {error}")
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if config.get("yolo_name") != "yolov5":
        parser.error("this exporter supports yolo_name: yolov5 only")
    model = args.model.resolve()
    if model.suffix.lower() != ".onnx":
        parser.error("--model must be an ONNX file")
    score_floor = float(config.get("yolov5_score_threshold", 0.55 if config.get("backend") == "tensorrt" else 0.7))
    session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    images = sorted(path for path in args.images.iterdir()
                    if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images or args.output.exists():
        parser.error("images must be nonempty and output must not already exist")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    total = 0
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("split", "image", "confidence", "x1", "y1", "x2", "y2", "label_class"))
        for path in images:
            image = cv2.imread(str(path))
            if image is None:
                raise ValueError(f"cannot read {path}")
            height, width = image.shape[:2]
            detections = []
            if config.get("use_roi"):
                roi = config["roi"]
                x, y = int(roi["x"]), int(roi["y"])
                w, h = int(roi["width"]), int(roi["height"])
                if x >= 0 and y >= 0 and x + w <= width and y + h <= height:
                    detections = detect(session, image[y:y+h, x:x+w], score_floor, x, y)
            if not detections:
                detections = detect(session, image, score_floor)
            split = "" if args.split == "unassigned" else args.split
            for confidence, class_id, box in detections:
                x, y, w, h = box
                writer.writerow((split, path.name, f"{confidence:.6f}",
                                 f"{max(0, x / width):.6f}", f"{max(0, y / height):.6f}",
                                 f"{min(1, (x + w) / width):.6f}",
                                 f"{min(1, (y + h) / height):.6f}", class_id))
                total += 1
            if not detections:
                writer.writerow((split, path.name, "", "", "", "", "", ""))
    metadata = {"model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
                "config": str(args.config), "score_floor": score_floor,
                "images": len(images), "detections": total,
                "limitations": "Host ONNX replay; compare with Jetson backend and traditional refinement."}
    args.output.with_suffix(".metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"exported {total} nyu-vision candidates from {len(images)} images")


if __name__ == "__main__":
    main()
