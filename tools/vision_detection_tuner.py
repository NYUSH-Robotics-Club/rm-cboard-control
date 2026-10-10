#!/usr/bin/env python3
"""Tune a vision confidence gate from independently labelled detections.

CSV columns: split,confidence,truth. Each row is one proposed detection with
truth 1 for a correctly matched armor, 0 for a false detection. Add one row
with an empty confidence and truth 1 for each missed ground-truth armor.
Splits must be train and holdout, preferably separated by recording session.
"""

import argparse
import csv
import json
import math
import re
from pathlib import Path


def read_rows(path):
    rows = {"train": [], "holdout": []}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not {"split", "confidence", "truth"} <= set(reader.fieldnames or []):
            raise ValueError("CSV requires split,confidence,truth columns")
        for line, row in enumerate(reader, 2):
            split = row["split"]
            if split not in rows or row["truth"] not in {"0", "1"}:
                raise ValueError(f"line {line}: invalid split or truth")
            truth = row["truth"] == "1"
            raw = row["confidence"].strip()
            confidence = None if raw == "" else float(raw)
            if confidence is not None and not (math.isfinite(confidence) and 0 <= confidence <= 1):
                raise ValueError(f"line {line}: confidence must be finite in [0,1]")
            if confidence is None and not truth:
                raise ValueError(f"line {line}: only a missed true armor may omit confidence")
            rows[split].append((confidence, truth))
    for split, values in rows.items():
        if len(values) < 10 or not any(truth for _, truth in values) or not any(
            confidence is not None and not truth for confidence, truth in values
        ):
            raise ValueError(f"{split}: need at least 10 rows, true armors and false detections")
    return rows


def measure(rows, threshold):
    tp = sum(confidence is not None and confidence > threshold and truth
             for confidence, truth in rows)
    fp = sum(confidence is not None and confidence > threshold and not truth
             for confidence, truth in rows)
    positives = sum(truth for _, truth in rows)
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / positives
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return {"tp": tp, "fp": fp, "fn": positives - tp,
            "precision": round(precision, 6), "recall": round(recall, 6),
            "f1": round(f1, 6)}


def baseline_from_yaml(path):
    matches = re.findall(r"^min_confidence:[ \t]*([0-9]*\.?[0-9]+)[ \t]*(?:#.*)?$",
                         path.read_text(encoding="utf-8"), re.MULTILINE)
    if len(matches) != 1:
        raise ValueError("config must contain exactly one min_confidence entry")
    value = float(matches[0])
    if not 0 <= value <= 1:
        raise ValueError("baseline min_confidence must be in [0,1]")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("predictions", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--candidate-config", type=Path,
                        help="write a separate YAML copy only if holdout improves")
    args = parser.parse_args()
    rows = read_rows(args.predictions)
    baseline = baseline_from_yaml(args.config)
    # The input detections must have been saved before the current runtime gate.
    # Candidates below the model's own score gate cannot recover discarded boxes.
    candidates = sorted({baseline} | {confidence for confidence, _ in rows["train"]
                                      if confidence is not None and confidence >= baseline})
    proposal = max(candidates, key=lambda threshold:
                   (measure(rows["train"], threshold)["f1"], -threshold))
    base_holdout = measure(rows["holdout"], baseline)
    new_holdout = measure(rows["holdout"], proposal)
    accepted = (proposal != baseline and
                new_holdout["f1"] > base_holdout["f1"] and
                new_holdout["recall"] >= base_holdout["recall"] - 0.02)
    report = {
        "source": str(args.predictions),
        "baseline_min_confidence": baseline,
        "proposed_min_confidence": proposal,
        "accepted_on_holdout": accepted,
        "train_baseline": measure(rows["train"], baseline),
        "train_proposed": measure(rows["train"], proposal),
        "holdout_baseline": base_holdout,
        "holdout_proposed": new_holdout,
        "limitation": "Optimizes confidence filtering only; model weights and upstream score gate stay unchanged.",
    }
    output = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.report:
        args.report.write_text(output, encoding="utf-8")
    else:
        print(output, end="")
    if args.candidate_config and accepted:
        source = args.config.read_text(encoding="utf-8")
        updated = re.sub(r"^(min_confidence:[ \t]*)([0-9]*\.?[0-9]+)([ \t]*(?:#.*)?)$",
                         lambda match: f"{match[1]}{proposal:.6f}{match[3]}",
                         source, count=1, flags=re.MULTILINE)
        args.candidate_config.write_text(updated, encoding="utf-8")


if __name__ == "__main__":
    main()
