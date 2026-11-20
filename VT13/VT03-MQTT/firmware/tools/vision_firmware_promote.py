#!/usr/bin/env python3
"""Promote a measured vision speed trial into infantry firmware source.

This only edits the vision-specific speed limit after paired board trials pass.
It never uses synthetic tuner reports and never flashes a board.
"""

import argparse
import csv
import hashlib
import json
import math
import re
import subprocess
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/robots/infantry_standard.c"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load_trials(path):
    required = {"variant", "scenario", "session", "yaw_mae_deg", "yaw_peak_deg",
                "saturation_fraction", "remote_loss_safe", "elf_sha256"}
    runs = defaultdict(dict)
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not required <= set(reader.fieldnames or []):
            raise ValueError("trial CSV missing required columns")
        for line, row in enumerate(reader, 2):
            variant = row["variant"]
            if variant not in {"baseline", "candidate"} or not row["scenario"] or not row["session"]:
                raise ValueError(f"line {line}: invalid variant, scenario or session")
            key = (row["scenario"], row["session"])
            if variant in runs[key]:
                raise ValueError(f"line {line}: duplicate paired trial")
            values = tuple(float(row[name]) for name in
                           ("yaw_mae_deg", "yaw_peak_deg", "saturation_fraction"))
            if (not all(math.isfinite(value) and value >= 0 for value in values) or
                    values[2] > 1 or row["remote_loss_safe"] != "1" or
                    not re.fullmatch(r"[0-9a-fA-F]{64}", row["elf_sha256"])):
                raise ValueError(f"line {line}: invalid metrics, safety result or ELF hash")
            runs[key][variant] = (values, row["elf_sha256"].lower())
    if len({scenario for scenario, _ in runs}) < 2:
        raise ValueError("need at least two measured scenarios")
    counts = defaultdict(int)
    hashes = {"baseline": set(), "candidate": set()}
    for (scenario, _), pair in runs.items():
        if set(pair) != {"baseline", "candidate"}:
            raise ValueError("each scenario/session needs both firmware variants")
        counts[scenario] += 1
        base, proposal = pair["baseline"][0], pair["candidate"][0]
        if not (proposal[0] <= base[0] * 0.95 and
                proposal[1] <= base[1] and
                proposal[2] <= base[2] + 0.02):
            raise ValueError(f"candidate failed holdout gate in {scenario}")
        for variant in hashes:
            hashes[variant].add(pair[variant][1])
    if any(count < 3 for count in counts.values()) or any(len(values) != 1 for values in hashes.values()):
        raise ValueError("need three paired sessions per scenario and one ELF per variant")
    if hashes["baseline"] == hashes["candidate"]:
        raise ValueError("candidate and baseline ELF hashes must differ")
    return {"scenarios": dict(counts), "elf_sha256": {key: next(iter(value))
                                                   for key, value in hashes.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("proposal", type=Path, help="JSON with source, config_sha256, candidate_rpm, bounds_rpm and trials_csv")
    parser.add_argument("--apply", action="store_true", help="write the validated value to upper-layer firmware config")
    args = parser.parse_args()
    proposal = json.loads(args.proposal.read_text(encoding="utf-8"))
    if proposal.get("source") != "measured_board_ab" or proposal.get("robot_type") != "infantry_standard":
        raise ValueError("only measured infantry board A/B evidence is accepted")
    original = CONFIG.read_bytes()
    if proposal.get("config_sha256") != digest(original):
        raise ValueError("firmware config hash changed since proposal creation")
    candidate = float(proposal["candidate_rpm"])
    lower, upper = (float(value) for value in proposal["bounds_rpm"])
    if not (math.isfinite(lower) and math.isfinite(upper) and
            0 < lower <= candidate <= upper):
        raise ValueError("candidate must be within the measured safe bounds")
    trials = Path(proposal["trials_csv"])
    if not trials.is_absolute():
        trials = args.proposal.parent / trials
    if proposal.get("trials_sha256") != digest(trials.read_bytes()):
        raise ValueError("trial data hash changed since proposal creation")
    evidence = load_trials(trials)
    source = original.decode("utf-8")
    pattern = r"(static const YawControlConfig s_yaw_control\s*=\s*\{[\s\S]*?\.vision_speed_rpm\s*=\s*)([0-9.]+)(f\s*,)"
    matches = list(re.finditer(pattern, source))
    if len(matches) != 1:
        raise ValueError("could not uniquely locate infantry vision_speed_rpm")
    old_value = float(matches[0][2])
    if not lower <= old_value <= upper:
        raise ValueError("existing value is outside supplied trial bounds")
    updated = source[:matches[0].start(2)] + f"{candidate:.6f}" + source[matches[0].end(2):]
    report = {"old_rpm": old_value, "candidate_rpm": candidate, "evidence": evidence,
              "applied": bool(args.apply)}
    if args.apply:
        CONFIG.write_text(updated, encoding="utf-8")
        try:
            subprocess.run(
                ["bash", "-lc", "source tools/activate.sh && CC=gcc sh tests/host/run_tests.sh && just build infantry_standard"],
                cwd=ROOT, check=True,
            )
        except BaseException:
            CONFIG.write_bytes(original)
            raise
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
