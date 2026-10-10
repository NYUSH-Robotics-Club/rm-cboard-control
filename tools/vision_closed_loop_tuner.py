#!/usr/bin/env python3
"""Deterministic, hardware-free auto-aim control sandbox.

The plant and vision measurements are synthetic. Parameters here are virtual
controller values, not firmware configuration or evidence of real motor safety.
"""

import argparse
import itertools
import json
import math
from dataclasses import asdict, dataclass


DT_S = 0.005
FRAME_PERIOD_S = 0.010
FRAME_TIMEOUT_S = 0.020
LATENCY_S = 0.030
MOTOR_TAU_S = 0.040
MOTOR_ACCEL_LIMIT_DEG_S2 = 900.0


@dataclass(frozen=True)
class Candidate:
    gain_per_s: float
    damping: float
    speed_limit_deg_s: float


@dataclass(frozen=True)
class Scenario:
    name: str
    kind: str
    dropout_start_s: float = -1.0
    dropout_end_s: float = -1.0


TRAIN = (
    Scenario("step_15deg", "step"),
    Scenario("sine_0p5hz", "sine"),
    Scenario("sine_dropout", "sine", 1.6, 1.72),
)
HOLDOUT = (
    Scenario("reverse_step", "reverse"),
    Scenario("sine_0p8hz", "fast_sine"),
    Scenario("reverse_dropout", "reverse", 2.0, 2.12),
)


def target_deg(kind, t_s):
    if kind == "step":
        return 15.0 if t_s >= 0.5 else 0.0
    if kind == "reverse":
        return 15.0 if 0.5 <= t_s < 2.0 else (-10.0 if t_s >= 2.0 else 0.0)
    if kind == "sine":
        return 12.0 * math.sin(2.0 * math.pi * 0.5 * t_s)
    return 9.0 * math.sin(2.0 * math.pi * 0.8 * t_s)


def simulate(candidate, scenario):
    position_deg = velocity_deg_s = requested_velocity_deg_s = 0.0
    last_frame_s = -1.0
    next_frame_s = 0.0
    samples = []
    previous_error = 0.0
    error_sum = reversal_sum = saturation_steps = active_steps = 0.0
    for step in range(int(4.0 / DT_S)):
        t_s = step * DT_S
        if t_s + 1e-9 >= next_frame_s:
            next_frame_s += FRAME_PERIOD_S
            if not scenario.dropout_start_s <= t_s < scenario.dropout_end_s:
                # A delayed image reports the target offset from the current axis.
                measured_target = target_deg(scenario.kind, max(0.0, t_s - LATENCY_S))
                error_deg = measured_target - position_deg
                requested_velocity_deg_s = max(
                    -candidate.speed_limit_deg_s,
                    min(candidate.speed_limit_deg_s,
                        candidate.gain_per_s * error_deg - candidate.damping * velocity_deg_s),
                )
                if abs(requested_velocity_deg_s) >= candidate.speed_limit_deg_s - 1e-9:
                    saturation_steps += 1.0
                last_frame_s = t_s
        if last_frame_s < 0 or t_s - last_frame_s > FRAME_TIMEOUT_S + 1e-9:
            requested_velocity_deg_s = 0.0
        else:
            active_steps += 1.0

        acceleration = max(
            -MOTOR_ACCEL_LIMIT_DEG_S2,
            min(MOTOR_ACCEL_LIMIT_DEG_S2,
                (requested_velocity_deg_s - velocity_deg_s) / MOTOR_TAU_S),
        )
        velocity_deg_s += acceleration * DT_S
        position_deg += velocity_deg_s * DT_S
        actual_error = target_deg(scenario.kind, t_s) - position_deg
        error_sum += abs(actual_error) * DT_S
        if actual_error * previous_error < 0 and abs(previous_error) > 0.1:
            reversal_sum += 1.0
        previous_error = actual_error
        samples.append((round(t_s, 3), round(position_deg, 4),
                        round(target_deg(scenario.kind, t_s), 4)))

    return {
        "mean_absolute_error_deg": round(error_sum / 4.0, 4),
        "error_sign_reversals": int(reversal_sum),
        "saturated_frames": int(saturation_steps),
        "active_fraction": round(active_steps / len(samples), 4),
        "samples": samples,
    }


def score(metrics):
    return sum(
        value["mean_absolute_error_deg"] +
        0.05 * value["error_sign_reversals"] +
        0.002 * value["saturated_frames"]
        for value in metrics.values()
    ) / len(metrics)


def evaluate(candidate, scenarios):
    return {scenario.name: simulate(candidate, scenario) for scenario in scenarios}


def summary(metrics):
    return {name: {key: value for key, value in result.items() if key != "samples"}
            for name, result in metrics.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", help="write a JSON report to this path")
    parser.add_argument("--trace", help="write the selected candidate's holdout trace as CSV")
    args = parser.parse_args()

    baseline = Candidate(4.0, 0.2, 60.0)
    candidates = [Candidate(*values) for values in itertools.product(
        (4.0, 6.0, 8.0, 10.0), (0.2, 0.5, 0.8), (60.0, 120.0, 180.0))]
    training = [(score(evaluate(item, TRAIN)), item) for item in candidates]
    training.sort(key=lambda entry: (entry[0], asdict(entry[1])["gain_per_s"],
                                     entry[1].damping, entry[1].speed_limit_deg_s))
    proposed = training[0][1]
    baseline_holdout = evaluate(baseline, HOLDOUT)
    proposed_holdout = evaluate(proposed, HOLDOUT)
    baseline_score = score(baseline_holdout)
    proposed_score = score(proposed_holdout)
    # Every unseen scenario must improve before the virtual candidate is accepted.
    accepted = proposed_score < baseline_score and all(
        score({name: proposed_holdout[name]}) < score({name: baseline_holdout[name]})
        for name in baseline_holdout
    )
    selected = proposed if accepted else baseline
    report = {
        "model": "synthetic first-order gimbal; no measured plant fit",
        "units": "degrees, seconds; candidate values do not map to firmware fields",
        "baseline": asdict(baseline),
        "proposed": asdict(proposed),
        "selected": asdict(selected),
        "accepted_on_holdout": accepted,
        "train_score_baseline": round(score(evaluate(baseline, TRAIN)), 4),
        "train_score_proposed": round(training[0][0], 4),
        "holdout_score_baseline": round(baseline_score, 4),
        "holdout_score_proposed": round(proposed_score, 4),
        "holdout_baseline": summary(baseline_holdout),
        "holdout_proposed": summary(proposed_holdout),
    }
    output = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(output)
    else:
        print(output, end="")
    if args.trace:
        with open(args.trace, "w", encoding="utf-8") as stream:
            stream.write("scenario,time_s,target_deg,position_deg\n")
            for name, result in evaluate(selected, HOLDOUT).items():
                for t_s, position_deg, target in result["samples"]:
                    stream.write(f"{name},{t_s},{target},{position_deg}\n")


if __name__ == "__main__":
    main()
