"""Reject ambiguous direction captures before enabling the chassis input."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vr_config import PROFILE


def vector(value):
    result = tuple(float(value[k]) for k in "xyz")
    if not all(map(math.isfinite, result)):
        raise ValueError("Calibration contains non-finite coordinates")
    return result


def quality(data):
    if data.get("version") != 5 or data.get("state") != "ready":
        raise ValueError("Complete and save the original calibration first")
    vector(data["origin"])
    right, forward, up = (vector(data[k]) for k in ("right", "forward", "up"))
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    for axis in (right, forward, up):
        if abs(dot(axis, axis) - 1) > .01:
            raise ValueError("Calibration axes must be unit vectors")
    if max(abs(dot(a, b)) for a, b in ((right, forward), (right, up), (forward, up))) > .01:
        raise ValueError("Calibration axes must be perpendicular")
    captured = []
    for name in ("right", "forward"):
        start, end = vector(data[name + "Start"]), vector(data[name + "End"])
        delta = tuple(b-a for a, b in zip(start, end))
        vertical = dot(delta, up)
        horizontal = tuple(v-vertical*u for v, u in zip(delta, up))
        distance = math.sqrt(dot(horizontal, horizontal))
        if distance < .10:
            raise ValueError(f"{name} capture is too short; move at least 10 cm")
        captured.append((horizontal, distance))
    cosine = dot(captured[0][0], captured[1][0]) / (captured[0][1] * captured[1][1])
    angle = math.degrees(math.acos(max(-1, min(1, cosine))))
    if abs(cosine) > .85:
        raise ValueError(f"Right and forward captures are nearly collinear ({angle:.1f} degrees); recalibrate perpendicular directions")
    return dict(capture_angle_deg=angle, right_distance_m=captured[0][1], forward_distance_m=captured[1][1])


def check(path=PROFILE):
    path = Path(path)
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                **quality(json.loads(path.read_text(encoding="utf-8-sig"))))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", type=Path, default=PROFILE)
    args = parser.parse_args()
    print(json.dumps(check(args.path), ensure_ascii=False, indent=2))
