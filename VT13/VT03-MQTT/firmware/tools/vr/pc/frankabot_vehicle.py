"""FrankaBot translation input for the RoboMaster custom-controller path."""

import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_runtime"))


def crc(data, state, poly):
    for value in data:
        state ^= value
        for _ in range(8):
            state = (state >> 1) ^ (poly if state & 1 else 0)
    return state


def pack_payload(seq, enabled, forward, right=0, diagnostic=False, rotation=None):
    if rotation is not None:
        axes = (forward, right, rotation)
        if any(type(v) is not int or abs(v) > 100 for v in axes) or (not enabled and any(axes)):
            raise ValueError("invalid dual-controller command")
        if diagnostic and (enabled or any(axes)):
            raise ValueError("diagnostic frames must be neutral")
        return struct.pack("<4sHBBhhh16x", b"VRD3", seq & 65535, enabled,
                           diagnostic, forward, right, rotation)
    if max(abs(forward), abs(right)) > 100 or (not enabled and (forward or right)):
        raise ValueError("invalid translation command")
    if diagnostic and (enabled or forward or right):
        raise ValueError("diagnostic frames must be neutral")
    return struct.pack("<4sHBBhh18x", b"VRD2", seq & 65535, enabled,
                       diagnostic, forward, right)


def pack(seq, enabled, forward, right=0, diagnostic=False, command=0x0302, rotation=None):
    if command not in (0x0302, 0x0311):
        raise ValueError("unsupported transport command")
    payload = pack_payload(seq, enabled, forward, right, diagnostic, rotation)
    header = struct.pack("<BHB", 0xA5, len(payload), seq & 255)
    frame = header + bytes([crc(header, 0xFF, 0x8C)]) + struct.pack("<H", command) + payload
    return frame + struct.pack("<H", crc(frame, 0xFFFF, 0x8408))


class TranslationInput:
    def __init__(self, axis, sign, lateral_axis, lateral_sign):
        self.axes = (axis, lateral_axis)
        self.signs = (sign, lateral_sign)
        self.origin = None
        self.released = False

    def update(self, packet, gate_open, now_ns):
        fresh = (packet and packet.get("tracking_valid") and
                 0 <= now_ns - packet["host_received_monotonic_ns"] <= 120_000_000)
        if not fresh:
            self.origin = None
            self.released = False
            return None
        position = tuple(packet["position"][axis] * sign for axis, sign in zip(self.axes, self.signs))
        if not all(map(math.isfinite, position)):
            return self.update(None, False, now_ns)
        trigger = packet["source_metadata"]["flag"] is True
        if not gate_open or not packet["gate_open"]:
            self.origin = None
            self.released = False
            return False, 0, 0
        if not trigger:
            self.origin = position
            self.released = True
            return False, 0, 0
        if not self.released or self.origin is None:
            return False, 0, 0
        # A 3 cm dead zone and a bounded low-speed test command.
        dx, dy = (v - origin for v, origin in zip(position, self.origin))
        magnitude = math.hypot(dx, dy)
        if magnitude <= 0.03:
            return True, 0, 0
        scale = 100 * min(1.0, (magnitude - 0.03) / 0.12) / magnitude
        return True, round(dx * scale), round(dy * scale)


class HandheldCalibration:
    """Three trigger presses establish an origin and two horizontal directions."""

    prompts = ("Hold at center", "Move forward at least 12 cm",
               "Return to center, then move right at least 12 cm")

    def __init__(self):
        self.points = []
        self.axes = None
        self.released = False
        print("Calibration: enable with B (Low then High). Center the hand and tap the trigger.",
              flush=True)

    def convert(self, packet, gate_open, now_ns):
        if (not packet or not packet.get("tracking_valid") or
                not 0 <= now_ns - packet["host_received_monotonic_ns"] <= 120_000_000):
            self.released = False
            return None
        raw = packet["source_metadata"]["raw_position"]
        pos = (float(raw[0]), float(raw[2]))
        if not all(map(math.isfinite, pos)):
            return None
        if self.axes is not None:
            f, r = self.axes
            return {**packet, "position": [sum(a*b for a, b in zip(pos, r)),
                                            sum(a*b for a, b in zip(pos, f)), 0]}
        if not gate_open or not packet["gate_open"]:
            self.released = False
            return None
        if not packet["source_metadata"]["flag"]:
            self.released = True
            return None
        if not self.released:
            return None
        self.released = False
        if not self.points:
            self.points.append(pos)
        else:
            origin = self.points[0]
            delta = tuple(x-y for x, y in zip(pos, origin))
            length = math.hypot(*delta)
            if length < 0.12:
                print("Calibration movement too short; release and retry this point.", flush=True)
                return None
            if len(self.points) == 1:
                self.points.append(tuple(v/length for v in delta))
            else:
                f = self.points[1]
                projection = sum(a*b for a, b in zip(delta, f))
                orthogonal = tuple(v-projection*a for v, a in zip(delta, f))
                lateral = math.hypot(*orthogonal)
                if lateral < 0.10 or lateral < 0.65 * length:
                    print("Right point is too close to the forward axis; release and retry.",
                          flush=True)
                    return None
                self.axes = f, tuple(v/lateral for v in orthogonal)
        if self.axes is None:
            print("Calibration: " + self.prompts[len(self.points)] + "; then tap trigger.",
                  flush=True)
        else:
            print("Calibration complete. Return to center and release trigger before driving.",
                  flush=True)
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--dry-run", action="store_true")
    target.add_argument("--port", help="VR writer COM port (physical or virtual pair), never the VT12 COM port")
    target.add_argument("--test-mailbox", type=Path, help="Mailbox of the local test client only")
    # The existing calibrated Quest adapter emits [right, forward, up].
    parser.add_argument("--axis", default="y", choices=("x", "y", "z"))
    parser.add_argument("--sign", default=1, type=int, choices=(-1, 1))
    parser.add_argument("--lateral-axis", default="x", choices=("x", "y", "z"))
    parser.add_argument("--lateral-sign", default=1, type=int, choices=(-1, 1))
    parser.add_argument("--neutral-only", action="store_true", help="Transmit neutral for link validation")
    parser.add_argument("--calibrate-handheld", action="store_true")
    parser.add_argument("--calibration", type=Path, help="Saved profile from the original calibration page")
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--prepare-usb", action="store_true")
    parser.add_argument("--existing-usb", action="store_true",
                        help="Start listeners using already configured ADB reverse mappings")
    parser.add_argument("--serial", help="ADB serial when using a different Quest.")
    parser.add_argument("--status-file", type=Path, help="Local input/transport diagnostics JSON")
    args = parser.parse_args()
    if args.axis == args.lateral_axis or (args.seconds is not None and args.seconds <= 0):
        parser.error("distinct axes and a positive duration are required")
    if args.existing_usb and args.prepare_usb:
        parser.error("choose existing mappings or prepare USB")
    import vr_input
    from vr_input import QuestInput, adb_command, prepare_usb
    if args.serial:
        vr_input.SERIAL = args.serial
    if not args.existing_usb:
        state = adb_command("get-state")
        if state.returncode or state.stdout.strip() != "device":
            parser.exit(2, "Quest not detected by ADB; no frames sent.\n")
    if args.prepare_usb and not prepare_usb()["ready"]:
        parser.exit(2, "ADB reverse mapping failed; no frames sent.\n")
    port = None
    if args.port:
        from serial import Serial
        port = Serial(args.port, baudrate=115200, write_timeout=0.05)
    source = QuestInput(calibration_path=args.calibration)
    mapping = TranslationInput("xyz".index(args.axis), args.sign,
                               "xyz".index(args.lateral_axis), args.lateral_sign)
    calibration = HandheldCalibration() if args.calibrate_handheld else None
    seq = 0
    previous = None
    last_input_status = None
    had_input = False
    last_status = 0.0
    sent = 0
    deadline = time.monotonic() + args.seconds if args.seconds else math.inf

    def send(frame):
        nonlocal sent
        if port:
            if port.write(frame) != len(frame):
                raise IOError("incomplete VR frame serial write")
            sent += 1
        elif args.test_mailbox:
            if args.test_mailbox.exists():
                return
            temp = args.test_mailbox.with_suffix(".pending")
            temp.write_bytes(frame)
            temp.replace(args.test_mailbox)
            sent += 1

    source.start()
    try:
        while source.thread.is_alive() and time.monotonic() < deadline:
            t = time.monotonic()
            snap = source.snapshot()
            status = (snap["count"] > 0, snap["gate_open"])
            if status != last_input_status:
                print(f"Tracking messages seen={status[0]}, B enabled={status[1]}", flush=True)
                last_input_status = status
            now_ns = time.monotonic_ns()
            last_gate_ns = snap["diagnostics"].get("last_pause_received_monotonic_ns", 0)
            gate_open = snap["gate_open"] and 0 <= now_ns - last_gate_ns <= 200_000_000
            packet = snap["packet"]
            if calibration:
                packet = calibration.convert(packet, gate_open, now_ns)
            output = mapping.update(packet, gate_open, now_ns)
            if output is not None:
                had_input = True
            elif had_input:
                output = (False, 0, 0)
                had_input = False
            if output is not None:
                seq += 1
                send(pack(seq, False, 0, diagnostic=True) if args.neutral_only else pack(seq, *output))
            if output != previous:
                print(f"input={output} samples={snap['count']}", flush=True)
                previous = output
            if args.status_file and t - last_status >= .25:
                last_status = t
                stamp = (packet or {}).get("host_received_monotonic_ns", 0)
                status_data = dict(samples=snap["count"], invalid=snap["invalid"],
                    gate_requested=snap["gate_open"], gate_fresh=gate_open,
                    last_b=snap["diagnostics"].get("last_pause"),
                    b_age_ms=(now_ns-last_gate_ns)/1e6 if last_gate_ns else None,
                    pose_age_ms=(now_ns-stamp)/1e6 if stamp else None,
                    trigger=(packet or {}).get("source_metadata", {}).get("flag"),
                    tracking_valid=(packet or {}).get("tracking_valid", False),
                    tracking_reason=(packet or {}).get("source_metadata", {}).get("reason"),
                    position=(packet or {}).get("position"),
                    raw_position=(packet or {}).get("source_metadata", {}).get("raw_position"),
                    origin=mapping.origin,
                    calibration_path=str(source.profile),
                    calibration_sha256=source.calibration_sha256,
                    hand_released=mapping.released, output=output, sequence=seq,
                    frames_sent=sent, host_time=time.time(),
                    transport=args.port or ("test-mailbox" if args.test_mailbox else "dry-run"))
                try:
                    pending=args.status_file.with_suffix(".pending")
                    pending.write_text(json.dumps(status_data, allow_nan=False), encoding="utf-8")
                    pending.replace(args.status_file)
                except PermissionError:
                    # Windows readers may briefly deny atomic replacement. A
                    # missed diagnostic update must not interrupt control.
                    pass
                except OSError as exc:
                    print(f"Status file unavailable: {exc}", flush=True)
            # Remain below the custom-controller 30 Hz ceiling.
            time.sleep(max(0.0, 0.05 - (time.monotonic() - t)))
        tail = source.snapshot()
        print(f"{tail['reason']}; samples={tail['count']}; packets_prepared={seq}", flush=True)
        return 0 if tail["count"] > tail["invalid"] else 3
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            if seq:
                send(pack(seq + 1, False, 0, diagnostic=args.neutral_only))
        finally:
            if port:
                port.close()
            source.stop_event.set()
            source.thread.join(timeout=2)


if __name__ == "__main__":
    raise SystemExit(main())
