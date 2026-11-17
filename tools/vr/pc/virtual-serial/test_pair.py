"""Verify a virtual pair locally, or send diagnostic-only frames to stock client.

--send-only reports only local writes, never remote reception. C-board reception
must be observed independently through its read-only diagnostic counters.
"""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from frankabot_vehicle import pack


def main():
    from serial import Serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--writer", default="COM21")
    parser.add_argument("--reader", default="COM22")
    parser.add_argument("--send-only", action="store_true")
    parser.add_argument("--count", type=int, default=10)
    args = parser.parse_args()
    if not 1 <= args.count <= 100:
        parser.error("count must be 1..100")
    if args.writer.upper() == args.reader.upper():
        parser.error("writer and reader must differ")
    # VRD2 diagnostic flag: all axes/enable zero; C board counts but never publishes.
    frames = [pack(i, False, 0, 0, diagnostic=True) for i in range(1, args.count + 1)]
    reader = None
    try:
        if not args.send_only:
            reader = Serial(args.reader, 115200, timeout=1, write_timeout=.2)
            reader.reset_input_buffer()
        with Serial(args.writer, 115200, timeout=1, write_timeout=.2) as writer:
            writer.reset_output_buffer()
            received = bytearray()
            for frame in frames:
                if writer.write(frame) != len(frame):
                    raise IOError("short serial write")
                if reader:
                    received += reader.read(len(frame))
                time.sleep(.05)
        if reader and received != b"".join(frames):
            raise RuntimeError("virtual pair byte mismatch: %d/%d bytes" %
                               (len(received), 39 * args.count))
        print(json.dumps({"stage": "local_write_only" if args.send_only else "virtual_pair_exact_bytes",
                          "frames": args.count, "bytes": 39 * args.count,
                          "diagnostic_only": True, "remote_reception_verified": False}))
    finally:
        if reader:
            reader.close()


if __name__ == "__main__":
    main()
