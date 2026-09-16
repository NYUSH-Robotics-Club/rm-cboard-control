#!/usr/bin/env python3
import argparse
import collections
import os
import struct
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from scripts.rtt_common.rtt_transport import RTTTransport, RTTTransportConfig
from scripts.rtt_common.telemetry import PAYLOAD_SIZES, SUPPORTED_VERSIONS


DASHBOARD_MAGIC = 0x4452
DASHBOARD_MAGIC_ALT = 0x5244


def crc16_firmware(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Probe raw RTT dashboard frames on channel 1.")
    p.add_argument("--backend", choices=["auto", "pyocd", "jlink"], default="auto")
    p.add_argument("--device", default="STM32F407IG")
    p.add_argument("--iface", default="SWD")
    p.add_argument("--speed", type=int, default=4000)
    p.add_argument("--serial", default=None)
    p.add_argument("--connect", choices=["normal", "halt", "under-reset"], default="normal")
    p.add_argument("--elf", default=os.path.join("build", "basic_framework.elf"))
    p.add_argument("--channel", type=int, default=1)
    p.add_argument("--seconds", type=float, default=3.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    transport = RTTTransport(
        RTTTransportConfig(
            device=args.device,
            serial=args.serial,
            speed_khz=args.speed,
            connect_mode=args.connect,
            backend=args.backend,
            iface=args.iface,
            elf=args.elf,
        )
    )

    buf = bytearray()
    hits = 0
    crc_ok_by_version = collections.Counter()
    try:
        transport.open()
        print(f"RTT connected backend={transport.backend_in_use} channels={transport.channel_info or '<unknown>'}")
        t_end = time.time() + max(args.seconds, 0.5)
        while time.time() < t_end:
            chunk = transport.read(args.channel, 4096)
            if chunk:
                buf.extend(chunk)
            if len(buf) < 8:
                time.sleep(0.01)
                continue

            i = 0
            while i + 8 <= len(buf):
                magic = struct.unpack_from("<H", buf, i)[0]
                if magic not in (DASHBOARD_MAGIC, DASHBOARD_MAGIC_ALT):
                    i += 1
                    continue
                if i + 8 > len(buf):
                    break
                version = buf[i + 2]
                payload_len_hdr = buf[i + 3]
                seq = struct.unpack_from("<I", buf, i + 4)[0]

                line = f"cand off={i} ver={version} payload_len_hdr={payload_len_hdr} seq={seq}"
                checked = False
                payload_size = PAYLOAD_SIZES.get(version)
                if payload_size is not None:
                    expected_low8 = payload_size & 0xFF
                    line += f" payload_size={payload_size}"
                    if payload_len_hdr != expected_low8:
                        line += f" len=BAD exp_low8={expected_low8}"
                        checked = True
                    else:
                        frame_size = 8 + payload_size + 2
                        if i + frame_size <= len(buf):
                            crc_rx = struct.unpack_from("<H", buf, i + frame_size - 2)[0]
                            crc_ok = crc16_firmware(bytes(buf[i : i + frame_size - 2])) == crc_rx
                            line += f" crc={'OK' if crc_ok else 'BAD'}"
                            if crc_ok:
                                crc_ok_by_version[version] += 1
                            checked = True
                else:
                    line += f" unsupported expect_ver={SUPPORTED_VERSIONS}"
                if checked:
                    print(line)
                    hits += 1
                    if hits >= 20:
                        break
                i += 1
            if hits >= 20:
                break
            if len(buf) > 8192:
                del buf[:4096]
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()

    summary = " ".join(
        f"v{version}_crc_ok={crc_ok_by_version.get(version, 0)}" for version in SUPPORTED_VERSIONS
    )
    print(f"summary: candidates_printed={hits} {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
