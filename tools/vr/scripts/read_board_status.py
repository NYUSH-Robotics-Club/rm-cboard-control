"""Read live SWD diagnostic symbols; never writes RAM or sends control commands."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

SYMBOLS = ["vr_mqtt_firmware_tag", "vr_uart_valid_count", "vr_uart_publish_count",
           "vr_uart_last_axes", "vr_uart_mqtt_count", "vr_uart_legacy_count",
           "vr_uart_last_sequence", "vr_uart_last_valid_ms", "vr_uart_rx_bytes",
           "vr_uart_ring_overflows", "vr_uart_errors", "vr_uart_last_error", "vr_uart_actual_baud"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    symbols = {}
    nm = subprocess.check_output(["arm-none-eabi-nm", "-n", str(args.elf)], text=True)
    for line in nm.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in SYMBOLS:
            symbols[parts[2]] = int(parts[0], 16)
    if set(symbols) != set(SYMBOLS):
        raise RuntimeError("ELF is not the MQTT receiver build")
    commands = ["init"]
    if args.verify:
        if not re.fullmatch(r"[A-Za-z0-9_./-]+", str(args.elf)):
            raise ValueError("ELF path contains unsupported OpenOCD characters")
        commands.append("verify_image " + str(args.elf))
    commands.extend(f"echo [mdw 0x{symbols[name]:08x} 1]" for name in SYMBOLS)
    commands.append("shutdown")
    completed = subprocess.run([shutil.which("openocd") or "openocd", "-f", "interface/stlink.cfg",
        "-c", "transport select hla_swd", "-f", "target/stm32f4x.cfg", "-c", "; ".join(commands)],
        text=True, capture_output=True, timeout=15)
    output = completed.stdout + completed.stderr
    if completed.returncode:
        raise RuntimeError(output[-2500:])
    readings = {int(address, 16): int(value, 16) for address, value in
                re.findall(r"0x([0-9a-fA-F]{8}):\s*([0-9a-fA-F]{8})", output)}
    values = {name: readings[symbols[name]] for name in SYMBOLS}
    if values["vr_mqtt_firmware_tag"] != 0x20261120:
        raise RuntimeError("Running firmware tag does not match the selected ELF")
    axes = values["vr_uart_last_axes"]
    forward, right = axes & 65535, axes >> 16
    values.update(forward=forward - 65536 if forward > 32767 else forward,
                  right=right - 65536 if right > 32767 else right,
                  checked_at=time.time(), elf_sha256=hashlib.sha256(args.elf.read_bytes()).hexdigest(),
                  flash_verified=args.verify)
    print(json.dumps(values))


if __name__ == "__main__":
    main()
