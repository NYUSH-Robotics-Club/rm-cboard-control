"""Explicit build, probe attachment and checked flashing for Diagnosis."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import stlink_usb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['build', 'connect', 'flash'])
    parser.add_argument('--mode', choices=['Debug', 'Release'], default='Debug')
    parser.add_argument('--serial', help='ST-Link serial; required if multiple probes are connected')
    parser.add_argument('--probe-first', action='store_true', help='Explicitly use the first OpenOCD probe; connect exactly one physical probe')
    parser.add_argument('--openocd', default='openocd')
    parser.add_argument('--tcl-port', type=int, default=6666)
    parser.add_argument('--rtt-port', type=int, default=9090)
    parser.add_argument('--nm', default='arm-none-eabi-nm')
    args = parser.parse_args()
    if args.command == 'build':
        subprocess.run(['cmake', '--preset', args.mode], cwd=ROOT, check=True)
        subprocess.run(['cmake', '--build', '--preset', args.mode], cwd=ROOT, check=True)
        return
    serial = args.serial
    if args.probe_first and serial:
        parser.error('--probe-first and --serial are mutually exclusive')
    if serial is None and not args.probe_first:
        probes = stlink_usb.probe_serials()
        if len(probes) != 1:
            parser.error(f'Expected one ST-Link; found {len(probes)}. Use --serial to select a probe.')
        serial = probes[0]
    if serial is not None and not re.fullmatch(r'[A-Za-z0-9]+', serial):
        parser.error('ST-Link serial must contain only letters and digits')
    if not 1 <= args.tcl_port <= 65535:
        parser.error('Invalid Tcl TCP port')
    if not 1 <= args.rtt_port <= 65535:
        parser.error('Invalid RTT TCP port')
    command = [args.openocd, '-f', str(ROOT/'tools/openocd/stm32f407-stlink.cfg')]
    env = dict(os.environ)
    if serial:
        env['DIAG_SERIAL'] = serial
        command += ['-c', 'fw_select_serial $::env(DIAG_SERIAL)']
    command += ['-c', f'bindto 127.0.0.1; gdb_port disabled; telnet_port disabled; tcl_port {args.tcl_port}']
    elf = ROOT/'build'/args.mode/'CBoard_Diagnosis.elf'
    if args.command == 'flash':
        if not elf.is_file():
            parser.error(f'Build {args.mode} before flashing')
        env['DIAG_ELF'] = str(elf)
        command += ['-f', str(ROOT/'tools/openocd/flash_checked.tcl'),
                    '-c', 'set FW_ELF $::env(DIAG_ELF); set FW_RUN_AFTER 1; fw_program']
    else:
        # No reset/halt/resume/program commands during live attachment.
        from interface import elf_address
        address = elf_address(elf, args.nm)
        command += ['-c', f'init; rtt setup {address} 128 "SEGGER RTT"; rtt polling_interval 5; rtt start; rtt server start {args.rtt_port} 0']
    try:
        subprocess.run(command, cwd=ROOT, env=env, check=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
