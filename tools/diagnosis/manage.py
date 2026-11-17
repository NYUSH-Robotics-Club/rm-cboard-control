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
    parser.add_argument('--openocd', default='openocd')
    parser.add_argument('--tcl-port', type=int, default=6666)
    args = parser.parse_args()
    if args.command == 'build':
        subprocess.run(['cmake', '--preset', args.mode], cwd=ROOT, check=True)
        subprocess.run(['cmake', '--build', '--preset', args.mode], cwd=ROOT, check=True)
        return
    serial = args.serial
    if serial is None:
        probes = stlink_usb.probe_serials()
        if len(probes) != 1:
            parser.error(f'Expected one ST-Link; found {len(probes)}. Use --serial to select a probe.')
        serial = probes[0]
    if not re.fullmatch(r'[A-Za-z0-9]+', serial):
        parser.error('ST-Link serial must contain only letters and digits')
    if not 1 <= args.tcl_port <= 65535:
        parser.error('Invalid Tcl TCP port')
    command = [args.openocd, '-f', str(ROOT/'tools/openocd/stm32f407-stlink.cfg'),
               '-c', 'fw_select_serial $::env(DIAG_SERIAL)',
               '-c', f'bindto 127.0.0.1; gdb_port disabled; telnet_port disabled; tcl_port {args.tcl_port}']
    env = dict(os.environ, DIAG_SERIAL=serial)
    if args.command == 'flash':
        elf = ROOT/'build'/args.mode/'CBoard_Diagnosis.elf'
        if not elf.is_file():
            parser.error(f'Build {args.mode} before flashing')
        env['DIAG_ELF'] = str(elf)
        command += ['-f', str(ROOT/'tools/openocd/flash_checked.tcl'),
                    '-c', 'set FW_ELF $::env(DIAG_ELF); set FW_RUN_AFTER 1; fw_program']
    else:
        # No reset/halt/resume/program commands during live attachment.
        command += ['-c', 'init']
    try:
        subprocess.run(command, cwd=ROOT, env=env, check=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
