"""Select Linux ST-Link probes and generate checked STM32F407 SWD transactions.
This module does not execute commands; firmware.py owns builds, logs and invocation.
"""
from pathlib import Path
import re


def probes(root=Path('/sys/bus/usb/devices')):
    """Read sysfs without opening SWD; unknown serials or duplicate probes stay errors."""
    result = []
    # USB IDs from OpenOCD's interface/stlink.cfg; loader modes are not programmers.
    supported = {'3748', '374b', '374e', '374f', '3752', '3753', '3754', '3757'}
    for device in sorted(root.iterdir()):
        if not (device / 'idVendor').exists():
            continue
        if (device / 'idVendor').read_text().strip().lower() != '0483':
            continue
        pid = (device / 'idProduct').read_text().strip().lower()
        if pid in {'374d', '3755'}:
            raise ValueError('ST-Link USB loader detected; no programming attempted.')
        if pid not in supported:
            continue
        raw = (device / 'serial').read_text().removesuffix('\n')
        # Older V2 USB descriptors expose 12 raw byte values as Unicode. OpenOCD
        # accepts their 24-digit hexadecimal spelling (stlink_usb_get_alternate_serial).
        if len(raw) == 12 and all(ord(c) <= 255 for c in raw):
            serial = raw.encode('latin1').hex().upper()
        elif re.fullmatch(r'[0-9A-Fa-f]{24}', raw):
            serial = raw.upper()
        else:
            raise ValueError(f'Unrecognized ST-Link serial at {device.name}; refusing ambiguous selection.')
        result.append(serial)
    return result


def tcl_word(value):
    """Quote one Tcl argument, including paths containing spaces or substitution syntax."""
    value = str(value)
    for char in ('\\', '"', '$', '[', ']'):
        value = value.replace(char, '\\' + char)
    return '"' + value.replace('\n', '\\n').replace('\r', '\\r') + '"'


def script(serial, *, image=None, backup=None, run_after=False):
    """Return a read-only identity check, or backup/write/verify transaction.
    Flash is 1 MiB; writes require identity and backup checks first. Any failure
    shuts OpenOCD down with an error and never reaches the optional reset/run.
    """
    if not re.fullmatch(r'[0-9A-Fa-f]{24}', serial):
        raise ValueError('OpenOCD requires a 24-digit ST-Link serial.')
    if image is not None and backup is None:
        raise ValueError('A flash backup path is required before programming.')
    setup = f'''source [find interface/stlink.cfg]
transport select swd
source [find target/stm32f4x.cfg]
adapter serial {tcl_word(serial.upper())}
adapter speed 1000
reset_config none
cortex_m reset_config sysresetreq
gdb port disabled
tcl port disabled
telnet port disabled
# Keep the initial identity check from writing DBGMCU registers.
stm32f4x.cpu configure -event examine-end {{}}
'''
    operations = '''init
poll
set chip_id [expr {[lindex [read_memory 0xe0042000 32 1] 0] & 0xfff}]
if {$chip_id != 0x413} {error "Target mismatch: expected STM32F405/407 device ID 0x413"}
# FLASHSIZE_BASE and UID_BASE are defined in the project's STM32F407 CMSIS header.
set flash_kb [lindex [read_memory 0x1fff7a22 16 1] 0]
if {$flash_kb != 1024} {error "Target mismatch: expected 1024 KiB Flash"}
echo "FW_TARGET_OK id=$chip_id flash_kb=$flash_kb uid=[read_memory 0x1fff7a10 32 3]"
'''
    if image is not None:
        operations += f'''dump_image {tcl_word(backup)} 0x08000000 1048576
if {{[file size {tcl_word(backup)}] != 1048576}} {{error "Incomplete Flash backup"}}
echo "FW_BACKUP_OK"
reset halt
wait_halt 5000
echo "FW_WRITE_STARTED"
flash write_image erase {tcl_word(image)}
verify_image {tcl_word(image)}
echo "FW_VERIFY_OK"
'''
        if run_after:
            operations += 'reset run\n'
        else:
            operations += 'if {[stm32f4x.cpu curstate] ne "halted"} {error "Target did not remain halted"}\n'
        operations += 'echo "FW_FLASH_OK"\n'
    operations += 'echo "FW_STATE=[stm32f4x.cpu curstate]"\n'
    return setup + 'if {[catch {\n' + operations + '''} reason]} {
    echo "FW_ERROR: $reason"
    shutdown error
} else {
    shutdown
}
'''
