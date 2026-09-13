"""Check probe selection and execute generated Tcl against a simulated target.
No test opens USB/SWD; erase and verification failures must prevent reset/run.
"""
import contextlib
import io
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import firmware as fw
import openocd_programmer as oc

SERIAL = '53FF6F067187485514522487'


class ProbeTests(unittest.TestCase):
    def test_sysfs_raw_and_ascii_serials(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for name, raw in [('1-1', bytes.fromhex(SERIAL).decode('latin1')),
                              ('1-2', SERIAL.lower())]:
                device = root / name
                device.mkdir()
                (device / 'idVendor').write_text('0483\n')
                (device / 'idProduct').write_text('3748\n')
                (device / 'serial').write_text(raw + '\n')
            self.assertEqual(oc.probes(root), [SERIAL, SERIAL])
            with self.assertRaises(fw.Failure):
                fw.choose_probe(oc.probes(root), {'allow_single': True})
            (root / '1-1/serial').write_text('unknown\n')
            with self.assertRaises(ValueError):
                oc.probes(root)

    def test_explicit_serial_must_be_unique(self):
        with self.assertRaises(fw.Failure):
            fw.choose_probe([SERIAL, SERIAL], {'serial': SERIAL})

    def test_plan_and_main_dispatch_do_not_connect(self):
        cfg = {'robot': 'infantry_standard', 'mode': 'Debug', 'flash': {
            'backend': 'openocd', 'interface': 'SWD', 'allow_single': True,
            'serial': None, 'run_after': False}}
        with tempfile.TemporaryDirectory() as d, patch.object(fw, 'ROOT', Path(d)), patch.object(fw.platform, 'system', return_value='Linux'), patch.object(fw, 'load_config', return_value=cfg), patch.object(fw, 'toolset', return_value=({'gcc': 'compiler'}, {})), patch.object(fw, 'host', return_value='linux-aarch64'), patch.object(fw, 'run') as external, patch.object(oc, 'probes') as usb, contextlib.redirect_stdout(io.StringIO()) as output:
            fw.main(['flash-plan'])
            external.assert_not_called()
            usb.assert_not_called()
            self.assertIn('"hardware_commands_executed": 0', output.getvalue())
            with patch.object(fw, 'flash_openocd') as opened, patch.object(fw, 'flash') as cube:
                fw.main(['flash'])
                opened.assert_called_once()
                cube.assert_not_called()

    def test_build_failure_and_changed_elf_never_open_target(self):
        cfg = {'flash': {'interface': 'SWD', 'allow_single': True, 'run_after': False}}
        with patch.object(fw, 'build', side_effect=fw.Failure('compile failed')), patch.object(fw, 'openocd_probe') as probe:
            with self.assertRaises(fw.Failure):
                fw.flash_openocd(cfg, 'infantry_standard', {}, {})
            probe.assert_not_called()
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / 'image.elf'
            elf.write_bytes(b'changed')
            with patch.object(fw, 'build', return_value={'elf': str(elf), 'sha256': 'old'}), patch.object(fw, 'openocd_probe', return_value=SERIAL), patch.object(fw, 'run') as external:
                with self.assertRaisesRegex(fw.Failure, 'ELF changed'):
                    fw.flash_openocd(cfg, 'infantry_standard', {}, {})
                external.assert_not_called()

    def test_flash_requires_positive_verification_and_retains_backup(self):
        cfg = {'flash': {'interface': 'SWD', 'allow_single': True, 'run_after': False}}
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / 'image.elf'
            elf.write_bytes(b'test image')

            def completed(args, **kwargs):
                directory = Path(args[-1]).parent
                (directory / 'before-flash.bin').write_bytes(b'\xff' * 1048576)
                return 'FW_BACKUP_OK\nFW_VERIFY_OK\nFW_FLASH_OK\n'

            def record():
                return {'elf': str(elf), 'sha256': fw.sha256(elf)}

            with patch.object(fw, 'build', side_effect=lambda *args: record()), patch.object(fw, 'openocd_probe', return_value=SERIAL), patch.object(fw, 'run', side_effect=completed), contextlib.redirect_stdout(io.StringIO()) as output:
                fw.flash_openocd(cfg, 'infantry_standard', {'openocd': 'openocd'}, {})
                self.assertIn('[flash OK]', output.getvalue())
                manifest = next(Path(d).glob('flash-*/flash-manifest.json'))
                self.assertIn('"verified": true', manifest.read_text())
                self.assertIn('"backup_sha256"', manifest.read_text())
            with patch.object(fw, 'build', side_effect=lambda *args: record()), patch.object(fw, 'openocd_probe', return_value=SERIAL), patch.object(fw, 'run', return_value='FW_BACKUP_OK\n'), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(fw.Failure, 'did not report completed'):
                    fw.flash_openocd(cfg, 'infantry_standard', {'openocd': 'openocd'}, {})


@unittest.skipUnless(shutil.which('tclsh'), 'Tcl interpreter unavailable')
class TransactionTests(unittest.TestCase):
    def simulate(self, *, phase='', chip='0x413', capacity=1024, run_after=True, readonly=False):
        # Explicit stubs expose the order of every destructive action. Tcl still
        # parses actual generated paths and control flow, including catch/shutdown.
        harness = r'''
proc find {value} {return $value}
proc source {args} {}
foreach name {transport adapter reset_config cortex_m gdb tcl telnet init poll wait_halt} {
    proc $name {args} {}
}
set state running
proc stm32f4x.cpu {args} {
    global state
    if {[lindex $args 0] eq "curstate"} {return $state}
}
proc echo {value} {puts $value}
proc read_memory {address width count} {
    global chip capacity
    if {$address == 0xe0042000} {return $chip}
    if {$address == 0x1fff7a22} {return $capacity}
    return {1 2 3}
}
proc dump_image {path address count} {
    global phase
    if {$phase eq "backup"} {error "backup failure"}
    set f [open $path wb]
    puts -nonewline $f [string repeat \0 $count]
    close $f
}
proc reset {mode} {
    global state
    puts "CALL:reset $mode"
    set state [expr {$mode eq "halt" ? "halted" : "running"}]
}
proc flash {args} {
    global phase
    puts "CALL:flash"
    if {$phase eq "write"} {error "write failure"}
}
proc verify_image {path} {
    global phase
    puts "CALL:verify"
    if {$phase eq "verify"} {error "verify failure"}
}
proc shutdown {args} {exit [expr {[llength $args] > 0 ? 1 : 0}]}
'''
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            image = root / '路径 [error INJECTED] $var; {x}.elf'
            image.write_bytes(b'ELF')
            script = root / 'test.tcl'
            code = oc.script(SERIAL, image=None if readonly else image,
                             backup=root / '备份 [error BAD] $a.bin', run_after=run_after)
            script.write_text(f'set phase {oc.tcl_word(phase)}\nset chip {chip}\nset capacity {capacity}\n' + harness + code)
            return subprocess.run(['tclsh', str(script)], capture_output=True, text=True)

    def test_readonly_does_not_halt_write_or_run(self):
        result = self.simulate(readonly=True)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn('FW_TARGET_OK', result.stdout)
        self.assertNotIn('CALL:', result.stdout)

    def test_target_or_backup_failure_prevents_reset_and_erase(self):
        for args in ({'chip': '0x419'}, {'capacity': 512}, {'phase': 'backup'}):
            with self.subTest(args=args):
                result = self.simulate(**args)
                self.assertEqual(result.returncode, 1)
                self.assertNotIn('CALL:', result.stdout)

    def test_write_or_verify_failure_never_runs(self):
        for phase in ('write', 'verify'):
            result = self.simulate(phase=phase)
            self.assertEqual(result.returncode, 1)
            self.assertIn('CALL:flash', result.stdout)
            self.assertNotIn('CALL:reset run', result.stdout)
            self.assertNotIn('FW_FLASH_OK', result.stdout)

    def test_success_runs_only_after_verification_when_requested(self):
        for run_after in (False, True):
            result = self.simulate(run_after=run_after)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            self.assertIn('FW_FLASH_OK', result.stdout)
            self.assertLess(result.stdout.index('FW_BACKUP_OK'), result.stdout.index('CALL:reset halt'))
            if run_after:
                self.assertLess(result.stdout.index('FW_VERIFY_OK'), result.stdout.index('CALL:reset run'))
            else:
                self.assertNotIn('CALL:reset run', result.stdout)
                self.assertIn('FW_STATE=halted', result.stdout)


if __name__ == '__main__':
    unittest.main()
