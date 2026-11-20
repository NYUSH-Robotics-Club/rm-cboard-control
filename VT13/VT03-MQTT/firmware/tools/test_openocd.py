"""Run the real OpenOCD Tcl interpreter with all target operations substituted.
No interface is loaded; noinit prevents automatic target access even on test errors.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import firmware as fw


class OpenOcdScriptTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.openocd = os.environ.get("OPENOCD") or fw.load_config(False).get("tools", {}).get("openocd") or shutil.which("openocd")
        if not cls.openocd:
            raise unittest.SkipTest("OpenOCD is not installed")

    def execute(self, scenario, run_after="0"):
        guard = fw.ROOT / "tools/openocd/flash_checked.tcl"
        command = [self.openocd, "-c", "noinit",
                   "-f", str(fw.ROOT / "tools/openocd/test_flash_mock.tcl"),
                   "-f", str(guard), "-c", "set SCENARIO " + scenario,
                   "-c", "set FW_ELF " + fw.tcl_word(str(guard)),
                   "-c", "set FW_RUN_AFTER " + run_after, "-c", "fw_program"]
        return subprocess.run(command, capture_output=True, text=True, timeout=15)

    def test_wrong_identity_or_missing_reads_never_reset_or_write(self):
        for scenario in ("wrong-id", "wrong-size", "read-failure", "init-failure"):
            with self.subTest(scenario=scenario):
                result = self.execute(scenario, "1")
                output = result.stdout + result.stderr
                self.assertNotEqual(result.returncode, 0, output)
                self.assertNotIn("MOCK_RESET", output)
                self.assertNotIn("MOCK_WRITE", output)

    def test_identity_change_after_reset_never_writes(self):
        result = self.execute("changed-id", "1")
        output = result.stdout + result.stderr
        self.assertNotEqual(result.returncode, 0, output)
        self.assertIn("MOCK_RESET_halt", output)
        self.assertNotIn("MOCK_WRITE", output)
        self.assertNotIn("MOCK_RESET_run", output)

    def test_reset_write_verify_failures_never_run(self):
        for scenario in ("reset-failure", "write-failure", "verify-failure"):
            with self.subTest(scenario=scenario):
                result = self.execute(scenario, "1")
                output = result.stdout + result.stderr
                self.assertNotEqual(result.returncode, 0, output)
                self.assertNotIn("MOCK_RESET_run", output)
                self.assertNotIn("FIRMWARE_FLASH_OK", output)
                self.assertNotIn("FIRMWARE_VERIFY_OK", output)

    def test_success_order_and_explicit_run_choice(self):
        for run_after in ("0", "1"):
            result = self.execute("success", run_after)
            output = result.stdout + result.stderr
            self.assertEqual(result.returncode, 0, output)
            order = ["MOCK_INIT", "FIRMWARE_TARGET", "MOCK_RESET_halt",
                     "MOCK_WRITE", "MOCK_VERIFY", "FIRMWARE_VERIFY_OK"]
            if run_after == "1":
                order.append("MOCK_RESET_run")
            else:
                self.assertNotIn("MOCK_RESET_run", output)
            order += ["FIRMWARE_FLASH_OK", "MOCK_SHUTDOWN"]
            positions = [output.index(token) for token in order]
            self.assertEqual(positions, sorted(positions), output)
            self.assertEqual(output.count("FIRMWARE_TARGET"), 2)

    def test_missing_or_invalid_run_choice_never_connects(self):
        result = self.execute("success", "maybe")
        output = result.stdout + result.stderr
        self.assertNotEqual(result.returncode, 0, output)
        self.assertNotIn("MOCK_INIT", output)

    def test_tcl_argument_roundtrip_does_not_execute_substitutions(self):
        value = '路径 with spaces; [error injected] $env(HOME) {brace} "quote" \\slash\nline'
        with tempfile.TemporaryDirectory() as directory:
            output_file = Path(directory) / "roundtrip.txt"
            script = ("set value " + fw.tcl_word(value) + "; set f [open " +
                      fw.tcl_word(str(output_file)) + " w]; puts -nonewline $f $value; close $f; shutdown")
            result = subprocess.run([self.openocd, "-c", "noinit", "-c", script],
                                    capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(output_file.read_text(), value)


if __name__ == "__main__":
    unittest.main()
