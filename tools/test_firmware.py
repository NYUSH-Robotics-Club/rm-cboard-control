"""Exercise OpenOCD orchestration failures without connecting to real hardware."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import firmware as fw


class FirmwareSafetyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {"robot": "infantry_standard", "mode": "Debug", "flash": {
            "backend": "openocd", "interface": "SWD", "serial": None,
            "allow_single": True, "run_after": False}}
        self.output = io.StringIO()
        self.redirect = contextlib.redirect_stdout(self.output)
        self.redirect.__enter__()
        self.addCleanup(self.redirect.__exit__, None, None, None)
        self.usb = patch.object(fw.stlink_usb, "probe_serials", return_value=["AAA"]).start()
        patch.object(fw.stlink_usb, "report").start()
        self.addCleanup(patch.stopall)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.elf = Path(self.temp.name) / "firmware.elf"
        self.elf.write_bytes(b"mock firmware")
        self.record = {"elf": str(self.elf), "sha256": fw.sha256(self.elf)}
        self.paths = {"gcc": "compiler", "openocd": "openocd"}

    def test_missing_config_never_selects_robot(self):
        with patch.object(fw, "CONFIG", Path(self.temp.name) / "missing"):
            with self.assertRaises(fw.Failure):
                fw.load_config()
        with self.assertRaises(fw.Failure):
            fw.selected({})

    def test_logger_shares_http_port_and_allows_explicit_split(self):
        from scripts.dashboard.rtt_ws_bridge import parse_args
        for extra, expected in (([], None), (["--ws-port", "8765"], 8765)):
            with self.subTest(extra=extra), patch.object(fw.subprocess, "call", return_value=0) as launch:
                self.assertEqual(fw.main(["logger", *extra]), 0)
                command = launch.call_args.args[0]
                with patch.object(sys, "argv", command[1:]):
                    args = parse_args()
                self.assertEqual(args.http_port, 8080)
                self.assertEqual(args.ws_port, expected)
                self.assertIsNone(args.ws_host)
        self.usb.assert_not_called()

    def test_configure_defaults_to_reset_run_and_allows_explicit_halt(self):
        # Exercise CLI defaults without configuring tools or accessing the target.
        for extra, expected in (([], "yes"), (["--run-after", "no"], "no")):
            with self.subTest(extra=extra), patch.object(fw, "configure") as configure, \
                    patch.object(fw, "operation_lock", return_value=contextlib.nullcontext()):
                fw.main(["configure", "infantry_standard", *extra])
                self.assertEqual(configure.call_args.args[0].run_after, expected)
        self.usb.assert_not_called()

    def test_explicit_override_does_not_modify_saved_robot(self):
        self.assertEqual(fw.selected(self.cfg, "sentry_swerve"), "sentry_swerve")
        self.assertEqual(self.cfg["robot"], "infantry_standard")

    def test_probe_boundaries(self):
        settings = self.cfg["flash"]
        for probes in ([], ["AAA", "BBB"]):
            with self.assertRaises(fw.Failure):
                fw.choose_probe(probes, settings)
        settings["serial"] = "BBB"
        self.assertEqual(fw.choose_probe(["AAA", "BBB"], settings), "BBB")
        with self.assertRaises(fw.Failure):
            fw.choose_probe(["AAA"], settings)

    def test_legacy_config_uses_openocd_and_rejects_explicit_other_backend(self):
        del self.cfg["flash"]["backend"]
        self.assertEqual(fw.flash_settings(self.cfg)["interface"], "SWD")
        self.cfg["flash"]["backend"] = "cube"
        with self.assertRaises(fw.Failure):
            fw.flash_settings(self.cfg)

    def test_enumeration_failure_never_connects(self):
        self.usb.side_effect = OSError("USB query failed")
        with patch.object(fw, "build", return_value=self.record), patch.object(fw, "run") as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", self.paths, {})
            external.assert_not_called()

    def test_plan_invokes_no_process_or_usb(self):
        with patch.object(fw, "run") as external:
            fw.plan(self.cfg, "infantry_standard", self.paths)
            external.assert_not_called()
            self.usb.assert_not_called()
        plan = json.loads(self.output.getvalue())
        self.assertEqual(plan["backend"], "openocd")
        self.assertEqual(plan["hardware_commands_executed"], 0)
        self.assertIn("reset halt", plan["steps"])
        self.assertIn("verify_image", plan["steps"])
        self.assertEqual(plan["write_verify"][-1], "fw_program")

    def test_build_failure_never_enumerates_or_writes(self):
        with patch.object(fw, "build", side_effect=fw.Failure("compile error")), patch.object(fw, "run") as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", self.paths, {})
            self.usb.assert_not_called()
            external.assert_not_called()

    def test_no_or_multiple_probes_never_connect(self):
        for probes in ([], ["AAA", "BBB"]):
            self.usb.return_value = probes
            with self.subTest(probes=probes), patch.object(fw, "build", return_value=self.record), patch.object(fw, "run") as external:
                with self.assertRaises(fw.Failure):
                    fw.flash(self.cfg, "infantry_standard", self.paths, {})
                external.assert_not_called()

    def test_changed_elf_never_connects(self):
        self.elf.write_bytes(b"changed")
        with patch.object(fw, "build", return_value=self.record), patch.object(fw, "run") as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", self.paths, {})
            external.assert_not_called()

    def test_failures_never_retry_or_open_second_session(self):
        self.cfg["flash"]["run_after"] = True
        for outcome in (fw.Failure("write failed"), "verified", "FIRMWARE_VERIFY_OK\n", "FIRMWARE_FLASH_OK\n"):
            with self.subTest(outcome=outcome), patch.object(fw, "build", return_value=self.record), \
                    patch.object(fw, "run", side_effect=[outcome]) as external:
                with self.assertRaises(fw.Failure):
                    fw.flash(self.cfg, "infantry_standard", self.paths, {})
                self.assertEqual(external.call_count, 1)

    def test_run_option_is_explicit_and_one_session_checks_and_programs(self):
        for run_after in (False, True):
            self.cfg["flash"]["run_after"] = run_after
            with patch.object(fw, "build", return_value=self.record), \
                    patch.object(fw, "run", return_value="FIRMWARE_VERIFY_OK\nFIRMWARE_FLASH_OK\n") as external:
                fw.flash(self.cfg, "infantry_standard", self.paths, {})
                self.assertEqual(external.call_count, 1)
                command = external.call_args.args[0]
                self.assertIn("set FW_RUN_AFTER " + str(int(run_after)), command)
                self.assertIn('fw_select_serial "AAA"', command)
                self.assertEqual(command[-1], "fw_program")
                self.assertEqual(external.call_args.kwargs["log"].name, "openocd-flash.log")

    def test_doctor_config_cannot_initialize_target(self):
        with patch.object(fw, "run", return_value="FIRMWARE_CONFIG_OK\n") as external:
            fw.check_openocd(self.paths)
            command = external.call_args.args[0]
            self.assertIn("noinit; gdb_port disabled; tcl_port disabled; telnet_port disabled", command)
            self.assertNotIn("fw_program", command)
        self.usb.assert_not_called()

    def test_config_error_without_marker_is_rejected(self):
        with patch.object(fw, "run", return_value="Error: missing interface file"):
            with self.assertRaises(fw.Failure):
                fw.check_openocd(self.paths)

    def test_process_failure_is_nonzero_and_log_is_preserved(self):
        log = Path(self.temp.name) / "failure.log"
        with self.assertRaises(fw.Failure):
            fw.run([sys.executable, "-c", "print('failure evidence'); raise SystemExit(7)"], log=log)
        self.assertIn("failure evidence", log.read_text())

    def test_arguments_preserve_spaces_and_chinese(self):
        arg = "路径 with spaces; harmless $(text)"
        result = fw.run([sys.executable, "-X", "utf8", "-c", "import sys; print(sys.argv[1])", arg])
        self.assertEqual(result.strip(), arg)

    def test_bad_elf_rejected(self):
        with self.assertRaises(fw.Failure):
            fw.inspect_elf(self.elf, {})


if __name__ == "__main__":
    unittest.main()
