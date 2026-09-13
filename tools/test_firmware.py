"""Exercise failure boundaries without connecting to or writing any hardware."""
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
import bootstrap


class FirmwareSafetyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {"robot": "infantry_standard", "mode": "Debug", "flash": {
            "interface": "SWD", "serial": None, "allow_single": True, "run_after": False}}
        self.output = io.StringIO()
        self.redirect = contextlib.redirect_stdout(self.output)
        self.redirect.__enter__()
        # Failure-path tests must never inspect the real user's USB hardware.
        self.usb_report = patch.object(fw.stlink_usb, "report").start()
        self.addCleanup(patch.stopall)

    def tearDown(self):
        self.redirect.__exit__(None, None, None)

    def test_missing_config_never_selects_robot(self):
        with tempfile.TemporaryDirectory() as d, patch.object(fw, "CONFIG", Path(d) / "missing"):
            with self.assertRaises(fw.Failure):
                fw.load_config()
        with self.assertRaises(fw.Failure):
            fw.selected({})

    def test_explicit_override_does_not_modify_saved_robot(self):
        self.assertEqual(fw.selected(self.cfg, "sentry_swerve"), "sentry_swerve")
        self.assertEqual(self.cfg["robot"], "infantry_standard")

    def test_linux_arm64_host_and_version_pins(self):
        with patch.object(fw.platform, "system", return_value="Linux"), patch.dict(fw.os.environ, {}, clear=True):
            for arch in ("aarch64", "arm64"):
                with patch.object(fw.platform, "machine", return_value=arch):
                    self.assertEqual(fw.host(), "linux-aarch64")
            self.assertEqual(fw.tool_pins()["gcc"], "15.3.1")
            self.assertEqual(fw.tool_pins()["cmake"], "4.4.3")
            with patch.object(fw.platform, "machine", return_value="x86_64"):
                with self.assertRaises(fw.Failure):
                    fw.host()

    def test_existing_host_pins_and_windows_arm64_rejection(self):
        with patch.dict(fw.os.environ, {}, clear=True):
            for system, arch, gcc in (("Windows", "AMD64", "14.3.1"),
                                      ("Darwin", "arm64", "14.3.1"),
                                      ("Darwin", "x86_64", "14.2.1")):
                with patch.object(fw.platform, "system", return_value=system), patch.object(fw.platform, "machine", return_value=arch):
                    self.assertEqual(fw.host(), system.lower() + "-" + arch.lower())
                    self.assertEqual(fw.tool_pins()["gcc"], gcc)
                    self.assertEqual(fw.tool_pins()["cmake"], "4.2.3")
            with patch.object(fw.platform, "system", return_value="Windows"), patch.object(fw.platform, "machine", return_value="ARM64"):
                with self.assertRaises(fw.Failure):
                    fw.host()

    def test_linux_still_rejects_wrong_tool_version(self):
        with patch.object(fw, "host", return_value="linux-aarch64"), patch.object(fw.platform, "system", return_value="Linux"), patch.object(fw, "discover", return_value="cmake"), patch.object(fw, "run", return_value="cmake version 4.2.3"):
            with self.assertRaisesRegex(fw.Failure, "expected 4.4.3, got 4.2.3"):
                fw.toolset({})

    def test_linux_build_preserves_legacy_artifact_directory(self):
        with patch.object(fw.platform, "system", return_value="Linux"), patch.object(fw, "host", return_value="linux-aarch64"):
            result = fw.build_dir(self.cfg, "infantry_standard", {"gcc": "compiler"})
            self.assertTrue(result.is_relative_to(fw.ROOT / "build" / "verified"))
            self.assertEqual(result.name, "inf-debug")

    def test_linux_cache_checks_compiler_from_matching_cmake_version(self):
        # Modern CMake stores the compiler in this file rather than the cache.
        with tempfile.TemporaryDirectory() as d, patch.object(fw.platform, "system", return_value="Linux"):
            root = Path(d)
            compiler = root / "arm-none-eabi-gcc"
            compiler.touch()
            (root / "CMakeCache.txt").write_text(
                f"ROBOT_TYPE:STRING=infantry_standard\nCMAKE_BUILD_TYPE:STRING=Debug\n"
                f"CMAKE_GENERATOR:INTERNAL=Ninja\nCMAKE_HOME_DIRECTORY:INTERNAL={fw.ROOT}\n")
            info = root / "CMakeFiles/4.4.3/CMakeCCompiler.cmake"
            info.parent.mkdir(parents=True)
            info.write_text(f'set(CMAKE_C_COMPILER "{compiler.as_posix()}")\n')
            fw.check_cache(root, self.cfg, "infantry_standard", {"gcc": str(compiler)})
            info.write_text('set(CMAKE_C_COMPILER "/missing/compiler")\n')
            with self.assertRaisesRegex(fw.Failure, "Cache mismatch CMAKE_C_COMPILER"):
                fw.check_cache(root, self.cfg, "infantry_standard", {"gcc": str(compiler)})

    def test_linux_bootstrap_stops_before_external_commands(self):
        with patch.object(fw.platform, "system", return_value="Linux"), patch.object(fw, "run") as external:
            with self.assertRaisesRegex(fw.Failure, "Linux bootstrap is not implemented"):
                list(bootstrap.sources())
            external.assert_not_called()

    def test_probe_boundaries(self):
        settings = self.cfg["flash"]
        for probes in ([], ["AAA", "BBB"]):
            with self.assertRaises(fw.Failure):
                fw.choose_probe(probes, settings)
        self.assertEqual(fw.choose_probe(["AAA"], settings), "AAA")
        settings["serial"] = "BBB"
        self.assertEqual(fw.choose_probe(["AAA", "BBB"], settings), "BBB")
        with self.assertRaises(fw.Failure):
            fw.choose_probe(["AAA"], settings)

    def test_probe_parser(self):
        self.assertEqual(fw.parse_probes("ST-LINK SN : ABC123\nST-LINK SN  : DEF456"), ["ABC123", "DEF456"])
        self.assertEqual(fw.parse_probes("No ST-Link detected!"), [])

    def test_usb_diagnosis_distinguishes_detection_layers(self):
        camera = {"Name": "Camera DFU Device", "PNPDeviceID": "USB\\VID_174F&PID_1820", "ConfigManagerErrorCode": 0}
        probe = {"Name": "USB Device", "PNPDeviceID": "USB\\VID_0483&PID_3748", "ConfigManagerErrorCode": 0}
        self.assertEqual(fw.stlink_usb.diagnose([camera])[0], "NO_STLINK_USB")
        self.assertEqual(fw.stlink_usb.diagnose([probe])[0], "USB_PRESENT_CLI_UNAVAILABLE")
        probe["ConfigManagerErrorCode"] = 28
        self.assertEqual(fw.stlink_usb.diagnose([probe])[0], "USB_DRIVER_ERROR")
        unknown = {"Name": "Unknown USB Device", "ConfigManagerErrorCode": 43}
        self.assertEqual(fw.stlink_usb.diagnose([unknown])[0], "USB_DEVICE_ERROR")
        self.assertEqual(fw.stlink_usb.diagnose([{"PNPDeviceID": "USB\\VID_0483&PID_3755"}])[0], "STLINK_LOADER")

    def test_enumeration_failure_never_connects(self):
        with patch.object(fw, "build", return_value={}), patch.object(fw, "run", side_effect=fw.Failure("CLI exit 1")) as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", {"cube": "cube"}, {})
            self.assertEqual(external.call_count, 1)
            self.usb_report.assert_called_once()

    def test_target_is_fail_closed(self):
        good = "Device ID : 0x413\nDevice name : STM32F405xx/F407xx/F415xx/F417xx\nFlash size : 1024 KBytes"
        self.assertEqual(fw.validate_target(good)["flash_kbytes"], 1024)
        for bad in ("", good.replace("413", "419"), good.replace("1024", "512"), good.replace("STM32F40", "STM32H70")):
            with self.assertRaises(fw.Failure):
                fw.validate_target(bad)

    def test_target_capacity_accepts_cubeprogrammer_megabytes(self):
        observed = "Device ID   : 0x413\nDevice name : STM32F405xx/F407xx/F415xx/F417xx\nFlash size  : 1 MBytes\nDevice CPU : Cortex-M4"
        self.assertEqual(fw.validate_target(observed)["flash_kbytes"], 1024)
        for capacity in ("2 MBytes", "1 KBytes", "512 KBytes", "1 GBytes", "unknown"):
            with self.subTest(capacity=capacity), self.assertRaises(fw.Failure) as failure:
                fw.validate_target(observed.replace("1 MBytes", capacity))
            self.assertIn(capacity, str(failure.exception))
        for invalid in (observed.replace("0x413", "0x450"), observed.replace("STM32F405xx/F407xx/F415xx/F417xx", "STM32H750")):
            with self.assertRaises(fw.Failure):
                fw.validate_target(invalid)

    def test_plan_invokes_no_external_process(self):
        with patch.object(fw, "run") as external:
            fw.plan(self.cfg, "infantry_standard", {"gcc": "compiler", "cube": "cube"})
            external.assert_not_called()
        self.assertIn('"hardware_commands_executed": 0', self.output.getvalue())
        self.assertIn('mode=NORMAL', self.output.getvalue())
        self.assertIn('mode=HOTPLUG', self.output.getvalue())

    def test_build_failure_never_enumerates_or_writes(self):
        with patch.object(fw, "build", side_effect=fw.Failure("compile error")), patch.object(fw, "run") as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", {"cube": "cube"}, {})
            external.assert_not_called()

    def test_no_probe_never_connects(self):
        with patch.object(fw, "build", return_value={}), patch.object(fw, "run", return_value="No ST-Link detected!") as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", {"cube": "cube"}, {})
            self.assertEqual(external.call_args_list[0].args[0], ["cube", "-l", "stlink-only"])
            self.assertEqual(external.call_count, 1)

    def test_multiple_probes_never_connects(self):
        with patch.object(fw, "build", return_value={}), patch.object(fw, "run", return_value="ST-LINK SN : AAA\nST-LINK SN : BBB") as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", {"cube": "cube"}, {})
            self.assertEqual(external.call_count, 1)

    def test_verification_failure_never_resets(self):
        self.cfg["flash"]["run_after"] = True
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / "firmware.elf"
            elf.write_bytes(b"test")
            record = {"elf": str(elf), "sha256": fw.sha256(elf)}
            info = "Device ID : 0x413\nDevice name : STM32F405xx/F407xx\nFlash size : 1024 KBytes"
            for outcome in ("Verification failed", fw.Failure("download exit 1")):
                with patch.object(fw, "build", return_value=record), patch.object(fw, "run", side_effect=["ST-LINK SN : AAA", info, outcome]) as external:
                    with self.assertRaises(fw.Failure) as failure:
                        fw.flash(self.cfg, "infantry_standard", {"cube": "cube"}, {})
                    self.assertEqual(external.call_count, 3)
                    self.assertNotIn("-rst", external.call_args.args[0])
                    self.assertNotIn("-run", external.call_args.args[0])
                    if isinstance(outcome, fw.Failure):
                        self.assertIn("partially programmed", str(failure.exception))

    def test_wrong_chip_never_writes(self):
        with patch.object(fw, "build", return_value={}), patch.object(fw, "run", side_effect=["ST-LINK SN : AAA", "Device ID : 0x419"]) as external:
            with self.assertRaises(fw.Failure):
                fw.flash(self.cfg, "infantry_standard", {"cube": "cube"}, {})
            self.assertEqual(external.call_count, 2)

    def test_successful_verify_resets_only_when_selected(self):
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / "firmware.elf"
            elf.write_bytes(b"test")
            record = {"elf": str(elf), "sha256": fw.sha256(elf)}
            info = "Device ID : 0x413\nDevice name : STM32F405xx/F407xx\nFlash size : 1024 KBytes"
            for reset in (False, True):
                self.cfg["flash"]["run_after"] = reset
                with patch.object(fw, "build", return_value=record), patch.object(fw, "run", side_effect=["ST-LINK SN : AAA", info, "Download verified successfully", "reset OK"]) as external:
                    fw.flash(self.cfg, "infantry_standard", {"cube": "cube"}, {})
                    self.assertEqual(external.call_count, 4 if reset else 3)
                    identify = external.call_args_list[1].args[0]
                    write = external.call_args_list[2].args[0]
                    self.assertIn("mode=HOTPLUG", identify)
                    self.assertNotIn("reset=SWrst", identify)
                    self.assertIn("mode=NORMAL", write)
                    self.assertIn("reset=SWrst", write)
                    self.assertIn("-halt", write)
                    self.assertNotIn("-run", write)
                    self.assertIn("-log", write)

    def test_process_failure_is_nonzero(self):
        with self.assertRaises(fw.Failure):
            fw.run([sys.executable, "-c", "raise SystemExit(7)"])

    def test_arguments_preserve_spaces_and_chinese(self):
        arg = "路径 with spaces; harmless $(text)"
        result = fw.run([sys.executable, "-X", "utf8", "-c", "import sys; print(sys.argv[1])", arg])
        self.assertEqual(result.strip(), arg)

    def test_bad_elf_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / "bad.elf"
            elf.write_bytes(b"not an ELF")
            with self.assertRaises(fw.Failure):
                fw.inspect_elf(elf, {})


if __name__ == "__main__":
    unittest.main()
