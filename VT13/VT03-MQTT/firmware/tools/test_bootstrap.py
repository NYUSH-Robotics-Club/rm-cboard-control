"""Check host package selection without downloading or executing vendor tools."""
import unittest
import argparse
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

import bootstrap
import firmware
import stlink_usb


class HostPackagesTests(unittest.TestCase):
    def test_linux_editor_preserves_settings_and_prioritizes_python(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / ".vscode").mkdir()
            settings_file = root / ".vscode/settings.json"
            settings_file.write_text('{"editor.tabSize": 4}')
            args = argparse.Namespace(robot="infantry_standard", mode="Debug", serial=None,
                                      allow_single="yes", run_after="no", tool=[])
            paths = {"git": "/usr/bin/git", "gcc": "/opt/arm/bin/arm-none-eabi-gcc", "just": "/opt/just/just"}
            with patch.object(firmware, "ROOT", root), patch.object(firmware, "CONFIG", root / "local.json"), \
                    patch.object(firmware, "toolset", return_value=(paths, {})), \
                    patch.object(firmware.sys, "executable", "/project/.venv/bin/python"), \
                    patch.object(firmware.platform, "system", return_value="Linux"):
                firmware.configure(args)
            settings = json.loads(settings_file.read_text())
            self.assertEqual(settings["editor.tabSize"], 4)
            self.assertTrue(settings["terminal.integrated.env.linux"]["PATH"].startswith("/project/.venv/bin:"))
            self.assertNotIn("terminal.integrated.env.osx", settings)

    def test_linux_packages_match_host_and_keep_mcu_target(self):
        for arch in ("aarch64", "x86_64"):
            with self.subTest(arch=arch), patch.object(bootstrap.platform, "system", return_value="Linux"), \
                    patch.object(bootstrap.platform, "machine", return_value=arch), \
                    patch.object(bootstrap, "github_asset", side_effect=lambda repo, tag, name: (name, "digest")), \
                    patch.object(bootstrap, "latest_tag", return_value="v9.8.7"), \
                    patch.object(bootstrap, "latest_arm_release", return_value="15.3.rel1"), \
                    patch.object(bootstrap, "fetch", return_value=b"a" * 64):
                packages = {name: (pin, source()) for name, pin, source in bootstrap.sources()}
                self.assertIn(arch + "-unknown-linux-musl", packages["just"][1][0])
                self.assertIn("linux-" + arch, packages["cmake"][1][0])
                self.assertEqual(packages["ninja"][1][0], "ninja-linux-aarch64.zip" if arch == "aarch64" else "ninja-linux.zip")
                self.assertEqual(packages["gcc"][0], "15.3.rel1")
                self.assertIn(arch + "-arm-none-eabi.tar.xz", packages["gcc"][1][0])
                self.assertIn("linux-" + ("arm64" if arch == "aarch64" else "x64"), packages["openocd"][1][0])
                self.assertEqual(firmware.host(), "linux-" + arch)

    def test_unknown_linux_architecture_is_rejected(self):
        with patch.object(firmware.platform, "system", return_value="Linux"), \
                patch.object(firmware.platform, "machine", return_value="armv7l"):
            with self.assertRaises(firmware.Failure):
                firmware.host()

    def test_intel_mac_does_not_silently_select_an_old_release(self):
        with patch.object(bootstrap.platform, "system", return_value="Darwin"), \
                patch.object(bootstrap.platform, "machine", return_value="x86_64"), \
                patch.object(bootstrap, "latest_tag", return_value="v9.8.7"), \
                patch.object(bootstrap, "latest_arm_release", return_value="15.3.rel1"):
            packages = {name: pin for name, pin, source in bootstrap.sources()}
            self.assertEqual(packages["gcc"], "15.3.rel1")

    def test_latest_arm_release_ignores_betas_and_sorts_numerically(self):
        branches = [{"name": name} for name in ("main", "releases/9.3.rel1", "releases/15.3.Rel1", "releases/16.0.beta1")]
        with patch.object(bootstrap, "fetch", return_value=json.dumps(branches).encode()):
            self.assertEqual(bootstrap.latest_arm_release(), "15.3.rel1")

    def test_latest_github_release_rejects_prerelease(self):
        with patch.object(bootstrap, "fetch", return_value=b'{"tag_name":"v99","prerelease":true}'):
            with self.assertRaises(firmware.Failure):
                bootstrap.latest_tag("mock/repo")


class UsbMetadataTests(unittest.TestCase):
    def test_linux_only_uses_physical_debug_mode_devices(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, vendor, product, serial in (("1-1", "0483", "374b", "AAA"),
                                                   ("1-2", "0483", "3755", "LOADER"),
                                                   ("1-3", "174f", "1820", "CAMERA")):
                device = root / name
                device.mkdir()
                for key, value in (("idVendor", vendor), ("idProduct", product), ("serial", serial)):
                    (device / key).write_text(value)
            (root / "1-1:1.0").mkdir()
            self.assertEqual(stlink_usb.linux_serials(root), ["AAA"])

    def test_windows_ignores_composite_interfaces(self):
        devices = [{"PNPDeviceID": "USB\\VID_0483&PID_374B\\AAA", "ConfigManagerErrorCode": 0},
                   {"PNPDeviceID": "USB\\VID_0483&PID_374B&MI_00\\6&generated", "ConfigManagerErrorCode": 0}]
        self.assertEqual(stlink_usb.windows_serials(devices), ["AAA"])
        devices[0]["ConfigManagerErrorCode"] = 28
        with self.assertRaises(ValueError):
            stlink_usb.windows_serials(devices)

    def test_mac_recurses_hubs(self):
        data = {"SPUSBDataType": [{"_items": [{"vendor_id": "0x0483 (STMicroelectronics)",
                  "product_id": "0x374b", "serial_num": "AAA"}]}]}
        self.assertEqual(stlink_usb.mac_serials(data), ["AAA"])

    def test_duplicate_or_missing_usb_serials_are_rejected(self):
        for values in (["AAA", "AAA"], [""], ["6&generated"]):
            with patch.object(stlink_usb.platform, "system", return_value="Linux"), \
                    patch.object(stlink_usb, "linux_serials", return_value=values):
                with self.assertRaises(ValueError):
                    stlink_usb.probe_serials()


if __name__ == "__main__":
    unittest.main()
