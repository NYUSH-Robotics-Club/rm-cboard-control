"""Check Linux legacy ST-Link descriptor conversion and ambiguous-probe rejection."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import stlink_usb


class LinuxSerialTests(unittest.TestCase):
    def discover(self, serial, product="3748"):
        with tempfile.TemporaryDirectory() as directory:
            device = Path(directory) / "1-2"
            device.mkdir()
            (device / "idVendor").write_text("0483\n")
            (device / "idProduct").write_text(product + "\n")
            (device / "serial").write_bytes((serial + "\n").encode("utf-8"))
            return stlink_usb.linux_serials(Path(directory))

    def test_observed_legacy_descriptor_matches_openocd_serial(self):
        raw = bytes.fromhex("53FF6F067187485514522487").decode("latin-1")
        self.assertEqual(self.discover(raw), ["53FF6F067187485514522487"])

    def test_raw_whitespace_bytes_are_not_stripped_or_translated(self):
        raw = "\r" + "0123456789" + "\n"
        self.assertEqual(self.discover(raw), [raw.encode("latin-1").hex().upper()])

    def test_modern_descriptor_is_unchanged(self):
        serial = "53FF6F067187485514522487"
        self.assertEqual(self.discover(serial), [serial])
        self.assertEqual(self.discover(serial, "374b"), [serial])

    def test_unknown_unicode_is_rejected(self):
        with self.assertRaises(UnicodeEncodeError):
            self.discover("界" + "01234567890")

    def test_invalid_and_duplicate_serials_still_stop_selection(self):
        for serials in ([""], ["bad$serial"], ["A", "A"]):
            with self.subTest(serials=serials), patch.object(stlink_usb.platform, "system", return_value="Linux"), patch.object(stlink_usb, "linux_serials", return_value=serials):
                with self.assertRaises(ValueError):
                    stlink_usb.probe_serials()


if __name__ == "__main__":
    unittest.main()
