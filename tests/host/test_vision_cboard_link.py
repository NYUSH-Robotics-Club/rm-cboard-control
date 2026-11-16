#!/usr/bin/env python3
"""Verify Python's read-only C-board decoder against a C Seasky frame."""

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from vision_cboard_link import SeaskyTelemetryParser, crc8, crc16  # noqa: E402


# Produced by get_protocol_send_data(0x0002, 0x1E01, {0.25,-0.5,1.0}, 3).
C_FRAME = bytes.fromhex("a50e00ae0200011e0000803e000000bf0000803f8463")


class SeaskyTelemetryTest(unittest.TestCase):
    def test_crc_and_fragmented_frame(self):
        self.assertEqual(crc8(C_FRAME[:3]), C_FRAME[3])
        self.assertEqual(crc16(C_FRAME[:-2]), int.from_bytes(C_FRAME[-2:], "little"))
        parser = SeaskyTelemetryParser()
        self.assertEqual(parser.feed(b"USB log\r\n" + C_FRAME[:7]), [])
        sample, = parser.feed(C_FRAME[7:])
        self.assertEqual((sample.gyro_z_rad_s, sample.gyro_x_rad_s,
                          sample.gyro_y_rad_s, sample.flags), (0.25, -0.5, 1.0, 0x1E01))

    def test_corrupt_frame_resyncs_to_valid_one(self):
        parser = SeaskyTelemetryParser()
        damaged = bytearray(C_FRAME)
        damaged[9] ^= 1
        samples = parser.feed(bytes(damaged) + C_FRAME)
        self.assertEqual(len(samples), 1)
        self.assertGreaterEqual(parser.crc_errors, 1)

    def test_does_not_treat_command_frame_as_telemetry(self):
        parser = SeaskyTelemetryParser()
        command = bytearray(C_FRAME)
        command[4] = 1
        command[-2:] = crc16(command[:-2]).to_bytes(2, "little")
        self.assertEqual(parser.feed(command), [])


if __name__ == "__main__":
    unittest.main()
