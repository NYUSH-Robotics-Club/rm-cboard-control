"""MQTT wire contract, no USB, broker, SSH, or robot access."""
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
sys.path.insert(0, str(ROOT / "pc/mqtt"))
from frankabot_vehicle import pack, pack_payload
from custom_control import CustomControl, encode
from calibration_check import quality


class WireTests(unittest.TestCase):
    def test_perpendicular_and_collinear_calibration_captures(self):
        p = lambda x, y, z: dict(x=x, y=y, z=z)
        data = dict(version=5, state="ready", origin=p(0,0,0), right=p(1,0,0),
                    forward=p(0,0,1), up=p(0,1,0), rightStart=p(0,0,0),
                    rightEnd=p(.2,0,0), forwardStart=p(0,0,0), forwardEnd=p(0,0,.2))
        self.assertAlmostEqual(quality(data)["capture_angle_deg"], 90)
        for end in [p(-.2,0,.01), p(.01,0,0), p(float("nan"),0,0)]:
            with self.assertRaises(ValueError): quality({**data, "forwardEnd": end})

    def test_mqtt_carries_only_the_thirty_byte_payload(self):
        payload = pack_payload(123, True, -100, 50)
        encoded = encode(payload)
        self.assertEqual(len(payload), 30)
        self.assertEqual(encoded[:2], b"\x0a\x1e")
        self.assertEqual(CustomControl.FromString(encoded).data, payload)
        self.assertEqual(payload, pack(123, True, -100, 50, command=0x0311)[7:-2])
        self.assertEqual(struct.unpack("<4sHBBhh18x", payload), (b"VRD2",123,1,0,-100,50))

    def test_payload_limits_and_diagnostic_cannot_drive(self):
        for arguments in [(0, False, 1), (0, True, 101)]:
            with self.assertRaises(ValueError): pack_payload(*arguments)
        with self.assertRaises(ValueError): pack_payload(1, True, 1, diagnostic=True)
        with self.assertRaises(ValueError): encode(bytes(31))
        self.assertEqual(pack_payload(65536, False, 0)[4:8], b"\0\0\0\0")

    def test_legacy_and_mqtt_uart_frames_differ_only_by_command_and_crc(self):
        old, new = (pack(1, False, 0, command=command) for command in (0x0302,0x0311))
        self.assertEqual(old[:5], new[:5])
        self.assertEqual(new[5:7], b"\x11\x03")
        self.assertEqual(old[7:-2], new[7:-2])


if __name__ == "__main__": unittest.main()
