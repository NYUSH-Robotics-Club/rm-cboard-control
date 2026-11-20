import json
import hashlib
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
sys.path.insert(0, str(ROOT / "quest_runtime"))
from frankabot_vehicle import TranslationInput, pack


class InputTests(unittest.TestCase):
    def packet(self, position=(0, 0, 0), trigger=False):
        return dict(position=position, tracking_valid=True, gate_open=True,
                    host_received_monotonic_ns=time.monotonic_ns(), source_metadata=dict(flag=trigger))

    def test_hold_requires_prior_release(self):
        mapping = TranslationInput(1, 1, 0, 1)
        self.assertEqual(mapping.update(self.packet((0, .2, 0), True), True, time.monotonic_ns()), (False, 0, 0))
        mapping.update(self.packet(), True, time.monotonic_ns())
        self.assertEqual(mapping.update(self.packet((0, .15, 0), True), True, time.monotonic_ns()), (True, 100, 0))

    def test_directions_and_radial_limit(self):
        for position, expected in [((0, .15, 0), (100, 0)), ((0, -.15, 0), (-100, 0)),
                                   ((.15, 0, 0), (0, 100)), ((-.15, 0, 0), (0, -100))]:
            mapping = TranslationInput(1, 1, 0, 1)
            mapping.update(self.packet(), True, time.monotonic_ns())
            self.assertEqual(mapping.update(self.packet(position, True), True, time.monotonic_ns()), (True, *expected))
        mapping.update(self.packet(), True, time.monotonic_ns())
        enabled, f, r = mapping.update(self.packet((1, 1, 0), True), True, time.monotonic_ns())
        self.assertTrue(enabled)
        self.assertLessEqual((f*f+r*r)**.5, 101)

    def test_dead_zone_release_and_loss(self):
        mapping = TranslationInput(1, 1, 0, 1)
        mapping.update(self.packet(), True, time.monotonic_ns())
        self.assertEqual(mapping.update(self.packet((.01, .01, 0), True), True, time.monotonic_ns()), (True, 0, 0))
        stale = self.packet()
        stale["host_received_monotonic_ns"] -= 121_000_000
        self.assertIsNone(mapping.update(stale, True, time.monotonic_ns()))
        self.assertEqual(mapping.update(self.packet((0, .2, 0), True), True, time.monotonic_ns()), (False, 0, 0))
        mapping.update(self.packet(), True, time.monotonic_ns())
        self.assertEqual(mapping.update(self.packet((0, .2, 0), True), False, time.monotonic_ns()), (False, 0, 0))

    def test_frame_contract(self):
        frame = pack(65535, True, -100, 100)
        self.assertEqual(len(frame), 39)
        self.assertEqual(frame[:4], bytes.fromhex("a51e00ff"))
        self.assertEqual(frame[5:7], bytes.fromhex("0203"))
        self.assertEqual(frame[7:15], b"VRD2\xff\xff\x01\x00")
        self.assertEqual(frame[15:19], bytes.fromhex("9cff6400"))
        self.assertEqual(frame[19:37], bytes(18))
        with self.assertRaises(ValueError): pack(1, False, 1)
        with self.assertRaises(ValueError): pack(1, True, 101)
        with self.assertRaises(ValueError): pack(1, True, 0, diagnostic=True)

    def test_saved_calibration_and_original_page(self):
        from vr_input import QuestInput
        profile = ROOT / "quest_runtime/repository_calibrations/quest_lab_current.json"
        source = QuestInput(calibration_path=profile)
        self.assertEqual(source.calibration_sha256, hashlib.sha256(profile.read_bytes()).hexdigest())
        # Imports/AST adaptation only: no sockets, headset files or control output.
        import run_repository_calibration as calibration
        page = calibration.load_original_web()
        self.assertIn("<html", page.HTML)
        from quest_trajectory_recorder.live3d import parse_args
        self.assertTrue(callable(parse_args))


if __name__ == "__main__":
    unittest.main()
