"""Checks for display-only light pairs; run with Python, OpenCV and NumPy installed."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vision_live_web import choose_candidate, find_lightbar_candidates, find_saturated_blue_pair


class LightPairTests(unittest.TestCase):
    def test_saturated_pair_uses_core_midpoint_not_union_box_center(self):
        frame = np.full((720, 1280, 3), (220, 80, 30), dtype=np.uint8)
        cv2.ellipse(frame, (410, 460), (40, 30), 0, 0, 360, (255, 255, 255), -1)
        cv2.ellipse(frame, (690, 460), (80, 70), 0, 0, 360, (255, 255, 255), -1)
        box, center = find_saturated_blue_pair(frame)
        self.assertEqual(center, (550, 460))
        self.assertNotEqual(center[0], round(box[0] + box[2] / 2))

    def test_neutral_lights_and_single_core_are_not_pairs(self):
        neutral = np.full((720, 1280, 3), 90, dtype=np.uint8)
        for x in (410, 690):
            cv2.circle(neutral, (x, 460), 40, (255, 255, 255), -1)
        self.assertIsNone(find_saturated_blue_pair(neutral))
        blue = np.full((720, 1280, 3), (220, 80, 30), dtype=np.uint8)
        cv2.circle(blue, (410, 460), 40, (255, 255, 255), -1)
        self.assertIsNone(find_saturated_blue_pair(blue))

    def test_brightest_contiguous_core_corrects_asymmetric_bloom(self):
        frame = np.full((720, 1280, 3), (220, 80, 30), dtype=np.uint8)
        cv2.ellipse(frame, (410, 460), (40, 30), 0, 0, 360, (250, 250, 250), -1)
        cv2.ellipse(frame, (690, 460), (80, 70), 0, 0, 360, (250, 250, 250), -1)
        cv2.ellipse(frame, (405, 460), (25, 20), 0, 0, 360, (255, 255, 255), -1)
        cv2.ellipse(frame, (670, 460), (50, 50), 0, 0, 360, (255, 255, 255), -1)
        _box, center = find_saturated_blue_pair(frame)
        self.assertEqual(center, (538, 460))

    def test_narrow_white_bars_still_find_their_plate(self):
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (394, 417), (402, 475), (255, 255, 255), -1)
        cv2.rectangle(frame, (545, 418), (553, 474), (255, 255, 255), -1)
        self.assertEqual(find_lightbar_candidates(frame), [(394, 417, 160, 59)])
        self.assertIsNone(find_saturated_blue_pair(frame))

    def test_low_score_model_box_cannot_supply_the_aim_center(self):
        diagnostic = (.385, 0, (889, 194, 32, 13))
        self.assertEqual(choose_candidate(diagnostic, [], None, .55),
                         (None, None, 'model diagnostic'))
        pair = ((349, 369, 436, 184), (553, 463))
        self.assertEqual(choose_candidate(diagnostic, [], pair, .55),
                         (pair[0], pair[1], 'saturated blue pair'))


if __name__ == '__main__':
    unittest.main()
