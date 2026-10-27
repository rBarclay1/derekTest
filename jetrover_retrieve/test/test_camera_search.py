"""Exercise stationary search behavior without ROS or robot hardware."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[2]/'scripts/camera_search.py'
spec = importlib.util.spec_from_file_location('camera_search', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
SearchPolicy = module.SearchPolicy


class SearchTests(unittest.TestCase):
    def test_scan_reverses_at_both_limits(self):
        policy = SearchPolicy(295, 0)
        self.assertEqual(policy.update(1, None), (300, None))
        self.assertEqual(policy.update(2, None), (290, None))
        for tick in range(3, 40):
            target, _ = policy.update(tick, None)
            self.assertTrue(149 <= target <= 300)
        self.assertTrue(policy.target > 149)

    def test_detection_pauses_scan_and_requires_repeated_frames(self):
        policy = SearchPolicy(250, 0)
        for tick in range(4):
            self.assertEqual(policy.update(1+tick*0.03, 230), (None, None))
        self.assertEqual(policy.update(1.2, 230), (247, None))
        self.assertEqual(policy.update(1.3, None), (None, None))
        self.assertEqual(policy.update(2.3, None), (257, None))

    def test_tilt_direction_and_step_bound(self):
        for y, expected in [(230, 247), (130, 253)]:
            policy = SearchPolicy(250, 0)
            for tick in range(5):
                target, reason = policy.update(1+tick*0.03, y)
            self.assertEqual(target, expected)
            self.assertIsNone(reason)

    def test_center_requires_settled_repeated_observations(self):
        policy = SearchPolicy(250, 0)
        for tick in range(11):
            self.assertEqual(policy.update(1+tick*0.03, 180), (None, None))
        self.assertEqual(policy.update(1.4, 180), (None, 'centered_vertically'))

    def test_rejects_unsafe_range_and_stops_at_tracking_limit(self):
        for position in [148, 301]:
            with self.assertRaises(ValueError):
                SearchPolicy(position, 0)
        policy = SearchPolicy(149, 0)
        for tick in range(5):
            result = policy.update(1+tick*0.03, 230)
        self.assertEqual(result, (None, 'tilt_limit_before_center'))


if __name__ == '__main__':
    unittest.main()
