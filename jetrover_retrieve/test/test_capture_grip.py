"""Check capture pairing and servo responses without ROS or camera hardware."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

path = Path(__file__).resolve().parents[2] / 'scripts/capture_grip.py'
spec = importlib.util.spec_from_file_location('capture_grip', path)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def frame(seconds):
    sec = int(seconds)
    timestamp = SimpleNamespace(sec=sec, nanosec=round((seconds - sec) * 1e9))
    return SimpleNamespace(header=SimpleNamespace(stamp=timestamp))


class CaptureTests(unittest.TestCase):
    def test_selects_closest_depth_for_latest_color(self):
        color = frame(100.5)
        near = frame(100.49)
        other = frame(100.42)
        result = capture.choose_images([frame(100.4), color], [near, other], 100.6)
        self.assertIs(result[0], color)
        self.assertIs(result[1], near)

    def test_rejects_stale_zero_future_and_unsynchronized_frames(self):
        for rgb, depth in [(98, 98), (0, 0), (101, 101), (100.5, 100.2)]:
            with self.subTest(rgb=rgb, depth=depth):
                self.assertEqual(capture.choose_images([frame(rgb)], [frame(depth)], 100.6),
                                 (None, None))

    def test_missing_depth_does_not_form_a_pair(self):
        self.assertEqual(capture.choose_images([frame(100.5)], [], 100.6), (None, None))

    def test_uses_previous_rgb_if_newest_has_no_matching_depth(self):
        previous = frame(100.2)
        result = capture.choose_images([previous, frame(100.5)], [frame(100.2)], 100.6)
        self.assertIs(result[0], previous)

    def test_readings_follow_request_order(self):
        response = SimpleNamespace(success=True,
            state=[SimpleNamespace(position=[value]) for value in [500, 750, 0, 325, 500, 200]])
        self.assertEqual(capture.servo_positions(response),
                         {'1': 500, '2': 750, '3': 0, '4': 325, '5': 500, '10': 200})

    def test_success_flag_does_not_override_missing_positions(self):
        response = SimpleNamespace(success=True,
            state=[SimpleNamespace(position=[]) for _ in range(6)])
        with self.assertRaises(ValueError):
            capture.servo_positions(response)
        response.state = []
        with self.assertRaises(ValueError):
            capture.servo_positions(response)


if __name__ == '__main__':
    unittest.main()
