"""Decision checks before publishing a live box measurement."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from cargo_live_view import LiveView, SAMPLE_COUNT, average_results, gate_result, height_slot  # noqa: E402
from grid_board import Board, config_path  # noqa: E402


class LiveViewGateTest(unittest.TestCase):
    def setUp(self):
        self.result = {'width': 0.188, 'height': 0.31, 'z_cam': 0.85,
                       'face_yaw_deg': 1.0, 'lidar_width': 0.174,
                       'lidar_points': 12, 'warnings': []}
        self.status = {'x': 0.01, 'y': 0.02, 'yaw': math.radians(0.5), 'state': 'done'}

    def check(self):
        return gate_result(self.result, self.status, 0.1, (0.0, 0.0), 0.0)

    def test_valid_box_at_goal(self):
        self.assertIsNone(self.check())

    def test_rejects_motion_and_stale_pose(self):
        self.status['state'] = 'moving'
        self.assertIn('moving', self.check())
        self.status['state'] = 'done'
        self.assertIn('Waiting', gate_result(self.result, self.status, 3.0, (0, 0), 0))

    def test_rejects_nonparallel_or_misaligned_robot(self):
        self.result['face_yaw_deg'] = 3.1
        self.assertIn('angle', self.check())
        self.result['face_yaw_deg'] = 0.0
        self.status['yaw'] = math.radians(1.1)
        self.assertIn('Align', self.check())

    def test_rejects_unreliable_edges(self):
        self.result['lidar_width'] = 0.12
        self.assertIn('differs', self.check())
        self.result['lidar_width'] = 0.18
        self.result['warnings'] = ['camera cao hon hang, khong thay vach mat tren/mat truoc']
        self.assertIn('uncertain', self.check())

    def test_five_measurements_are_averaged_once(self):
        values = [dict(self.result, width=w, height=h, z_cam=z)
                  for w, h, z in ((0.18, 0.30, 0.80),
                                  (0.19, 0.31, 0.81),
                                  (0.20, 0.32, 0.82),
                                  (0.21, 0.33, 0.83),
                                  (0.22, 0.34, 0.84))]
        self.assertEqual(SAMPLE_COUNT, 5)
        with self.assertRaises(ValueError):
            average_results(values[:4])
        result = average_results(values)
        self.assertAlmostEqual(result['width'], 0.20)
        self.assertAlmostEqual(result['height'], 0.32)
        self.assertAlmostEqual(result['z_cam'], 0.82)

    def test_height_goes_to_four_equal_bins(self):
        sorting = {'min_m': 0.24, 'max_m': 0.50, 'columns': [1, 2, 3, 4]}
        self.assertIsNone(height_slot(0.2399, sorting))
        self.assertEqual(height_slot(0.24, sorting), 1)
        self.assertEqual(height_slot(0.3049, sorting), 1)
        self.assertEqual(height_slot(0.305, sorting), 2)
        self.assertEqual(height_slot(0.37, sorting), 3)
        self.assertEqual(height_slot(0.435, sorting), 4)
        self.assertEqual(height_slot(0.50, sorting), 4)
        self.assertIsNone(height_slot(0.5001, sorting))

    def test_confirmation_sends_one_goal_only_after_valid_result(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def get_subscription_count(self):
                return 1

            def publish(self, message):
                self.messages.append(message.data)

        board = Board.load(config_path(True))
        x, y = board.xy(2, 4)  # agv1 pickup (3,5)
        state = {'accepted': None, 'sent': False, 'slot': None,
                 'status': {'x': x, 'y': y, 'yaw': board.yaw, 'state': 'done'},
                 'status_at': time.monotonic()}
        node = SimpleNamespace(cars={'agv1': state}, board=board,
                               sorting={'min_m': 0.24, 'max_m': 0.50}, drop_row=1,
                               goal_pub=Publisher(), result_pub=Publisher())
        ok, _ = LiveView.confirm(node, 'agv1')
        self.assertFalse(ok)
        self.assertEqual(node.goal_pub.messages, [])
        state['accepted'] = {'width': 0.18, 'height': 0.313, 'z_cam': 0.84}
        ok, _ = LiveView.confirm(node, 'agv1')  # out-of-range result has no slot
        self.assertFalse(ok)
        state['slot'] = 2
        state['status_at'] = time.monotonic() - 3
        ok, _ = LiveView.confirm(node, 'agv1')  # stale pose cannot dispatch
        self.assertFalse(ok)
        self.assertEqual(node.goal_pub.messages, [])
        state['status_at'] = time.monotonic()
        ok, _ = LiveView.confirm(node, 'agv1')
        self.assertTrue(ok)
        self.assertEqual(node.goal_pub.messages, ['1 1 2'])
        ok, _ = LiveView.confirm(node, 'agv1')
        self.assertFalse(ok)
        self.assertEqual(node.goal_pub.messages, ['1 1 2'])


if __name__ == '__main__':
    unittest.main()
