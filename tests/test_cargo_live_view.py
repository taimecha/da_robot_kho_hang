"""Decision checks before publishing a live box measurement."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from cargo_live_view import LiveView, SAMPLE_COUNT, TARGETS, average_results, deliveries_complete, gate_result, height_slot, pickup_arg, safe_height_slot, stable_result_reason, supervised_height_slot  # noqa: E402
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
        self.result['warnings'] = []
        self.result['lidar_edge_inset_px'] = 5.5
        self.assertIn('cut off', self.check())

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

    def test_five_frames_must_agree_before_locking_or_routing(self):
        sorting = {'min_m': 0.24, 'max_m': 0.50, 'columns': [1, 2, 3, 4]}
        values = [dict(self.result, width=0.185, height=0.320,
                       z_cam=0.82, lidar_width=0.175) for _ in range(5)]
        self.assertIsNone(stable_result_reason(values))
        values[4]['height'] = 0.345
        self.assertIn('Unstable height', stable_result_reason(values))
        values[4]['height'] = 0.320
        values[4]['width'] = 0.225
        self.assertIn('Unstable width', stable_result_reason(values))
        values[4]['width'] = 0.185
        values[4]['height'] = 0.305
        for v in values[:4]:
            v['height'] = 0.304
        self.assertIsNone(stable_result_reason(values))
        self.assertIsNone(safe_height_slot(average_results(values)['height'], sorting))

    def test_verified_lower_height_is_green_and_internal_edges_need_review(self):
        sorting = {'min_m': 0.24, 'max_m': 0.50, 'columns': [1, 2, 3, 4]}
        self.assertEqual(height_slot(0.242, sorting), 1)
        self.assertEqual(safe_height_slot(0.242, sorting), 1)
        self.assertIsNone(height_slot(0.239, sorting))
        self.assertEqual(supervised_height_slot(0.239, sorting), 1)
        self.assertEqual(safe_height_slot(0.239, sorting), 1)
        self.assertIsNone(safe_height_slot(0.237, sorting))
        self.assertIsNone(supervised_height_slot(0.234, sorting))
        self.assertEqual(safe_height_slot(0.320, sorting), 2)
        self.assertIsNone(safe_height_slot(0.305, sorting))

    def test_camera_closes_only_after_both_confirmed_deliveries_arrive(self):
        board = Board.load(config_path(True))
        now = time.monotonic()
        def car(slot):
            x, y = board.xy(0, slot - 1)
            return {'sent': True, 'slot': slot, 'status_at': now,
                    'status': {'x': x, 'y': y, 'state': 'done'}}
        cars = {'agv1': car(1), 'agv2': car(2)}
        self.assertTrue(deliveries_complete(cars, board, 1, now))
        cars['agv2']['sent'] = False
        self.assertFalse(deliveries_complete(cars, board, 1, now))
        cars['agv2']['sent'] = True
        cars['agv2']['status']['state'] = 'moving'
        self.assertFalse(deliveries_complete(cars, board, 1, now))
        cars['agv2']['status']['state'] = 'done'
        cars['agv2']['status_at'] = now - 3
        self.assertFalse(deliveries_complete(cars, board, 1, now))
        cars['agv2']['status_at'] = now
        cars['agv2']['status']['x'] += 0.1
        self.assertFalse(deliveries_complete(cars, board, 1, now))

    def test_confirmation_sends_one_goal_only_after_valid_result(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def get_subscription_count(self):
                return 1

            def publish(self, message):
                self.messages.append(message.data)

        board = Board.load(config_path(True))
        self.assertEqual(TARGETS, {'agv1': (4, 5), 'agv2': (3, 5)})
        self.assertEqual(pickup_arg('3,5'), (3, 5))
        current_targets = {'agv1': (3, 5), 'agv2': (4, 5)}
        r, c = current_targets['agv1']
        x, y = board.xy(r - 1, c - 1)
        state = {'accepted': None, 'sent': False, 'slot': None,
                 'status': {'x': x, 'y': y, 'yaw': board.yaw, 'state': 'done'},
                 'status_at': time.monotonic()}
        node = SimpleNamespace(cars={'agv1': state}, board=board, targets=current_targets,
                               sorting={'min_m': 0.24, 'max_m': 0.50, 'columns': [1, 2, 3, 4]},
                               drop_row=1,
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

        # Near a height boundary the amber button is an explicit supervisor
        # override after loading; it must still publish the chosen destination.
        state.update(accepted={'width': 0.13, 'height': 0.305, 'z_cam': 0.84},
                     slot=2, near_boundary=True, sent=False,
                     status_at=time.monotonic())
        ok, message = LiveView.confirm(node, 'agv1')
        self.assertTrue(ok)
        self.assertIn('sat nguong', message)
        self.assertEqual(node.goal_pub.messages[-1], '1 1 2')


if __name__ == '__main__':
    unittest.main()
