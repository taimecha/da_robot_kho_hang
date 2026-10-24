"""Decision checks before publishing a live box measurement."""
import math
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from types import MethodType
from collections import deque
from unittest.mock import patch
import numpy as np
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from cargo_live_view import LiveView, SAMPLE_COUNT, TARGETS, average_results, deliveries_complete, gate_result, height_slot, pickup_arg, safe_height_slot, stable_result_reason, supervised_height_slot  # noqa: E402
from cargo_live_view import observation_cell, display_frame
import cargo_measure as cm
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

    def test_camera_accepts_new_heading_tolerance_and_rejects_boundary(self):
        self.status['yaw']=math.radians(2.4)
        self.assertIsNone(self.check())
        self.status['yaw']=math.radians(2.5)
        self.assertIn('Align',self.check())
        self.status['yaw']=math.radians(3.0)
        self.assertIn('Align',self.check())

    def test_live_video_updates_in_all_phases_while_results_stay_locked(self):
        old = np.zeros((24,32,3), np.uint8)
        fresh = np.full_like(old, 100)
        result = dict(self.result)
        state = dict(frame=old, view=old, phase='measuring', accepted=result,
                     frame_seq=10, last_measured_frame_seq=10)
        for phase in ('ready', 'to_observe', 'measuring', 'to_pickup', 'loading',
                      'delivering', 'delivered', 'error'):
            state.update(phase=phase, frame=fresh, frame_seq=11)
            self.assertIs(display_frame(state), fresh, phase)
            self.assertEqual(state['accepted'], result)

    def test_edge_overlay_expires_when_a_new_camera_frame_arrives(self):
        raw = np.zeros((24,32,3), np.uint8)
        overlay = np.full_like(raw, 50)
        fresh = np.full_like(raw, 100)
        state = dict(frame=raw, view=overlay, phase='measuring', accepted=None,
                     frame_seq=10, last_measured_frame_seq=10)
        self.assertIs(display_frame(state), overlay)
        state.update(frame=fresh, frame_seq=11)
        self.assertIs(display_frame(state), fresh)
        state.update(frame_seq=10, phase='to_pickup')
        self.assertIs(display_frame(state), fresh)

    def test_rejects_motion_and_stale_pose(self):
        self.status['state'] = 'moving'
        self.assertIn('moving', self.check())
        self.status['state'] = 'done'
        self.assertIn('Waiting', gate_result(self.result, self.status, 3.0, (0, 0), 0))

    def test_rejects_nonparallel_or_misaligned_robot(self):
        self.result['face_yaw_deg'] = 15.1
        self.assertIn('oblique', self.check())
        self.result['face_yaw_deg'] = 0.0
        self.status['yaw'] = math.radians(2.6)
        self.assertIn('Align', self.check())

    def test_moderate_face_yaw_is_compensated_not_rejected_at_three_degrees(self):
        for angle in (-14.,-3.9,3.9,14.):
            self.result['face_yaw_deg']=angle
            self.assertIsNone(self.check())
        self.result['face_rms_m']=.02
        self.assertIn('noisy',self.check())

    def test_nonfinite_geometry_never_becomes_an_accepted_measurement(self):
        self.result['face_yaw_deg']=float('nan')
        self.assertIn('Invalid',self.check())

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

    def test_completion_requires_both_confirmed_deliveries_arrive(self):
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

    def test_delivery_success_event_is_sent_once_at_real_destination(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(json.loads(message.data))

        board = Board.load(config_path(True))
        x, y = board.xy(0, 1)
        now = time.monotonic()
        state = {'sent': True, 'slot': 2, 'delivered_at': None,
                 'pickup_route_id': 10,
                 'accepted': {'height': 0.314, 'width': 0.185, 'z_cam': 0.872},
                 'status_at': now,
                 'status': {'x': x, 'y': y, 'yaw': board.yaw, 'state': 'moving',
                            'route_id': 10}}
        node = SimpleNamespace(cars={'agv1': state}, board=board, drop_row=1,
                               result_pub=Publisher())
        self.assertFalse(LiveView.record_delivery(node, 'agv1', now))
        self.assertEqual(node.result_pub.messages, [])
        state['status']['state'] = 'done'
        self.assertFalse(LiveView.record_delivery(node, 'agv1', now))
        state['status']['route_id'] = 11
        self.assertTrue(LiveView.record_delivery(node, 'agv1', now))
        self.assertFalse(LiveView.record_delivery(node, 'agv1', now + 0.1))
        self.assertEqual(len(node.result_pub.messages), 1)
        self.assertEqual(node.result_pub.messages[0]['event'], 'delivered')
        self.assertEqual(node.result_pub.messages[0]['deliver_col'], 2)
        self.assertEqual(node.result_pub.messages[0]['route_id'], 11)

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
                 'phase': 'loading',
                 'status': {'x': x, 'y': y, 'yaw': board.yaw, 'state': 'done'},
                 'status_at': time.monotonic()}
        node = SimpleNamespace(cars={'agv1': state}, board=board, targets=current_targets,
                               sorting={'min_m': 0.24, 'max_m': 0.50, 'columns': [1, 2, 3, 4]},
                               drop_row=1,
                               pick_yaw=board.yaw, pick_heading='d',
                               goal_pub=Publisher(), result_pub=Publisher())
        node.at_cell = MethodType(LiveView.at_cell, node)
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
        self.assertEqual(node.goal_pub.messages, ['agv1 1 2'])
        ok, _ = LiveView.confirm(node, 'agv1')
        self.assertFalse(ok)
        self.assertEqual(node.goal_pub.messages, ['agv1 1 2'])

        # Near a height boundary the amber button is an explicit supervisor
        # override after loading; it must still publish the chosen destination.
        state.update(accepted={'width': 0.13, 'height': 0.305, 'z_cam': 0.84},
                     slot=2, near_boundary=True, sent=False, phase='loading',
                     status_at=time.monotonic())
        ok, message = LiveView.confirm(node, 'agv1')
        self.assertTrue(ok)
        self.assertIn('sat nguong', message)
        self.assertEqual(node.goal_pub.messages[-1], 'agv1 1 2')


class CargoCycleTest(unittest.TestCase):
    def setUp(self):
        class Publisher:
            def __init__(self):
                self.messages = []
            def get_subscription_count(self):
                return 1
            def publish(self, msg):
                self.messages.append(msg.data)
        self.now = time.monotonic()
        board = Board.load(config_path(True))
        cfg = cm.load_cfg()
        self.node = SimpleNamespace(
            board=board, targets={'agv1': (3,5), 'agv2': (4,5)},
            observations={'agv1': (3,4), 'agv2': (4,4)},
            pick_yaw=board.yaw, pick_heading='d', drop_row=1,
            sorting=cfg['height_sort'], goal_pub=Publisher(), result_pub=Publisher(), cars={})
        for car in self.node.targets:
            self.node.cars[car] = dict(
                phase='ready', status=None, status_at=self.now, cycle=0,
                history=deque(maxlen=5), scans=deque(maxlen=10),
                frame=np.zeros((480,640,3),np.uint8), frame_seq=0,
                last_measured_frame_seq=0, last_process=0., frame_at=self.now,
                cfg=cm.car_cfg(cfg,car), sent=False, slot=None,
                accepted=None, delivered_at=None, leg_route_id=None, action_note='')
        for name in ('at_cell', 'send_leg', 'advance', 'start_cycle', 'action',
                     'confirm', 'record_delivery', 'process', 'reset'):
            setattr(self.node,name,MethodType(getattr(LiveView,name),self.node))
        self.pose('agv1',(3,3),10)
        self.pose('agv2',(4,3),20)

    def pose(self, car, cell, route, state='done'):
        x,y=self.node.board.xy(cell[0]-1,cell[1]-1)
        self.node.cars[car].update(status=dict(x=x,y=y,yaw=self.node.board.yaw,
                                             state=state,route_id=route),
                                   status_at=time.monotonic())

    def measure_box(self, height):
        s=self.node.cars['agv1']
        result=dict(width=.18,height=height,z_cam=.82,face_yaw_deg=0.,
                    lidar_width=.18,lidar_points=12,warnings=[])
        with patch('cargo_live_view.cm.measure',return_value=(result,s['frame'].copy())):
            for _ in range(5):
                now=time.monotonic()
                s.update(frame_seq=s['frame_seq']+1,frame_at=now,last_process=0.)
                s['scans'].clear()
                s['scans'].extend([(now,0.,.01,[1.]*10)]*s['cfg']['lidar']['scans'])
                self.node.process('agv1',now)

    def test_two_cycles_same_car_can_deliver_different_classes(self):
        n=self.node; s=n.cars['agv1']
        self.assertTrue(n.action('agv1')[0])
        self.assertEqual(n.goal_pub.messages,['agv1 3 4 d'])
        self.pose('agv1',(3,4),10)  # stale previous route cannot trigger measurement
        n.advance('agv1',time.monotonic())
        self.assertEqual(s['phase'],'to_observe')
        self.pose('agv1',(3,4),11)
        n.advance('agv1',time.monotonic())
        self.assertEqual(s['phase'],'measuring')
        self.measure_box(.32)
        self.assertEqual(s['phase'],'to_pickup')
        self.assertEqual(n.goal_pub.messages[-1],'agv1 3 5 d')
        self.assertFalse(n.confirm('agv1')[0])
        self.pose('agv1',(3,5),12)
        n.advance('agv1',time.monotonic())
        self.assertEqual(s['phase'],'loading')
        self.assertEqual(len(n.goal_pub.messages),2)  # never dispatch before supervisor
        self.assertTrue(n.confirm('agv1')[0])
        self.assertFalse(n.confirm('agv1')[0])
        self.assertEqual(n.goal_pub.messages[-1],'agv1 1 2')
        self.pose('agv1',(1,2),13)
        self.assertTrue(n.record_delivery('agv1',time.monotonic()))
        self.assertFalse(n.record_delivery('agv1',time.monotonic()))
        self.assertEqual(s['phase'],'delivered')
        self.assertTrue(n.action('agv1')[0])
        self.assertIsNone(s['accepted'])
        self.assertIsNone(s['slot'])
        self.assertEqual(s['cycle'],2)
        self.pose('agv1',(3,4),14)
        n.advance('agv1',time.monotonic())
        self.measure_box(.40)
        self.assertEqual(s['slot'],3)
        self.pose('agv1',(3,5),15)
        n.advance('agv1',time.monotonic())
        self.assertTrue(n.confirm('agv1')[0])
        self.assertEqual(n.goal_pub.messages[-1],'agv1 1 3')

    def test_preview_of_four_good_frames_cannot_dispatch_pickup(self):
        n=self.node;s=n.cars['agv1']
        n.action('agv1');self.pose('agv1',(3,4),11)
        n.advance('agv1',time.monotonic())
        result=dict(width=.185,height=.32,z_cam=.89,face_yaw_deg=-3.9,
                    lidar_width=.18,lidar_points=10,warnings=[])
        with patch('cargo_live_view.cm.measure',return_value=(result,s['frame'].copy())):
            for _ in range(4):
                now=time.monotonic()
                s.update(frame_seq=s['frame_seq']+1,frame_at=now,last_process=0.)
                s['scans'].clear()
                s['scans'].extend([(now,0.,.01,[1.]*10)]*s['cfg']['lidar']['scans'])
                n.process('agv1',now)
        self.assertIsNotNone(s['candidate'])
        self.assertIsNone(s['accepted'])
        self.assertEqual(len(s['history']),4)
        self.assertEqual(n.goal_pub.messages,['agv1 3 4 d'])

    def test_invalid_height_does_not_dispatch_pickup(self):
        self.node.action('agv1')
        self.pose('agv1',(3,4),11)
        self.node.advance('agv1',time.monotonic())
        self.measure_box(.60)
        self.assertIsNone(self.node.cars['agv1']['slot'])
        self.assertEqual(self.node.goal_pub.messages,['agv1 3 4 d'])

    def test_same_class_is_reserved_across_both_cars(self):
        n=self.node
        self.pose('agv1',(3,5),11)
        self.pose('agv2',(4,5),21)
        for s in n.cars.values():
            s.update(phase='loading',accepted=dict(width=.18,height=.32,z_cam=.82),slot=2)
        self.assertTrue(n.confirm('agv1')[0])
        self.assertFalse(n.confirm('agv2')[0])
        self.assertEqual(len(n.goal_pub.messages),1)

    def test_pickup_requires_two_cm_and_under_two_point_five_degrees(self):
        n=self.node; s=n.cars['agv1']
        self.pose('agv1',(3,5),11)
        s.update(phase='loading',accepted=dict(width=.18,height=.32,z_cam=.82),slot=2)
        x=s['status']['x']
        s['status']['x']=x+.025
        self.assertFalse(n.confirm('agv1')[0])
        s['status']['x']=x
        s['status']['yaw']=n.pick_yaw+math.radians(2.6)
        self.assertFalse(n.confirm('agv1')[0])
        s['status']['yaw']=n.pick_yaw
        self.assertTrue(n.confirm('agv1')[0])
        self.assertEqual(n.reset(),[])  # cannot erase an item already in transit

    def test_both_cars_can_carry_the_same_or_different_class_over_time(self):
        n=self.node
        self.pose('agv1',(3,5),11)
        self.pose('agv2',(4,5),21)
        n.cars['agv1'].update(phase='loading',accepted=dict(width=.18,height=.32,z_cam=.82),slot=2)
        n.cars['agv2'].update(phase='loading',accepted=dict(width=.18,height=.40,z_cam=.82),slot=3)
        self.assertTrue(n.confirm('agv1')[0])
        self.assertTrue(n.confirm('agv2')[0])
        self.assertEqual(n.goal_pub.messages,['agv1 1 2','agv2 1 3'])

    def test_delivery_keeps_route_heading_instead_of_forcing_east(self):
        n=self.node;s=n.cars['agv1']
        self.pose('agv1',(3,5),11)
        s.update(phase='loading',accepted=dict(width=.18,height=.32,z_cam=.82),slot=2)
        self.assertTrue(n.confirm('agv1')[0])
        self.assertEqual(n.goal_pub.messages[-1],'agv1 1 2')
        self.assertIsNone(s['delivery_yaw'])
        self.pose('agv1',(1,2),12)
        s['status']['yaw']=-math.pi/2  # South is the final edge, not camera East.
        self.assertTrue(n.record_delivery('agv1',time.monotonic()))

    def test_delivery_retry_keeps_the_confirmed_item_and_only_resends_its_leg(self):
        from cargo_live_view import navigation_error
        n=self.node;s=n.cars['agv1']
        accepted=dict(width=.18,height=.32,z_cam=.82)
        s.update(phase='delivering',accepted=accepted,slot=2,sent=True)
        self.pose('agv1',(1,2),30,state='idle')
        navigation_error(s,'sai so vi tri tai dich 3.2 cm')
        navigation_error(s,'navigation error repeated')
        self.assertEqual(s['failed_phase'],'delivering')
        self.assertTrue(n.action('agv1')[0])
        self.assertEqual(s['phase'],'delivering')
        self.assertEqual(n.goal_pub.messages,['agv1 1 2'])
        self.assertIs(s['accepted'],accepted)
        self.assertTrue(s['sent'])
        self.assertEqual(s['pickup_route_id'],30)

    def test_resumed_delivery_does_not_invent_measurements_or_dispatch_on_open(self):
        n=self.node;s=n.cars['agv1']
        s.update(phase='delivery_ready',slot=3,sent=True,resuming_delivery=True)
        self.assertEqual(n.goal_pub.messages,[])
        self.assertTrue(n.action('agv1')[0])
        self.assertEqual(n.goal_pub.messages,['agv1 1 3'])
        self.pose('agv1',(1,3),31)
        self.assertTrue(n.record_delivery('agv1',time.monotonic()))
        import json
        event=json.loads(n.result_pub.messages[-1])
        self.assertTrue(event['delivered'])
        self.assertTrue(event['resumed_delivery'])
        self.assertIsNone(event['height_m'])
        self.assertIsNone(event['width_m'])
        self.assertTrue(n.action('agv1')[0])
        self.assertFalse(s['resuming_delivery'])

    def test_delivery_success_rejects_wrong_heading_and_excess_position_error(self):
        n=self.node; s=n.cars['agv1']
        s.update(sent=True,phase='delivering',slot=2,pickup_route_id=10,
                 delivery_yaw=n.pick_yaw,accepted=dict(width=.18,height=.32,z_cam=.82))
        self.pose('agv1',(1,2),11)
        s['status']['yaw']=n.pick_yaw+math.radians(2.6)
        self.assertFalse(n.record_delivery('agv1',time.monotonic()))
        s['status']['yaw']=n.pick_yaw
        s['status']['x']+=.025
        self.assertFalse(n.record_delivery('agv1',time.monotonic()))
        self.pose('agv1',(1,2),11)
        self.assertTrue(n.record_delivery('agv1',time.monotonic()))

    def test_offset_is_one_step_in_facing_direction(self):
        b=self.node.board
        self.assertEqual(observation_cell((3,5),'d',b),(3,4))
        self.assertEqual(observation_cell((3,3),'b',b),(2,3))
        self.assertEqual(observation_cell((3,3),'t',b),(3,4))
        with self.assertRaises(ValueError):
            observation_cell((3,1),'d',b)

    def test_agv2_alone_uses_name_in_every_goal_without_agv1(self):
        n=self.node
        n.targets={'agv2': (4,5)}
        n.observations={'agv2': (4,4)}
        n.cars={'agv2': n.cars['agv2']}
        s=n.cars['agv2']
        self.assertTrue(n.action('agv2')[0])
        self.assertEqual(n.goal_pub.messages,['agv2 4 4 d'])
        self.pose('agv2',(4,4),21)
        n.advance('agv2',time.monotonic())
        s.update(accepted=dict(width=.18,height=.32,z_cam=.82),slot=2)
        self.assertTrue(n.action('agv2')[0])
        self.assertEqual(n.goal_pub.messages[-1],'agv2 4 5 d')
        self.pose('agv2',(4,5),22)
        n.advance('agv2',time.monotonic())
        self.assertTrue(n.action('agv2')[0])
        self.assertEqual(n.goal_pub.messages[-1],'agv2 1 2')
        self.pose('agv2',(1,2),23)
        self.assertTrue(n.record_delivery('agv2',time.monotonic()))
        self.assertTrue(n.action('agv2')[0])
        self.assertEqual(n.goal_pub.messages[-1],'agv2 4 4 d')

    def test_precision_failure_displays_position_error_and_blocks_loading(self):
        n=self.node; s=n.cars['agv1']
        n.action('agv1')
        s['status'].update(state='precision_failed', error='sai so vi tri tai dich 2.8 cm > 2 cm')
        n.advance('agv1',time.monotonic())
        self.assertEqual(s['phase'],'error')
        self.assertIn('Position error 2.8 cm',s['reason'])
        self.assertFalse(n.confirm('agv1')[0])
        self.assertEqual(n.goal_pub.messages,['agv1 3 4 d'])

    def test_original_precision_log_is_not_hidden_by_generic_stop_log(self):
        n=self.node; s=n.cars['agv1'];n.action('agv1')
        LiveView.route_log(n,SimpleNamespace(name='agv1.fleet_agent',level=40,
            msg='[agv1] sai so vi tri tai dich 2.8 cm > 2 cm sau 2 lan chinh'))
        LiveView.route_log(n,SimpleNamespace(name='traffic_manager',level=30,
            msg='[agv1] dung: sai so vi tri tai dich > 2 cm'))
        self.assertEqual(s['phase'],'error')
        self.assertIn('2.8 cm',s['reason'])


if __name__ == '__main__':
    unittest.main()
