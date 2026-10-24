"""Shared-bay access, staged waiting and directional exit safety."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from delivery_queue import DeliveryQueue, bay_exit_clear
from grid_board import Board
from warehouse_queue import parse_orders


def board(**extra):
    cfg=dict(rows=5,cols=5,spacing=.6,center_row=3,center_col=3,
             center_x=0.,center_y=0.,robot_radius=.13,node_clearance=.25,edge_clearance=.2)
    return Board(dict(cfg,**extra))


class DeliveryQueueTests(unittest.TestCase):
    def test_ready_order_is_fifo_not_robot_name_and_duplicates_do_not_reorder(self):
        q=DeliveryQueue()
        for i,n in enumerate(('agv3','agv1','agv2')):q.enqueue(n,2,i)
        self.assertFalse(q.enqueue('agv3',2,99))
        self.assertFalse(q.admit('agv1'))
        self.assertTrue(q.admit('agv3'))
        self.assertFalse(q.admit('agv1'))
        q.release('agv3')
        self.assertTrue(q.admit('agv1'))
        q.release('agv1')
        self.assertTrue(q.admit('agv2'))

    def test_only_owner_can_release_and_different_bays_work_independently(self):
        q=DeliveryQueue();q.enqueue('a',1,0);q.enqueue('b',1,1);q.enqueue('c',3,2)
        q.admit('a');self.assertTrue(q.admit('c'))
        with self.assertRaises(ValueError):q.release('b')
        with self.assertRaises(ValueError):q.enqueue('a',4,3)

    def test_clearance_releases_bay_before_owner_returns_home(self):
        b=board();x,y=b.xy(0,1)
        self.assertFalse(bay_exit_clear((x+.44,y,0),b,2))
        self.assertTrue(bay_exit_clear((x+.46,y,0),b,2))
        self.assertFalse(bay_exit_clear((x-.6,y,math.pi),b,2))
        self.assertFalse(bay_exit_clear((x,y+.6,0),b,2))

    def test_clearance_follows_rotated_grid(self):
        b=board(yaw_deg=30);x,y=b.xy(0,2)
        self.assertTrue(bay_exit_clear((x+.46*math.cos(b.yaw),y+.46*math.sin(b.yaw),0),b,3))

    def test_waiting_slots_are_distinct_leave_egress_and_camera_stations_clear(self):
        b=board();q=DeliveryQueue()
        for n in ('a','b','c'):q.enqueue(n,4,0)
        q.admit('a')
        self.assertEqual(q.assign_staging('b',b),(2,3))
        self.assertEqual(q.assign_staging('c',b),(3,3))
        for r,c in q.staging.values():self.assertGreater(r,1);self.assertLess(c,4)

    def test_next_waiter_advances_before_current_owner_returns(self):
        b=board();q=DeliveryQueue()
        for n in ('a','b','c'):q.enqueue(n,2,0)
        q.admit('a');q.assign_staging('b',b);q.assign_staging('c',b)
        q.release('a');q.admit('b')
        self.assertEqual(q.assign_staging('c',b,promote=True),(3,2))
        x,y=b.xy(1,1)
        self.assertFalse(q.release_staging_when_clear('b',(x,y-.44,0),b))
        self.assertTrue(q.release_staging_when_clear('b',(x,y-.46,0),b))
        self.assertEqual(q.assign_staging('c',b,promote=True),(2,2))

    def test_no_waiting_space_leaves_vehicle_unassigned(self):
        b=board();q=DeliveryQueue();q.enqueue('a',1,0)
        blocked={(r,c) for r in range(2,6) for c in range(1,4)}
        self.assertIsNone(q.assign_staging('a',b,blocked))

    def test_one_way_service_row_and_default_grid(self):
        b=board();self.assertIn((0,0),list(b.neighbours(0,1)))
        b=board(forbidden_directions=[[[1,2],[1,1]],[[1,2],[2,2]]])
        self.assertNotIn((0,0),list(b.neighbours(0,1)))
        self.assertNotIn((1,1),list(b.neighbours(0,1)))
        self.assertIn((0,1),list(b.neighbours(1,1)))
        self.assertIn((0,2),list(b.neighbours(0,1)))
        with self.assertRaises(ValueError):board(forbidden_directions=[[[1,1],[3,1]]])

    def test_planned_return_clears_the_entire_service_row_before_heading_north(self):
        from types import SimpleNamespace
        from traffic_manager import Robot,TrafficManager
        rules=[[[1,c+1],[1,c]] for c in range(1,5)]
        rules += [[[1,c],[2,c]] for c in range(1,5)]
        b=board(forbidden_directions=rules)
        planner=SimpleNamespace(board=b,avoid=False,blocked_sets=lambda:(set(),set()),
                                xy=lambda node:b.xy(*node))
        planner.heading_dir=TrafficManager.heading_dir.__get__(planner)
        planner.dist=TrafficManager.dist.__get__(planner)
        robot=Robot('test',0);robot.x,robot.y=b.xy(0,1);robot.yaw=0.
        path=TrafficManager.plan(planner,robot,(0,1),(1,0))
        self.assertEqual(path[:5],[(0,1),(0,2),(0,3),(0,4),(1,4)])
        self.assertEqual(path[-1],(1,0))

    def test_custom_orders_are_validated_and_do_not_change_defaults(self):
        self.assertEqual(parse_orders('agv1:2;agv2:2;agv3:2'),
                         dict(agv1=[2],agv2=[2],agv3=[2]))
        for text in ('agv1:0','agv4:1','agv1:1;agv1:2','agv1:'):
            with self.assertRaises(ValueError):parse_orders(text)
        self.assertEqual(parse_orders('',1)['agv1'],[1])
        self.assertEqual(parse_orders('')['agv1'],[1,4,2,3])


if __name__=='__main__':unittest.main()
