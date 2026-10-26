"""Straight delivery columns: spatial joining, progressive permissions, safe exit."""
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from delivery_queue import DeliveryQueue,column_controls,bay_exit_clear
from grid_board import Board
from traffic_manager import TrafficManager,Robot
from warehouse_scene import Warehouse


def board():
    return Board(dict(rows=5,cols=5,spacing=.6,center_row=3,center_col=3,
        center_x=0.,center_y=0.,robot_radius=.13,node_clearance=.25,edge_clearance=.2,
        column_queues=True,delivery_flow='flexible',
        goal_approaches=[[[1,c],[2,c]] for c in range(1,5)]))


def planner():
    p=SimpleNamespace(board=board(),avoid=False,delivery_queues={},
        blocked_sets=lambda:(set(),set()),xy=None,robots={},static_nodes=set())
    for name in ('xy','dist','heading_dir','others','goal_error','plan'):
        setattr(p,name,getattr(TrafficManager,name).__get__(p))
    p.lbl=TrafficManager.lbl
    r=Robot('agv1',0);r.x,r.y=p.board.xy(3,4);r.yaw=0.;r.goal=(0,0);r.goal_heading=-math.pi/2;r.mode='go'
    p.robots[r.name]=r
    return p,r


class ColumnControlsTests(unittest.TestCase):
    def setUp(self):
        self.b=board();self.q=DeliveryQueue();self.entered=set()
        self.poses={n:(*self.b.xy(i+1,4),0.) for i,n in enumerate(('a','b','c'))}
        for n in self.poses:self.q.enqueue(n,1,0)
        self.entries={'a':2,'b':3,'c':4}

    def controls(self):
        return column_controls(self.q,self.b,self.poses,self.entries,self.entered)

    def test_initial_three_go_to_the_same_column_and_stay_behind_the_leader(self):
        self.q.admit('a');c=self.controls()
        self.assertEqual([c[n]['entry_row'] for n in self.poses],[2,3,4])
        self.assertEqual([c[n]['gate_row'] for n in self.poses],[1,3,4])
        self.assertEqual({d['slot'] for d in c.values()},{1})

    def test_followers_advance_as_the_preceding_vehicle_physically_clears(self):
        self.q.admit('a')
        self.poses['a']=(*self.b.xy(0,0),-math.pi/2)
        self.poses['b']=(*self.b.xy(2,0),-math.pi/2)
        self.poses['c']=(*self.b.xy(3,0),-math.pi/2)
        self.assertEqual([self.controls()[n]['gate_row'] for n in self.poses],[1,2,4])
        self.poses['b']=(*self.b.xy(1,0),-math.pi/2)
        self.assertEqual(self.controls()['c']['gate_row'],3)

    def test_gate_does_not_force_follower_to_enter_at_a_distant_row(self):
        q=DeliveryQueue()
        for n in ('leader','follower'):q.enqueue(n,1,0)
        q.admit('leader')
        poses={'leader':(*self.b.xy(2,0),-math.pi/2),
               'follower':(*self.b.xy(1,4),math.pi)}
        controls=column_controls(q,self.b,poses,{'leader':3},{'leader'})
        self.assertEqual(controls['follower']['gate_row'],4)
        self.assertEqual(controls['follower']['entry_row'],2)

    def test_third_car_waits_outside_nearby_row_instead_of_touring_perimeter(self):
        q=DeliveryQueue()
        for n in ('owner','second','third'):q.enqueue(n,2,0)
        q.admit('owner')
        poses={'owner':(*self.b.xy(0,1),-math.pi/2),
               'second':(*self.b.xy(3,1),-math.pi/2),
               'third':(*self.b.xy(2,4),math.pi)}
        controls=column_controls(q,self.b,poses,{'owner':3,'second':4},
                                 {'owner','second'})
        self.assertEqual(controls['third']['gate_row'],5)
        self.assertEqual(controls['third']['entry_row'],3)

    def test_releasing_bay_opens_next_route_without_a_staging_goal(self):
        self.q.admit('a');self.poses['a']=(*self.b.xy(0,0),-math.pi/2);self.controls()
        self.q.release('a');self.q.admit('b')
        c=self.controls();self.assertNotIn('a',c);self.assertEqual(c['b']['gate_row'],1)

    def test_flexible_exit_releases_bay_after_clearance_in_either_direction(self):
        x,y=self.b.xy(0,0)
        self.assertTrue(bay_exit_clear((x,y+.6,0.),self.b,1))
        self.assertTrue(bay_exit_clear((x+.46,y,0.),self.b,1))

    def test_known_pickups_register_front_to_back_without_waiting_for_all_to_load(self):
        q=DeliveryQueue()
        w=SimpleNamespace(lanes={'c':None,'b':None,'a':None},
            phase={'c':'wait_bay','b':'loading','a':'pickup'},locked={n:{'kind':1} for n in self.poses},
            delivery=q,board=self.b,pose=self.poses,event=Mock(),
            column_orders=set(),column_reported=set())
        Warehouse.register_column_orders(w,10)
        self.assertEqual(list(q.waiting[1]),['a','b','c'])
        self.assertTrue(q.can_admit('a'))


class ColumnPlannerTests(unittest.TestCase):
    def test_nearby_entry_keeps_short_route_while_gate_is_closed(self):
        p,r=planner();r.x,r.y=p.board.xy(2,4);r.goal=(0,1)
        p.delivery_queues[r.name]=dict(slot=2,entry_row=3,gate_row=5,
                                       until=time.time()+3)
        path=p.plan(r,(2,4),r.goal)
        self.assertEqual(path,[(2,4),(2,3),(2,2),(2,1),(1,1),(0,1)])
        self.assertFalse(TrafficManager.column_allows(p,r,(2,1)))
        self.assertTrue(TrafficManager.column_allows(p,r,(2,2)))

    def test_route_joins_at_assigned_row_then_only_follows_that_column(self):
        p,r=planner()
        p.delivery_queues[r.name]=dict(slot=1,entry_row=4,gate_row=4,until=time.time()+3)
        path=p.plan(r,(3,4),(0,0))
        self.assertEqual(path,[(3,4),(3,3),(3,2),(3,1),(3,0),(2,0),(1,0),(0,0)])

    def test_next_vehicle_gets_same_goal_but_no_permission_to_enter_early(self):
        p,r=planner();other=Robot('agv2',1);other.goal=(0,0);other.mode='go';p.robots[other.name]=other
        self.assertIsNotNone(p.goal_error(r,(0,0)))
        p.delivery_queues={n:dict(slot=1,entry_row=i+2,gate_row=i+1,until=time.time()+3)
                           for i,n in enumerate(p.robots)}
        self.assertIsNone(p.goal_error(r,(0,0)))
        self.assertFalse(TrafficManager.column_allows(p,other,(0,0)))
        self.assertTrue(TrafficManager.column_allows(p,other,(1,0)))

    def test_stale_permission_never_opens_the_delivery_bay(self):
        p,r=planner();p.delivery_queues[r.name]=dict(slot=1,entry_row=4,gate_row=1,until=0.)
        self.assertFalse(TrafficManager.column_allows(p,r,(0,0)))
        self.assertTrue(TrafficManager.column_allows(p,r,(3,1)))

    def test_return_path_never_enters_an_active_column_against_incoming_traffic(self):
        p,r=planner();r.goal=(1,3);r.x,r.y=p.board.xy(0,1)
        p.delivery_queues['agv2']=dict(slot=2,entry_row=3,gate_row=2,until=time.time()+3)
        path=p.plan(r,(0,1),r.goal)
        self.assertNotEqual(path[1],(1,1))

    def test_departure_goal_is_off_the_service_row_and_can_leave_sideways(self):
        p,r=planner();r.goal=(1,1);r.x,r.y=p.board.xy(0,0)
        p.delivery_queues[r.name]=dict(slot=1,entry_row=2,gate_row=1,until=time.time()+3.)
        self.assertEqual(p.plan(r,(0,0),r.goal),[(0,0),(0,1),(1,1)])

    def test_empty_column_allows_short_direct_return_north(self):
        p,r=planner();r.goal=(3,0);r.goal_heading=math.pi/2;r.x,r.y=p.board.xy(0,0)
        p.delivery_queues[r.name]=dict(slot=1,entry_row=2,gate_row=1,until=time.time()+3.)
        self.assertEqual(p.plan(r,(0,0),r.goal),[(0,0),(1,0),(2,0),(3,0)])

    def test_waiting_follower_allows_west_exit_instead_of_forcing_east(self):
        p,r=planner();r.goal=(3,3);r.goal_heading=0.;r.x,r.y=p.board.xy(0,3);r.yaw=-math.pi/2
        p.delivery_queues['agv2']=dict(slot=4,entry_row=3,gate_row=2,until=time.time()+3.)
        path=p.plan(r,(0,3),r.goal)
        self.assertEqual(path[1],(0,2))
        self.assertEqual(path[-1],r.goal)

    def test_time_costs_include_the_final_camera_heading(self):
        p,r=planner();r.goal=(3,3);r.goal_heading=0.;r.x,r.y=p.board.xy(0,3);r.yaw=-math.pi/2
        p.board.cfg['route_time_costs']=dict(linear_speed_mps=.22,turn_start_s=1.8,quarter_turn_s=1.6,
            route_node_wait_s=.4,occupied_wait_s=3.,goal_wait_s=3.,opposite_wait_s=6.)
        p.delivery_queues['agv2']=dict(slot=4,entry_row=3,gate_row=2,until=time.time()+3.)
        path=p.plan(r,(0,3),r.goal)
        self.assertEqual(path[1],(0,2))
        self.assertEqual(path[-2:],[(3,2),(3,3)])

    def test_continuous_return_has_no_intermediate_exit_goal(self):
        w=SimpleNamespace(lanes={'agv1':SimpleNamespace(row=2,front={'id':'next'})},
                          args=SimpleNamespace(sync_rounds=False),go=Mock())
        Warehouse.return_after_delivery(w,'agv1')
        w.go.assert_called_once_with('agv1','returning',2,4,'d')

    def test_opening_queue_gate_extends_same_route_without_replanning_or_pivot_reset(self):
        from test_traffic_three_robot_recovery import manager
        p=manager();a,b,c=p.robots.values()
        for rb,node in ((a,(3,0)),(b,(4,3)),(c,(4,4))):
            rb.x,rb.y=p.board.xy(*node);rb.route=None;rb.goal=None;rb.state='done';rb.mode='idle'
        a.route=[(3,0),(2,0),(1,0),(0,0)];a.goal=(0,0)
        a.mode='go';a.state='waiting';a.upto=a.progress=0
        p.delivery_queues={a.name:dict(slot=1,entry_row=4,gate_row=4,until=time.time()+3.)}
        route_id=a.route_id
        TrafficManager.tick(p)
        self.assertEqual(a.upto,0)
        self.assertIsNone(p.recovery)
        p.delivery_queues[a.name]['gate_row']=2
        TrafficManager.tick(p)
        self.assertEqual(a.upto,2)
        self.assertEqual(a.route_id,route_id)
        self.assertEqual(a.route,[(3,0),(2,0),(1,0),(0,0)])


if __name__=='__main__':unittest.main()
