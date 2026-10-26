"""A temporary detour is shortened after the opposing car clears the edge."""
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from grid_board import Board
from traffic_manager import Robot,TrafficManager


def manager():
    board=Board(dict(rows=5,cols=5,spacing=.6,center_row=3,center_col=3,
                     center_x=0.,center_y=0.,robot_radius=.13,
                     node_clearance=.25,edge_clearance=.2,
                     route_time_costs=dict(linear_speed_mps=.22,turn_start_s=1.8,
                         quarter_turn_s=1.6,route_node_wait_s=.4,
                         occupied_wait_s=3.,goal_wait_s=3.,opposite_wait_s=6.)))
    blue,orange=Robot('agv1',0),Robot('agv2',1)
    blue.x,blue.y=board.xy(0,3);blue.yaw=-math.pi/2
    blue.goal=(1,3);blue.goal_heading=0.;blue.final_heading=0.
    blue.route=[(0,3),(0,2),(1,2),(1,3)]
    blue.route_id=blue.ack=7;blue.mode='go';blue.state='waiting'
    blue.upto=1;blue.progress=0;blue.route_changed_at=time.time()-5
    orange.x,orange.y=board.xy(2,4)
    p=SimpleNamespace(board=board,occ_r=.43,robots={'agv1':blue,'agv2':orange},
                      static_nodes=set(),static_edges=set(),avoid=True,
                      service_holds={},delivery_queues={},next_route_id=8,
                      res={'agv1':{(0,3),(0,2)},'agv2':{(1,3)}},log=Mock(),
                      recovery=None,map_ok=True,publish_markers=Mock(),
                      cmd_pubs={'agv1':Mock(),'agv2':Mock()})
    for method in ('blocked_sets','others','remaining','heading_dir','dist','xy',
                   'plan','node_of','route_str','assign_route','in_service',
                   'occupied','parked','prio','owner','waits_for','recovery_active',
                   'on_route_done','on_trouble','try_resume'):
        setattr(p,method,getattr(TrafficManager,method).__get__(p))
    p.lbl=TrafficManager.lbl
    return p,blue


class ShortcutTests(unittest.TestCase):
    def test_detour_is_shortened_only_after_opposing_reservation_clears(self):
        p,blue=manager();now=time.time()
        self.assertFalse(TrafficManager.shorten_clear_route(p,blue,now))
        self.assertEqual(blue.route,[(0,3),(0,2),(1,2),(1,3)])
        p.res['agv2']={(2,4)}
        self.assertTrue(TrafficManager.shorten_clear_route(p,blue,now))
        self.assertEqual(blue.route,[(0,3),(1,3)])
        self.assertEqual(blue.route_id,8)
        self.assertIn((0,2),blue.transition_res)

    def test_active_motion_and_unacknowledged_route_are_not_interrupted(self):
        p,blue=manager();p.res['agv2']={(2,4)};now=time.time()
        blue.state='moving';blue.route_busy=True
        self.assertFalse(TrafficManager.shorten_clear_route(p,blue,now))
        blue.route_busy=False;blue.linear_speed=.08;blue.angular_speed=0.
        self.assertFalse(TrafficManager.shorten_clear_route(p,blue,now))
        blue.linear_speed=0.;blue.angular_speed=0.;blue.ack=6
        self.assertFalse(TrafficManager.shorten_clear_route(p,blue,now))
        blue.ack=7
        self.assertTrue(TrafficManager.shorten_clear_route(p,blue,now))

    def test_queue_vehicle_keeps_its_admitted_column(self):
        p,blue=manager();p.res['agv2']={(2,4)}
        p.delivery_queues['agv1']=dict(slot=4,entry_row=2,gate_row=1,until=time.time()+3)
        blue.goal=(0,3);blue.route[-1]=(0,3)
        self.assertFalse(TrafficManager.shorten_clear_route(p,blue,time.time()))

    def test_tick_publishes_direct_route_after_road_clears(self):
        p,blue=manager();orange=p.robots['agv2'];now=time.time()
        orange.x,orange.y=p.board.xy(1,3)
        blue.upto=0;blue.t=orange.t=now
        TrafficManager.tick(p)
        self.assertEqual(blue.route_id,7)
        orange.x,orange.y=p.board.xy(2,4)
        TrafficManager.tick(p)
        self.assertEqual(blue.route,[(0,3),(1,3)])
        self.assertIn('"route_id": 8',p.cmd_pubs['agv1'].publish.call_args.args[0].data)

    def test_bay_car_exits_straight_before_incoming_car_enters_approach(self):
        p,blue=manager();orange=p.robots['agv2'];now=time.time()
        blue.route=None;blue.goal=None;blue.state='done';blue.x,blue.y=p.board.xy(0,3)
        orange.x,orange.y=p.board.xy(2,4)
        orange.goal=(0,3);orange.route=[(2,4),(2,3),(1,3),(0,3)]
        orange.route_id=orange.ack=9;orange.mode='go';orange.state='waiting'
        orange.upto=orange.progress=0;blue.t=orange.t=now
        p.delivery_queues['agv2']=dict(slot=4,entry_row=3,gate_row=1,until=now+3)
        TrafficManager.tick(p)
        self.assertEqual(orange.upto,1)
        self.assertNotIn((1,3),p.res['agv2'])
        blue.goal=(1,3);blue.goal_heading=0.
        self.assertEqual(p.plan(blue,(0,3),blue.goal),[(0,3),(1,3)])
        p.assign_route(blue,[(0,3),(1,3)],0.,'go')
        blue.state='waiting';blue.t=time.time()
        TrafficManager.tick(p)  # The new command has not been ACKed yet.
        self.assertEqual(orange.upto,1)
        self.assertNotIn((1,3),p.res['agv2'])
        blue.ack=blue.route_id;blue.state='waiting';blue.t=time.time()
        TrafficManager.tick(p)
        self.assertIn((1,3),p.res['agv1'])
        self.assertNotIn((1,3),p.res['agv2'])
        blue.x,blue.y=p.board.xy(1,4);blue.route=None;blue.goal=None
        blue.state='done';blue.t=time.time()
        TrafficManager.tick(p)
        self.assertEqual(orange.upto,2)
        orange.x,orange.y=p.board.xy(2,3);orange.t=time.time()
        TrafficManager.tick(p)
        self.assertEqual(orange.upto,len(orange.route)-1)

    def test_bay_exit_stays_closed_when_approach_is_already_reserved(self):
        p,blue=manager();orange=p.robots['agv2'];now=time.time()
        blue.x,blue.y=p.board.xy(0,3);blue.goal=(1,3)
        orange.goal=(0,3)
        orange.route=[(2,4),(2,3),(1,3),(0,3)]
        p.delivery_queues['agv2']=dict(slot=4,entry_row=3,gate_row=1,until=now+3)
        p.res['agv2']={(2,4),(2,3),(1,3)}
        self.assertNotEqual(p.plan(blue,(0,3),blue.goal),[(0,3),(1,3)])

    def test_incoming_car_still_at_pickup_does_not_force_bay_detour(self):
        p,blue=manager();orange=p.robots['agv2'];now=time.time()
        blue.x,blue.y=p.board.xy(0,3);blue.goal=(1,3)
        orange.goal=(2,4);orange.route=[(2,4)]
        p.delivery_queues['agv2']=dict(slot=4,entry_row=3,gate_row=2,until=now+3)
        p.res['agv2']={(2,4)}
        self.assertEqual(p.plan(blue,(0,3),blue.goal),[(0,3),(1,3)])

    def test_incoming_car_does_not_reserve_approach_before_arrival_ack(self):
        p,blue=manager();orange=p.robots['agv2'];now=time.time()
        blue.x,blue.y=p.board.xy(0,3)
        blue.route=[(1,3),(0,3)];blue.goal=(0,3)
        blue.route_id=blue.ack=7;blue.progress=blue.upto=1
        blue.state='moving';blue.t=now
        orange.goal=(0,3);orange.route=[(2,4),(2,3),(1,3),(0,3)]
        orange.route_id=orange.ack=9;orange.mode='go';orange.state='waiting'
        orange.progress=orange.upto=0;orange.t=now
        TrafficManager.tick(p)
        self.assertEqual(orange.upto,1)
        self.assertNotIn((1,3),p.res['agv2'])



if __name__=='__main__':unittest.main()
