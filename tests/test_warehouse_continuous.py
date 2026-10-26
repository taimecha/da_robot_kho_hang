"""Continuous collection/delivery, unbroken bay approach, safe receiving belts."""
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from grid_board import Board
from traffic_manager import TrafficManager,Robot
from delivery_queue import DeliveryQueue,bay_exit_clear
from warehouse_queue import DeliveryOutfeed,DECK_Z,Lane
from warehouse_scene import Warehouse
from warehouse_telemetry import TelemetryGuard


def board(**extra):
    return Board(dict(rows=5,cols=5,spacing=.6,center_row=3,center_col=3,
        center_x=0.,center_y=0.,robot_radius=.13,node_clearance=.25,edge_clearance=.2,
        delivery_flow='flexible',**extra))


def planner(b):
    p=SimpleNamespace(board=b,avoid=False,blocked_sets=lambda:(set(),set()),xy=lambda n:b.xy(*n))
    p.heading_dir=TrafficManager.heading_dir.__get__(p)
    p.dist=TrafficManager.dist.__get__(p)
    return p


class ContinuousTests(unittest.TestCase):
    def test_named_demo30_does_not_silently_enable_a_group_barrier(self):
        import os
        import tempfile
        from unittest.mock import patch
        import warehouse_launch
        from warehouse_queue import DEMO_30
        class Prepared(Exception):pass
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(sys,'argv',['warehouse_launch.py','up','--scenario','demo30','--out',folder]), \
                patch.dict(os.environ,{'ROS_DOMAIN_ID':'88'}), \
                patch.object(warehouse_launch,'owner',return_value=None), \
                patch.object(warehouse_launch,'prepare',side_effect=Prepared) as prepare:
            with self.assertRaises(Prepared):warehouse_launch.main()
            self.assertEqual(prepare.call_args.args[2],DEMO_30)
            self.assertFalse(prepare.call_args.args[4])
            self.assertEqual(prepare.call_args.args[5],'flexible')

    def test_free_bay_is_one_delivery_goal_not_an_approach_stop(self):
        w=SimpleNamespace(locked={'agv1':{'kind':3}},delivery=DeliveryQueue(),event=Mock(),go=Mock(),
                          unavailable_nodes=lambda n:set(),phase={'agv1':'wait_bay'})
        Warehouse.dispatch_delivery(w,'agv1',1)
        w.go.assert_called_once_with('agv1','deliver',1,3,'n')
        self.assertEqual(w.delivery.owners,{3:'agv1'})

    def test_waiter_uses_nearby_slot_outside_owner_corridor_and_does_not_shuffle(self):
        b=board();q=DeliveryQueue();q.enqueue('owner',1,0);q.admit('owner')
        w=SimpleNamespace(locked={'waiter':{'kind':1}},delivery=q,event=Mock(),go=Mock(),
            unavailable_nodes=lambda n:set(),board=b,
            pose={'owner':(*b.xy(3,4),0.),'waiter':(*b.xy(1,4),0.)},
            phase={'owner':'deliver','waiter':'wait_bay'})
        Warehouse.dispatch_delivery(w,'waiter',1)
        self.assertEqual(q.staging['waiter'],(3,2))
        w.go.assert_called_once_with('waiter','staging',3,2,'')
        w.phase.update(owner='unload',waiter='queued')
        w.go.reset_mock()
        Warehouse.dispatch_delivery(w,'waiter',2)
        self.assertEqual(q.staging['waiter'],(3,2))
        w.go.assert_not_called()

    def test_free_bay_does_not_replace_a_staging_route_halfway_along_an_edge(self):
        q=DeliveryQueue();q.enqueue('waiter',1,0);q.staging['waiter']=(3,2)
        w=SimpleNamespace(locked={'waiter':{'kind':1}},delivery=q,event=Mock(),go=Mock(),
            phase={'waiter':'staging'},reached=Mock(return_value=False),
            unavailable_nodes=lambda n:set())
        Warehouse.dispatch_delivery(w,'waiter',1)
        w.go.assert_not_called()
        self.assertEqual(q.owners,{})
        w.reached.return_value=True
        Warehouse.dispatch_delivery(w,'waiter',2)
        w.go.assert_called_once_with('waiter','deliver',1,1,'n')
        self.assertEqual(q.owners,{1:'waiter'})

    def test_any_of_the_three_initial_owners_leaves_distinct_nearby_waiting_slots(self):
        b=board();names=('agv1','agv2','agv3')
        for owner_index,owner in enumerate(names):
            with self.subTest(owner=owner):
                q=DeliveryQueue();q.enqueue(owner,1,0);q.admit(owner)
                w=SimpleNamespace(locked={n:{'kind':1} for n in names},delivery=q,
                    event=Mock(),go=Mock(),unavailable_nodes=lambda n:set(),board=b,
                    pose={n:(*b.xy(i+1,4),0.) for i,n in enumerate(names)},
                    phase={n:'deliver' if n==owner else 'wait_bay' for n in names})
                for n in names:
                    if n!=owner:Warehouse.dispatch_delivery(w,n,1)
                self.assertEqual(len(set(q.staging.values())),2)
                for r,c in q.staging.values():
                    self.assertGreaterEqual(r,3)
                    self.assertLessEqual(c,3)
                    self.assertNotEqual(r,owner_index+2)
                    self.assertFalse(c==1 and r<=owner_index+2)
                if owner=='agv3':
                    self.assertEqual(q.staging,{'agv1':(3,2),'agv2':(3,3)})

    def test_each_robot_returns_to_its_own_observation_without_a_group_barrier(self):
        w=SimpleNamespace(lanes={n:SimpleNamespace(row=i+2,front={'id':'next'})
                          for i,n in enumerate(('agv1','agv2','agv3'))},
                          args=SimpleNamespace(sync_rounds=False),go=Mock(),
                          board=SimpleNamespace(cfg={}))
        for n in w.lanes:Warehouse.return_after_delivery(w,n)
        self.assertEqual([c.args for c in w.go.call_args_list],[
            ('agv1','returning',2,4,'d'),('agv2','returning',3,4,'d'),('agv3','returning',4,4,'d')])
        w.lanes['agv2'].front=None;w.go.reset_mock()
        Warehouse.return_after_delivery(w,'agv2')
        w.go.assert_called_once_with('agv2','parking',5,2,'d')

    def test_mapped_observation_detects_robot_in_front_of_package(self):
        b=board(lidar_map=True)
        station=b.xy(3,3)
        lane=Lane('agv3',4,b.xy(3,4),[1])
        w=SimpleNamespace(board=b,lanes={'agv3':lane},status={
            'agv3':dict(x=station[0],y=station[1]),
            'agv2':dict(x=lane.pickup[0],y=lane.pickup[1])})
        self.assertEqual(Warehouse.observation_blocker(w,'agv3'),'agv2')
        w.status['agv2']['y']+=b.spacing
        self.assertIsNone(Warehouse.observation_blocker(w,'agv3'))

    def test_occluded_measurement_yields_station_without_changing_dispatcher(self):
        b=board(lidar_map=True)
        lane=Lane('agv3',4,b.xy(3,4),[1])
        w=SimpleNamespace(started=True,failed=False,done=False,
            telemetry=TelemetryGuard(),column_queues=False,
            args=SimpleNamespace(sync_rounds=False,delivery_mode='serial'),
            phase={'agv3':'measure'},lanes={'agv3':lane},board=b,
            pose={'agv3':(*b.xy(3,3),0.)},pending_goals={},deadlines={'agv3':55.},
            inspection_blocked_since={},inspection_blocker={},inspection_wait_since={},
            samples={'agv3':[object()]},frame_epoch=0,
            observation_blocker=Mock(return_value='agv2'),event=Mock(),go=Mock())
        Warehouse.mission(w,10.)
        self.assertEqual(w.inspection_blocked_since['agv3'],10.)
        Warehouse.mission(w,12.1)
        w.go.assert_called_once_with('agv3','inspection_yield',5,4)
        self.assertEqual(w.inspection_blocker['agv3'],'agv2')
        self.assertEqual(w.samples['agv3'],[])
        w.phase['agv3']='inspection_yield';w.reached=Mock(return_value=True)
        Warehouse.mission(w,13.)
        self.assertEqual(w.phase['agv3'],'inspection_wait')
        Warehouse.mission(w,14.)
        self.assertEqual(w.go.call_count,1)
        w.observation_blocker.return_value=None
        Warehouse.mission(w,15.)
        w.go.assert_called_with('agv3','observe',4,4,'d')

    def test_loaded_robot_becomes_ready_even_while_other_robots_are_loading(self):
        lane=Lane('agv1',2,(1.2,-.6),[1,4]);lane.load(0)
        w=SimpleNamespace(lanes={'agv1':lane},pose={'agv1':(1.2,-.6,0.)},
                          phase={'agv1':'loading','agv2':'loading','agv3':'measure'},
                          args=SimpleNamespace(sync_rounds=False),telemetry=TelemetryGuard(),failed=False,
                          outfeed=DeliveryOutfeed(board()),dropped=[],box_pub=Mock(),mirror=None,event=Mock())
        Warehouse.mechanics(w,2.1)
        self.assertEqual(w.phase,{'agv1':'wait_bay','agv2':'loading','agv3':'measure'})

    def test_final_edge_is_constrained_without_closing_the_return_lane(self):
        b=board(goal_approaches=[[[1,c],[2,c]] for c in range(1,5)])
        p=planner(b);r=Robot('agv1',0);r.x,r.y=b.xy(3,4);r.yaw=0.
        r.goal=(0,2);r.goal_heading=-math.pi/2
        path=TrafficManager.plan(p,r,(3,4),(0,2))
        self.assertEqual(path[-2:],[(1,2),(0,2)])
        r.x,r.y=b.xy(0,0);r.yaw=-math.pi/2;r.goal=(1,3);r.goal_heading=0.
        path=TrafficManager.plan(p,r,(0,0),(1,3))
        self.assertEqual(path,[(0,0),(1,0),(1,1),(1,2),(1,3)])
        self.assertTrue(bay_exit_clear((*b.xy(1,0),math.pi/2),b,1))

    def test_serviced_robot_is_not_treated_as_a_deadlock_or_moved_aside(self):
        w=SimpleNamespace(service_holds={'agv3':time.time()+2},clear_route=Mock(),side_step=Mock())
        w.in_service=TrafficManager.in_service.__get__(w)
        a,b=Robot('agv1',0),Robot('agv3',2)
        TrafficManager.resolve(w,a,b)
        w.clear_route.assert_not_called();w.side_step.assert_not_called()
        self.assertFalse(TrafficManager.side_step(w,b,a))

    def test_yield_route_cannot_acknowledge_the_requested_delivery_goal(self):
        b=board();x,y=b.xy(0,0)
        w=SimpleNamespace(pending_goals={'agv1':dict(old_route=8,ack=False,command='agv1 1 1 n')},
            active_goals={'agv1':(1,1,'n')},status={'agv1':dict(route_id=9,route_goal=[1,0],route_mode='yield',state='done')},
            board=b,pose={'agv1':(x,y,-math.pi/2)},last_goal_send={'agv1':time.monotonic()},goal_pub=Mock(),event=Mock(),
            phase={'agv1':'deliver'})
        self.assertFalse(Warehouse.reached(w,'agv1'))
        self.assertFalse(w.pending_goals['agv1']['ack'])
        w.status['agv1'].update(route_goal=[0,0],route_mode='go')
        self.assertTrue(Warehouse.reached(w,'agv1'))
        self.assertNotIn('agv1',w.pending_goals)


class OutfeedTests(unittest.TestCase):
    def test_no_stacking_overlap_or_lost_delivery_ids(self):
        out=DeliveryOutfeed(board())
        for i in range(10):
            out.update(i*2)
            item=dict(id=f'box{i}',kind=2,w=.185,d=.18,h=.34)
            self.assertTrue(out.can_accept(2,item['d']))
            out.accept(item,2,i*2)
            self.assertFalse(out.can_accept(2,item['d']))
            active=[b for b in out.items if not b['outbound_cleared']]
            self.assertTrue(all(b['z']==DECK_Z for b in active))
            for a,b in zip(active,active[1:]):
                self.assertGreater(math.dist((a['x'],a['y']),(b['x'],b['y'])),(a['d']+b['d'])/2)
        out.update(30)
        self.assertEqual(len(out.items),10)
        self.assertTrue(all(b['outbound_cleared'] and b['z']<0 for b in out.items))
        self.assertEqual(out.update(31),[])

    def test_wrong_slot_is_refused_and_pause_does_not_advance_the_belt(self):
        out=DeliveryOutfeed(board());item=dict(id='a',kind=1,w=.12,d=.14,h=.27)
        with self.assertRaises(ValueError):out.accept(dict(item),2,0)
        out.accept(item,1,0);out.update(1);pose=(item['x'],item['y'],item['z'])
        out.shift_time(5);out.update(6)
        self.assertEqual(pose,(item['x'],item['y'],item['z']))


if __name__=='__main__':unittest.main()
