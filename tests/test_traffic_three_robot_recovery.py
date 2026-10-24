"""Replay the real 3-robot circular wait at (2,4)/(1,4)/(2,3)."""
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

from grid_board import Board
from traffic_manager import Robot,TrafficManager


def manager():
    b=Board(dict(rows=5,cols=5,spacing=.6,center_row=3,center_col=3,
                 center_x=0.,center_y=0.,robot_radius=.13,node_clearance=.25,edge_clearance=.2,
                 goal_approaches=[[[1,c],[2,c]] for c in range(1,5)]))
    p=SimpleNamespace(board=b,robots={f'agv{i+1}':Robot(f'agv{i+1}',i) for i in range(3)},
        avoid=True,static_nodes=set(),static_edges=set(),occ_r=.43,service_holds={},
        next_route_id=1000,map_ok=True,recovery=None,log=Mock(),publish_markers=Mock())
    for name in ('blocked_sets','others','remaining','heading_dir','dist','xy','plan','node_of',
                 'occupied','parked','prio','route_str','assign_route','clear_route','owner',
                 'waits_for','in_service','resolve','yield_to','side_step','try_resume',
                 'on_route_done','on_trouble','route_to_goal','set_goal','recovery_active'):
        setattr(p,name,getattr(TrafficManager,name).__get__(p))
    p.lbl=TrafficManager.lbl
    p.cmd_pubs={n:Mock() for n in p.robots}
    a,c,d=p.robots.values()
    a.route=[(1,3),(1,2),(1,1),(0,1)];a.goal=(0,1);a.yaw=math.pi
    c.route=[(0,3),(1,3),(2,3)];c.goal=(2,3);c.yaw=-math.pi/2
    for i,(r,node) in enumerate(zip((a,c,d),((1,3),(0,3),(1,2)))):
        r.x,r.y=b.xy(*node);r.t=time.time();r.last_move_t=time.time()-20
        r.state='waiting' if r.route else 'done';r.mode='go' if r.route else 'idle'
        r.route_id=r.ack=i+1;r.wait_since=time.time()-10
    p.occ={n:p.occupied(r) for n,r in p.robots.items()}
    p.res={n:set(nodes) for n,nodes in p.occ.items()}
    return p


class ThreeRobotRecoveryTests(unittest.TestCase):
    def test_cycle_has_one_escape_route_and_is_not_overwritten_while_pivoting(self):
        p=manager();a,b,c=p.robots.values();now=time.time()
        TrafficManager.tick(p)
        chosen=list(a.route);route_id=a.route_id
        self.assertEqual(chosen[-1],(0,1))
        self.assertNotIn(p.node_of(c),chosen)
        self.assertNotIn(p.node_of(b),chosen)
        self.assertEqual(b.route_id,2)
        self.assertIsNotNone(p.recovery)
        for dt in (.2,3.3,6.):
            a.ack=a.route_id;a.state='moving';a.last_move_t=now+dt
            for r in p.robots.values():r.t=now+dt
            with patch('traffic_manager.time.time',return_value=now+dt):
                TrafficManager.tick(p)
            self.assertEqual(a.route_id,route_id)
            self.assertEqual(a.route,chosen)
        x0,y0=p.board.xy(*chosen[0]);x1,y1=p.board.xy(*chosen[1])
        a.x=x0+(x1-x0)*.46/.6;a.y=y0+(y1-y0)*.46/.6
        for r in p.robots.values():r.t=now+7
        with patch('traffic_manager.time.time',return_value=now+7):TrafficManager.tick(p)
        self.assertGreaterEqual(b.upto,1)  # Yellow can leave its delivery bay.

    def test_old_forward_permissions_remain_reserved_until_new_route_ack(self):
        p=manager();a=p.robots['agv1'];old=(1,4)
        a.route=[(1,3),old];a.upto=1;p.res['agv1']|={old}
        p.assign_route(a,[(1,3),(2,3)],None,'go')
        TrafficManager.tick(p)
        self.assertIn(old,p.res['agv1'])
        a.ack=a.route_id
        TrafficManager.tick(p)
        self.assertNotIn(old,p.res['agv1'])

    def test_refuge_cannot_be_the_occupied_node_of_a_third_robot(self):
        p=manager();a,b,c=p.robots.values()
        b.x,b.y=p.board.xy(0,4);b.route=[(0,4),(1,4),(2,4)]
        c.x,c.y=p.board.xy(2,3)
        p.occ={n:p.occupied(r) for n,r in p.robots.items()}
        p.res={n:set(v) for n,v in p.occ.items()}
        self.assertTrue(p.side_step(a,b))
        self.assertNotIn((2,3),a.route)
        self.assertEqual(a.route[-1],(1,2))

    def test_new_workflow_goal_does_not_cancel_a_coordinated_yield(self):
        p=manager();a=p.robots['agv1'];a.mode='yield';a.resume=((0,1),None)
        before=list(a.route);rid=a.route_id
        TrafficManager.set_goal(p,a,(0,3),-math.pi/2)
        self.assertEqual(a.resume,((0,3),-math.pi/2))
        self.assertEqual(a.route,before)
        self.assertEqual(a.route_id,rid)

    def test_old_planning_retry_cannot_interrupt_a_yield_or_erase_a_pending_goal(self):
        p=manager();a,b,_=p.robots.values()
        a.route=None;a.mode='idle';a.pending=True;a.goal=(0,1);a.goal_heading=-math.pi/2
        self.assertTrue(p.side_step(a,b))
        self.assertFalse(a.pending)
        self.assertEqual(a.resume,((0,1),-math.pi/2))

    def test_new_goal_is_deferred_until_the_recovering_robot_clears_its_cell(self):
        p=manager();a=p.robots['agv1'];now=time.time()
        p.recovery=dict(robots={a.name:(a.route_id,(a.x,a.y))},until=now+12)
        before=list(a.route);rid=a.route_id
        p.set_goal(a,(0,3),-math.pi/2)
        self.assertEqual(a.deferred_goal,((0,3),-math.pi/2))
        self.assertEqual(a.route,before)
        self.assertEqual(a.route_id,rid)
        a.x+=.46
        TrafficManager.tick(p)
        self.assertIsNone(a.deferred_goal)
        self.assertEqual(a.goal,(0,3))

    def test_duplicate_goal_does_not_restart_an_active_route_or_a_planning_retry(self):
        p=manager();a=p.robots['agv1'];rid=a.route_id;path=list(a.route)
        p.set_goal(a,a.goal,a.goal_heading)
        self.assertEqual(a.route_id,rid);self.assertEqual(a.route,path)
        a.route=None;a.pending=True;a.fail_count=3;a.retry_at=time.time()+1
        p.set_goal(a,a.goal,a.goal_heading)
        self.assertEqual(a.route_id,rid);self.assertEqual(a.fail_count,3)

    def test_yielded_owner_resumes_when_parked_waiters_block_the_only_entrance(self):
        p=manager();a,b,c=p.robots.values()
        for r,node in ((a,(1,0)),(b,(2,1)),(c,(2,0))):
            r.x,r.y=p.board.xy(*node);r.route=None;r.state='done';r.mode='idle'
        b.mode='yield_wait';b.resume=((0,0),-math.pi/2);b.goal=(0,0);b.yield_since=time.time()-5
        p.occ={n:p.occupied(r) for n,r in p.robots.items()}
        p.res={n:set(v) for n,v in p.occ.items()}
        p.try_resume(b,time.time())
        self.assertEqual(b.mode,'go')
        self.assertEqual(b.route[-1],(0,0))
        self.assertEqual(b.upto,0)  # permission control still protects parked cars


if __name__=='__main__':unittest.main()
