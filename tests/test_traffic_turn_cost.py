import unittest
import math
from functools import lru_cache

from grid_board import Board, edge_key
from traffic_manager import DIRS, Robot, TrafficManager


class Planner:
    """Run the real grid planner without starting ROS publishers or moving robots."""

    blocked_sets = TrafficManager.blocked_sets
    others = TrafficManager.others
    remaining = TrafficManager.remaining
    heading_dir = TrafficManager.heading_dir
    dist = TrafficManager.dist
    xy = TrafficManager.xy
    plan = TrafficManager.plan

    def __init__(self):
        self.board = Board({
            'rows': 5, 'cols': 5, 'spacing': 0.6,
            'center_row': 3, 'center_col': 3, 'center_x': 0.0, 'center_y': 0.0,
            'robot_radius': 0.13, 'node_clearance': 0.25, 'edge_clearance': 0.2,
        })
        self.static_nodes = {(0, 4)}  # The real map blocks (1,5).
        self.static_edges = set()
        self.avoid = True
        self.agv1 = Robot('agv1', 0)
        self.agv2 = Robot('agv2', 1)
        self.agv1.x, self.agv1.y = self.board.xy(2, 4)
        self.agv1.yaw = 0.0
        self.agv1.goal = (0, 1)
        self.agv2.x, self.agv2.y = self.board.xy(3, 4)
        self.agv2.yaw = 0.0
        self.robots = {'agv1': self.agv1, 'agv2': self.agv2}


class TurnCostTests(unittest.TestCase):
    def test_empty_grid_routes_are_shortest_and_use_fewest_pivots(self):
        p=Planner()
        p.avoid=False
        p.board.cfg['route_time_costs']=dict(
            linear_speed_mps=.22,turn_start_s=1.8,quarter_turn_s=1.6,
            route_node_wait_s=.4,occupied_wait_s=3.,goal_wait_s=3.,opposite_wait_s=6.)
        p.static_nodes.clear()

        @lru_cache(None)
        def shortest_paths(start,goal):
            if start==goal:return ((start,),)
            paths=[]
            for axis in (0,1):
                if start[axis]!=goal[axis]:
                    nxt=list(start)
                    nxt[axis]+=1 if goal[axis]>start[axis] else -1
                    paths.extend((start,)+tail for tail in shortest_paths(tuple(nxt),goal))
            return tuple(paths)

        def pivots(path,heading,final):
            directions=[DIRS[heading]]
            directions.extend((b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))
            directions.append(DIRS[final])
            return sum(a!=b for a,b in zip(directions,directions[1:]))

        robot=p.agv1
        for start in p.board.nodes():
            robot.x,robot.y=p.board.xy(*start)
            for goal in p.board.nodes():
                if start==goal:continue
                robot.goal=goal
                options=shortest_paths(start,goal)
                for heading in range(4):
                    robot.yaw=heading*math.pi/2
                    for final in range(4):
                        robot.goal_heading=final*math.pi/2
                        path=p.plan(robot,start,goal)
                        self.assertEqual(len(path)-1,len(options[0])-1)
                        self.assertEqual(pivots(path,heading,final),
                                         min(pivots(candidate,heading,final) for candidate in options))

    def test_new_goal_survives_failed_planning_at_end_of_previous_route(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        p=SimpleNamespace(next_route_id=100, others=lambda rb:[],
                          node_of=lambda rb:(0,1), plan=Mock(return_value=None),
                          lbl=str, log=Mock())
        p.clear_route=lambda rb:TrafficManager.clear_route(p,rb)
        p.assign_route=lambda *args:TrafficManager.assign_route(p,*args)
        p.route_str=str
        rb=Robot('agv1',0)
        rb.route=[(0,0),(0,1)];rb.route_id=99;rb.mode='go'
        rb.goal=(4,0);rb.goal_heading=0.
        self.assertFalse(TrafficManager.route_to_goal(p,rb))
        self.assertEqual(rb.goal,(4,0))
        self.assertTrue(rb.pending)
        self.assertIsNone(rb.route)  # stale done cannot erase the pending goal
        self.assertEqual(rb.mode,'idle')
        self.assertEqual(rb.route_id,100)
        p.plan.return_value=[(0,1),(1,1),(2,1),(3,1),(4,1),(4,0)]
        rb.pending=False  # the actual retry loop consumes this flag
        self.assertTrue(TrafficManager.route_to_goal(p,rb))
        self.assertEqual(rb.route[-1],(4,0))
        self.assertEqual(rb.goal,(4,0))
        self.assertEqual(rb.mode,'go')

    def test_delivery_uses_two_turns_when_other_robot_is_parked(self):
        p = Planner()
        path = p.plan(p.agv1, (2, 4), (0, 1), hard={(3, 4)})
        self.assertEqual(path, [(2, 4), (2, 3), (2, 2), (2, 1), (1, 1), (0, 1)])

    def test_parallel_delivery_keeps_shorter_turn_route(self):
        p = Planner()
        p.agv2.route = [(3, 4), (3, 3), (3, 2), (3, 1), (3, 0),
                        (2, 0), (1, 0), (0, 0)]
        p.agv2.goal = (0, 0)
        path = p.plan(p.agv1, (2, 4), (0, 1))
        self.assertEqual(path, [(2, 4), (2, 3), (2, 2), (2, 1), (1, 1), (0, 1)])

    def test_explicit_heading_is_counted_before_choosing_the_goal_path(self):
        import math
        p=Planner();r=p.agv1
        r.x,r.y=p.board.xy(0,2);r.yaw=-math.pi/2
        r.goal=(3,3);r.goal_heading=0.
        path=p.plan(r,(0,2),(3,3))
        self.assertEqual(len(path),5)  # same shortest grid distance
        self.assertEqual(path[-2:],[(3,2),(3,3)])  # arrive East

    def test_blocked_edge_still_forces_detour(self):
        p = Planner()
        p.static_edges.add(edge_key((2, 4), (2, 3)))
        path = p.plan(p.agv1, (2, 4), (0, 1), hard={(3, 4)})
        self.assertEqual(path, [(2, 4), (1, 4), (1, 3), (1, 2), (1, 1), (0, 1)])


if __name__ == '__main__':
    unittest.main()
