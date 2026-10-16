import unittest

from grid_board import Board, edge_key
from traffic_manager import Robot, TrafficManager


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

    def test_blocked_edge_still_forces_detour(self):
        p = Planner()
        p.static_edges.add(edge_key((2, 4), (2, 3)))
        path = p.plan(p.agv1, (2, 4), (0, 1), hard={(3, 4)})
        self.assertEqual(path, [(2, 4), (1, 4), (1, 3), (1, 2), (1, 1), (0, 1)])


if __name__ == '__main__':
    unittest.main()
