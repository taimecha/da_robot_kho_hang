import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from fleet_agent import FleetAgent
from grid_navigator import GridNavigator


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=int((self.seconds - other.seconds) * 1e9))


def agent_at(yaw_deg, final_done=False):
    logger = SimpleNamespace(info=Mock())
    agent = SimpleNamespace(
        name='agv1',
        waypoints=[], active_segment=None, route=[(2, 4)], sent_upto=0,
        final_heading=0.0, final_done=final_done, rot_done=math.radians(0.9),
        state='moving', target_node=(0.655, 0.035),
        get_robot_pose=lambda: (0.66, 0.04, math.radians(yaw_deg)),
        get_coordinates=lambda r, c: (0.655, 0.035),
        normalize_angle=lambda a: math.atan2(math.sin(a), math.cos(a)),
        start_rotation=Mock(), abort_route=Mock(), get_logger=lambda: logger,
    )
    return agent


class GoalHeadingTests(unittest.TestCase):
    def test_misaligned_goal_starts_correction(self):
        agent = agent_at(1.1)
        FleetAgent.send_next_waypoint(agent)
        agent.start_rotation.assert_called_once_with(0.0, finish=True)
        self.assertNotEqual(agent.state, 'done')

    def test_goal_is_not_done_if_correction_still_misses(self):
        agent = agent_at(1.1, final_done=True)
        FleetAgent.send_next_waypoint(agent)
        agent.abort_route.assert_called_once()
        self.assertNotEqual(agent.state, 'done')

    def test_goal_is_done_only_below_tolerance(self):
        agent = agent_at(0.8)
        FleetAgent.send_next_waypoint(agent)
        self.assertEqual(agent.state, 'done')
        agent.start_rotation.assert_not_called()
        agent.abort_route.assert_not_called()
        self.assertIn('sai so goc 0.80°', agent.get_logger().info.call_args.args[0])

    def test_tolerance_boundary_does_not_pass(self):
        agent = agent_at(0.9)
        FleetAgent.send_next_waypoint(agent)
        agent.start_rotation.assert_called_once()
        self.assertNotEqual(agent.state, 'done')

    def test_rotation_exhaustion_reports_failure(self):
        logger = SimpleNamespace(error=Mock(), info=Mock())
        agent = SimpleNamespace(
            rot={'target': 0.0, 'phase': 'settle', 't': FakeTime(0),
                 't0': FakeTime(0), 'tries': 12, 'yaw0': 0.0,
                 'err0': -math.radians(1.2), 'wpeak': 0.7,
                 'log': [], 'finish': True},
            get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(2)),
            get_robot_pose=lambda: (0.0, 0.0, math.radians(1.2)),
            normalize_angle=lambda a: math.atan2(math.sin(a), math.cos(a)),
            publish_w=Mock(), rot_w=lambda: 0.0, rot_lag=0.3, rot_tau=0.12,
            rot_done=math.radians(0.9), ROT_TRIES=12,
            get_logger=lambda: logger, stop_rotation=Mock(),
            abort_route=Mock(), send_next_waypoint=Mock(), send_follow_path=Mock(),
        )
        GridNavigator.rotate_tick(agent)
        agent.abort_route.assert_called_once()
        agent.send_next_waypoint.assert_not_called()


if __name__ == '__main__':
    unittest.main()
