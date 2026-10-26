"""Moving robots must not leave phantom obstacles in another car's route."""
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from std_msgs.msg import String

from fleet_agent import FleetAgent


class RobotEchoTests(unittest.TestCase):
    def test_recent_robot_trail_is_removed_but_unrelated_obstacle_remains(self):
        agent=SimpleNamespace(name='agv1',others={},other_pose_history={},
                              robot_echo_history_s=3.)
        for t,x in ((97.,0.),(99.9,.6)):
            msg=String(data=json.dumps(dict(name='agv3',x=x,y=0.)))
            with patch('fleet_agent.time.time',return_value=t):
                FleetAgent.cb_status(agent,msg)
        obstacles=[(.02,0.),(.62,0.),(1.5,0.)]
        self.assertEqual(FleetAgent.remove_robot_obstacles(agent,obstacles,100.),
                         [(1.5,0.)])

    def test_real_config_keeps_historical_obstacles_and_stale_pose_masks_none(self):
        agent=SimpleNamespace(name='agv1',others={},other_pose_history={},
                              robot_echo_history_s=0.)
        msg=String(data=json.dumps(dict(name='agv3',x=.6,y=0.)))
        with patch('fleet_agent.time.time',return_value=99.9):
            FleetAgent.cb_status(agent,msg)
        obstacles=[(.02,0.),(.62,0.),(1.5,0.)]
        self.assertEqual(FleetAgent.remove_robot_obstacles(agent,obstacles,100.),
                         [(.02,0.),(1.5,0.)])
        self.assertEqual(FleetAgent.remove_robot_obstacles(agent,obstacles,102.),
                         obstacles)


if __name__=='__main__':unittest.main()
