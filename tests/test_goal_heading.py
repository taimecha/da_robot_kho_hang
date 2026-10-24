import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path
import yaml
from nav_msgs.msg import Path as RosPath

from fleet_agent import FleetAgent
from grid_navigator import GridNavigator


ROT_CFG = yaml.safe_load((Path(__file__).resolve().parents[1]/'src/my_bot_bringup/config/grid_real.yaml').read_text())['rotate']


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __add__(self, duration):
        return FakeTime(self.seconds + duration.nanoseconds * 1e-9)

    def __lt__(self, other):
        return self.seconds < other.seconds

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=int((self.seconds - other.seconds) * 1e9))


def agent_at(yaw_deg, final_done=False, y=0.04, position_corrections=0, x=0.66):
    logger = SimpleNamespace(info=Mock(), warn=Mock(), error=Mock())
    agent = SimpleNamespace(
        name='agv1',
        waypoints=[], active_segment=None, route=[(2, 4)], sent_upto=0,
        final_heading=0.0, final_done=final_done, rot_done=math.radians(ROT_CFG['done_deg']),
        state='moving', target_node=(0.655, 0.035),
        position_corrections=position_corrections,
        POSITION_TOL_M=FleetAgent.POSITION_TOL_M,
        MAX_POSITION_CORRECTIONS=FleetAgent.MAX_POSITION_CORRECTIONS,
        SMALL_CORRECTION_MAX_M=FleetAgent.SMALL_CORRECTION_MAX_M,
        get_robot_pose=lambda: (x, y, math.radians(yaw_deg)),
        get_coordinates=lambda r, c: (0.655, 0.035),
        normalize_angle=lambda a: math.atan2(math.sin(a), math.cos(a)),
        start_rotation=Mock(), abort_route=Mock(), rotation_failed=Mock(), send_next_waypoint=Mock(),
        halt=Mock(), start_position_alignment=Mock(),
        get_logger=lambda: logger,
    )
    return agent


class GoalHeadingTests(unittest.TestCase):
    def test_real_tolerance_is_under_three_degrees_and_no_pulse_retries(self):
        self.assertEqual(ROT_CFG['done_deg'], 2.5)
        self.assertLess(ROT_CFG['done_deg'], 3.0)
        self.assertNotIn('tries', ROT_CFG)
        self.assertNotIn('pulse_max_deg', ROT_CFG)

    def test_short_position_correction_uses_tighter_goal_checker(self):
        for corrections, checker in ((0, 'general_goal_checker'), (1, 'precision_goal_checker')):
            client = SimpleNamespace(send_goal_async=Mock(return_value=Mock()))
            agent = SimpleNamespace(_seq=0, active_segment=(0., 0., 0., .028),
                                    position_corrections=corrections,
                                    route=[], waypoints=[],
                                    CORRECTION_TIMEOUT_S=FleetAgent.CORRECTION_TIMEOUT_S,
                                    make_path=lambda *args: RosPath(), _action_client=client,
                                    _on_goal_response=Mock())
            FleetAgent.send_follow_path(agent)
            self.assertEqual(client.send_goal_async.call_args.args[0].goal_checker_id, checker)
            goal=client.send_goal_async.call_args.args[0]
            self.assertEqual(goal.controller_id,'PrecisionFollowPath' if corrections else 'FollowPath')
            self.assertEqual(goal.progress_checker_id,'precision_progress_checker' if corrections else 'progress_checker')

    def test_precision_checker_is_configured_with_margin_below_final_tolerance(self):
        cfg=Path(__file__).resolve().parents[1]/'src/my_bot_bringup/config/navigation.yaml'
        params=yaml.safe_load(cfg.read_text())['controller_server']['ros__parameters']
        self.assertIn('precision_goal_checker',params['goal_checker_plugins'])
        checker=params['precision_goal_checker']
        self.assertLess(checker['xy_goal_tolerance'], FleetAgent.POSITION_TOL_M)
        self.assertGreaterEqual(checker['xy_goal_tolerance'], .01)
        self.assertFalse(checker['stateful'])
        progress=params['precision_progress_checker']
        self.assertLess(progress['required_movement_radius'],.01)
        self.assertLess(progress['movement_time_allowance'],6.)
        self.assertLess(params['PrecisionFollowPath']['min_approach_linear_velocity'],.04)

    def test_misaligned_goal_starts_correction(self):
        agent = agent_at(2.6)
        FleetAgent.send_next_waypoint(agent)
        agent.start_rotation.assert_called_once_with(0.0, finish=True)
        self.assertNotEqual(agent.state, 'done')

    def test_goal_is_not_done_if_correction_still_misses(self):
        agent = agent_at(2.6, final_done=True)
        FleetAgent.send_next_waypoint(agent)
        agent.rotation_failed.assert_called_once()
        self.assertNotEqual(agent.state, 'done')

    def test_goal_is_done_only_below_tolerance(self):
        agent = agent_at(2.4)
        FleetAgent.send_next_waypoint(agent)
        self.assertEqual(agent.state, 'done')
        agent.start_rotation.assert_not_called()
        agent.abort_route.assert_not_called()
        self.assertIn('sai so goc 2.40°', agent.get_logger().info.call_args.args[0])

    def test_tolerance_boundary_does_not_pass(self):
        agent = agent_at(ROT_CFG['done_deg'])
        FleetAgent.send_next_waypoint(agent)
        agent.start_rotation.assert_called_once()
        self.assertNotEqual(agent.state, 'done')

    def test_aligned_position_is_corrected_before_final_rotation(self):
        agent=agent_at(.2,y=.035,x=.61)
        FleetAgent.send_next_waypoint(agent)
        agent.start_position_alignment.assert_called_once_with(
            (.655,.035),(.61,.035,math.radians(.2)))
        agent.start_rotation.assert_not_called()

    def test_position_is_centered_before_turning_to_final_heading(self):
        agent=agent_at(90.,y=.007,x=.655)
        FleetAgent.send_next_waypoint(agent)
        agent.start_rotation.assert_not_called()
        agent.start_position_alignment.assert_called_once()

    def test_centimeter_lateral_error_cannot_trigger_ninety_degree_pivot(self):
        agent=agent_at(0.,y=.007,x=.655)
        FleetAgent.start_position_alignment(agent,(.655,.035),(.655,.007,0.))
        agent.rotation_failed.assert_called_once()
        agent.start_rotation.assert_not_called()
        agent.send_next_waypoint.assert_not_called()

    def test_final_original_segment_also_uses_gentle_approach(self):
        client=SimpleNamespace(send_goal_async=Mock(return_value=Mock()))
        agent=SimpleNamespace(_seq=0,active_segment=(0.,0.,.6,0.),
                              position_corrections=0,route=[(0,0),(0,1)],waypoints=[],
                              get_coordinates=lambda r,c:(.6,0.),
                              make_path=lambda *args:RosPath(),_action_client=client,
                              _on_goal_response=Mock())
        FleetAgent.send_follow_path(agent)
        self.assertEqual(client.send_goal_async.call_args.args[0].controller_id,'PrecisionFollowPath')

    def test_position_correction_is_bounded(self):
        agent=agent_at(.2,y=.035,x=.61,position_corrections=2)
        FleetAgent.start_position_alignment(agent,(.655,.035),(.61,.035,0.))
        agent.rotation_failed.assert_called_once()
        agent.send_next_waypoint.assert_not_called()

    def test_position_drift_after_rotation_uses_straight_correction(self):
        agent=agent_at(.2,final_done=True,y=.035,x=.69)
        FleetAgent.send_next_waypoint(agent)
        agent.start_position_alignment.assert_called_once()
        agent.start_rotation.assert_not_called()
        self.assertTrue(agent.final_done)

    def test_free_heading_delivery_can_align_once_and_keep_that_heading(self):
        agent=agent_at(0.,y=.003,x=.655)
        agent.final_heading=None
        agent.free_heading_aligned=False
        FleetAgent.send_next_waypoint(agent)
        agent.start_rotation.assert_called_once()
        self.assertAlmostEqual(abs(agent.final_heading),math.pi/2)
        self.assertTrue(agent.final_done)
        self.assertTrue(agent.free_heading_aligned)
        agent.start_rotation.reset_mock()
        FleetAgent.send_next_waypoint(agent)
        agent.start_rotation.assert_not_called()
        agent.start_position_alignment.assert_called_once()

    def test_explicit_goal_heading_is_not_changed_to_hide_lateral_error(self):
        agent=agent_at(0.,y=.003,x=.655)
        FleetAgent.send_next_waypoint(agent)
        self.assertEqual(agent.final_heading,0.)
        agent.start_rotation.assert_not_called()
        agent.start_position_alignment.assert_called_once()

    def test_nonfinite_goal_pose_cannot_pass_as_done(self):
        agent=agent_at(0.,x=math.nan)
        FleetAgent.send_next_waypoint(agent)
        agent.rotation_failed.assert_called_once()
        self.assertNotEqual(agent.state,'done')

    def test_rotation_failure_is_terminal_for_fleet_and_has_reason(self):
        a=SimpleNamespace(name='agv2', halt=Mock(), get_logger=lambda: SimpleNamespace(error=Mock()))
        FleetAgent.rotation_failed(a,'xoay khong dat')
        a.halt.assert_called_once()
        self.assertEqual(a.state,'precision_failed')
        self.assertEqual(a.last_error,'xoay khong dat')


class RotationFeedbackTests(unittest.TestCase):
    def controller(self, yaw_deg, speed=0., now_s=0.):
        clock=SimpleNamespace(seconds=now_s)
        pose=SimpleNamespace(yaw=math.radians(yaw_deg), speed=speed)
        r=dict(target=0.,phase='turn',t0=FakeTime(0.),t_last=FakeTime(0.),
               finish=True,direction=None,command=0.,progress_yaw=None,
               progress_t=FakeTime(0.),wpeak=0.)
        a=SimpleNamespace(
            rot=r, real=False, gyro_t=time.monotonic(),
            get_clock=lambda:SimpleNamespace(now=lambda:FakeTime(clock.seconds)),
            get_robot_pose=lambda:(0.,0.,pose.yaw),
            normalize_angle=lambda x: math.atan2(math.sin(x),math.cos(x)),
            rot_done=math.radians(ROT_CFG['done_deg']),
            rot_tol=math.radians(ROT_CFG['stop_deg']),
            rot_wmax=ROT_CFG['max_speed'],rot_wmin=ROT_CFG['min_speed'],
            rot_accel=ROT_CFG['decel'],rot_gain=ROT_CFG['gain'],
            rot_brake_delay=ROT_CFG['brake_delay'],rot_settle=ROT_CFG['settle_s'],
            rot_timeout=ROT_CFG['timeout_s'],rot_stall_timeout=ROT_CFG['stall_timeout_s'],
            rot_w=lambda:pose.speed,publish_w=Mock(),rotation_failed=Mock(),
            get_logger=lambda:SimpleNamespace(info=Mock()),stop_rotation=Mock(),
            send_next_waypoint=Mock(),send_follow_path=Mock())
        return a,clock,pose

    def test_already_acceptable_angle_never_commands_rotation(self):
        a,c,p=self.controller(-2.4)
        GridNavigator.rotate_tick(a)
        self.assertEqual(a.rot['phase'],'settle')
        a.publish_w.assert_called_once_with(0.)
        c.seconds=.4;GridNavigator.rotate_tick(a)
        a.send_next_waypoint.assert_called_once()

    def test_decelerates_continuously_before_goal(self):
        a,c,p=self.controller(-90.)
        commands=[]
        for t,yaw in ((.1,-90),(.2,-70),(.3,-30),(.4,-10),(.5,-5),(.6,-3)):
            c.seconds=t;p.yaw=math.radians(yaw)
            GridNavigator.rotate_tick(a)
            commands.append(a.publish_w.call_args.args[0])
        self.assertTrue(all(w>0 for w in commands))
        self.assertLess(commands[-1],commands[2])
        self.assertEqual(a.rot['phase'],'turn')

    def test_crossing_goal_brakes_without_reversing_or_retrying(self):
        a,c,p=self.controller(-10.)
        c.seconds=.1;GridNavigator.rotate_tick(a)
        p.yaw=math.radians(4.);p.speed=.3;c.seconds=.2
        GridNavigator.rotate_tick(a)
        self.assertEqual(a.rot['phase'],'settle')
        a.publish_w.assert_called_with(0.)
        p.speed=0.;c.seconds=.5;GridNavigator.rotate_tick(a)
        c.seconds=.9;GridNavigator.rotate_tick(a)
        a.rotation_failed.assert_called_once()
        a.send_next_waypoint.assert_not_called()
        self.assertTrue(all(call.args[0]>=0 for call in a.publish_w.call_args_list))

    def test_predicted_brake_uses_measured_gyro_and_does_not_stop_ten_degrees_short(self):
        a,c,p=self.controller(-10.,speed=.65,now_s=.1)
        GridNavigator.rotate_tick(a)
        self.assertEqual(a.rot['phase'],'turn')
        p.yaw=math.radians(-2.);c.seconds=.15
        GridNavigator.rotate_tick(a)
        self.assertEqual(a.rot['phase'],'settle')

    def test_stopped_robot_must_remain_still_before_continuing(self):
        a,c,p=self.controller(-.5,speed=.08)
        GridNavigator.rotate_tick(a)
        c.seconds=.4;GridNavigator.rotate_tick(a)
        a.send_next_waypoint.assert_not_called()
        p.speed=0.;c.seconds=.5;GridNavigator.rotate_tick(a)
        c.seconds=.7;GridNavigator.rotate_tick(a)
        a.send_next_waypoint.assert_not_called()
        c.seconds=.9;GridNavigator.rotate_tick(a)
        a.send_next_waypoint.assert_called_once()

    def test_timeout_and_stall_stop_without_going_to_next_segment(self):
        for elapsed in (2.1,16.):
            a,c,p=self.controller(-90.)
            GridNavigator.rotate_tick(a)
            c.seconds=elapsed;GridNavigator.rotate_tick(a)
            a.rotation_failed.assert_called_once()
            a.send_next_waypoint.assert_not_called()
            a.send_follow_path.assert_not_called()

    def test_missing_pose_or_real_imu_stops(self):
        a,c,p=self.controller(-90.)
        a.get_robot_pose=lambda:(None,None,None)
        GridNavigator.rotate_tick(a)
        a.rotation_failed.assert_called_once()
        a,c,p=self.controller(-90.)
        a.real=True;a.gyro_t=time.monotonic()-1.
        GridNavigator.rotate_tick(a)
        a.rotation_failed.assert_called_once()

    def test_lagged_motor_model_turns_once_in_both_directions(self):
        # Synthetic dynamics verify controller behavior, not hardware accuracy.
        for angle in (-180.,-90.,-5.,-3.,3.,5.,90.,180.):
            for lag in (.05,.1,.2):
                with self.subTest(angle=angle,lag=lag):
                    a,c,p=self.controller(angle)
                    for i in range(1,1501):
                        c.seconds=i*.01
                        if i%5==0:
                            GridNavigator.rotate_tick(a)
                        cmd=a.publish_w.call_args.args[0] if a.publish_w.called else 0.
                        p.speed+=(cmd-p.speed)*min(1.,.01/lag)
                        p.yaw+=p.speed*.01
                        if a.rotation_failed.called or a.send_next_waypoint.called:
                            break
                    a.rotation_failed.assert_not_called()
                    a.send_next_waypoint.assert_called_once()
                    self.assertLess(abs(p.yaw),a.rot_done)
                    commands=[x.args[0] for x in a.publish_w.call_args_list]
                    self.assertTrue(all(x*angle<=0 for x in commands))
                    moving=[i for i,x in enumerate(commands) if abs(x)>1e-6]
                    self.assertTrue(moving)
                    self.assertTrue(all(abs(commands[i])>0 for i in range(moving[0],moving[-1]+1)))


if __name__ == '__main__':
    unittest.main()
