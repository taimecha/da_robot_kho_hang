"""Straight correction geometry, swept-footprint checks, and live state machine."""
import math
import time
import unittest
from types import SimpleNamespace, MethodType
from unittest.mock import Mock, patch
from nav_msgs.msg import OccupancyGrid
from fleet_agent import FleetAgent
from grid_navigator import GridNavigator
from goal_alignment import correction_plan, swept_costmap_clear


def free_map():
    m=OccupancyGrid();m.header.frame_id='map'
    m.info.width=m.info.height=100;m.info.resolution=.01
    m.info.origin.position.x=m.info.origin.position.y=-.5
    m.info.origin.orientation.w=1.;m.data=[0]*10000
    return m


def mark(m,x,y,value):
    ix=int((x-m.info.origin.position.x)/m.info.resolution)
    iy=int((y-m.info.origin.position.y)/m.info.resolution)
    m.data[iy*m.info.width+ix]=value


class StraightGeometryTests(unittest.TestCase):
    def test_actual_failure_has_a_short_reverse_solution(self):
        along,across=correction_plan((.0909,.6185,.001294),(.055,.635))
        self.assertLess(along,0.)
        self.assertAlmostEqual(along,-.0359,delta=.0001)
        self.assertLess(abs(across),.018)

    def test_forward_reverse_and_rotated_frame(self):
        self.assertEqual(correction_plan((0.,0.,0.),(.04,0.)),(.04,0.))
        self.assertEqual(correction_plan((0.,0.,0.),(-.04,0.)),(-.04,0.))
        along,across=correction_plan((0.,0.,math.pi/2),(0.,-.04))
        self.assertAlmostEqual(along,-.04)
        self.assertAlmostEqual(across,0.)

    def test_lateral_error_range_and_invalid_pose_are_not_hidden(self):
        for target in ((0.,.021),(.081,0.)):
            with self.assertRaises(ValueError):correction_plan((0.,0.,0.),target)
        with self.assertRaises(ValueError):correction_plan((math.nan,0.,0.),(0.,0.))

    def test_both_directions_require_clear_swept_footprint(self):
        for direction in (-1.,1.):
            m=free_map()
            self.assertTrue(swept_costmap_clear(m,(0.,0.),(.08*direction,0.),.13))
            mark(m,.19*direction,0.,100)
            self.assertFalse(swept_costmap_clear(m,(0.,0.),(.08*direction,0.),.13))

    def test_unknown_space_stale_frames_and_map_bounds_do_not_pass(self):
        m=free_map();mark(m,-.19,0.,-1)
        self.assertFalse(swept_costmap_clear(m,(0.,0.),(-.08,0.),.13))
        self.assertFalse(swept_costmap_clear(free_map(),(.45,0.),(.49,0.),.13))
        m=free_map();m.header.frame_id='odom'
        self.assertFalse(swept_costmap_clear(m,(0.,0.),(.04,0.),.13))


class StraightControllerTests(unittest.TestCase):
    def agent(self,x=.036,y=-.016,final_done=True):
        now=time.monotonic()
        pose=SimpleNamespace(x=x,y=y,yaw=0.)
        logger=SimpleNamespace(info=Mock(),error=Mock())
        a=SimpleNamespace(name='agv2',rot=None,goal_handle=None,alignment=None,
            position_corrections=0,final_done=final_done,
            MAX_POSITION_CORRECTIONS=2,CORRECTION_TIMEOUT_S=6.,POSITION_TOL_M=.02,
            rot_done=math.radians(2.5),odom_xy=(0.,0.),odom_t=now,gyro_t=now,
            odom_v=0.,esp_estop='',last_cmd_t=time.time(),
            alignment_costmap=free_map(),alignment_costmap_t=now,robot_radius=.13,
            normalize_angle=GridNavigator.normalize_angle,rot_w=lambda:0.,
            get_robot_pose=lambda:(pose.x,pose.y,pose.yaw),
            get_logger=lambda:logger,create_timer=Mock(return_value='timer'),
            destroy_timer=Mock(),publish_w=Mock(),cmd_pub=SimpleNamespace(publish=Mock()),
            rotation_failed=Mock(),send_next_waypoint=Mock())
        a.alignment_tick=MethodType(FleetAgent.alignment_tick,a)
        return a,pose

    def start(self,a,pose):
        FleetAgent.start_position_alignment(a,(0.,0.),(pose.x,pose.y,pose.yaw))

    def test_reverse_preserves_final_heading_without_pivot(self):
        a,p=self.agent();self.start(a,p)
        self.assertTrue(a.final_done)
        a.alignment_tick()
        cmd=a.cmd_pub.publish.call_args.args[0]
        self.assertLess(cmd.linear.x,0.)
        self.assertEqual(cmd.angular.z,0.)
        self.assertLessEqual(abs(cmd.linear.x),.025)

    def test_accepts_full_radius_only_after_stopping(self):
        a,p=self.agent();self.start(a,p)
        p.x=.006;a.odom_xy=(-.03,0.)
        a.alignment_tick();self.assertEqual(a.alignment['phase'],'settle')
        a.send_next_waypoint.assert_not_called()
        a.alignment_tick()
        a.alignment['still_since']-=.5
        a.alignment_tick()
        a.send_next_waypoint.assert_called_once()
        self.assertIsNone(a.alignment)
        a.rotation_failed.assert_not_called()
        self.assertTrue(a.final_done)

    def test_lateral_error_outside_radius_never_becomes_success(self):
        a,p=self.agent();self.start(a,p);p.x=0.;p.y=.021
        a.alignment_tick()
        a.rotation_failed.assert_called_once()
        a.send_next_waypoint.assert_not_called()

    def test_feedback_watchdog_obstacles_and_travel_limit_stop(self):
        for case in ('odom','imu','costmap','command','obstacle','travel','yaw','timeout','estop'):
            with self.subTest(case=case):
                a,p=self.agent();self.start(a,p)
                if case=='odom':a.odom_t-=1.
                if case=='imu':a.gyro_t-=1.
                if case=='costmap':a.alignment_costmap_t-=1.
                if case=='command':a.last_cmd_t-=2.
                if case=='obstacle':mark(a.alignment_costmap,-.13,-.016,100)
                if case=='travel':a.odom_xy=(-.11,0.)
                if case=='yaw':p.yaw=math.radians(3.)
                if case=='timeout':a.correction_deadline-=10.
                if case=='estop':a.esp_estop='wheel_stall'
                a.alignment_tick()
                a.rotation_failed.assert_called_once()
                a.cmd_pub.publish.assert_not_called()
                a.send_next_waypoint.assert_not_called()

    def test_no_direction_reversal_after_longitudinal_overshoot(self):
        a,p=self.agent(y=0.);self.start(a,p)
        a.alignment_tick()
        p.x=-.004;a.alignment_tick()
        self.assertEqual(a.alignment['phase'],'settle')
        a.publish_w.assert_called_with(0.)
        self.assertTrue(all(c.args[0].linear.x<0 for c in a.cmd_pub.publish.call_args_list))

    def test_stalled_correction_is_terminal(self):
        a,p=self.agent();self.start(a,p)
        a.alignment['progress_t']-=3.
        a.alignment_tick()
        a.rotation_failed.assert_called_once()
        a.cmd_pub.publish.assert_not_called()

    def test_cannot_start_while_nav2_or_rotation_is_active(self):
        for field in ('rot','goal_handle'):
            a,p=self.agent();setattr(a,field,object());self.start(a,p)
            a.rotation_failed.assert_called_once()
            a.create_timer.assert_not_called()

    def test_forward_and_reverse_converge_with_lagged_motion(self):
        # Synthetic plant: demonstrates bounded behavior, not measured accuracy.
        for initial_x in (-.04,.04):
            with self.subTest(initial_x=initial_x):
                a,p=self.agent(x=initial_x,y=.012)
                with patch('fleet_agent.time.monotonic',return_value=100.) as clock:
                    self.start(a,p);velocity=0.
                    a.publish_w.side_effect=lambda w:a.cmd_pub.publish(SimpleNamespace(linear=SimpleNamespace(x=0.)))
                    for k in range(1,111):
                        clock.return_value=100.+k*.05
                        a.odom_t=a.gyro_t=a.alignment_costmap_t=clock.return_value
                        a.alignment_tick()
                        if a.send_next_waypoint.called or a.rotation_failed.called:break
                        cmd=a.cmd_pub.publish.call_args.args[0].linear.x
                        velocity+=(cmd-velocity)*.5
                        p.x+=velocity*.05
                        a.odom_xy=(p.x-initial_x,0.)
                        a.odom_v=velocity
                    a.rotation_failed.assert_not_called()
                    a.send_next_waypoint.assert_called_once()
                    self.assertLessEqual(math.hypot(p.x,p.y),.02)


if __name__=='__main__':unittest.main()
