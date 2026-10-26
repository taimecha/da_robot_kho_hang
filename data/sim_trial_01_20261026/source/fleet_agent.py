#!/usr/bin/env python3
"""Bo thuc thi cua MOT xe khi chay nhieu xe (traffic_manager.py dieu phoi).

Dung lai toan bo GridNavigator (doan thang tren canh, xoay tai cho theo gyro,
Nav2 FollowPath), chi khac:
  - khong tu nhap dich, khong tu lap duong: nhan lo trinh va "duoc di toi o
    thu k" tu traffic_manager qua /fleet/<ten>/cmd (JSON);
  - chi chay toi o da duoc cho phep. Duoc cho them o CUNG HUONG trong luc dang
    chay thi noi dai doan dang chay (gui lai FollowPath, Nav2 thay duong ma
    khong dung xe) - khong dung lai o moi o;
  - vat can tren local costmap: BO vung quanh xe khac (traffic_manager lo
    chuyen hai xe), vat can that tren duong thi dung xe va bao 'blocked';
  - bao trang thai len /fleet/status (JSON) 5 lan/giay;
  - mat lenh tu traffic_manager > 1.5 s thi dung xe.

    python3 fleet_agent.py agv1 [--real] [--grid FILE] \\
        --ros-args -r __ns:=/agv1 -r /tf:=/agv1/tf -r /tf_static:=/agv1/tf_static
"""
import json
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from grid_navigator import GridNavigator
from goal_alignment import (ALIGN_STOP_RADIUS, MAX_TRAVEL, MAX_SPEED, MIN_SPEED,
                            MAX_LATERAL, body_error, correction_plan, swept_costmap_clear)

STATUS_PERIOD = 0.2     # s
CMD_TIMEOUT = 1.0       # s khong co lenh tu traffic_manager (5 lan/s) -> dung xe
OTHER_ROBOT_R = 0.25    # m - bo vat can trong ban kinh nay quanh xe khac
OTHER_STALE = 2.0       # s - vi tri xe khac cu hon muc nay thi khong loc nua
NAV2_WAIT = 5.0         # s - cho Nav2 (action follow_path) khi vua bat agent


class FleetAgent(GridNavigator):
    # FollowPath accepts a goal within 2 cm, but the final in-place rotation
    # can move the wheel axle after that check. Keep the skip distance below
    # the final tolerance so a short corrective segment is actually executed.
    SNAP_DIST = 0.015
    POSITION_TOL_M = 0.02
    MAX_POSITION_CORRECTIONS = 2
    SMALL_CORRECTION_MAX_M = 0.08
    CORRECTION_TIMEOUT_S = 6.0

    def __init__(self, name, use_sim_time=False, grid_file=None):
        # costmap_decay 1 s: o vat can KET tren local costmap mat sau ~1-2 s (xem
        # GridNavigator); dong INFO moi lan xoa da tat trong tools/fleet_nav.launch.py
        super().__init__(use_sim_time=use_sim_time, grid_file=grid_file,
                         node_name='fleet_agent', costmap_decay=1.0)
        self.POSITION_TOL_M=float(self.board.cfg.get('position_tolerance_m',self.POSITION_TOL_M))
        self.name = name
        self.route_id = None
        self.route = []             # [(r, c)] chi so tu 0
        self.final_heading = None   # rad he map, None = giu huong canh cuoi
        self.upto = 0               # duoc phep di toi route[upto]
        self.sent_upto = 0          # da giao cho Nav2 toi route[sent_upto]
        self.final_done = False
        self.free_heading_aligned = False
        self.position_corrections = 0
        self.last_error = ''
        self.correction_deadline = None
        self.alignment = None
        self.alignment_timer = None
        self.alignment_costmap = None
        self.alignment_costmap_t = -math.inf
        self.odom_xy = None
        self.odom_t = -math.inf
        self.odom_v = 0.0
        self.state = 'idle'          # idle | moving | waiting | done | blocked | failed | precision_failed
        self.dyn_blocked_nodes = set()
        self.dyn_blocked_edges = set()
        self.others = {}            # ten -> (x, y, t)
        # Simulation costmaps can retain a moving robot's old scan returns.
        # A generated sim grid opts into filtering that short pose trail.
        self.robot_echo_history_s = max(0., float(self.board.cfg.get('robot_echo_history_s', 0.)))
        self.other_pose_history = {}
        self.last_cmd_t = None
        self.cmd_lost = False
        self.nav2_wait_since = None
        self._seq = 0               # moi FollowPath mot so; ket qua cua goal cu bi bo qua
        self.status_pub = self.create_publisher(String, '/fleet/status', 10)
        self.create_subscription(String, f'/fleet/{name}/cmd', self.cb_cmd, 10)
        self.create_subscription(String, '/fleet/status', self.cb_status, 10)
        # ESTOP tren ESP32: GridNavigator da dang ky /agvN/diag -> self.cb_diag (ham
        # o duoi ghi de): dung xe, bao state 'estop', bo qua lenh cho toi khi go.
        self.create_timer(STATUS_PERIOD, self.tick)
        self.get_logger().info(f'[{name}] cho lenh tu traffic_manager tren /fleet/{name}/cmd')

    # ------------------------------------------------------------------
    # Ve sa ban la viec cua traffic_manager (mot ban cho ca doi xe)
    def publish_markers(self):
        pass

    def busy(self):
        return (self.active_segment is not None or self.rot is not None
                or self.alignment is not None)

    def halt(self):
        """Dung xe ngay: huy FollowPath, dung xoay, bo cac doan con lai."""
        self._seq += 1
        self.correction_deadline = None
        self.alignment = None
        if self.alignment_timer is not None:
            self.destroy_timer(self.alignment_timer)
            self.alignment_timer = None
        self.stop_rotation()
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:      # noqa: BLE001 - chi ghi lai, van dung xe
                self.get_logger().warn(f'[{self.name}] huy goal loi: {e}')
            self.goal_handle = None
        self.waypoints = []
        self.active_segment = None
        self.current_path_points = []
        self.publish_w(0.0)

    # ------------------------------------------------------------------
    def cb_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('name') != self.name and d.get('x') is not None:
            now=time.time();name=d['name'];x,y=d['x'],d['y']
            self.others[name] = (x, y, now)
            if self.robot_echo_history_s:
                history=self.other_pose_history.setdefault(name,deque())
                if not history or math.hypot(x-history[-1][0],y-history[-1][1])>.025:
                    history.append((x,y,now))
                while history and now-history[0][2]>self.robot_echo_history_s:
                    history.popleft()

    def remove_robot_obstacles(self, obstacles, now):
        """Remove live robot bodies and their brief costmap trails."""
        centers=[]
        for name,(x,y,t) in self.others.items():
            if now-t>=OTHER_STALE:continue
            centers.append((x,y))
            if self.robot_echo_history_s:
                centers.extend((hx,hy) for hx,hy,ht in self.other_pose_history.get(name,())
                               if now-ht<=self.robot_echo_history_s)
        if not centers:return obstacles
        lim=OTHER_ROBOT_R*OTHER_ROBOT_R
        return [(ox,oy) for ox,oy in obstacles
                if all((ox-x)**2+(oy-y)**2>lim for x,y in centers)]

    def cb_diag(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        reason = d.get('estop') or ''
        if reason and not self.esp_estop:
            self.esp_estop = reason
            self.halt()
            self.state = 'estop'
            self.get_logger().error(
                f'[{self.name}] !!! ESP32 ESTOP: {reason} - xe dung han. Kiem tra xe '
                f'(pin, banh ket, day dong co) roi go: ros2 topic pub --once '
                f'/{self.name}/estop_reset std_msgs/msg/Empty  (hoac tat/bat nguon xe)')
        elif not reason and self.esp_estop and d.get('event') != 'ESTOP':
            self.get_logger().warn(f'[{self.name}] ESP32 da go ESTOP ({self.esp_estop})')
            self.esp_estop = ''
            self.route, self.waypoints = [], []
            self.state = 'idle'

    def cb_cmd(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError as e:
            self.get_logger().error(f'[{self.name}] lenh hong: {e}')
            return
        self.last_cmd_t = time.time()
        if self.esp_estop:
            return                  # dang ESTOP: khong chay, traffic_manager da thay 'estop'

        nodes = [tuple(n) for n in d.get('nodes', [])]
        self.manager_mode=d.get('mode')
        if d.get('route_id') != self.route_id or self.cmd_lost:
            self.cmd_lost = False
            self.start_route(d.get('route_id'), nodes, d.get('upto', 0),
                             d.get('final_heading'))
        elif d.get('upto', 0) > self.upto:
            self.extend(d['upto'])

    # ------------------------------------------------------------------
    def start_route(self, route_id, nodes, upto, final_heading):
        self.halt()
        self.route_id = route_id
        self.route = nodes
        self.final_heading = final_heading
        self.final_done = False
        self.free_heading_aligned = False
        self.position_corrections = 0
        self.replan_attempts = 0
        if nodes:
            self.last_error = ''
        if not nodes:
            self.state = 'idle'
            self.upto = self.sent_upto = 0
            return
        self.upto = min(upto, len(nodes) - 1)
        if not self._action_client.server_is_ready():
            # Vua bat lai: chua thay Nav2 ngay - cho toi NAV2_WAIT giay (tick thu lai)
            if self.nav2_wait_since is None:
                self.nav2_wait_since = time.time()
            if time.time() - self.nav2_wait_since < NAV2_WAIT:
                self.route_id = None        # lenh ke tiep (0.2 s) goi lai start_route
                self.state = 'waiting'
                return
            self.nav2_wait_since = None
            self.route_id = route_id
            self.abort_route('khong thay Nav2 (action follow_path) - Nav2 cua xe nay chay chua?')
            return
        self.nav2_wait_since = None
        names = ' -> '.join(f'({r + 1},{c + 1})' for r, c in nodes)
        self.get_logger().info(f'[{self.name}] lo trinh #{route_id}: {names}, duoc di toi o {self.upto}')
        segs, i0 = self.segments_from_pose(0, self.upto)
        self.target_node = self.get_coordinates(*nodes[-1])
        self.sent_upto = self.upto
        self.waypoints = segs
        self.state = 'moving'
        self.send_next_waypoint()

    def segments_from_pose(self, i_from, i_to):
        """Cac doan tu VI TRI XE toi route[i_to]. Xe co the dang o giua canh
        route[i]->route[i+1] (vd lap lai duong khi dang chay): bat dau tu cho xe,
        khong lui ve diem truoc."""
        nodes = self.route
        rx, ry, _ = self.get_robot_pose()
        if rx is None:
            return self.build_segments(nodes[i_from:i_to + 1]), i_from
        # o gan xe nhat trong doan duoc phep
        i0 = min(range(i_from, i_to + 1),
                 key=lambda i: math.hypot(self.get_coordinates(*nodes[i])[0] - rx,
                                          self.get_coordinates(*nodes[i])[1] - ry))
        if i0 < i_to:
            ax, ay = self.get_coordinates(*nodes[i0])
            bx, by = self.get_coordinates(*nodes[i0 + 1])
            vx, vy = bx - ax, by - ay
            t = ((rx - ax) * vx + (ry - ay) * vy) / (vx * vx + vy * vy)
            if t > 0.05:
                i0 += 1     # da qua route[i0] ve phia route[i0+1]
        segs = self.build_segments(nodes[i0:i_to + 1])
        sx, sy = self.get_coordinates(*nodes[i0])
        off = math.hypot(sx - rx, sy - ry)
        if not segs:
            # Dich la o dang dung (xem GridNavigator.stay_put)
            if not self.stay_put(off, rx, ry, sx, sy):
                segs = [(rx, ry, sx, sy)]
        elif off > self.SNAP_DIST:
            x0, y0, x1, y1 = segs[0]
            on_line = abs((x1 - x0) * (ry - y0) - (y1 - y0) * (rx - x0)) / max(
                1e-9, math.hypot(x1 - x0, y1 - y0)) < 0.05
            if off <= self.MERGE_START_DIST or (on_line and i0 > i_from):
                segs[0] = (rx, ry, x1, y1)
            else:
                segs.insert(0, (rx, ry, sx, sy))
        return segs, i0

    @staticmethod
    def _collinear(s1, s2):
        """s2 noi tiep s1 va cung huong."""
        x0, y0, x1, y1 = s1
        a0, b0, a1, b1 = s2
        l1, l2 = math.hypot(x1 - x0, y1 - y0), math.hypot(a1 - a0, b1 - b0)
        if l1 < 1e-6 or l2 < 1e-6 or math.hypot(x1 - a0, y1 - b0) > 1e-3:
            return False
        return (abs((x1 - x0) / l1 - (a1 - a0) / l2) < 1e-3 and
                abs((y1 - y0) / l1 - (b1 - b0) / l2) < 1e-3)

    def extend(self, new_upto):
        new_upto = min(new_upto, len(self.route) - 1)
        if new_upto <= self.sent_upto:
            self.upto = max(self.upto, new_upto)
            return
        self.upto = new_upto
        if not self.busy():
            # Dang dung cho o cuoi phan duoc phep: di tiep tu cho xe dung
            segs, _ = self.segments_from_pose(self.sent_upto, new_upto)
            self.sent_upto = new_upto
            self.waypoints = segs
            self.state = 'moving'
            self.send_next_waypoint()
            return
        segs = self.build_segments(self.route[self.sent_upto:new_upto + 1])
        self.sent_upto = new_upto
        if self.waypoints and self._collinear(self.waypoints[-1], segs[0]):
            x0, y0, _, _ = self.waypoints[-1]
            self.waypoints[-1] = (x0, y0, segs[0][2], segs[0][3])
            segs = segs[1:]
        elif (not self.waypoints and self.rot is None and self.active_segment is not None
              and self._collinear(self.active_segment, segs[0])):
            # Dang chay thang dung huong: noi dai doan dang chay, gui lai
            # FollowPath tu cho xe - Nav2 thay duong, xe khong dung.
            rx, ry, _ = self.get_robot_pose()
            x0, y0 = (rx, ry) if rx is not None else self.active_segment[:2]
            self.active_segment = (x0, y0, segs[0][2], segs[0][3])
            segs = segs[1:]
            self.send_follow_path()
        elif (not self.waypoints and self.rot is not None and self.active_segment is not None
              and self._collinear(self.active_segment, segs[0])):
            # Dang xoay o dau doan: doan chua giao cho Nav2, chi can noi dai
            x0, y0, _, _ = self.active_segment
            self.active_segment = (x0, y0, segs[0][2], segs[0][3])
            segs = segs[1:]
        self.waypoints.extend(segs)
        self.refresh_path_points()

    # ------------------------------------------------------------------
    def send_next_waypoint(self):
        if not self.waypoints:
            self.active_segment = None
            if not self.route:
                self.state = 'idle'
                return
            if self.sent_upto < len(self.route) - 1:
                self.state = 'waiting'          # het phan duoc phep, cho them o
                return
            fh = self.final_heading
            if fh is None and len(self.route) >= 2:
                (ra, ca), (rb, cb) = self.route[-2], self.route[-1]
                xa, ya = self.get_coordinates(ra, ca)
                xb, yb = self.get_coordinates(rb, cb)
                fh = math.atan2(yb - ya, xb - xa)
            rx, ry, yaw = self.get_robot_pose()
            if not all(v is not None and math.isfinite(v) for v in (rx,ry,yaw)):
                self.rotation_failed('khong do duoc vi tri/goc tai dich')
                return
            r, c = self.route[-1]
            x, y = self.get_coordinates(r, c)
            pos_err = math.hypot(rx - x, ry - y)
            if pos_err > self.POSITION_TOL_M:
                # Correct only the longitudinal component without a pivot.
                # This also handles a goal at the current cell, behind us,
                # and small position drift after the one final heading turn.
                if (self.final_heading is None and not self.free_heading_aligned and
                        pos_err <= self.SMALL_CORRECTION_MAX_M and
                        abs(body_error((rx,ry,yaw),(x,y))[1]) > MAX_LATERAL):
                    # No heading was requested (e.g. delivery). Choose the
                    # closest forward/reverse approach axis once and KEEP it.
                    # Never rotate to an arbitrary East heading afterwards.
                    self.free_heading_aligned = True
                    bearing = math.atan2(y-ry,x-rx)
                    candidates = (bearing,self.normalize_angle(bearing+math.pi))
                    chosen = min(candidates,key=lambda h:abs(self.normalize_angle(h-yaw)))
                    self.final_heading = chosen
                    self.final_done = True
                    self.start_rotation(chosen,finish=True)
                    return
                self.start_position_alignment((x, y), (rx, ry, yaw))
                return
            if not self.final_done:
                self.final_done = True
                if fh is not None and yaw is not None and \
                        abs(self.normalize_angle(fh - yaw)) >= self.rot_done:
                    self.state = 'moving'
                    self.start_rotation(fh, finish=True)
                    return
            if fh is not None and abs(self.normalize_angle(fh - yaw)) >= self.rot_done:
                self.rotation_failed(f'sai so goc tai dich {abs(math.degrees(self.normalize_angle(fh - yaw))):.2f}° '
                                 f'>= {math.degrees(self.rot_done):.1f}°')
                return
            if self.state != 'done':
                heading_err = abs(math.degrees(self.normalize_angle(fh - yaw))) if fh is not None else None
                msg = f'[{self.name}] da toi ({r + 1},{c + 1}), sai so vi tri {pos_err * 100:.1f} cm'
                if heading_err is not None:
                    msg += f', sai so goc {heading_err:.2f}°'
                self.get_logger().info(msg)
            self.state = 'done'
            self.target_node = None
            return
        if (len(self.waypoints) == 1 and self.route and
                self.sent_upto == len(self.route)-1):
            x, y = self.get_coordinates(*self.route[-1])
            rx, ry, yaw = self.get_robot_pose()
            _, _, ex, ey = self.waypoints[0]
            if (rx is not None and yaw is not None and
                    math.hypot(ex-x, ey-y) < 1e-3 and
                    math.hypot(rx-x, ry-y) <= self.SMALL_CORRECTION_MAX_M):
                self.waypoints = []
                self.send_next_waypoint()
                return
        self.state = 'moving'
        super().send_next_waypoint()

    def cb_odom(self, msg):
        super().cb_odom(msg)
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.odom_v = msg.twist.twist.linear.x
        self.odom_t = time.monotonic()

    def start_position_alignment(self, target, pose):
        if self.position_corrections >= self.MAX_POSITION_CORRECTIONS:
            self.rotation_failed('vi chinh vi tri da het luot - dung, khong lap xoay')
            return
        try:
            along, across = correction_plan(pose, target)
        except ValueError as e:
            self.rotation_failed(f'sai so vi tri tai dich '
                                 f'{math.hypot(pose[0]-target[0],pose[1]-target[1])*100:.1f} cm '
                                 f'> 2 cm; {e}')
            return
        if self.rot is not None or self.goal_handle is not None:
            self.rotation_failed('con lenh chuyen dong cu - khong vi chinh dong thoi')
            return
        self.position_corrections += 1
        now = time.monotonic()
        self.correction_deadline = now + self.CORRECTION_TIMEOUT_S
        self.alignment = dict(target=target, yaw=pose[2], direction=math.copysign(1.,along),
                              phase='drive', best=abs(along), progress_t=now,
                              odom_last=self.odom_xy, travel=0., still_since=None)
        self.active_segment = None
        self.waypoints = []
        self.current_path_points = [pose[:2], target]
        self.state = 'moving'
        self.get_logger().info(f'[{self.name}] can tam khong xoay: '
                               f'{"tien" if along>0 else "lui"} {abs(along)*100:.1f} cm, '
                               f'lech ngang {abs(across)*100:.1f} cm')
        self.alignment_timer = self.create_timer(0.05, self.alignment_tick)

    def alignment_tick(self):
        a = self.alignment
        if a is None:
            return
        now = time.monotonic()
        fail = self.rotation_failed
        if self.esp_estop or self.last_cmd_t is None or time.time()-self.last_cmd_t > CMD_TIMEOUT:
            fail('vi chinh mat lenh dieu phoi/ESTOP - dung xe')
            return
        if now > self.correction_deadline:
            fail('vi chinh vi tri qua 6 giay - dung, khong lap xoay')
            return
        if now-self.odom_t > .35 or now-self.gyro_t > .35:
            fail('vi chinh mat odometry/IMU - dung xe')
            return
        pose = self.get_robot_pose()
        if not all(v is not None and math.isfinite(v) for v in (*pose, self.odom_v, self.rot_w())):
            fail('vi chinh khong co vi tri/toc do hop le')
            return
        if self.odom_xy is None or not all(math.isfinite(v) for v in self.odom_xy):
            fail('vi chinh khong co odometry hop le')
            return
        if a['odom_last'] is not None:
            a['travel'] += math.dist(self.odom_xy, a['odom_last'])
        a['odom_last'] = self.odom_xy
        if a['travel'] > MAX_TRAVEL:
            fail('vi chinh vuot 10 cm - dung xe')
            return
        x, y, yaw = pose
        heading_error = self.normalize_angle(a['yaw']-yaw)
        if abs(heading_error) >= self.rot_done:
            fail('vi chinh lech huong >= 2.5 do - dung, khong xoay bu')
            return
        along, across = body_error((x,y,a['yaw']),a['target'])
        error = math.hypot(along,across)
        if abs(across) > MAX_LATERAL and error > self.POSITION_TOL_M:
            fail(f'vi chinh lech ngang {abs(across)*100:.1f} cm - khong the truot ngang')
            return
        # Include the whole straight correction and stopping margin. Positive
        # and negative velocity use the same obstacle/unknown-space check.
        end = (x+along*math.cos(a['yaw']),y+along*math.sin(a['yaw']))
        if (now-self.alignment_costmap_t > .6 or self.alignment_costmap is None or
                not swept_costmap_clear(self.alignment_costmap,(x,y),end,self.robot_radius+.015)):
            fail('vi chinh: costmap cu/khong ro hoac vat can truoc-sau xe')
            return
        if a['phase'] == 'drive':
            if error <= ALIGN_STOP_RADIUS or along*a['direction'] <= 0.:
                a.update(phase='settle',still_since=None)
                self.publish_w(0.)
                return
            if abs(along) < a['best']-.002:
                a['best'],a['progress_t'] = abs(along),now
            elif now-a['progress_t'] > 2.:
                fail('vi chinh khong tien trien - dung, khong lap lai')
                return
            cmd = Twist()
            cmd.linear.x = a['direction']*min(MAX_SPEED,max(MIN_SPEED,1.2*abs(along)))
            cmd.angular.z = max(-.08,min(.08,2.*heading_error))
            self.cmd_pub.publish(cmd)  # smoother -> collision_monitor -> ESP32
            return
        self.publish_w(0.)
        if abs(self.odom_v)<.01 and abs(self.rot_w())<.04:
            if a['still_since'] is None:
                a['still_since'] = now
        else:
            a['still_since'] = None
        if a['still_since'] is None or now-a['still_since']<.4:
            return
        if error > self.POSITION_TOL_M:
            fail(f'sai so vi tri sau vi chinh {error*100:.1f} cm > 2 cm - dung')
            return
        self.get_logger().info(f'[{self.name}] can tam xong, sai so {error*100:.1f} cm, '
                               f'quang duong odom {a["travel"]*100:.1f} cm, khong xoay them')
        self.alignment = None
        self.correction_deadline = None
        self.current_path_points = []
        self.destroy_timer(self.alignment_timer)
        self.alignment_timer = None
        self.send_next_waypoint()  # recheck radius AND final yaw, not just projection

    def send_follow_path(self):
        self._seq += 1
        seq = self._seq
        x0, y0, x1, y1 = self.active_segment
        goal = FollowPath.Goal()
        goal.path = self.make_path(x0, y0, x1, y1)
        # Use the gentle approach also on the LAST original segment: center
        # first, final yaw second. Earlier corners keep the normal controller.
        final_segment = False
        if self.route and not self.waypoints:
            gx, gy = self.get_coordinates(*self.route[-1])
            final_segment = math.hypot(x1-gx, y1-gy) < 1e-3
        precision = self.position_corrections > 0 or final_segment
        goal.controller_id = 'PrecisionFollowPath' if precision else 'FollowPath'
        goal.goal_checker_id = 'precision_goal_checker' if precision else 'general_goal_checker'
        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = 'precision_progress_checker' if precision else 'progress_checker'
        self.correction_deadline = (time.monotonic()+self.CORRECTION_TIMEOUT_S
                                    if self.position_corrections else None)
        fut = self._action_client.send_goal_async(goal)
        fut.add_done_callback(lambda f, s=seq: self._on_goal_response(f, s))

    def _on_goal_response(self, future, seq):
        gh = future.result()
        if seq != self._seq:
            return              # da co goal moi hon (noi dai / dung xe)
        if not gh.accepted:
            self.abort_route('Nav2 tu choi doan duong')
            return
        self.goal_handle = gh
        self.is_canceling = False
        gh.get_result_async().add_done_callback(lambda f, s=seq: self._on_result(f, s))

    def _on_result(self, future, seq):
        if seq != self._seq:
            return              # goal cu bi thay the - Nav2 bao ABORTED, bo qua
        status = getattr(future.result(), 'status', None)
        self.goal_handle = None
        self.correction_deadline = None
        if status == GoalStatus.STATUS_CANCELED:
            return
        if status is not None and status != GoalStatus.STATUS_SUCCEEDED:
            if self.position_corrections:
                # Don't restart a failed centimeter-long correction as a full
                # navigation segment with another large heading change.
                rx, ry, _ = self.get_robot_pose()
                x, y = self.get_coordinates(*self.route[-1])
                if rx is not None and math.hypot(rx-x, ry-y) <= self.POSITION_TOL_M:
                    self.send_next_waypoint()  # recheck final heading and position
                else:
                    self.halt()
                    self.state = 'precision_failed'
                    self.last_error = 'vi chinh vi tri khong hoi tu - dung, khong lap xoay'
                    self.get_logger().error(f'[{self.name}] {self.last_error}')
                return
            self.replan_attempts += 1
            if self.replan_attempts > self.max_replan_attempts:
                self.abort_route(f'Nav2 that bai {self.replan_attempts} lan lien tiep')
                return
            self.get_logger().warn(f'[{self.name}] Nav2 khong toi duoc (status={status}), '
                                   f'di tiep doan dang do {self.replan_attempts}/{self.max_replan_attempts}')
            self.retry_active_segment()
            return
        self.replan_attempts = 0
        self.send_next_waypoint()

    def rotation_failed(self, reason):
        # Generic 'failed' triggers traffic_manager's route retry. A heading
        # failure must terminate the goal, otherwise a new route repeats the
        # supposedly single turn indefinitely.
        self.halt()
        self.state = 'precision_failed'
        self.last_error = reason
        self.get_logger().error(f'[{self.name}] {reason}')

    def abort_route(self, reason):
        self.halt()
        self.state = 'failed'
        self.get_logger().error(f'[{self.name}] {reason} - dung xe, bao traffic_manager')

    def schedule_replan(self, delay=1.0):
        # Lap lai duong la viec cua traffic_manager
        self.halt()
        self.state = 'failed'

    # ------------------------------------------------------------------
    def costmap_callback(self, msg):
        """Nhu GridNavigator nhung BO vat can quanh xe khac, va khong tu lap lai
        duong: chi dung xe + bao 'blocked' de traffic_manager lap lai."""
        self.alignment_costmap = msg
        self.alignment_costmap_t = time.monotonic()
        if not self.map_received:
            return
        obstacles = self.collect_obstacles(msg)
        now = time.time()
        obstacles = self.remove_robot_obstacles(obstacles, now)
        nodes, edges = set(), set()
        for r, c in self.board.nodes():
            x, y = self.get_coordinates(r, c)
            if self.point_blocked(x, y, obstacles, self.node_clearance):
                nodes.add((r, c))
        for (r, c), (nr, nc) in self.board.edges():
            x1, y1 = self.get_coordinates(r, c)
            x2, y2 = self.get_coordinates(nr, nc)
            if self.segment_blocked(x1, y1, x2, y2, obstacles, self.edge_clearance):
                edges.add(self.edge_key(r, c, nr, nc))
        self.dyn_blocked_nodes, self.dyn_blocked_edges = nodes, edges
        if self.busy() and self.state != 'blocked' and self.is_path_blocked(obstacles):
            if self.alignment is not None:
                self.rotation_failed('vat can khi vi chinh - dung, khong tu lap lai')
                return
            self.get_logger().warn(f'[{self.name}] vat can tren duong - dung xe, cho lap lai duong')
            self.halt()
            self.state = 'blocked'

    # ------------------------------------------------------------------
    def tick(self):
        now = time.time()
        if self.correction_deadline is not None and time.monotonic() > self.correction_deadline:
            self.halt()
            self.state = 'precision_failed'
            self.last_error = 'vi chinh vi tri qua 6 giay - dung, khong lap xoay'
            self.get_logger().error(f'[{self.name}] {self.last_error}')
        if (self.last_cmd_t is not None and now - self.last_cmd_t > CMD_TIMEOUT
                and not self.cmd_lost):
            self.cmd_lost = True
            if self.busy():
                self.get_logger().error(f'[{self.name}] mat lenh tu traffic_manager '
                                        f'{now - self.last_cmd_t:.1f} s - dung xe')
            self.halt()
            if self.state == 'moving':
                self.state = 'waiting'
        x, y, yaw = self.get_robot_pose()
        msg = String()
        msg.data = json.dumps({
            'name': self.name, 't': now,
            'x': x, 'y': y, 'yaw': yaw,
            'state': 'estop' if self.esp_estop else self.state, 'route_id': self.route_id,
            'route_goal': self.route[-1] if self.route else None,
            'route_mode':getattr(self,'manager_mode',None),
            'route_busy':self.busy(),
            'linear_speed_mps':self.odom_v,
            'angular_speed_radps':self.rot_w(),
            'estop': self.esp_estop,
            'error': self.last_error,
            'blocked_nodes': sorted(self.dyn_blocked_nodes),
            'blocked_edges': sorted([list(map(list, e)) for e in self.dyn_blocked_edges]),
        })
        self.status_pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    names = [a for a in argv if not a.startswith('--')]
    grid_file = None
    if '--grid' in argv:
        grid_file = argv[argv.index('--grid') + 1]
        names = [n for n in names if n != grid_file]
    if not names:
        sys.exit('Thieu ten xe: python3 fleet_agent.py agv1 [--real] [--grid FILE] --ros-args ...')
    agent = FleetAgent(names[0], use_sim_time='--sim-time' in argv, grid_file=grid_file)
    # MOT luong: moi callback von da chung mot nhom loai tru nhau (chay noi
    # tiep). MultiThreadedExecutor voi imu/data + odom 50 Hz (xe that) thi luong
    # ranh quay vong kiem tra lai lien tuc - do duoc 77-90% CPU, agent doi CPU,
    # vi tri xe kia toi tre -> bao nut bi chan sai.
    ex = SingleThreadedExecutor()
    ex.add_node(agent)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            agent.halt()
        agent.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
