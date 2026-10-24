#!/usr/bin/env python3
"""Nhiem vu DO HANG cho doi xe: toi o LAY hang (mac dinh cot 5, quay Dong) -> do
rong x cao hang phia truoc (cargo_measure.py: LiDAR + camera) -> phan loai
(cargo.yaml classes) -> chay ve o TRA hang (mac dinh hang 1, cot theo loai).
Xe KHONG cho hang, chi chay ve o.

    python3 cargo_task.py agv1 agv2              # chay cung traffic_manager.py
    > 1 3          xe 1 do hang o (3,5)  (so thu hai = HANG cua o lay hang o cot 5)
    > 2 4          xe 2 do hang o (4,5)  (hai xe lam song song duoc)
    python3 cargo_task.py agv1 agv2 --sim        # mo phong: camera mac dinh, bo qua camera_agvN.yaml
Ngoai terminal: ros2 topic pub --once /fleet/cargo std_msgs/msg/String "{data: '1 3'}"

O lay / o tra / huong / khoang dat hang: cargo.yaml place.*. Thong so camera rieng
tung xe: camera_agvN.yaml (tools/cam_calib.py). Di chuyen giao cho traffic_manager
qua /fleet/goal (van giu cho, ne xe kia). Xe da giao dang do o o tra ma xe khac can
o do thi lui sang hang ke ben nhuong o.
Ket qua: in ra, /fleet/cargo_result (JSON), anh chu thich + results.csv trong
~/agv_logs/cargo/, chu kich thuoc tren o tra trong RViz (/fleet/markers).
"""
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

import cargo_measure as cm
from grid_board import Board, config_path
from grid_navigator import HEADING_NAMES, parse_heading
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))
import fleet_localize as fl   # noqa: E402  (ICP anh quet <-> /map)

def hd_name(h):
    return HEADING_NAMES[parse_heading(h)[0]]


OUT = os.path.expanduser('~/agv_logs/cargo')
ARRIVE_D = 0.05            # m: coi la da toi o
ARRIVE_YAW = math.radians(2.5)  # cung nguong goc tai goal voi fleet_agent/grid_real.yaml
SETTLE = 1.0               # s dung yen truoc khi do
TIMEOUT = 300.0            # s cho moi chang (cho xe kia nhuong + di vong: 2026-10-14 > 180 s)
# traffic_manager bo lo trinh sau 5 lan "blocked" (~10 s, vd nguoi dung canh sa ban):
# cho RETRY_WAIT s roi giao lai cung dich, toi da RETRY_MAX lan moi chang.
RETRY_MAX = 3
RETRY_WAIT = 5.0
# O lay hang (xe dung yen): khop anh quet voi /map (ICP nhu tools/fleet_localize.py);
# AMCL lech hon muc nay thi gui lai /agvN/initialpose -> chang ve o tra di dung.
# (2026-10-14: phong co xe may / do dac ngoai ban do, AMCL troi 5 cm, 4-7 do sau 1.8 m.)
RELOC_SHIFT = 0.015        # m
RELOC_YAW = math.radians(1.5)


class Task:
    def __init__(self, car, row):
        self.car, self.row = car, row      # row: hang o lay hang (tu 1)
        self.state = 'pickup'              # pickup | settle | collect | waitslot | deliver | done | failed
        self.t = 0.0                       # luc vao trang thai hien tai
        self.target = None                 # (r, c) tu 0
        self.slot = None                   # cot o tra (tu 1)
        self.result = None
        self.tries = 0
        self.reject = None
        self.gaveup = None          # traffic_manager bao "[xe] dung: ..." (bo lo trinh)
        self.retries = 0
        self.retry_at = None
        self.heading = ''
        self.noted = None
        self.asked = None


class CargoTask(Node):
    def __init__(self, cars, sim=False):
        super().__init__('cargo_task')
        self.cfg = cm.load_cfg()
        pl = self.cfg['place']
        self.board = Board.load(config_path(True))
        self.pick_col = int(pl['pickup_col'])
        self.pick_rows = [int(r) for r in pl['pickup_rows']]
        self.pick_hd = pl['pickup_heading']
        self.pick_yaw = parse_heading(self.pick_hd, self.board.yaw)[1]
        self.drop_row = int(pl['deliver_row'])
        self.cars = cars
        self.ccfg = {c: cm.car_cfg(self.cfg, c, sim) for c in cars}
        self.status, self.scans, self.img, self.lidar_x = {}, {}, {}, {}
        self.static_tf = {c: {} for c in cars}
        self.tasks = {}
        self.goal_pub = self.create_publisher(String, '/fleet/goal', 10)
        self.res_pub = self.create_publisher(String, '/fleet/cargo_result', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.create_subscription(String, '/fleet/status', self.cb_status, 50)
        self.create_subscription(String, '/fleet/cargo', lambda m: self.command(m.data), 10)
        self.create_subscription(Log, '/rosout', self.cb_rosout, 100)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_tree = None
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, latched)
        self.amcl, self.init_pubs = {}, {}
        for c in cars:
            self.create_subscription(PoseWithCovarianceStamped, f'/{c}/amcl_pose',
                                     lambda m, c=c: self.amcl.__setitem__(c, m),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.init_pubs[c] = self.create_publisher(PoseWithCovarianceStamped, f'/{c}/initialpose', 1)
        static = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        for c in cars:
            self.scans[c] = []
            self.create_subscription(LaserScan, f'/{c}/scan', lambda m, c=c: self.cb_scan(c, m),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, f'/{c}/camera/image_raw',
                                     lambda m, c=c: self.img.__setitem__(c, (time.time(), m)), 2)
            self.create_subscription(TFMessage, f'/{c}/tf_static', lambda m, c=c: self.cb_static(c, m), static)
        self.labels = {}
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_labels)
        os.makedirs(OUT, exist_ok=True)
        rows = ','.join(map(str, self.pick_rows))
        self.get_logger().info(
            f'Do hang: {", ".join(cars)}. Lay hang o cot {self.pick_col} (hang {rows}), quay '
            f'{hd_name(self.pick_hd)}; tra o hang {self.drop_row}. Lenh "<xe> <hang>", '
            f'vd "1 3" = xe 1 do hang o (3,{self.pick_col})')
        for c in cars:
            cam = self.ccfg[c]['camera']
            src = ('MO PHONG (camera mac dinh)' if sim else
                   f'{os.path.basename(cm.calib_path(c))}' if os.path.exists(cm.calib_path(c)) else
                   'CHUA HIEU CHUAN - dang dung so mac dinh, sai vai mm toi vai cm (tools/cam_calib.py)')
            log = self.get_logger().info if sim or os.path.exists(cm.calib_path(c)) else self.get_logger().warn
            log(f'[{c}] camera: {src}; fx {cam["fx"]:.0f}, lap o x {cam["x"]:.3f} z {cam["z"]:.3f} m')

    # -------------------------------------------------------------- dau vao
    def cb_status(self, m):
        d = json.loads(m.data)
        self.status[d['name']] = d

    def cb_map(self, m):
        from scipy.spatial import cKDTree
        W, H, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        occ = np.argwhere(np.array(m.data).reshape(H, W) > 50)
        self.map_pts = np.c_[ox + (occ[:, 1] + 0.5) * res, oy + (occ[:, 0] + 0.5) * res]
        self.map_tree = cKDTree(self.map_pts)

    def relocalize(self, car, since):
        """Xe dung yen: ICP anh quet (tu `since`) voi /map quanh vi tri AMCL; lech thi
        gui initialpose. Khong co AMCL (mo phong) / khop kem thi bo qua."""
        if self.map_tree is None or car not in self.amcl:
            return
        sc = [x for t, x in self.scans[car] if t > since][-10:]
        if len(sc) < 5:
            return
        RA = []
        for amin, inc, rr in sc:
            rr = np.asarray(rr, float)
            ang = amin + np.arange(len(rr)) * inc
            ok = np.isfinite(rr) & (rr > 0.15) & (rr < 6)
            RA.append(np.c_[rr[ok], ang[ok]])
        RA = np.vstack(RA)
        p0 = self.amcl[car].pose.pose
        th = fl.yaw_of(p0.orientation)
        lx, ly = p0.position.x + fl.LIDAR_X * math.cos(th), p0.position.y + fl.LIDAR_X * math.sin(th)
        p, (med, good) = fl.icp(self.map_tree, self.map_pts, RA, lx, ly, th)
        bx, by = p[0] - fl.LIDAR_X * math.cos(p[2]), p[1] - fl.LIDAR_X * math.sin(p[2])
        shift = math.hypot(bx - p0.position.x, by - p0.position.y)
        dyaw = math.atan2(math.sin(p[2] - th), math.cos(p[2] - th))
        if good < fl.MIN_GOOD:
            self.get_logger().warn(f'[{car}] khop ban do kem ({good * 100:.0f}%) - giu AMCL')
            return
        if shift < RELOC_SHIFT and abs(dyaw) < RELOC_YAW:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(bx), float(by)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 ** 2
        msg.pose.covariance[35] = math.radians(1.5) ** 2
        self.init_pubs[car].publish(msg)
        self.get_logger().info(f'[{car}] sua AMCL theo ban do: lech {shift * 100:.1f} cm, '
                               f'{math.degrees(dyaw):+.1f} do ({good * 100:.0f}% tia khop)')

    def cb_scan(self, c, m):
        self.scans[c].append((time.time(), (m.angle_min, m.angle_increment, list(m.ranges))))
        del self.scans[c][:-30]

    def cb_static(self, c, m):
        for t in m.transforms:
            q = t.transform.rotation
            self.static_tf[c][t.child_frame_id] = (
                t.header.frame_id, t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        x, y, f = 0.0, 0.0, 'lidar_link'
        for _ in range(10):              # lidar_link -> ... -> base_footprint (2D)
            if f == 'base_footprint':
                self.lidar_x[c] = x
                return
            if f not in self.static_tf[c]:
                return
            p, tx, ty, yaw = self.static_tf[c][f]
            x, y = tx + x * math.cos(yaw) - y * math.sin(yaw), ty + x * math.sin(yaw) + y * math.cos(yaw)
            f = p

    def cb_rosout(self, m):
        if m.name != 'traffic_manager' or m.level < 30:
            return
        for tk in self.tasks.values():
            if not m.msg.startswith(f'[{tk.car}]') or tk.state not in ('pickup', 'deliver'):
                continue
            if m.level >= 40:
                tk.reject = m.msg          # tu choi dich (o la dich xe kia, ngoai sa ban...)
            elif m.msg.startswith(f'[{tk.car}] dung:') and 'ESTOP' not in m.msg:
                tk.gaveup = m.msg          # bo lo trinh vi bi chan / Nav2 that bai lien tiep

    def command(self, text):
        tok = text.split()
        if len(tok) != 2 or not tok[1].isdigit():
            self.get_logger().error(f'Lenh "{text}": can "<xe> <hang>", vd "1 3"')
            return
        car = self.cars[int(tok[0]) - 1] if tok[0].isdigit() and 1 <= int(tok[0]) <= len(self.cars) else tok[0]
        if car not in self.cars:
            self.get_logger().error(f'Khong co xe "{tok[0]}"')
            return
        row = int(tok[1])
        if row not in self.pick_rows:
            self.get_logger().error(f'Hang {row}: o lay hang chi o hang {self.pick_rows} (cargo.yaml pickup_rows)')
            return
        if car in self.tasks and self.tasks[car].state not in ('done', 'failed'):
            self.get_logger().error(f'[{car}] dang lam nhiem vu khac')
            return
        for o in self.tasks.values():
            if o.car != car and o.row == row and o.state in ('pickup', 'settle', 'collect'):
                self.get_logger().error(f'[{car}] o ({row},{self.pick_col}) dang do {o.car} lam')
                return
        tk = Task(car, row)
        self.tasks[car] = tk
        self.go(tk, (row - 1, self.pick_col - 1), self.pick_hd)

    # ------------------------------------------------------------ dieu khien
    def lbl(self, node):
        return f'({node[0] + 1},{node[1] + 1})'

    def go(self, tk, node, heading=''):
        if node != tk.target:
            tk.retries = 0                 # chang moi
        tk.target, tk.t, tk.reject, tk.heading = node, time.time(), None, heading
        tk.gaveup, tk.retry_at = None, None
        r, c = node
        self.goal_pub.publish(String(data=f'{self.cars.index(tk.car) + 1} {r + 1} {c + 1} {heading}'.strip()))
        self.get_logger().info(f'[{tk.car}] -> {self.lbl(node)}' +
                               (f' quay {hd_name(heading)}' if heading else ''))

    def arrived(self, tk, yaw=None):
        s = self.status.get(tk.car)
        if not s or s.get('x') is None or time.time() - tk.t < 1.5:
            return False
        x, y = self.board.xy(*tk.target)
        if math.hypot(s['x'] - x, s['y'] - y) > ARRIVE_D or s['state'] not in ('done', 'idle'):
            return False
        if yaw is not None:
            return abs(math.atan2(math.sin(s['yaw'] - yaw), math.cos(s['yaw'] - yaw))) < ARRIVE_YAW
        return True

    def fail(self, tk, why):
        tk.state = 'failed'
        self.get_logger().error(f'[{tk.car}] DO HANG ({tk.row},{self.pick_col}) THAT BAI: {why}')
        self.res_pub.publish(String(data=json.dumps({'car': tk.car, 'row': tk.row, 'col': self.pick_col,
                                                     'ok': False, 'error': why})))

    def tick(self):
        now = time.time()
        for tk in list(self.tasks.values()):
            if tk.state in ('done', 'failed'):
                continue
            s = self.status.get(tk.car, {})
            if tk.state in ('pickup', 'deliver'):
                if tk.reject:
                    self.fail(tk, f'traffic_manager tu choi: {tk.reject}')
                elif tk.gaveup and tk.retry_at is None:
                    if tk.retries >= RETRY_MAX:
                        self.fail(tk, f'bi chan sau {RETRY_MAX} lan giao lai: {tk.gaveup}')
                    else:
                        tk.retries += 1
                        tk.retry_at = now + RETRY_WAIT
                        self.get_logger().warn(f'[{tk.car}] traffic_manager bo lo trinh ({tk.gaveup}) - '
                                               f'cho {RETRY_WAIT:.0f} s roi giao lai (lan {tk.retries}/{RETRY_MAX})')
                elif tk.retry_at is not None:
                    if now >= tk.retry_at:
                        self.go(tk, tk.target, tk.heading)
                elif s.get('state') == 'estop':
                    # 'failed' thoang qua (Nav2 tu choi mot doan) thi traffic_manager tu lap lai
                    # duong (toi 5 lan); bo han thi no bao ERROR "[agvN] dung: ..." -> cb_rosout.
                    self.fail(tk, 'xe bao ESTOP')
                elif now - tk.t > TIMEOUT:
                    self.fail(tk, f'qua {TIMEOUT:.0f} s chua toi {self.lbl(tk.target)}')
                elif tk.state == 'pickup' and self.arrived(tk, self.pick_yaw):
                    tk.state, tk.t = 'settle', now
                elif tk.state == 'deliver' and self.arrived(tk):
                    tk.state = 'done'
                    self.get_logger().info(f'[{tk.car}] XONG: hang ({tk.row},{self.pick_col}) -> '
                                           f'da ve o {self.lbl(tk.target)}')
            elif tk.state == 'settle' and now - tk.t > SETTLE:
                self.relocalize(tk.car, tk.t)
                tk.state, tk.t = 'collect', now
            elif tk.state == 'collect':
                sc = [x for t, x in self.scans[tk.car] if t > tk.t]
                im = self.img.get(tk.car)
                if len(sc) >= self.cfg['lidar']['scans'] and im and im[0] > tk.t:
                    self.measure(tk, sc, im[1])
                elif now - tk.t > 10:
                    why = ('khong co anh camera moi - esp32_cam_node.py chay chua? ESP32-CAM co nguon?'
                           if not (im and im[0] > tk.t) else 'thieu anh quet LiDAR')
                    self.fail(tk, why)
            elif tk.state == 'waitslot':
                self.try_slot(tk)

    def measure(self, tk, scans, m):
        cfg = self.ccfg[tk.car]
        frame = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        lx = self.lidar_x.get(tk.car, cfg['lidar']['x'])
        try:
            res, vis = cm.measure(frame, scans[-cfg['lidar']['scans']:], cfg, lx)
        except ValueError as e:
            tk.tries += 1
            if tk.tries < 3:
                self.get_logger().warn(f'[{tk.car}] do loi ({e}) - do lai')
                tk.state, tk.t = 'settle', time.time()
            else:
                stamp = time.strftime('%Y%m%d-%H%M%S')
                path = f'{OUT}/{stamp}_{tk.car}_r{tk.row}_LOI.png'
                cv2.imwrite(path, frame)
                self.fail(tk, f'khong do duoc: {e} (anh: {path})')
            return
        cls = cm.classify(cfg, res['width'], res['height'])
        stamp = time.strftime('%Y%m%d-%H%M%S')
        path = f'{OUT}/{stamp}_{tk.car}_r{tk.row}.png'
        cv2.imwrite(path, vis)
        cv2.imwrite(path.replace('.png', '_raw.png'), frame)   # anh goc: phan tich lai offline
        tk.result = dict(res, car=tk.car, row=tk.row, col=self.pick_col, image=path,
                         cls=cls['name'] if cls else None, slot=cls['col'] if cls else None, ok=cls is not None)
        pick = self.lbl((tk.row - 1, self.pick_col - 1))
        msg = (f'[{tk.car}] Hang o {pick}: rong {res["width"] * 100:.1f} cm, cao {res["height"] * 100:.1f} cm '
               f'(Z {res["z_cam"] * 100:.1f} cm, mat lech {res["face_yaw_deg"]:+.1f} do) -> '
               + (f'loai "{cls["name"]}" -> o ({self.drop_row},{cls["col"]})' if cls else 'KHONG khop loai nao'))
        self.get_logger().info(msg)
        for w in res['warnings']:
            self.get_logger().warn(f'[{tk.car}] canh bao: {w}')
        new = not os.path.exists(f'{OUT}/results.csv')
        with open(f'{OUT}/results.csv', 'a') as f:
            if new:
                f.write('thoi_gian,xe,o_lay,rong_cm,cao_cm,z_cm,lech_do,loai,o_tra,canh_bao,anh\n')
            f.write(f'{stamp},{tk.car},"{pick}",{res["width"] * 100:.2f},{res["height"] * 100:.2f},'
                    f'{res["z_cam"] * 100:.2f},{res["face_yaw_deg"]:.1f},{tk.result["cls"]},'
                    f'"({self.drop_row},{tk.result["slot"]})","{"; ".join(res["warnings"])}",{path}\n')
        out = {k: v for k, v in tk.result.items() if k != 'edges_px'}
        self.res_pub.publish(String(data=json.dumps(out)))
        if not cls:
            self.fail(tk, 'kich thuoc khong khop loai nao trong cargo.yaml')
            return
        tk.slot = cls['col']
        self.labels[tk.slot] = f'{cls["name"]}: {res["width"] * 100:.1f}x{res["height"] * 100:.1f} cm'
        tk.state, tk.t = 'waitslot', time.time()
        self.try_slot(tk)

    def try_slot(self, tk):
        """O tra dang la dich cua xe khac / co xe khac do -> cho (xe da giao xong thi nhuong)."""
        node = (self.drop_row - 1, tk.slot - 1)
        x, y = self.board.xy(*node)
        for o in self.tasks.values():
            if o is not tk and o.state == 'deliver' and o.target == node:
                return self.wait_note(tk, f'o {self.lbl(node)} la dich cua {o.car}')
        for name, s in self.status.items():
            if name != tk.car and s.get('x') is not None and math.hypot(s['x'] - x, s['y'] - y) < 0.3:
                o = self.tasks.get(name)
                if (o is None or o.state in ('done', 'failed')) and s.get('state') in ('done', 'idle') \
                        and tk.asked != name:
                    tk.asked = name
                    free = self.free_node_beside(tk.slot - 1, name)
                    if free is not None:
                        self.goal_pub.publish(String(
                            data=f'{self.cars.index(name) + 1} {free[0] + 1} {free[1] + 1}'))
                        self.get_logger().info(f'[{name}] nhuong o {self.lbl(node)} cho {tk.car} '
                                               f'-> sang {self.lbl(free)}')
                return self.wait_note(tk, f'{name} dang o {self.lbl(node)}')
        tk.state = 'deliver'
        self.go(tk, node)

    def free_node_beside(self, c, who):
        """O o hang ke hang tra (vao trong sa ban), gan cot c nhat, khong co xe, khong
        phai o lay hang."""
        r = self.drop_row if self.drop_row < self.board.rows else self.drop_row - 2   # tu 0
        busy = set()
        for name, s in self.status.items():
            if name != who and s.get('x') is not None:
                busy.add(self.board.closest(s['x'], s['y']))
        busy |= {o.target for o in self.tasks.values() if o.state in ('pickup', 'deliver')}
        busy |= {(pr - 1, self.pick_col - 1) for pr in self.pick_rows}
        for cc in sorted(range(self.board.cols), key=lambda k: abs(k - c)):
            if (r, cc) not in busy:
                return (r, cc)
        return None

    def wait_note(self, tk, why):
        if tk.noted != why:
            tk.noted = why
            self.get_logger().info(f'[{tk.car}] cho: {why}')
        if time.time() - tk.t > TIMEOUT:
            self.fail(tk, f'cho qua lau: {why}')

    def publish_labels(self):
        ma = MarkerArray()
        for k, text in self.labels.items():
            x, y = self.board.xy(self.drop_row - 1, k - 1)
            t = Marker()
            t.header.frame_id, t.ns, t.id, t.type = 'map', 'cargo_result', k, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y - 0.18, 0.15
            t.scale.z = 0.06
            t.color = ColorRGBA(r=1.0, g=0.85, b=0.2, a=1.0)
            t.text = text
            ma.markers.append(t)
        if ma.markers:
            self.mk_pub.publish(ma)


def stdin_loop(node):
    if sys.stdin.isatty():
        print(f'Nhap "<xe> <hang>" (vd "1 3" = xe 1 do hang o (3,{node.pick_col})), "q" thoat.')
    for line in sys.stdin:
        line = line.strip()
        if line in ('q', 'quit', 'exit'):
            break
        if line:
            node.command(line)
    else:
        return                 # het dau vao (chay nen): van nhan lenh qua /fleet/cargo
    rclpy.try_shutdown()


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    sim = '--sim' in argv
    cars = [a for a in argv if not a.startswith('--')] or ['agv1', 'agv2']
    node = CargoTask(cars, sim)
    threading.Thread(target=stdin_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
