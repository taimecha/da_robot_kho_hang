#!/usr/bin/env python3
"""Tu tim xe dang o dau tren ban do roi bao cho AMCL - thay cho viec dat xe
dung cho cu hoac bam "2D Pose Estimate" trong RViz.

    cd ~/robot_ws_3 && python3 tools/auto_initial_pose.py

Chay SAU localization.launch.py, luc xe DUNG YEN. Khong dieu khien xe - chi
phat mot message /initialpose cho AMCL.

CACH LAM
  Thu dat anh quet LiDAR len moi o trong cua /map (buoc 10 cm) theo moi huong
  (buoc 2 do), cham diem bang khoang cach tu moi tia toi tuong gan nhat, roi
  tinh chinh ung vien tot nhat toi 1 cm / 0.25 do.

KHI NAO NO TU CHOI
  Phong gan doi xung (hinh chu nhat) thi xe o goc nay va xe o goc doi dien
  quay nguoc 180 do nhin thay gan giong nhau. Neu ung vien thu hai (o cho
  khac) gan bang ung vien tot nhat, cong cu KHONG doan ma bao ban dung
  2D Pose Estimate. Doan sai la AMCL bam sai cho ngay tu dau."""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt
import tf2_ros

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

TRUNC = 0.30          # m - tia xa tuong hon the nay deu bi cham nhu nhau
HIT = 0.10            # m - tia cach tuong duoi muc nay coi la "trung tuong"
MIN_HIT = 0.60        # it hon 60% tia trung tuong -> khong tin
MIN_MARGIN = 0.15     # ung vien thu hai phai kem it nhat 15 diem %


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Loc(Node):
    def __init__(self, ns=''):
        # ns (vd 'agv2'): xe trong che do 2 xe - topic /agv2/scan_filtered, /agv2/initialpose,
        # cay TF rieng /agv2/tf. Ban do /map dung chung.
        remap = ['--ros-args', '-r', f'/tf:=/{ns}/tf', '-r', f'/tf_static:=/{ns}/tf_static'] if ns else None
        super().__init__('auto_initial_pose', cli_args=remap)
        pre = f'/{ns}' if ns else ''
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        tf2_ros.TransformListener(self.buf, self)
        self.grid = None
        self.scan = None
        self.amcl_pose = None
        self.create_subscription(
            OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m),
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(
            LaserScan, f'{pre}/scan_filtered', lambda m: setattr(self, 'scan', m),
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=2))
        self.create_subscription(
            PoseWithCovarianceStamped, f'{pre}/amcl_pose',
            lambda m: setattr(self, 'amcl_pose', (time.monotonic(), m)), 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, f'{pre}/initialpose', 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true',
                    help='chi in ket qua, khong gui cho AMCL')
    ap.add_argument('--ns', default='', help='agv1 | agv2 khi chay 2 xe (tools/fleet_real.sh)')
    ap.add_argument('--hint-cell', metavar='ROW,COL',
                    help='o xe da duoc nguoi van hanh xac nhan; chi tim quanh o nay')
    ap.add_argument('--hint-heading', choices=('d', 'b', 't', 'n'),
                    help='huong xe da xac nhan: d=Dong, b=Bac, t=Tay, n=Nam')
    a = ap.parse_args()
    if bool(a.hint_cell) != bool(a.hint_heading):
        ap.error('--hint-cell va --hint-heading phai di cung nhau')

    rclpy.init()
    n = Loc(a.ns)
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    end = time.time() + 25
    while time.time() < end and (n.grid is None or n.scan is None):
        time.sleep(0.2)
    if n.grid is None:
        print('Khong nhan duoc /map. localization.launch.py co dang chay khong?')
        rclpy.shutdown(); return 2
    if n.scan is None:
        print(f'Khong nhan duoc {f"/{a.ns}" if a.ns else ""}/scan_filtered.')
        rclpy.shutdown(); return 2

    g, s = n.grid, n.scan
    w, h, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    occ = np.array(g.data, dtype=np.int16).reshape(h, w)
    dist = distance_transform_edt(occ < 65) * res

    # Vi tri LiDAR so voi tam xe (lay tu URDF qua TF). Tim theo LiDAR, gui
    # cho AMCL theo base_footprint.
    try:
        bl = n.buf.lookup_transform('base_footprint', s.header.frame_id, Time(),
                                    timeout=Duration(seconds=3.0))
        lx, ly = bl.transform.translation.x, bl.transform.translation.y
        lth = yaw_of(bl.transform.rotation)
    except Exception as e:
        print(f'Khong tra duoc TF base_footprint -> {s.header.frame_id}: '
              f'{type(e).__name__}. bringup_real.launch.py co dang chay khong?')
        rclpy.shutdown(); return 2

    r = np.array(s.ranges, dtype=np.float64)
    ang = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = (r > s.range_min) & (r < s.range_max) & np.isfinite(r)
    r, ang = r[ok], ang[ok]
    if len(r) < 50:
        print(f'Anh quet chi co {len(r)} tia hop le - qua it de tim.')
        rclpy.shutdown(); return 2

    def cost(px, py):
        ix = np.floor((px - ox) / res).astype(np.int64)
        iy = np.floor((py - oy) / res).astype(np.int64)
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        d = np.full(px.shape, TRUNC)
        d[inside] = np.minimum(dist[iy[inside], ix[inside]], TRUNC)
        return d

    print(f'Dang tim tren ban do {w*res:.1f} x {h*res:.1f} m voi {len(r)} tia...')
    fy, fx = np.nonzero(occ == 0)
    keep = (fy % 2 == 0) & (fx % 2 == 0)              # buoc 10 cm
    cx = ox + (fx[keep] + 0.5) * res
    cy = oy + (fy[keep] + 0.5) * res
    hint = None
    angles = np.radians(np.arange(0.0, 360.0, 2.0))
    if a.hint_cell:
        from grid_board import Board, config_path
        board = Board.load(config_path(True))
        try:
            row, col = (int(v) for v in a.hint_cell.split(','))
        except (ValueError, TypeError):
            ap.error('--hint-cell can dang ROW,COL, vd 5,2')
        if not (1 <= row <= board.rows and 1 <= col <= board.cols):
            ap.error('--hint-cell nam ngoai sa ban')
        hx, hy = board.xy(row - 1, col - 1)
        hth = board.yaw + {'d': 0, 'b': math.pi / 2,
                           't': math.pi, 'n': -math.pi / 2}[a.hint_heading]
        hlx = hx + lx * math.cos(hth) - ly * math.sin(hth)
        hly = hy + lx * math.sin(hth) + ly * math.cos(hth)
        local = np.hypot(cx - hlx, cy - hly) <= 0.45
        cx, cy = cx[local], cy[local]
        angles = hth + lth + np.radians(np.arange(-24.0, 24.1, 2.0))
        hint = (hx, hy, hth)
        print(f'Chi tim quanh o ({row},{col}) huong {a.hint_heading}: '
              f'{len(cx)} vi tri, {len(angles)} goc; van kiem tra do khop LiDAR.')
    if len(cx) == 0:
        print('Ban do khong co o trong de tu dinh vi quanh vi tri da xac nhan.')
        n.destroy_node()
        rclpy.shutdown()
        return 2
    sub = np.linspace(0, len(r) - 1, min(len(r), 120)).astype(int)
    rs, angs = r[sub], ang[sub]
    coarse = []
    for th in angles:
        px, py = rs * np.cos(angs + th), rs * np.sin(angs + th)
        for i in range(0, len(cx), 4000):
            c = cost(cx[i:i+4000, None] + px[None, :],
                     cy[i:i+4000, None] + py[None, :]).mean(axis=1)
            # Lay ca cac cuc tieu o cho khac; nhieu diem sat nhau quanh mot
            # o se che mat nghiem thu hai neu chi lay 3 gia tri nho nhat.
            for _ in range(min(3, len(c))):
                k = int(np.argmin(c))
                if not np.isfinite(c[k]):
                    break
                coarse.append((c[k], cx[i + k], cy[i + k], th))
                near = ((cx[i:i+len(c)] - cx[i + k]) ** 2 +
                        (cy[i:i+len(c)] - cy[i + k]) ** 2 < 0.5 ** 2)
                c[near] = np.inf
    coarse.sort(key=lambda b: b[0])

    seeds = []
    for candidate in coarse:
        _, x, y, th = candidate
        if all(math.hypot(x - sx, y - sy) > 0.5 or
               abs(math.remainder(th - sth, 2 * math.pi)) > math.radians(30)
               for _, sx, sy, sth in seeds):
            seeds.append(candidate)
            if len(seeds) == 12:
                break
    if not seeds:
        print('Khong tim duoc ung vien vi tri tu anh quet.')
        n.destroy_node()
        rclpy.shutdown()
        return 2

    offs = np.arange(-0.1, 0.1001, 0.01)
    DX, DY = np.meshgrid(offs, offs, indexing='ij')
    DX, DY = DX.ravel(), DY.ravel()
    fine = []
    for _, bx, by, bth in seeds:
        best = (1e9, bx, by, bth)
        for dth in np.radians(np.arange(-2.0, 2.001, 0.25)):
            px, py = r * np.cos(ang + bth + dth), r * np.sin(ang + bth + dth)
            c = cost(bx + DX[:, None] + px[None, :],
                     by + DY[:, None] + py[None, :]).mean(axis=1)
            k = int(np.argmin(c))
            if c[k] < best[0]:
                best = (c[k], bx + DX[k], by + DY[k], bth + dth)
        c0, x, y, th = best
        hit = float((cost(x + r * np.cos(ang + th), y + r * np.sin(ang + th))
                     < HIT).mean())
        fine.append((c0, hit, x, y, th))
    # Xep theo khoang cach trung binh toi tuong (min tung cm), KHONG theo % tia
    # trung tuong: % do tho, lech 1 do van gan nhu y het, da lam chon sai ~1 do.
    fine.sort(key=lambda b: b[0])
    fine = [f[1:] for f in fine]

    # Ung vien KHAC cho (cach > 0.5 m hoac lech > 30 do) tot nhat
    hit1, x1, y1, th1 = fine[0]
    hit2 = 0.0
    for hit, x, y, th in fine[1:]:
        if (math.hypot(x - x1, y - y1) > 0.5 or
                abs(math.remainder(th - th1, 2 * math.pi)) > math.radians(30)):
            hit2 = hit
            x2, y2, th2 = x, y, th
            break

    # LiDAR (x1, y1, th1) -> tam xe
    bth = th1 - lth
    bx = x1 - (lx * math.cos(bth) - ly * math.sin(bth))
    by = y1 - (lx * math.sin(bth) + ly * math.cos(bth))
    print(f'Tot nhat : xe o ({bx:+.2f}, {by:+.2f}) huong '
          f'{math.degrees(math.remainder(bth, 2*math.pi)):+.1f} do, '
          f'{hit1*100:.0f}% tia trung tuong')
    if hit2 > 0:
        print(f'Thu hai  : LiDAR o ({x2:+.2f}, {y2:+.2f}) huong '
              f'{math.degrees(math.remainder(th2, 2*math.pi)):+.1f} do, '
              f'{hit2*100:.0f}% tia trung tuong')

    hint_ok = (hint is None or
               (math.hypot(bx - hint[0], by - hint[1]) <= 0.20 and
                abs(math.remainder(bth - hint[2], 2 * math.pi)) <= math.radians(15)))
    success = False
    if hit1 < MIN_HIT:
        print(f'\nKHONG GUI: chi {hit1*100:.0f}% tia trung tuong (can >= '
              f'{MIN_HIT*100:.0f}%). Phong da doi nhieu so voi luc ve ban do, '
              'hoac xe dang o ngoai vung da ve. Dung "2D Pose Estimate".')
    elif not hint_ok:
        print('\nKHONG GUI: ket qua LiDAR cach xa o/huong da xac nhan.')
    elif hint is None and hit1 - hit2 < MIN_MARGIN:
        print(f'\nKHONG GUI: hai cho khac nhau deu khop gan bang nhau - phong '
              'gan doi xung, doan la co the sai. Dung "2D Pose Estimate", hoac '
              'dua xe toi cho it doi xung hon (gan do dac) roi chay lai.')
    elif a.dry_run:
        print('\n--dry-run: khong gui.')
    else:
        m = PoseWithCovarianceStamped()
        m.header.frame_id = g.header.frame_id or 'map'
        # Viec tim kiem co the mat vai giay; tem scan cu se vuot qua bo dem TF.
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.pose.position.x = bx
        m.pose.pose.position.y = by
        m.pose.pose.orientation.z = math.sin(bth / 2)
        m.pose.pose.orientation.w = math.cos(bth / 2)
        # Sai so cua phep tim: ~5 cm, ~2 do
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(2.0) ** 2
        # Cho AMCL thay publisher roi moi gui
        end = time.time() + 5
        while time.time() < end and n.pub.get_subscription_count() == 0:
            time.sleep(0.1)
        if n.pub.get_subscription_count() == 0:
            print('\nKhong thay AMCL dang nghe /initialpose.')
        else:
            sent_at = time.monotonic()
            n.pub.publish(m)
            end = time.monotonic() + 8
            while time.monotonic() < end:
                received = n.amcl_pose
                if received is not None and received[0] > sent_at:
                    pose = received[1].pose.pose
                    if (math.hypot(pose.position.x - bx, pose.position.y - by) < 0.15
                            and abs(math.remainder(yaw_of(pose.orientation) - bth,
                                                   2 * math.pi)) < math.radians(15)):
                        success = True
                        break
                time.sleep(0.1)
            print('\nAMCL da nhan va xac nhan vi tri.' if success else
                  '\nAMCL chua xac nhan vi tri trong 8 s; khong cho xe chay.')

    n.destroy_node()
    rclpy.shutdown()
    return 0 if success or (a.dry_run and hit1 >= MIN_HIT and hint_ok and
                            (hint is not None or hit1-hit2 >= MIN_MARGIN)) else 2


if __name__ == '__main__':
    sys.exit(main())
