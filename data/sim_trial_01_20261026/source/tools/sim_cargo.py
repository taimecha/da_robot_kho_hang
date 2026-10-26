#!/usr/bin/env python3
"""Hang hoa + camera AO cho mo phong do kich thuoc hang (chay kem tools/fleet_sim.sh).

    ROS_DOMAIN_ID=87 python3 tools/sim_cargo.py agv1 agv2 \\
        --goods "3:0.15:0.20:0.12, 4:0.10:0.12:0.10:5"

Moi hang "hang:rong:cao:sau[:xoay_do[:lech_ngang]]" (m): hop dat o o LAY hang
(hang, cargo.yaml place.pickup_col) ve phia place.pickup_heading (mac dinh Dong),
mat truoc cach tam o `place.gap`; xoay_do quanh truc dung, lech_ngang doc mat
truoc (+ = ben phai khi nhin tu xe).
Doi hang luc dang chay: ros2 topic pub --once /sim/cargo_set std_msgs/msg/String
"{data: '3:0.2:0.25:0.15'}"  (chuoi rong = bo het).

Phat:
  /sim/boxes                 JSON hop -> LiDAR ao (fleet_fake_robot.py) thay hop
  /agvN/camera/image_raw     anh camera AO 640x480 (bgr8, 2 Hz): nhin thang tu dau
                             xe (cargo.yaml camera.*), co meo ong kinh, nhieu, JPEG
  /fleet/markers             hop trong RViz (ns "cargo")
Vi tri xe lay tu /agvN/odom (fleet_sim: map->odom dong nhat) hoac --pose truth
(xe ao --esp32).
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, WS)
from grid_board import Board, config_path   # noqa: E402
from grid_navigator import parse_heading     # noqa: E402

CFG = os.path.join(WS, 'src/my_bot_bringup/config/cargo.yaml')
MARGIN = 160                       # px: anh ve truoc khi meo phai rong hon anh that
WALL = 1.2                         # m: tuong nen sau mat hang
BOX_BGR = np.array([70, 125, 185], float)       # bia carton
SHADE = {'front': 1.0, 'side': 0.72, 'top': 1.18}


def parse_goods(text):
    out = []
    for item in text.replace(';', ',').split(','):
        f = [float(v) for v in item.strip().split(':') if v.strip()] if item.strip() else []
        if not f:
            continue
        if len(f) < 4:
            raise ValueError(f'"{item}": can hang:rong:cao:sau')
        out.append({'row': int(f[0]), 'w': f[1], 'h': f[2], 'd': f[3],
                    'rot': f[4] if len(f) > 4 else 0.0, 'off': f[5] if len(f) > 5 else 0.0})
    return out


class SimCargo(Node):
    def __init__(self, cars, goods, pose_topic):
        super().__init__('sim_cargo')
        self.cfg = yaml.safe_load(open(CFG))
        cam = self.cfg['camera']
        self.K = np.array([[cam['fx'], 0, cam['cx']], [0, cam['fy'], cam['cy']], [0, 0, 1]])
        self.D = np.array(cam['dist'], float)
        self.W, self.H = int(cam['width']), int(cam['height'])
        self.cam = (cam['x'], cam['y'], cam['z'])
        self.cam_yaw = math.radians(float(cam.get('yaw', 0.0)))   # camera lap xoay (thu bu goc)
        self.cam_pitch = math.radians(float(cam.get('pitch', 0.0)))   # camera ngoc (+ = len)
        self.board = Board.load(config_path(True))
        self.rng = np.random.default_rng(1)
        self.maps = self.distort_maps()
        self.poses = {}
        self.boxes = []
        self.marker_count = 0
        self.img_pubs = {}
        for c in cars:
            self.img_pubs[c] = self.create_publisher(Image, f'/{c}/camera/image_raw', 2)
            self.create_subscription(Odometry, f'/{c}/{pose_topic}',
                                     lambda m, c=c: self.cb_pose(c, m), 10)
        self.box_pub = self.create_publisher(String, '/sim/boxes', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.create_subscription(String, '/sim/cargo_set', lambda m: self.set_goods(m.data), 10)
        self.create_subscription(String, '/fleet/cargo_event', self.cargo_event, 20)
        self.set_goods(goods)
        self.create_timer(0.5, self.render_all)
        self.create_timer(0.1, self.publish_boxes)

    def cargo_event(self, msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        car = event.get('car')
        if car not in self.poses:
            return
        if event.get('event') == 'loaded_confirmed':
            box = next((b for b in self.boxes if b['row'] == event.get('pickup_row')
                        and not b.get('owner') and not b.get('delivered')), None)
            if box is not None:
                box.update(owner=car, z=0.14)
        elif event.get('event') == 'unloaded_confirmed':
            box = next((b for b in self.boxes if b.get('owner') == car), None)
            if box is not None:
                x, y = self.board.xy(int(event['deliver_row'])-1, int(event['deliver_col'])-1)
                box.update(x=x+0.32*math.sin(self.board.yaw),
                           y=y-0.32*math.cos(self.board.yaw),
                           owner=None, z=0.0, delivered=True)
        self.publish_boxes()

    # ------------------------------------------------------------------ hang
    def set_goods(self, text):
        try:
            goods = parse_goods(text)
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        pl = self.cfg['place']
        H = parse_heading(pl['pickup_heading'], self.board.yaw)[1]   # huong xe nhin vao hang
        side = H - math.pi / 2          # truc rong cua hop (ben phai khi nhin tu xe)
        gap, col = pl['gap'], int(pl['pickup_col'])
        self.boxes = []
        for g in goods:
            nx, ny = self.board.xy(g['row'] - 1, col - 1)
            dist = gap + g['d'] / 2
            self.boxes.append({
                'x': nx + dist * math.cos(H) + g['off'] * math.cos(side),
                'y': ny + dist * math.sin(H) + g['off'] * math.sin(side),
                # truc x rieng = rong, truc y rieng = sau (mat -y quay ve xe)
                'yaw': side + math.radians(g['rot']),
                'w': g['w'], 'd': g['d'], 'h': g['h'], 'row': g['row']})
        self.get_logger().info('Hang: ' + (', '.join(
            f'({b["row"]},{col}) {b["w"] * 100:.1f}x{b["h"] * 100:.1f} cm' for b in self.boxes) or 'khong co'))
        self.publish_boxes()

    def publish_boxes(self):
        for box in self.boxes:
            if box.get('owner') in self.poses:
                x, y, yaw = self.poses[box['owner']]
                box.update(x=x, y=y, yaw=yaw-math.pi/2)
        self.box_pub.publish(String(data=json.dumps(self.boxes)))
        ma = MarkerArray()
        for i in range(len(self.boxes), self.marker_count):
            for marker_id in (i, 100+i):
                old = Marker()
                old.ns, old.id, old.action = 'cargo', marker_id, Marker.DELETE
                ma.markers.append(old)
        self.marker_count = len(self.boxes)
        for i, b in enumerate(self.boxes):
            m = Marker()
            m.header.frame_id, m.ns, m.id, m.type = 'map', 'cargo', i, Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = b['x'], b['y'], b.get('z',0.0)+b['h'] / 2
            m.pose.orientation.z, m.pose.orientation.w = math.sin(b['yaw'] / 2), math.cos(b['yaw'] / 2)
            m.scale.x, m.scale.y, m.scale.z = b['w'], b['d'], b['h']
            m.color = ColorRGBA(r=0.72, g=0.49, b=0.27, a=0.9)
            ma.markers.append(m)
            t = Marker()
            t.header.frame_id, t.ns, t.id, t.type = 'map', 'cargo', 100 + i, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = b['x'], b['y'], b.get('z',0.0)+b['h'] + 0.08
            t.scale.z = 0.07
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f'{b["w"] * 100:.1f}x{b["h"] * 100:.1f} cm'
            ma.markers.append(t)
        self.mk_pub.publish(ma)

    # ---------------------------------------------------------------- camera
    def distort_maps(self):
        """Anh that (co meo) lay mau tu anh ve phang (khong meo, rong hon MARGIN)."""
        u, v = np.meshgrid(np.arange(self.W, dtype=np.float32), np.arange(self.H, dtype=np.float32))
        pts = np.stack([u.ravel(), v.ravel()], 1).reshape(-1, 1, 2)
        und = cv2.undistortPoints(pts, self.K, self.D).reshape(-1, 2)
        mx = (und[:, 0] * self.K[0, 0] + self.K[0, 2] + MARGIN).reshape(self.H, self.W)
        my = (und[:, 1] * self.K[1, 1] + self.K[1, 2] + MARGIN).reshape(self.H, self.W)
        return mx.astype(np.float32), my.astype(np.float32)

    def cb_pose(self, car, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.poses[car] = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def to_cam(self, pose, P):
        """Diem the gioi (N,3) -> he camera OpenCV (X phai, Y xuong, Z truoc)."""
        x, y, th = pose
        dx, dy = P[:, 0] - x, P[:, 1] - y
        fwd = dx * math.cos(th) + dy * math.sin(th) - self.cam[0]
        left = -dx * math.sin(th) + dy * math.cos(th) - self.cam[1]
        c, s = math.cos(self.cam_yaw), math.sin(self.cam_yaw)
        fwd, left = fwd * c + left * s, -fwd * s + left * c
        up = P[:, 2] - self.cam[2]
        cp, sp = math.cos(self.cam_pitch), math.sin(self.cam_pitch)
        fwd, up = fwd * cp + up * sp, -fwd * sp + up * cp
        return np.stack([-left, -up, fwd], 1)

    def render(self, pose):
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2] + MARGIN, self.K[1, 2] + MARGIN
        Wc, Hc = self.W + 2 * MARGIN, self.H + 2 * MARGIN
        img = np.zeros((Hc, Wc, 3), np.uint8)
        horizon = int(round(cy + fy * self.cam[2] / WALL))     # chan tuong nen
        img[:horizon] = (205, 208, 210)                          # tuong / tam nen trang
        img[horizon:] = (112, 118, 124)                          # san xam
        cam_w = np.array([pose[0] + self.cam[0] * math.cos(pose[2]) - self.cam[1] * math.sin(pose[2]),
                          pose[1] + self.cam[0] * math.sin(pose[2]) + self.cam[1] * math.cos(pose[2]),
                          self.cam[2]])
        drawn = []
        for b in self.boxes:
            c, s = math.cos(b['yaw']), math.sin(b['yaw'])
            hw, hd = b['w'] / 2, b['d'] / 2
            bottom = b.get('z', 0.0)
            centre = np.array([b['x'], b['y'], bottom + b['h'] / 2])

            def corner(a, bb, z):
                return (b['x'] + a * c - bb * s, b['y'] + a * s + bb * c, z)
            # mat truoc = phia -y rieng cua hop (quay ve phia xe dung o hang 5)
            sides = {'front': ((-hw, -hd), (hw, -hd)), 'back': ((hw, hd), (-hw, hd)),
                     'left': ((-hw, hd), (-hw, -hd)), 'right': ((hw, -hd), (hw, hd))}
            for name, ((a1, b1), (a2, b2)) in sides.items():
                P = np.array([corner(a1, b1, bottom), corner(a2, b2, bottom),
                              corner(a2, b2, bottom+b['h']), corner(a1, b1, bottom+b['h'])])
                ctr = P.mean(0)
                out = ctr - centre                       # phap tuyen huong ra ngoai
                if np.dot(out, cam_w - ctr) > 0:         # camera o phia ngoai mat nay
                    drawn.append((ctr, 'front' if name == 'front' else 'side', P))
            if cam_w[2] > bottom+b['h']:              # camera cao hon hop: thay mat tren
                P = np.array([corner(-hw, -hd, bottom+b['h']), corner(hw, -hd, bottom+b['h']),
                              corner(hw, hd, bottom+b['h']), corner(-hw, hd, bottom+b['h'])])
                drawn.append((P.mean(0), 'top', P))
        faces = []
        for ctr, kind, P in drawn:
            Q = self.to_cam(pose, P)
            if (Q[:, 2] < 0.02).any():
                continue
            faces.append((np.linalg.norm(ctr - cam_w), kind, Q))
        drawn = faces
        for _, kind, Q in sorted(drawn, key=lambda t: -t[0]):
            uv = np.stack([fx * Q[:, 0] / Q[:, 2] + cx, fy * Q[:, 1] / Q[:, 2] + cy], 1)
            col = np.clip(BOX_BGR * SHADE[kind], 0, 255)
            cv2.fillConvexPoly(img, np.round(uv * 16).astype(np.int32), tuple(float(v) for v in col),
                               cv2.LINE_AA, 4)
            if getattr(self, 'carton_edges', False):
                # Stylized physical seams/shadows, identical for all classes.
                # No class colours, IDs or dimensions are written into the RGB.
                cv2.polylines(img, [np.round(uv*16).astype(np.int32)], True,
                              tuple(float(v*.67) for v in col), 1, cv2.LINE_AA, 4)
        img = cv2.remap(img, self.maps[0], self.maps[1], cv2.INTER_LINEAR)
        img = cv2.GaussianBlur(img, (3, 3), 0.7)
        img = np.clip(img + self.rng.normal(0, 3.0, img.shape), 0, 255).astype(np.uint8)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return cv2.imdecode(jpg, cv2.IMREAD_COLOR)

    def render_all(self):
        for car, pub in self.img_pubs.items():
            if car not in self.poses:
                continue
            img = self.render(self.poses[car])
            m = Image()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'camera_link'
            m.height, m.width, m.encoding, m.step = img.shape[0], img.shape[1], 'bgr8', img.shape[1] * 3
            m.data = img.tobytes()
            pub.publish(m)


def main():
    rclpy.init(args=sys.argv)
    ap = argparse.ArgumentParser()
    ap.add_argument('cars', nargs='*', default=['agv1', 'agv2'])
    ap.add_argument('--goods', default='')
    ap.add_argument('--pose', default='odom', help='odom (fleet_sim) / truth (xe ao --esp32)')
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])
    node = SimCargo(a.cars, a.goods, a.pose)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
