#!/usr/bin/env python3
"""Xem mo phong HAI XE trong Gazebo + DAT VAT CAN bang chuot (kich ban 3
HUONG_DAN_CHAY.md).

    tools/fleet_sim.sh up                          # truoc
    ROS_DOMAIN_ID=87 python3 tools/fleet_gazebo.py

Tao the gioi tu ban do that (tuong = o den cua real_room_map.pgm, cao 25 cm),
ve vach sa ban 5x5, dat hai xe (agv1 xanh duong, agv2 cam - cung mau RViz) va
6 tru do (vat can) xep hang CHO o duoi sa ban, roi bat Gazebo.

- Xe: vi tri lay tu /agvN/odom cua xe ao (map = odom), day vao Gazebo bang
  set_pose. Gazebo KHONG tinh vat ly, KHONG lai xe: xe ao + Nav2 van la nguon
  su that cho xe.
- Vat can: nguoc lai, GAZEBO la nguon su that. Bam T (cong cu dich chuyen),
  bam mot tru do roi keo mui ten do/xanh la vao sa ban; Esc de thoi. Tru nao
  nam trong sa ban (them le 25 cm) duoc gui len /sim/obstacles -> LiDAR hai xe
  ao thay no. Keo ra ngoai = bo vat can. Lenh ros2 topic pub /sim/obstacles
  "x,y;..." van dung duoc: tru do tu nhay toi cho do.
Ctrl-C (hoac dong cua so Gazebo) de tat.
"""
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, WS)

import rclpy                                            # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.executors import ExternalShutdownException   # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from std_msgs.msg import String                         # noqa: E402
from ros_gz_interfaces.msg import Entity                # noqa: E402
from ros_gz_interfaces.srv import SetEntityPose         # noqa: E402
import yaml                                             # noqa: E402

from grid_board import Board, config_path               # noqa: E402

CFG = os.path.join(WS, 'src/my_bot_bringup/config')
PARTITION = os.environ.get('GZ_PARTITION') or 'fleet_sim'   # khong lan voi Gazebo khac
OUT = f'/tmp/fleet_sim/fleet_world_{PARTITION}.sdf'
WORLD = 'fleet'
ROBOTS = {'agv1': '0.15 0.45 1.0', 'agv2': '1.0 0.5 0.0'}   # mau RViz
N_OBST = 6                       # so tru vat can
OBST_R = 0.10                    # giong fleet_fake_robot.py
WALL_H = 0.25
RATE = 25.0                      # Hz cap nhat vi tri moi xe
MARGIN = 0.25                    # m, vung tinh la "tren sa ban" ngoai cac nut bien
SHELF_GAP = 0.45                 # m, hang cho duoi mep sa ban


# ---------------------------------------------------------------- the gioi
def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tok, i = [], 0
    while len(tok) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tok.append(data[i:j])
        i = j
    assert tok[0] == b'P5', f'{path}: chi doc PGM nhi phan (P5)'
    w, h = int(tok[1]), int(tok[2])
    return w, h, data[i + 1:i + 1 + w * h]


def wall_boxes(map_yaml):
    """O den ban do -> danh sach hop (cx, cy, sx, sy): gop theo hang roi gop
    cac hang lien tiep co cung doan."""
    m = yaml.safe_load(open(map_yaml))
    w, h, px = read_pgm(os.path.join(os.path.dirname(map_yaml), m['image']))
    res, (ox, oy) = float(m['resolution']), m['origin'][:2]
    thr = (1.0 - float(m['occupied_thresh'])) * 255
    if m.get('negate', 0):
        raise SystemExit('ban do negate=1 chua ho tro')
    runs = []                                  # (cot dau, cot cuoi) moi hang
    for r in range(h):
        row, cur, c = px[r * w:(r + 1) * w], [], 0
        while c < w:
            if row[c] <= thr:
                c0 = c
                while c < w and row[c] <= thr:
                    c += 1
                cur.append((c0, c))
            else:
                c += 1
        runs.append(cur)
    boxes, open_ = [], {}                      # (c0, c1) -> hang bat dau
    for r in range(h + 1):
        cur = set(runs[r]) if r < h else set()
        for seg in list(open_):
            if seg not in cur:
                r0 = open_.pop(seg)
                c0, c1 = seg
                # hang 0 cua anh la mep TREN ban do
                x0, x1 = ox + c0 * res, ox + c1 * res
                y1, y0 = oy + (h - r0) * res, oy + (h - r) * res
                boxes.append(((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0))
        for seg in cur:
            open_.setdefault(seg, r)
    return boxes


def box_visual(name, x, y, z, sx, sy, sz, rgb, yaw=0.0):
    return (f'<visual name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>'
            f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
            f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>')


def cyl_visual(name, x, y, z, r, l, rgb, roll=0.0):
    return (f'<visual name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} {roll:.6f} 0 0</pose>'
            f'<geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>'
            f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>')


def robot_model(name, rgb):
    """Hinh xe (kich thuoc robot.urdf.xacro). Khong va cham, khong trong luc:
    chi di chuyen khi set_pose."""
    v = [cyl_visual('than', 0, 0, 0.045, 0.0907, 0.040, rgb),
         cyl_visual('lidar', 0, 0, 0.080, 0.030, 0.030, '0.1 0.1 0.1'),
         box_visual('mui', 0.060, 0, 0.067, 0.050, 0.020, 0.004, '1 1 1'),
         # Cylinder axis is z by default; rotate it onto the wheel axle (y).
         cyl_visual('banh_trai', -0.014, 0.100, 0.03225, 0.03225, 0.026,
                    '0.1 0.1 0.1', roll=math.pi/2),
         cyl_visual('banh_phai', -0.014, -0.100, 0.03225, 0.03225, 0.026,
                    '0.1 0.1 0.1', roll=math.pi/2)]
    return (f'<model name="{name}"><pose>0 0 -5 0 0 0</pose><link name="link">'
            f'<gravity>false</gravity><inertial><mass>1</mass></inertial>'
            + ''.join(v) + '</link></model>')


GUI = '''<gui fullscreen="0">
  <plugin filename="MinimalScene" name="3D View">
    <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
      <property type="string" key="state">docked</property></gz-gui>
    <engine>ogre2</engine><scene>scene</scene>
    <ambient_light>0.5 0.5 0.5</ambient_light><background_color>0.75 0.8 0.85</background_color>
    <camera_pose>{cx:.3f} {cy:.3f} 2.4 0 0.95 1.5708</camera_pose>
  </plugin>
  <plugin filename="GzSceneManager" name="Scene Manager"><gz-gui>
    <property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
    <property key="height" type="double">5</property><property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="InteractiveViewControl" name="Interactive view control"><gz-gui>
    <property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
    <property key="height" type="double">5</property><property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="CameraTracking" name="Camera Tracking"><gz-gui>
    <property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
    <property key="height" type="double">5</property><property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="SelectEntities" name="Select Entities"><gz-gui>
    <property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
    <property key="height" type="double">5</property><property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="TransformControl" name="Transform control"><gz-gui>
    <property key="resizable" type="bool">false</property><property key="x" type="double">0</property>
    <property key="y" type="double">0</property><property key="width" type="double">250</property>
    <property key="height" type="double">50</property><property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
    <property key="cardBackground" type="string">#666666</property></gz-gui></plugin>
  <plugin filename="Screenshot" name="Screenshot"><gz-gui>
    <property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
    <property key="height" type="double">5</property><property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="EntityTree" name="Entity tree"><gz-gui>
    <property type="string" key="state">docked_collapsed</property></gz-gui></plugin>
</gui>'''


def shelf(board):
    """Cho de cac tru vat can chua dung: mot hang duoi mep sa ban."""
    x0, x1, y0, _ = board.bounds()
    step = (x1 - x0) / max(N_OBST - 1, 1)
    return [(x0 + i * step, y0 - SHELF_GAP) for i in range(N_OBST)]


def build_world(board):
    walls = wall_boxes(os.path.join(CFG, 'real_room_map.yaml'))
    wall_v = ''.join(box_visual(f'w{i}', x, y, WALL_H / 2, sx, sy, WALL_H, '0.55 0.55 0.6')
                     for i, (x, y, sx, sy) in enumerate(walls))
    # Vach sa ban: bang keo den rong 2 cm + diem nut
    tape = []
    for (r1, c1), (r2, c2) in board.edges():
        (x1, y1), (x2, y2) = board.xy(r1, c1), board.xy(r2, c2)
        tape.append(box_visual(f't{len(tape)}', (x1 + x2) / 2, (y1 + y2) / 2, 0.001,
                               math.hypot(x2 - x1, y2 - y1), 0.02, 0.002, '0.05 0.05 0.05',
                               math.atan2(y2 - y1, x2 - x1)))
    for r, c in board.nodes():
        x, y = board.xy(r, c)
        tape.append(cyl_visual(f'n{r}_{c}', x, y, 0.002, 0.025, 0.003, '0.85 0.1 0.1'))
    cx, cy = board.xy(board.center_row - 1, board.center_col - 1)
    obst = ''.join(
        f'<model name="vat_can_{i}"><pose>{x:.3f} {y:.3f} 0 0 0 0</pose><link name="link">'
        f'<gravity>false</gravity><inertial><mass>1</mass></inertial>'
        + cyl_visual('v', 0, 0, 0.10, OBST_R, 0.20, '0.85 0.15 0.15') + '</link></model>'
        for i, (x, y) in enumerate(shelf(board)))
    sdf = f'''<?xml version="1.0"?>
<sdf version="1.8"><world name="{WORLD}">
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  {GUI.format(cx=cx, cy=cy - 1.7)}
  <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.75 0.8 0.85 1</background><shadows>false</shadows></scene>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><cast_shadows>false</cast_shadows>
    <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -0.9</direction></light>
  <model name="san"><static>true</static><link name="link">
    {box_visual('san', cx, cy, -0.005, 12, 12, 0.01, '0.92 0.9 0.85')}
    {wall_v}{''.join(tape)}
  </link></model>
  {''.join(robot_model(n, rgb) for n, rgb in ROBOTS.items())}
  {obst}
</world></sdf>
'''
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write(sdf)
    return len(walls)


# ---------------------------------------------------------------- dong bo vi tri
class Mirror(Node):
    def __init__(self, board):
        super().__init__('fleet_gazebo')
        self.board = board
        x0, x1, y0, y1 = board.bounds()
        self.area = (x0 - MARGIN, x1 + MARGIN, y0 - MARGIN, y1 + MARGIN)
        self.shelf = shelf(board)
        self.cli = self.create_client(SetEntityPose, f'/world/{WORLD}/set_pose')
        self.pose = {}                 # xe: ten -> (x, y, z, yaw), gui vao Gazebo lien tuc
        self.sent = {}
        self.oneshot = []              # (ten, (x, y, z, yaw)) gui MOT lan (tru vat can)
        self.busy = False
        self.queue = []
        self.last_full = 0.0
        self.nudge = 0.0
        self.obst = {}                 # tru vat can -> (x, y, z) Gazebo dang co
        self.lock = threading.Lock()
        self.last_pub, self.last_pub_t = None, 0.0
        for n in ROBOTS:
            self.create_subscription(Odometry, f'/{n}/odom',
                                     lambda m, n=n: self.cb_odom(n, m), 10)
        self.obst_pub = self.create_publisher(String, '/sim/obstacles', 10)
        self.create_subscription(String, '/sim/obstacles', self.cb_obst, 10)
        self.create_timer(1.0 / (RATE * len(ROBOTS)), self.tick)
        self.create_timer(0.2, self.publish_obstacles)

    def cb_odom(self, n, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation   # map = odom (xe ao)
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose[n] = (p.x, p.y, 0.0, yaw)

    # ---- vat can: Gazebo -> /sim/obstacles
    def read_gz(self, proc):
        """Luong rieng: doc vi tri cac tru tu 'gz topic --json-output'."""
        for line in proc.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            got = {}
            for p in msg.get('pose', []):
                name = p.get('name', '')
                if name.startswith('vat_can_'):
                    pos = p.get('position', {})       # JSON bo truong = 0
                    got[name] = (pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0))
            if got:
                with self.lock:
                    self.obst.update(got)

    def active(self):
        x0, x1, y0, y1 = self.area
        with self.lock:
            items = sorted(self.obst.items())
        return [(x, y) for _, (x, y, z) in items
                if x0 <= x <= x1 and y0 <= y <= y1 and -0.5 < z < 1.0]

    def publish_obstacles(self):
        pts = self.active()
        data = ';'.join(f'{x:.3f},{y:.3f}' for x, y in pts)
        now = time.monotonic()
        changed = data != self.last_pub
        if not changed and now - self.last_pub_t < 2.0:
            return                     # gui lai moi 2 s: xe ao vua bat lai van nhan
        if changed and self.last_pub is not None and self.moving(data):
            return                     # dang keo: cho tha tay roi moi bao
        self.obst_pub.publish(String(data=data))
        self.last_pub_t = now
        if changed:
            self.last_pub = data
            if pts:
                where = ', '.join(self.describe(x, y) for x, y in pts)
                print(f'Vat can tren sa ban ({len(pts)}): {where}', flush=True)
            else:
                print('Sa ban khong con vat can', flush=True)

    def moving(self, data):
        """True neu vi tri vat can con doi trong 0.4 s vua qua (dang keo)."""
        now = time.monotonic()
        if data != getattr(self, '_cand', None):
            self._cand, self._cand_t = data, now
        return now - self._cand_t < 0.4

    def describe(self, x, y):
        r, c = self.board.closest(x, y)
        nx, ny = self.board.xy(r, c)
        d = math.hypot(x - nx, y - ny)
        return f'({x:+.2f}, {y:+.2f}) gan nut hang {r + 1} cot {c + 1} ({d * 100:.0f} cm)'

    def cb_obst(self, m):
        """Lenh ngoai (ros2 topic pub) -> dat tru toi cho do, tru thua ve hang cho."""
        if m.data == self.last_pub:
            return                     # chinh minh vua gui
        pts = []
        for item in m.data.split(';'):
            if item.strip():
                try:
                    x, y = (float(v) for v in item.split(','))
                except ValueError:
                    self.get_logger().warn(f'bo qua "{item}" (can dang x,y)')
                    continue
                pts.append((x, y))
        if len(pts) > N_OBST:
            self.get_logger().warn(f'chi co {N_OBST} tru, bo {len(pts) - N_OBST} vat can cuoi')
        for i in range(N_OBST):
            x, y = pts[i] if i < len(pts) else self.shelf[i]
            self.oneshot.append((f'vat_can_{i}', (x, y, 0.0, 0.0)))

    # ---- xe (va lenh dat tru): ROS -> Gazebo
    def tick(self):
        if self.busy or not self.cli.service_is_ready():
            return
        now = time.monotonic()
        # Moi giay gui lai HET, z nhich 0.1 mm: dat lai dung vi tri cu thi
        # Gazebo coi nhu khong doi va khong bao cho cua so -> xe dung yen tu
        # luc cua so chua mo se khong bao gio hien.
        if now - self.last_full > 1.0:
            self.sent.clear()
            self.nudge = 1e-4 - self.nudge
            self.last_full = now
        if self.oneshot:
            name, val = self.oneshot.pop(0)
            nudge, keep = 0.0, False
        else:
            if not self.queue:
                self.queue = [k for k, v in self.pose.items() if self.sent.get(k) != v]
            if not self.queue:
                return
            name = self.queue.pop(0)
            val, nudge, keep = self.pose[name], self.nudge, True
        x, y, z, yaw = val
        req = SetEntityPose.Request()
        req.entity.name, req.entity.type = name, Entity.MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z + nudge
        req.pose.orientation.z, req.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.busy = True
        fut = self.cli.call_async(req)
        fut.add_done_callback(lambda f, k=name, v=val, s=keep: self.done(f, k, v, s))

    def done(self, fut, name, val, keep):
        self.busy = False
        try:
            if fut.result().success and keep:
                self.sent[name] = val
        except Exception:
            pass


def main():
    board = Board.load(config_path(True))
    n_walls = build_world(board)
    print(f'The gioi: {OUT} ({n_walls} hop tuong)', flush=True)
    env = dict(os.environ, GZ_PARTITION=PARTITION)
    procs = [
        subprocess.Popen(['gz', 'sim', '-r', OUT], env=env, start_new_session=True),
        subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                          f'/world/{WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                          '--ros-args', '--log-level', 'warn'],
                         env=env, start_new_session=True),
        subprocess.Popen(['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info',
                          '--json-output'],
                         env=env, start_new_session=True, stdout=subprocess.PIPE, text=True),
    ]
    rclpy.init()
    node = Mirror(board)
    threading.Thread(target=node.read_gz, args=(procs[2],), daemon=True).start()
    print('Dang dong bo agv1, agv2 vao Gazebo. Dat vat can: bam T, bam tru do, keo vao '
          'sa ban (Esc de thoi). Ctrl-C de tat.', flush=True)
    try:
        while rclpy.ok() and procs[0].poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
