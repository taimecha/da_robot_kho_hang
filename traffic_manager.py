#!/usr/bin/env python3
"""Dieu phoi NHIEU XE tren sa ban: giu cho truoc khi di, go ket chu dong.

Moi xe chay fleet_agent.py (namespace rieng). Node nay lap duong cho tung xe,
cho phep tung xe di toi o nao (giu cho), phat hien va go ket.

Quy tac:
 1. Hai xe khong cung dich; khong nhan dich la o xe khac dang do.
 2. Lap duong: it canh nhat, it re (moi lan re = 0.3 canh), cong them chi phi
    khi di tren duong con lai cua xe kia (2/o) va di nguoc chieu xe kia tren
    cung canh (6/canh). Xe dang do (khong co lo trinh) la vat can cung.
 3. Giu cho: xe CHIEM moi o cach no duoi nua o + ban kinh xe (dang chay giua
    hai o thi chiem ca hai, qua khoi nua o moi nha o sau). Xe duoc giu truoc
    toi LOOKAHEAD o tren lo trinh va chi duoc di toi o da giu. O da giu khong
    bi lay lai.
 4. Hai xe cung xin mot o: xe GAN DICH HON (it o con lai) duoc truoc; bang
    nhau thi xe ten nho hon.
 5. Cho > 3 s vi bi xe kia chan:
      - xe kia dang do: lap duong vong quanh no; khong duoc thi xe dang do ne
        sang o trong roi quay ve cho cu;
      - xe kia cung dang cho minh (ket): xe XA DICH HON nhuong - lap duong vong
        tranh xe kia; khong duoc thi ne sang o trong, doi xe kia qua roi di
        tiep; khong duoc nua thi doi vai; van khong duoc thi dung ca hai, bao loi;
      - xe kia dang chay / dang xoay: cho tiep; dung yen han > 3 s thi go nhu tren.
 6. Xe mat trang thai > 1 s: giu nguyen o no chiem (xe khac khong vao), bao
    loi. Xe bao vat can / Nav2 that bai: lap lai duong tu cho no dung.

    python3 traffic_manager.py agv1 agv2 [--grid FILE] [--no-avoid]
Chay trong terminal: hoi lan luot xe 1 roi xe 2 (hang, cot, huong tai dich -
giong grid_navigator.py --real); nhap DU ca hai moi giao lenh, ca hai cung
xuat phat; cho ca hai toi noi roi hoi luot tiep. 'q' thoat.
Khong phai terminal (script/test): moi dong mot lenh "1 5 5 d" (xe 1 toi
hang 5 cot 5 roi quay Dong), "s" dung tat ca, "q" thoat. Topic /fleet/goal
nhan cung dang do, va "pair 1 3 5 d ; 2 3 1 t" (giao cung luc, mot dich sai
thi khong xe nao chay).
--no-avoid: bo chi phi tranh duong xe kia (de thu go ket).
"""
import heapq
import json
import math
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from grid_board import Board, config_path, edge_key
from grid_navigator import HEADING_NAMES, heading_name, parse_heading

TICK = 0.2                  # s
LOOKAHEAD = 2               # o giu truoc
WAIT_S = 3.0                # cho lau hon -> go ket
STALE_S = 1.0               # khong co trang thai lau hon -> mat lien lac
REACH_R = 0.15              # m - coi nhu da toi o
TURN_COST = 0.3              # chi phi moi 90 do quay
TURN_START_COST = 0.3        # moi lan dung de xoay co thoi gian co dinh
SOFT_NODE = 2.0
SOFT_GOAL = 3.0
SOFT_OPP = 6.0
RETRY_S = 2.0
MAX_FAIL = 5
COLORS = [(0.15, 0.45, 0.95), (0.95, 0.5, 0.1), (0.2, 0.75, 0.3), (0.8, 0.2, 0.7)]
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]       # Dong, Bac, Tay, Nam (theo truc sa ban)


class Robot:
    def __init__(self, name, idx):
        self.name, self.idx = name, idx
        self.estop = None               # ly do ESTOP tren ESP32 (None = khong)
        self.x = self.y = self.yaw = None
        self.t = 0.0
        self.state = 'unknown'
        self.ack = None                 # route_id agent dang chay
        self.dyn_nodes, self.dyn_edges = set(), set()
        self.goal = None                # dich nguoi dung dat (r, c)
        self.goal_heading = None
        self.route = None               # [(r, c)] hoac None
        self.route_id = 0
        self.upto = 0
        self.progress = 0
        self.final_heading = None
        self.mode = 'idle'              # idle | go | yield | yield_wait
        self.resume = None              # (dich, huong) sau khi ne
        self.wait_since = None
        self.yield_since = None
        self.stale = False
        self.fail_count = 0
        self.retry_at = 0.0
        self.pending = False            # co dich nhung chua co vi tri xe
        self.mark = (None, None, None)  # vi tri lan cuoi thay xe nhuc nhich
        self.last_move_t = 0.0


class TrafficManager(Node):
    def __init__(self, names, grid_file=None, avoid=True):
        super().__init__('traffic_manager')
        self.board = Board.load(grid_file or config_path(True))
        self.robots = {n: Robot(n, i) for i, n in enumerate(names)}
        self.order = list(names)
        self.avoid = avoid
        self.occ_r = self.board.spacing / 2 + self.board.robot_radius
        self.static_nodes, self.static_edges = set(), set()
        self.map_ok = False
        # So hieu lo trinh bat dau theo gio: chay lai traffic_manager ma dem tu 1
        # thi trung so voi lo trinh cu agent con giu (vd #1 da 'done') -> agent
        # tuong la lo trinh cu, bao xong ngay, xe khong chay.
        self.next_route_id = int(time.time() * 10) % 1_000_000_000
        self.cmd_pubs = {n: self.create_publisher(String, f'/fleet/{n}/cmd', 10) for n in names}
        self.goal_pub = self.create_publisher(String, '/fleet/goal', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.create_subscription(String, '/fleet/status', self.cb_status, 50)
        self.create_subscription(String, '/fleet/goal', self.cb_goal, 10)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qos)
        self.create_timer(TICK, self.tick)
        self.occ, self.res = {}, {}
        self.get_logger().info(
            f"Dieu phoi {', '.join(names)} tren sa ban {self.board.rows}x{self.board.cols}"
            f"{'' if avoid else ' (TAT tranh duong xe kia)'}. Cho /map va trang thai xe...")

    # ------------------------------------------------------------------ tien ich
    def log(self, text, level='info'):
        # rclpy cam mot cho goi log doi muc giua cac lan -> moi muc mot cho goi
        lg = self.get_logger()
        if level == 'error':
            lg.error(text)
        elif level == 'warn':
            lg.warn(text)
        else:
            lg.info(text)

    @staticmethod
    def lbl(n):
        return f'({n[0] + 1},{n[1] + 1})'

    def route_str(self, nodes):
        return ' -> '.join(self.lbl(n) for n in nodes)

    def xy(self, n):
        return self.board.xy(*n)

    def dist(self, rb, n):
        if rb.x is None:
            return float('inf')
        x, y = self.xy(n)
        return math.hypot(rb.x - x, rb.y - y)

    def node_of(self, rb):
        """O gan xe nhat ma khong bi chan tinh."""
        cands = [n for n in self.board.nodes() if n not in self.static_nodes]
        return min(cands, key=lambda n: self.dist(rb, n))

    def occupied(self, rb):
        if rb.x is None:
            return set()
        return {n for n in self.board.nodes() if self.dist(rb, n) < self.occ_r}

    def remaining(self, rb):
        return rb.route[rb.progress:] if rb.route else []

    def prio(self, rb):
        """Nho hon = uu tien hon: it o con lai (gan dich hon), roi ten."""
        left = len(rb.route) - 1 - rb.progress if rb.route else 0
        return (left, rb.idx)

    def parked(self, rb):
        return rb.route is None

    def others(self, rb):
        return [o for o in self.robots.values() if o is not rb]

    def blocked_sets(self):
        nodes = set(self.static_nodes)
        edges = set(self.static_edges)
        for rb in self.robots.values():
            nodes |= rb.dyn_nodes
            edges |= rb.dyn_edges
        return nodes, edges

    def heading_dir(self, yaw):
        if yaw is None:
            return None
        d = math.atan2(math.sin(yaw - self.board.yaw), math.cos(yaw - self.board.yaw))
        return DIRS[int(round(d / (math.pi / 2))) % 4]

    # ------------------------------------------------------------------ lap duong
    def plan(self, me, start, goal, hard=frozenset()):
        """Dijkstra tren (o, huong vua di). None neu khong co duong."""
        bn, be = self.blocked_sets()
        bn = (bn | set(hard)) - {start}
        if goal in bn:
            return None
        soft_nodes, opp_edges, soft_goals = set(), set(), set()
        if self.avoid:
            for o in self.others(me):
                rem = self.remaining(o)
                soft_nodes |= set(rem)
                for a, b in zip(rem, rem[1:]):
                    opp_edges.add((b, a))           # minh di b->a = nguoc chieu xe kia
                if o.goal is not None:
                    soft_goals.add(o.goal)
        d0 = self.heading_dir(me.yaw) if me.x is not None and self.dist(me, start) < 0.2 else None
        best = {(start, d0): 0.0}
        prev = {}
        heap = [(0.0, 0, start, d0)]
        seq = 0
        while heap:
            cost, _, n, d = heapq.heappop(heap)
            if best.get((n, d), float('inf')) < cost - 1e-9:
                continue
            if n == goal:
                path = [n]
                key = (n, d)
                while key in prev:
                    key = prev[key]
                    path.append(key[0])
                return path[::-1]
            for nb in self.board.neighbours(*n):
                if nb in bn or edge_key(n, nb) in be:
                    continue
                nd = (nb[0] - n[0], nb[1] - n[1])
                c = cost + 1.0
                if d is not None and nd != d:
                    c += TURN_START_COST + TURN_COST * (2 if nd == (-d[0], -d[1]) else 1)
                if nb in soft_nodes:
                    c += SOFT_NODE
                if nb in soft_goals and nb != goal:
                    c += SOFT_GOAL
                if (n, nb) in opp_edges:
                    c += SOFT_OPP
                key = (nb, nd)
                if c < best.get(key, float('inf')) - 1e-9:
                    best[key] = c
                    prev[key] = (n, d)
                    seq += 1
                    heapq.heappush(heap, (c, seq, nb, nd))
        return None

    def assign_route(self, rb, nodes, final_heading, mode):
        rb.route = list(nodes)
        rb.route_id = self.next_route_id
        self.next_route_id += 1
        rb.upto = 0
        rb.progress = 0
        rb.final_heading = final_heading
        rb.mode = mode
        rb.wait_since = None
        what = {'go': 'lo trinh', 'yield': 'NE DUONG'}.get(mode, mode)
        self.log(f'[{rb.name}] {what} #{rb.route_id}: {self.route_str(nodes)}')

    def clear_route(self, rb, reason=None):
        rb.route = None
        rb.route_id = self.next_route_id      # id moi -> agent dung xe
        self.next_route_id += 1
        rb.mode = 'idle'
        rb.resume = None
        rb.wait_since = None
        if reason:
            self.log(f'[{rb.name}] dung: {reason}', 'warn')

    # ------------------------------------------------------------------ callbacks
    def cb_map(self, msg):
        if self.map_ok:
            return
        i = msg.info
        self.static_nodes, self.static_edges, _ = self.board.check_map(
            msg.data, i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)
        self.map_ok = True
        self.log(f'Ban do: bo {len(self.static_nodes)} o, {len(self.static_edges)} canh sat vat can')

    def cb_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        rb = self.robots.get(d.get('name'))
        if rb is None:
            return
        rb.t = time.time()
        if d.get('x') is not None:
            rb.x, rb.y, rb.yaw = d['x'], d['y'], d['yaw']
            mx, my, myaw = rb.mark
            if (mx is None or math.hypot(rb.x - mx, rb.y - my) > 0.02 or
                    abs(math.atan2(math.sin(rb.yaw - myaw), math.cos(rb.yaw - myaw))) > 0.05):
                rb.mark = (rb.x, rb.y, rb.yaw)
                rb.last_move_t = rb.t
        rb.state = d.get('state', 'unknown')
        if rb.state == 'estop' and rb.estop is None:
            rb.estop = d.get('estop') or '?'
            self.log(f'!!! [{rb.name}] ESTOP tren ESP32: {rb.estop} - xe dung han, bo lo trinh; '
                     f'xe kia coi no nhu xe dang do. Kiem tra xe roi go: ros2 topic pub --once '
                     f'/{rb.name}/estop_reset std_msgs/msg/Empty (hoac tat/bat nguon xe)', 'error')
            if rb.route is not None or rb.goal is not None:
                rb.goal, rb.pending = None, False
                self.clear_route(rb, 'ESTOP')
        elif rb.state != 'estop' and rb.estop is not None:
            self.log(f'[{rb.name}] da go ESTOP ({rb.estop}) - nhap dich moi cho xe nay')
            rb.estop = None
        rb.ack = d.get('route_id')
        rb.dyn_nodes = {tuple(n) for n in d.get('blocked_nodes', [])}
        rb.dyn_edges = {edge_key(tuple(a), tuple(b)) for a, b in d.get('blocked_edges', [])}
        if rb.stale:
            rb.stale = False
            self.log(f'[{rb.name}] co lai lien lac')

    def parse_goal(self, text):
        """'1 5 5 d' / 'agv1 5 5' -> (xe, (r, c) tu 0, huong rad hoac None).
        Sai thi ValueError kem ly do."""
        tok = text.split()
        if len(tok) < 3:
            raise ValueError(f'"{text}" thieu so. Dang: "1 5 5 d"')
        name = tok[0]
        if name.isdigit() and 1 <= int(name) <= len(self.order):
            name = self.order[int(name) - 1]
        rb = self.robots.get(name)
        if rb is None:
            raise ValueError(f'khong co xe "{tok[0]}"')
        r, c = int(tok[1]) - 1, int(tok[2]) - 1
        heading = parse_heading(tok[3], self.board.yaw)[1] if len(tok) > 3 else None
        return rb, (r, c), heading

    def goal_error(self, rb, goal, taken=()):
        """Ly do khong nhan dich nay (None = duoc). taken: dich da nhan cho xe
        khac trong cung luot nhap."""
        r, c = goal
        if not (0 <= r < self.board.rows and 0 <= c < self.board.cols):
            return f'ô {self.lbl(goal)} nằm ngoài sa bàn {self.board.rows}x{self.board.cols}'
        if goal in self.static_nodes:
            return f'ô {self.lbl(goal)} sát vật cản, không tới được'
        if goal in taken:
            return f'ô {self.lbl(goal)} đã là đích của xe kia'
        for o in self.others(rb):
            # O xe kia DANG DO thi duoc: khi can, xe do ne sang o trong va o
            # lai do (vd hai xe doi cho). Chi cam trung dich / cho xe kia se ve.
            busy = {o.goal} if o.goal else set()
            if o.resume:
                busy.add(o.resume[0])
            if goal in busy:
                return f'ô {self.lbl(goal)} là đích của {o.name}'
        return None

    def cb_goal(self, msg):
        line = msg.data.strip()
        if not line:
            return
        if line.split()[0] in ('s', 'S', 'stop'):
            for rb in self.robots.values():
                rb.goal = None
                self.clear_route(rb, 'lenh dung tat ca')
            return
        # "pair 1 3 5 d ; 2 3 1 t": giao CUNG LUC cho moi xe - mot dich sai thi
        # khong xe nao chay. Khong co "pair" = mot xe.
        pair = line.lower().startswith('pair')
        parts = line[4:].split(';') if pair else [line]
        items, taken = [], set()
        for part in parts:
            try:
                rb, goal, heading = self.parse_goal(part.strip())
            except ValueError as e:
                self.log(f'Lenh sai: {e}', 'error')
                return
            err = self.goal_error(rb, goal, taken)
            if err:
                self.log(f'[{rb.name}] {err}' + (' - khong xe nao chay' if pair else ''), 'error')
                return
            taken.add(goal)
            items.append((rb, goal, heading))
        for rb, goal, heading in items:
            self.set_goal(rb, goal, heading)

    def set_goal(self, rb, goal, heading):
        rb.goal, rb.goal_heading = goal, heading
        rb.resume = None
        rb.fail_count = 0
        if rb.x is None or not self.map_ok:
            rb.pending = True
            self.log(f'[{rb.name}] chua co {"vi tri xe" if rb.x is None else "/map"} - '
                     'se lap duong khi co')
            return
        self.route_to_goal(rb)

    def route_to_goal(self, rb):
        hard = set()
        for o in self.others(rb):
            if self.parked(o):
                hard |= self.occupied(o)
        start = self.node_of(rb)
        path = self.plan(rb, start, rb.goal, hard=hard)
        if path is None:
            path = self.plan(rb, start, rb.goal)     # vong khong duoc: di, toi do go ket
        if path is None:
            # Vat can dong (vd costmap vua bat, chua loc xe kia) co the chan
            # thoang qua: thu lai nhu on_trouble roi moi bo.
            rb.fail_count += 1
            if rb.fail_count <= MAX_FAIL:
                rb.pending, rb.retry_at = True, time.time() + RETRY_S
                self.log(f'[{rb.name}] chua co duong toi {self.lbl(rb.goal)}, thu lai sau '
                         f'{RETRY_S:.0f} s ({rb.fail_count}/{MAX_FAIL})', 'warn')
                return False
            self.clear_route(rb, f'khong co duong toi {self.lbl(rb.goal)}')
            rb.goal = None
            return False
        self.assign_route(rb, path, rb.goal_heading, 'go')
        # Dich la cho xe khac dang do: bao xe do ne ngay, khoi doi toi sat roi
        # cho 3 s moi go ket
        for o in self.others(rb):
            if self.parked(o) and o.x is not None and self.node_of(o) == rb.goal:
                self.log(f'[{o.name}] dang do o dich cua {rb.name} -> ne truoc')
                self.occ[o.name] = self.occupied(o)
                self.occ[rb.name] = self.occupied(rb)
                if not self.side_step(o, rb):
                    self.log(f'[{o.name}] khong co o trong de ne', 'warn')
        return True

    # ------------------------------------------------------------------ vong chinh
    def tick(self):
        now = time.time()
        for rb in self.robots.values():
            if rb.t and now - rb.t > STALE_S and not rb.stale:
                rb.stale = True
                self.log(f'[{rb.name}] MAT LIEN LAC {now - rb.t:.1f} s - giu nguyen o xe chiem, '
                         'xe khac khong vao', 'error')
        self.occ = {n: self.occupied(rb) for n, rb in self.robots.items()}
        for rb in self.robots.values():
            if (rb.pending and rb.x is not None and self.map_ok and rb.goal is not None
                    and now >= rb.retry_at):
                rb.pending = False
                self.route_to_goal(rb)

        # 1. Tien do, xong lo trinh, su co
        for rb in self.robots.values():
            if not rb.route or rb.ack != rb.route_id:
                continue
            for i in range(rb.progress, rb.upto + 1):
                if self.dist(rb, rb.route[i]) < REACH_R:
                    rb.progress = i
            if rb.state == 'done':
                self.on_route_done(rb, now)
            elif rb.state == 'precision_failed':
                rb.goal, rb.pending = None, False
                self.clear_route(rb, 'sai so vi tri tai dich > 2 cm - can kiem tra, khong tu giao lai')
            elif rb.state in ('blocked', 'failed'):
                self.on_trouble(rb, now)

        # 2. O da giu cua tung xe
        self.res = {}
        for n, rb in self.robots.items():
            s = set(self.occ[n])
            if rb.route:
                s |= set(rb.route[rb.progress:rb.upto + 1])
            self.res[n] = s

        # 3. Cap them o, xe gan dich truoc
        movers = [rb for rb in self.robots.values()
                  if rb.route and rb.ack == rb.route_id and not rb.stale
                  and rb.state not in ('blocked', 'failed', 'precision_failed')]
        for rb in sorted(movers, key=self.prio):
            want = min(rb.progress + LOOKAHEAD, len(rb.route) - 1)
            while rb.upto < want:
                nxt = rb.route[rb.upto + 1]
                if self.owner(nxt, rb) is not None:
                    break
                rb.upto += 1
                self.res[rb.name].add(nxt)

        # 4. Cho / ket
        for rb in movers:
            if rb.route is None or rb.upto >= len(rb.route) - 1 or rb.state != 'waiting':
                rb.wait_since = None
                continue
            blk = self.owner(rb.route[rb.upto + 1], rb)
            if blk is None:
                rb.wait_since = None
                continue
            if rb.wait_since is None:
                rb.wait_since = now
                self.log(f'[{rb.name}] cho o {self.lbl(rb.route[rb.upto + 1])} ({blk.name} dang giu)')
                continue
            # Xe chan dang chay / dang xoay thi cu cho (xe that xoay ~5 s moi
            # lan). Chi go khi no do, mat lien lac, cho nguoc lai minh, hoac
            # dung yen han > WAIT_S.
            stuck = (self.parked(blk) or blk.stale or self.waits_for(blk, rb)
                     or now - blk.last_move_t > WAIT_S)
            if stuck and now - rb.wait_since > WAIT_S:
                rb.wait_since = None
                self.resolve(rb, blk)

        # 5. Xe da ne: quay lai khi xe kia di qua
        for rb in self.robots.values():
            if rb.mode == 'yield_wait':
                self.try_resume(rb, now)

        # 6. Gui lenh
        for n, rb in self.robots.items():
            msg = String()
            msg.data = json.dumps({'route_id': rb.route_id, 'nodes': rb.route or [],
                                   'upto': rb.upto, 'final_heading': rb.final_heading})
            self.cmd_pubs[n].publish(msg)
        self.publish_markers()

    def owner(self, node, me):
        for n, s in self.res.items():
            if n != me.name and node in s:
                return self.robots[n]
        return None

    def waits_for(self, a, b):
        """a dang dung cho o ma b giu?"""
        if not a.route or a.state != 'waiting' or a.upto >= len(a.route) - 1:
            return False
        return a.route[a.upto + 1] in self.res.get(b.name, set())

    # ------------------------------------------------------------------ su kien
    def on_route_done(self, rb, now):
        if rb.mode == 'yield':
            if rb.resume is None:
                self.log(f'[{rb.name}] da ne sang {self.lbl(rb.route[-1])}, do lai day')
                rb.route = None
                rb.mode = 'idle'
                return
            self.log(f'[{rb.name}] da ne sang {self.lbl(rb.route[-1])}, cho xe kia qua')
            rb.route = None
            rb.mode = 'yield_wait'
            rb.yield_since = now
            return
        if rb.mode == 'go':
            g = rb.route[-1]
            hd = '' if rb.final_heading is None else f', huong {self.heading_label(rb.final_heading)}'
            self.log(f'[{rb.name}] TOI DICH {self.lbl(g)}{hd}')
        rb.route = None
        rb.mode = 'idle'
        rb.goal = None
        rb.fail_count = 0

    def heading_label(self, yaw):
        d = self.heading_dir(yaw)
        return {(0, 1): HEADING_NAMES['dong'], (1, 0): HEADING_NAMES['bac'],
                (0, -1): HEADING_NAMES['tay'], (-1, 0): HEADING_NAMES['nam']}[d]

    def on_trouble(self, rb, now):
        if now < rb.retry_at:
            return
        rb.retry_at = now + RETRY_S
        rb.fail_count += 1
        target, heading, mode = rb.route[-1], rb.final_heading, rb.mode
        if rb.fail_count > MAX_FAIL:
            rb.goal = None
            self.clear_route(rb, f'{rb.state} {rb.fail_count - 1} lan, bo lo trinh - can nguoi kiem tra')
            return
        hard = set()
        for o in self.others(rb):
            hard |= self.res.get(o.name, self.occupied(o))
        path = self.plan(rb, self.node_of(rb), target, hard=hard)
        if path is None:
            self.log(f'[{rb.name}] {rb.state}: chua co duong toi {self.lbl(target)}, thu lai sau '
                     f'{RETRY_S:.0f} s ({rb.fail_count}/{MAX_FAIL})', 'warn')
            return
        self.log(f'[{rb.name}] {rb.state} - lap lai duong ({rb.fail_count}/{MAX_FAIL})', 'warn')
        self.assign_route(rb, path, heading, mode)

    def resolve(self, rb, blk):
        if self.parked(blk) or blk.stale:
            what = 'mat lien lac' if blk.stale else 'dang do'
            self.log(f'KET: {rb.name} bi {blk.name} ({what}) chan > {WAIT_S:.0f} s', 'warn')
            # Tranh moi o xe kia dang GIU (ca o phia truoc da cap): xe mat lien
            # lac van co the dang chay not doan Nav2 da nhan.
            path = self.plan(rb, self.node_of(rb), rb.route[-1], hard=self.res[blk.name])
            if path is not None:
                self.log(f'[{rb.name}] di vong quanh {blk.name}')
                self.assign_route(rb, path, rb.final_heading, rb.mode)
                return
            if not blk.stale and self.side_step(blk, rb):
                return
        else:
            y, x = (rb, blk) if self.prio(rb) > self.prio(blk) else (blk, rb)
            why = 'cho nhau' if self.waits_for(blk, rb) else f'{blk.name} dung yen > {WAIT_S:.0f} s'
            self.log(f'KET: {rb.name} - {blk.name} {why}. {y.name} xa dich hon -> nhuong', 'warn')
            for a, b in ((y, x), (x, y)):
                if self.yield_to(a, b):
                    return
        self.log(f'KHONG GO DUOC KET giua {rb.name} va {blk.name} - dung ca hai, can nguoi xu ly', 'error')
        for r in (rb, blk):
            r.goal = None
            self.clear_route(r)

    def yield_to(self, y, x):
        """y nhuong x: lap duong vong tranh x, khong duoc thi ne sang o trong."""
        forbid = set(self.occ[x.name]) | set(self.remaining(x))
        if y.route:
            path = self.plan(y, self.node_of(y), y.route[-1], hard=forbid)
            if path is not None:
                self.log(f'[{y.name}] lap duong vong tranh {x.name}')
                self.assign_route(y, path, y.final_heading, y.mode)
                return True
        return self.side_step(y, x)

    def side_step(self, y, x):
        """y ne sang o trong gan nhat khong nam tren duong con lai cua x."""
        forbid = set(self.occ[x.name]) | set(self.remaining(x))
        if x.route:
            forbid.add(x.route[-1])
        bn, be = self.blocked_sets()
        start = self.node_of(y)
        prev = {start: None}
        q = deque([start])
        refuge = None
        while q:
            n = q.popleft()
            if n != start and n not in forbid:
                refuge = n
                break
            for nb in self.board.neighbours(*n):
                if nb in prev or nb in bn or nb in forbid and nb != start:
                    continue
                if edge_key(n, nb) in be:
                    continue
                prev[nb] = n
                q.append(nb)
        if refuge is None:
            return False
        path = [refuge]
        while prev[path[-1]] is not None:
            path.append(prev[path[-1]])
        path.reverse()
        if y.resume is None:
            if y.route:
                y.resume = (y.route[-1], y.final_heading)
            elif x.goal != start and (not x.route or x.route[-1] != start):
                y.resume = (start, y.yaw)   # xe dang do: ne xong quay ve cho cu
            # cho cu chinh la dich xe kia -> o lai cho ne, khong quay ve
        self.assign_route(y, path, None, 'yield')
        return True

    def try_resume(self, rb, now):
        if now - rb.yield_since < 1.0:
            return
        target, heading = rb.resume
        hard = set()
        for o in self.others(rb):
            if self.parked(o):
                hard |= self.occupied(o)
        path = self.plan(rb, self.node_of(rb), target, hard=hard)
        if path is None:
            return
        for o in self.others(rb):
            if not self.parked(o) and set(path) & (set(self.remaining(o)) | self.occ[o.name]):
                return          # xe kia chua qua
        rb.resume = None
        if rb.goal is None:
            rb.goal = target
        self.log(f'[{rb.name}] xe kia da qua, quay lai {self.lbl(target)}')
        self.assign_route(rb, path, heading, 'go')

    # ------------------------------------------------------------------ RViz
    def _mk(self, ns, mid, mtype, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', stamp
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish_markers(self):
        """Mot topic /fleet/markers cho RViz: luoi sa ban, hai xe (than + huong
        dau + ten), duong con lai va o dang giu cua tung xe. Moi thu o khung
        'map' - RViz khong can TF cua xe (moi xe mot cay TF rieng /agvN/tf, mot
        cua so RViz chi doc duoc mot cay)."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        # Luoi: canh xanh nhat, diem xanh; diem / canh bi chan mau do
        bn, be = self.blocked_sets()
        edges = self._mk('grid_edges', 0, Marker.LINE_LIST, stamp)
        edges.scale.x = 0.015
        edges.color = ColorRGBA(r=0.35, g=0.6, b=0.9, a=0.8)
        bad_e = self._mk('grid_edges_blocked', 0, Marker.LINE_LIST, stamp)
        bad_e.scale.x = 0.02
        bad_e.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.9)
        for a, b in self.board.edges():
            tgt = bad_e if edge_key(a, b) in be or a in bn or b in bn else edges
            for nd in (a, b):
                x, y = self.xy(nd)
                tgt.points.append(Point(x=float(x), y=float(y), z=0.005))
        nodes = self._mk('grid_nodes', 0, Marker.SPHERE_LIST, stamp)
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.06
        for nd in self.board.nodes():
            x, y = self.xy(nd)
            nodes.points.append(Point(x=float(x), y=float(y), z=0.01))
            nodes.colors.append(ColorRGBA(r=0.9, g=0.2, b=0.2, a=1.0) if nd in bn
                                else ColorRGBA(r=0.15, g=0.4, b=0.85, a=1.0))
        arr.markers += [edges, nodes]
        if bad_e.points:
            arr.markers.append(bad_e)
        for i, (n, rb) in enumerate(self.robots.items()):
            col = COLORS[i % len(COLORS)]
            # Xe: than tron 18 cm, mui ten huong dau, ten xe
            if rb.x is not None:
                body = self._mk(f'robot_{n}', 0, Marker.CYLINDER, stamp)
                body.pose.position.x, body.pose.position.y, body.pose.position.z = rb.x, rb.y, 0.05
                body.scale.x = body.scale.y = 0.18
                body.scale.z = 0.10
                a = 0.4 if rb.stale else 0.95
                body.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=a)
                head = self._mk(f'robot_{n}', 1, Marker.ARROW, stamp)
                head.points = [Point(x=rb.x, y=rb.y, z=0.11),
                               Point(x=rb.x + 0.2 * math.cos(rb.yaw),
                                     y=rb.y + 0.2 * math.sin(rb.yaw), z=0.11)]
                head.scale.x, head.scale.y, head.scale.z = 0.03, 0.06, 0.06
                head.color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=0.9)
                label = self._mk(f'robot_{n}', 2, Marker.TEXT_VIEW_FACING, stamp)
                label.pose.position.x, label.pose.position.y, label.pose.position.z = rb.x, rb.y, 0.30
                label.scale.z = 0.12
                label.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0)
                label.text = n + (' (MAT LIEN LAC)' if rb.stale else '')
                arr.markers += [body, head, label]
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id = f'route_{n}', 0
            m.type, m.scale.x = Marker.LINE_STRIP, 0.04
            m.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=0.9)
            m.action = Marker.ADD if rb.route else Marker.DELETE
            for node in self.remaining(rb):
                x, y = self.xy(node)
                m.points.append(Point(x=float(x), y=float(y), z=0.03))
            arr.markers.append(m)
            r = Marker()
            r.header.frame_id, r.header.stamp = 'map', stamp
            r.ns, r.id = f'held_{n}', 0
            r.type = Marker.CUBE_LIST
            r.scale.x = r.scale.y = 0.14
            r.scale.z = 0.02
            r.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=0.55)
            for node in self.res.get(n, set()):
                x, y = self.xy(node)
                r.points.append(Point(x=float(x), y=float(y), z=0.01))
            r.action = Marker.ADD if r.points else Marker.DELETE
            arr.markers.append(r)
        self.marker_pub.publish(arr)


def send(node, text):
    msg = String()
    msg.data = text
    node.goal_pub.publish(msg)


def stdin_loop(node):
    """Khong phai terminal (script, test): moi dong mot lenh '1 5 5 d'."""
    print('Nhap: "1 5 5 d" (xe 1 toi hang 5 cot 5, quay Dong), "s" dung tat ca, "q" thoat.')
    for line in sys.stdin:
        line = line.strip()
        if line in ('q', 'quit', 'exit'):
            break
        if line:
            send(node, line)
    rclpy.try_shutdown()


class Quit(Exception):
    pass


def ask(prompt):
    """input() co 'q' = thoat. Het dau vao (Ctrl-D) cung la thoat."""
    try:
        s = input(prompt).strip()
    except EOFError:
        raise Quit()
    if s.lower() in ('q', 'quit', 'exit'):
        raise Quit()
    return s


def ask_goal(node, idx, rb, taken):
    """Hoi hang, cot, huong cho mot xe (giong grid_navigator.py --real).
    Tra ve chuoi lenh '1 3 5 d'."""
    b = node.board
    while True:
        try:
            r = int(ask(f'Xe {idx} - Hàng đích (1-{b.rows}): '))
            c = int(ask(f'Xe {idx} - Cột đích (1-{b.cols}): '))
        except ValueError:
            print('Lỗi: nhập số nguyên.')
            continue
        err = node.goal_error(rb, (r - 1, c - 1), taken)
        if err:
            print(f'Lỗi: {err}. Nhập lại cho xe {idx}.')
            continue
        while True:
            h = ask('Hướng tại đích - Đ(ông)=phải, T(ây)=trái, B(ắc)=lên, '
                    'N(am)=xuống, Enter=giữ hướng đi: ')
            try:
                key, _ = parse_heading(h, b.yaw)
                break
            except ValueError as e:
                print(f'Lỗi: {e}. Nhập D, T, B, N hoặc Enter.')
        taken.add((r - 1, c - 1))
        return f'{idx} {r} {c}' + (f' {key}' if key else ''), (r, c, key)


def interactive_loop(node):
    """Terminal: hoi lan luot xe 1 roi xe 2 (hang, cot, huong); nhap DU ca hai
    moi giao lenh, CUNG LUC; cho ca hai toi noi roi hoi luot tiep."""
    b = node.board
    print(f'\n=== ĐIỀU PHỐI {len(node.order)} XE TRÊN SA BÀN {b.rows}x{b.cols} ===')
    print('Hàng 1 = dưới cùng, cột 1 = bên trái. Gõ q để thoát, Ctrl-C để dừng khẩn.')
    shown = False
    while not (node.map_ok and all(rb.x is not None for rb in node.robots.values())):
        if not shown:
            print('Đang chờ bản đồ và vị trí các xe...')
            shown = True
        time.sleep(0.5)
    try:
        while True:
            print(f'\n--- NHẬP ĐÍCH CHO CẢ {len(node.order)} XE (phải nhập đủ mới chạy) ---')
            cmds, summary, taken, targets = [], [], set(), {}
            for idx, name in enumerate(node.order, start=1):
                rb = node.robots[name]
                r, c = node.node_of(rb)
                hname, _ = heading_name(rb.yaw, b.yaw) if rb.yaw is not None else ('?', 0)
                print(f'\n[Xe {idx} - {name}] đang ở gần (hàng {r + 1}, cột {c + 1}), hướng {hname}')
                cmd, (gr, gc, key) = ask_goal(node, idx, rb, taken)
                cmds.append(cmd)
                targets[name] = (gr - 1, gc - 1)
                summary.append(f'Xe {idx}: (hàng {gr}, cột {gc})'
                               + (f' quay {HEADING_NAMES[key]}' if key else ''))
            print('\n-> ' + ' | '.join(summary))
            send(node, 'pair ' + ' ; '.join(cmds))
            print('-> CẢ HAI XE XUẤT PHÁT. Đang chờ tới nơi... (Ctrl-C để dừng khẩn)')
            wait_all_done(node, targets)
    except Quit:
        pass
    rclpy.try_shutdown()


def wait_all_done(node, goals):
    """Cho moi xe het lo trinh (toi noi, hoac bi dung vi khong co duong).
    goals: ten xe -> (r, c) tu 0 vua nhap."""
    t0 = time.time()
    while True:
        time.sleep(0.5)
        if time.time() - t0 < 1.5:       # cho traffic_manager nhan lenh
            continue
        if all(rb.route is None and not rb.pending and rb.mode == 'idle'
               for rb in node.robots.values()):
            break
    for idx, (n, rb) in enumerate(node.robots.items(), start=1):
        g = goals.get(n)
        if g is None:
            continue
        if node.dist(rb, g) < 0.10:
            print(f'   Xe {idx} ĐÃ TỚI (hàng {g[0] + 1}, cột {g[1] + 1}).')
        else:
            print(f'   Xe {idx} DỪNG trước khi tới (hàng {g[0] + 1}, cột {g[1] + 1}) - xem log phía trên.')


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    grid_file = argv[argv.index('--grid') + 1] if '--grid' in argv else None
    names = [a for a in argv if not a.startswith('--') and a != grid_file] or ['agv1', 'agv2']
    node = TrafficManager(names, grid_file, avoid='--no-avoid' not in argv)
    if '--no-stdin' not in argv:
        loop = interactive_loop if sys.stdin.isatty() else stdin_loop
        threading.Thread(target=loop, args=(node,), daemon=True).start()
    else:
        print('Nhan dich qua /fleet/goal; khong doc lenh cu trong terminal.', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
