"""Sa ban luoi dung chung cho grid_navigator.py va cac cong cu trong tools/.

Doc cau hinh (config/grid_real.yaml hoac grid_sim.yaml), tinh toa do tung diem
tren ban do, va kiem tra diem/canh nao qua sat vat can.

Ben trong dung chi so tu 0 (r, c); NGUOI DUNG thay va nhap tu 1:
    hang = r + 1 (1 = duoi cung, y nho nhat)
    cot  = c + 1 (1 = ben trai,  x nho nhat)
(khi yaw_deg != 0 thi "duoi/trai" tinh theo truc sa ban da xoay)
"""
import math
import os

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(ROOT, 'src', 'my_bot_bringup', 'config')


def config_path(real):
    return os.path.join(CONFIG_DIR, 'grid_real.yaml' if real else 'grid_sim.yaml')


def edge_key(a, b):
    """Khoa cua canh noi hai diem (r, c), khong phu thuoc thu tu."""
    return tuple(sorted((tuple(a), tuple(b))))


class Board:
    KEYS = ('rows', 'cols', 'spacing', 'center_row', 'center_col', 'center_x',
            'center_y', 'robot_radius', 'node_clearance', 'edge_clearance')

    def __init__(self, cfg, path=None):
        missing = [k for k in self.KEYS if k not in cfg]
        if missing:
            raise ValueError(f"{path}: thieu {', '.join(missing)}")
        self.path = path
        self.cfg = cfg              # ca cac muc tuy chon (vd rotate:)
        self.rows, self.cols = int(cfg['rows']), int(cfg['cols'])
        self.spacing = float(cfg['spacing'])
        self.center_row, self.center_col = int(cfg['center_row']), int(cfg['center_col'])
        self.center_x, self.center_y = float(cfg['center_x']), float(cfg['center_y'])
        self.robot_radius = float(cfg['robot_radius'])
        self.node_clearance = float(cfg['node_clearance'])
        self.edge_clearance = float(cfg['edge_clearance'])
        if not (1 <= self.center_row <= self.rows and 1 <= self.center_col <= self.cols):
            raise ValueError(f"{path}: tam (hang {self.center_row}, cot {self.center_col}) "
                             f"nam ngoai sa ban {self.rows}x{self.cols}")
        # Goc cua truc "cot tang" so voi truc x ban do (tuy chon, mac dinh 0).
        # Ban do SLAM co huong tuy y (theo luc bat dau ve) nen vach gach that
        # thuong lech vai do so voi truc x/y ban do.
        self.yaw = math.radians(float(cfg.get('yaw_deg', 0.0)))
        self._cos, self._sin = math.cos(self.yaw), math.sin(self.yaw)
        # Optional directed traffic rules, expressed with public 1-based nodes.
        # Empty by default: existing real-robot grids remain bidirectional.
        self.forbidden_directions = set()
        for a, b in cfg.get('forbidden_directions', []):
            a, b = tuple(int(v)-1 for v in a), tuple(int(v)-1 for v in b)
            if (len(a)!=2 or len(b)!=2 or
                    not all(0<=r<self.rows and 0<=c<self.cols for r,c in (a,b)) or
                    abs(a[0]-b[0])+abs(a[1]-b[1])!=1):
                raise ValueError(f'{path}: invalid directed edge {a} -> {b}')
            self.forbidden_directions.add((a,b))
        # Only the final edge into a service goal is constrained; transit
        # through the same node remains available to return trips.
        self.goal_approaches = {}
        for goal, entry in cfg.get('goal_approaches', []):
            goal,entry=tuple(int(v)-1 for v in goal),tuple(int(v)-1 for v in entry)
            if (len(goal)!=2 or len(entry)!=2 or
                    not all(0<=r<self.rows and 0<=c<self.cols for r,c in (goal,entry)) or
                    abs(goal[0]-entry[0])+abs(goal[1]-entry[1])!=1):
                raise ValueError(f'{path}: invalid goal approach {entry} -> {goal}')
            self.goal_approaches[goal]=entry

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(yaml.safe_load(f), path)

    # ------------------------------------------------------------------
    def xy(self, r, c):
        """Toa do (x, y) tren ban do cua diem (r, c) tinh tu 0."""
        u = (c - (self.center_col - 1)) * self.spacing      # doc theo hang (cot tang)
        v = (r - (self.center_row - 1)) * self.spacing      # doc theo cot (hang tang)
        return (self.center_x + u * self._cos - v * self._sin,
                self.center_y + u * self._sin + v * self._cos)

    def closest(self, x, y):
        """Diem (r, c) gan (x, y) nhat, ep vao trong sa ban."""
        dx, dy = x - self.center_x, y - self.center_y
        u = dx * self._cos + dy * self._sin
        v = -dx * self._sin + dy * self._cos
        c = round(u / self.spacing) + self.center_col - 1
        r = round(v / self.spacing) + self.center_row - 1
        return min(max(r, 0), self.rows - 1), min(max(c, 0), self.cols - 1)

    def bounds(self):
        """(min_x, max_x, min_y, max_y) cua 4 goc sa ban tren ban do."""
        corners = [self.xy(r, c) for r in (0, self.rows - 1) for c in (0, self.cols - 1)]
        xs, ys = zip(*corners)
        return min(xs), max(xs), min(ys), max(ys)

    def nodes(self):
        for r in range(self.rows):
            for c in range(self.cols):
                yield r, c

    def edges(self):
        """Moi canh mot lan: noi sang phai va len tren."""
        for r, c in self.nodes():
            if c + 1 < self.cols:
                yield (r, c), (r, c + 1)
            if r + 1 < self.rows:
                yield (r, c), (r + 1, c)

    def neighbours(self, r, c):
        for dr, dc in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if ((r,c),(nr,nc)) not in self.forbidden_directions:
                    yield nr, nc

    @staticmethod
    def label(r, c):
        """Ten diem cho nguoi doc: 'hang 3, cot 3'."""
        return f"hàng {r + 1}, cột {c + 1}"

    # ------------------------------------------------------------------
    def check_map(self, data, width, height, resolution, origin_x, origin_y):
        """So sa ban voi ban do tinh (OccupancyGrid: 0 trong, 100 vat can,
        -1 chua biet).

        Tra ve (diem_bi_bo, canh_bi_bo, khoang_cach): diem/canh cach tuong,
        vat can HOAC vung chua biet gan hon node_clearance / edge_clearance.
        Vung chua biet cung tinh (co the la tuong LiDAR chua thay) - tru cac
        cham chua biet le te giua phong: slam_toolbox de lai, ban do mo phong
        co cham cach diem luoi 7 cm lam chan nham 4 diem.
        """
        from scipy.ndimage import distance_transform_edt, label
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        blocked = grid != 0
        unknown = grid < 0
        lab, n = label(unknown)
        if n:
            size = np.bincount(lab.ravel())
            border = np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]]))
            min_cells = 0.2 / (resolution * resolution)     # 0.2 m2
            speck = (size < min_cells)
            speck[0] = False
            speck[border] = False
            blocked &= ~speck[lab]
        dist = distance_transform_edt(~blocked) * resolution

        def d_at(x, y):
            ix = int(math.floor((x - origin_x) / resolution))
            iy = int(math.floor((y - origin_y) / resolution))
            if 0 <= ix < width and 0 <= iy < height:
                return float(dist[iy, ix])
            return 0.0                      # ngoai ban do = khong biet

        node_dist = {}
        bad_nodes = set()
        for r, c in self.nodes():
            node_dist[(r, c)] = d_at(*self.xy(r, c))
            if node_dist[(r, c)] < self.node_clearance:
                bad_nodes.add((r, c))
        bad_edges = set()
        n = max(2, int(math.ceil(self.spacing / (resolution / 2))) + 1)
        for a, b in self.edges():
            if a in bad_nodes or b in bad_nodes:
                bad_edges.add(edge_key(a, b))
                continue
            (xa, ya), (xb, yb) = self.xy(*a), self.xy(*b)
            for k in range(n):
                t = k / (n - 1)
                if d_at(xa + (xb - xa) * t, ya + (yb - ya) * t) < self.edge_clearance:
                    bad_edges.add(edge_key(a, b))
                    break
        return bad_nodes, bad_edges, node_dist
