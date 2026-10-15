import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
import threading
import sys
import time
import json
import math
import numpy as np

from grid_board import Board, config_path, edge_key as board_edge_key

class GridNavigator(Node):
    def __init__(self, use_sim_time=True, grid_file=None, node_name='grid_navigator',
                 costmap_decay=0.0):
        # use_sim_time truoc day bi dong cung = True. Tren xe that khong co
        # /clock nen node se treo vinh vien ma khong bao loi gi.
        #   Mo phong: python3 grid_navigator.py
        #   Xe that : python3 grid_navigator.py --real
        super().__init__(node_name, parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)
        ])

        # Goi thang controller_server (action follow_path) voi duong thang tren
        # canh - khong qua navigate_to_pose/planner A*, vi A* tu tim duong cong
        # cat qua o sa ban.
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.active_segment = None
        self.final_yaw = None
        self.requested_heading = None   # huong nguoi dung chon cho dich hien tai
        self.imu_yaw = None             # goc tu quaternion BNO055 (/imu/data)
        self.create_subscription(Imu, 'imu/data', self.cb_imu, qos_profile_sensor_data)
        # ESTOP tren ESP32 (firmware gui ngay qua /diag; 1 xe: ns_relay chuyen
        # /agvN/diag -> /diag, 2 xe: /agvN/diag). Bao do va huy lo trinh.
        self.esp_estop = ''
        self.create_subscription(String, 'diag', self.cb_diag, qos_profile_sensor_data)
        # Xoay tai cho do node nay tu lam (xem rotate_tick): lenh vao
        # cmd_vel_nav = dau vao velocity_smoother, cung cho controller phat.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.odom_w = 0.0
        self.gyro_w, self.gyro_t = 0.0, -1e9
        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.rot = None
        self.rot_timer = None
        self.rot_hist = []          # (t, lenh w) trong rot_lag giay qua
        self.marker_pub = self.create_publisher(MarkerArray, '/grid_visualization', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sa ban CO DINH tren ban do, doc tu file cau hinh - khong con tu suy
        # tu khung bao vung trong cua /map: tren phong that khung bao do bi tia
        # LiDAR lot qua cua keo ra ngoai, diem luoi roi vao tuong.
        #   xe that : src/my_bot_bringup/config/grid_real.yaml
        #   mo phong: src/my_bot_bringup/config/grid_sim.yaml
        self.real = not use_sim_time
        self.board = Board.load(grid_file or config_path(self.real))
        self.rows, self.cols = self.board.rows, self.board.cols
        self.map_received = False

        # Ban kinh xe that (0.13 m, giong navigation.yaml). Truoc day 0.6 m /
        # khoang ho 0.65 m - kich thuoc mo hinh xe cu duong kinh 1 m.
        self.robot_radius = self.board.robot_radius
        self.node_clearance = self.board.node_clearance
        self.edge_clearance = self.board.edge_clearance
        # Xoay tai cho (muc "rotate:" trong file sa ban, khong co thi mac dinh)
        rot = self.board.cfg.get('rotate', {}) or {}
        self.rot_start = math.radians(rot.get('start_deg', 10.0))   # lech hon -> xoay
        self.rot_tol = math.radians(rot.get('stop_deg', 2.0))       # du doan toi dich -> dung
        self.rot_done = math.radians(rot.get('done_deg', 5.0))      # con lech hon -> xoay bu
        self.ROT_TRIES = int(rot.get('tries', self.ROT_TRIES))
        self.rot_wmax = rot.get('max_speed', 1.3)                   # rad/s
        self.rot_wmin = rot.get('min_speed', 0.7)                   # rad/s, tren vung chet
        self.rot_accel = rot.get('decel', 2.5)                      # rad/s2, do doc giam toc
        self.rot_lag = rot.get('lag', 0.3)                          # s, lenh -> banh
        self.rot_tau = rot.get('motor_tau', 0.12)                   # s, quan tinh dong co
        # Goc xe con quay them sau lenh 0 = coast * w do duoc (s); 0 = mo hinh cu theo lenh
        self.rot_coast = rot.get('coast', 0.0)
        # Xung vong ho cho goc <= pulse_max_deg (0 = tat): bien do (rad/s) va
        # phan dau xung bi ma sat nuot (s, tu hoc)
        self.rot_pulse_max = math.radians(rot.get('pulse_max_deg', 0.0))
        self.rot_wp = rot.get('pulse_speed', self.rot_wmin)
        self.rot_T0 = rot.get('pulse_dead', 0.0)
        # Chỉ lấy ô vật cản THẬT (giá trị 100). Ngưỡng thấp hơn sẽ nuốt cả vùng
        # inflation của Nav2 và lại gây chặn quá tay.
        self.lethal_threshold = 99
        # Diem/canh sat vat can tren BAN DO TINH (tinh mot lan khi nhan /map),
        # cong voi diem/canh bi vat can moi tren local costmap chan luc chay.
        self.static_blocked_nodes = set()
        self.static_blocked_edges = set()

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos)
            
        self.blocked_nodes = [[False] * self.cols for _ in range(self.rows)]
        self.blocked_edges = set()
        # Ten TUONG DOI: xe don (namespace goc) van la /local_costmap/costmap,
        # nhieu xe (fleet_agent trong /agv1) thanh /agv1/local_costmap/costmap.
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            'local_costmap/costmap',
            self.costmap_callback,
            10)
        # Vat can "phai" tren local costmap: cu costmap_decay giay xoa lop vat can
        # (tru vung 0.5 m quanh xe); chu ky costmap ke tiep danh dau lai tu anh
        # quet 1 s qua (observation_persistence trong navigation.yaml) nen vat
        # that van con, chi o KET (khong tia nao di qua de raytrace xoa) mat.
        self.costmap_decay = costmap_decay
        if costmap_decay > 0:
            from nav2_msgs.srv import ClearCostmapExceptRegion
            self._clear_req = ClearCostmapExceptRegion.Request(reset_distance=0.5)
            self._clear_cli = self.create_client(
                ClearCostmapExceptRegion, 'local_costmap/clear_except_local_costmap')
            self.create_timer(costmap_decay, self.decay_costmap)

        b = self.board
        self.get_logger().info(
            f"Sa bàn {b.rows}x{b.cols}, ô {b.spacing:.2f} m, "
            f"(hàng {b.center_row}, cột {b.center_col}) = ({b.center_x:+.3f}, {b.center_y:+.3f}), "
            f"xoay {math.degrees(b.yaw):+.1f}° "
            f"- {b.path}. Đang chờ bản đồ...")
        
        # Publish static grid periodically
        self.create_timer(0.2, self.publish_markers)   # nut do/xanh doi theo costmap (5 Hz)
        
        self.waypoints = []
        self.current_path_points = []
        self.target_node = None
        self.goal_handle = None
        self.is_canceling = False
        self._replan_timer = None
        self._state_lock = threading.Lock()
        self.replan_attempts = 0
        self.max_replan_attempts = 3

    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            return

        info = msg.info
        bad_nodes, bad_edges, node_dist = self.board.check_map(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y)
        self.static_blocked_nodes = bad_nodes
        self.static_blocked_edges = bad_edges
        for r, c in bad_nodes:
            self.blocked_nodes[r][c] = True
        self.blocked_edges = set(bad_edges)

        self.map_received = True
        if bad_nodes:
            for r, c in sorted(bad_nodes):
                self.get_logger().warn(
                    f"Bỏ điểm ({self.board.label(r, c)}): cách vật cản trên bản đồ "
                    f"{node_dist[(r, c)] * 100:.0f} cm < {self.node_clearance * 100:.0f} cm")
        self.get_logger().info(
            f"Đã đối chiếu sa bàn với bản đồ: {self.rows * self.cols - len(bad_nodes)}/"
            f"{self.rows * self.cols} điểm, bỏ {len(bad_edges)} cạnh sát vật cản.")
        self.publish_markers()

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return x, y, yaw
        except Exception:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                return x, y, yaw
            except Exception as e2:
                return None, None, None

    def get_closest_grid(self, x, y):
        """Diem luoi gan (x, y) nhat (chi so tu 0)."""
        return self.board.closest(x, y)

    def get_start_grid(self, x, y):
        """Diem XUAT PHAT cho BFS: diem gan xe nhat MA KHONG bi chan.

        Xe dung canh mot diem bi bo (vd hang 1, cot 5 sat chan ghe) thi diem
        gan nhat la diem do; BFS tu do se bao 'khong co duong' du xe di duoc.
        """
        best, best_d = None, float('inf')
        for r, c in self.board.nodes():
            if self.blocked_nodes[r][c]:
                continue
            nx, ny = self.get_coordinates(r, c)
            d = math.hypot(nx - x, ny - y)
            if d < best_d:
                best, best_d = (r, c), d
        return best if best is not None else self.get_closest_grid(x, y)

    @staticmethod
    def edge_key(r1, c1, r2, c2):
        return board_edge_key((r1, c1), (r2, c2))

    @staticmethod
    def point_blocked(x, y, obstacles, clearance):
        """Có vật cản nào nằm trong bán kính `clearance` quanh điểm (x, y) không?"""
        limit = clearance * clearance
        for ox, oy in obstacles:
            dx = ox - x
            dy = oy - y
            if dx * dx + dy * dy < limit:
                return True
        return False

    @staticmethod
    def segment_blocked(x1, y1, x2, y2, obstacles, clearance):
        """Có vật cản nào cách đoạn thẳng [P1, P2] dưới `clearance` không?

        Dùng khoảng cách điểm-tới-đoạn-thẳng nên không bỏ sót vật cản nằm lọt
        giữa hai điểm lấy mẫu như cách sampling theo bước 10cm trước đây.
        """
        vx = x2 - x1
        vy = y2 - y1
        length_sq = vx * vx + vy * vy
        limit = clearance * clearance
        for ox, oy in obstacles:
            if length_sq == 0.0:
                t = 0.0
            else:
                t = ((ox - x1) * vx + (oy - y1) * vy) / length_sq
                t = max(0.0, min(1.0, t))
            dx = ox - (x1 + t * vx)
            dy = oy - (y1 + t * vy)
            if dx * dx + dy * dy < limit:
                return True
        return False

    def collect_obstacles(self, msg: OccupancyGrid):
        """Toạ độ thực (x, y) của các ô costmap là vật cản thật, đã lọc theo vùng lưới.

        Ô nằm xa lưới hơn `clearance` thì không thể chặn node hay cạnh nào, nên bỏ
        qua luôn cho nhẹ (thay cho ngưỡng cứng ±4.1m viết tay trước đây).
        """
        info = msg.info
        width = info.width
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        pad = max(self.node_clearance, self.edge_clearance) + resolution
        b = self.board
        min_x, max_x, min_y, max_y = b.bounds()
        grid_min_x, grid_max_x = min_x - pad, max_x + pad
        grid_min_y, grid_max_y = min_y - pad, max_y + pad

        # numpy: duyet tung o bang Python (120x120 o, 2 lan/s) ton ~60% mot nhan
        data = np.asarray(msg.data, dtype=np.int16)
        idx = np.nonzero(data > self.lethal_threshold)[0]
        px = origin_x + (idx % width + 0.5) * resolution
        py = origin_y + (idx // width + 0.5) * resolution
        keep = (px >= grid_min_x) & (px <= grid_max_x) & (py >= grid_min_y) & (py <= grid_max_y)
        return list(zip(px[keep].tolist(), py[keep].tolist()))

    def decay_costmap(self):
        if self._clear_cli.service_is_ready():
            self._clear_cli.call_async(self._clear_req)

    def costmap_callback(self, msg: OccupancyGrid):
        if not self.map_received:
            return

        obstacles = self.collect_obstacles(msg)

        changed = False
        for r, c in self.board.nodes():
            x, y = self.get_coordinates(r, c)
            # Bi chan = sat vat can tren ban do tinh HOAC vat can moi tren costmap
            is_blocked = ((r, c) in self.static_blocked_nodes or
                          self.point_blocked(x, y, obstacles, self.node_clearance))
            if self.blocked_nodes[r][c] != is_blocked:
                self.blocked_nodes[r][c] = is_blocked
                changed = True

        # Cạnh nối 2 node kề nhau cũng phải đủ rộng cho xe chui qua: vật cản nằm
        # đúng giữa 2 node thì không node nào bị chặn nhưng lối đi thì tắc.
        new_blocked_edges = set(self.static_blocked_edges)
        for (r, c), (nr, nc) in self.board.edges():
            x1, y1 = self.get_coordinates(r, c)
            x2, y2 = self.get_coordinates(nr, nc)
            if self.segment_blocked(x1, y1, x2, y2, obstacles, self.edge_clearance):
                new_blocked_edges.add(self.edge_key(r, c, nr, nc))

        if new_blocked_edges != self.blocked_edges:
            self.blocked_edges = new_blocked_edges
            changed = True

        if changed:
            self.publish_markers()

        # Kiểm tra xem lộ trình hiện tại có bị chặn không
        if self.target_node is not None and not self.is_canceling:
            if self.is_path_blocked(obstacles):
                self.get_logger().warn("Phát hiện vật cản trên đường đi! Đang tiến hành dừng xe khẩn cấp...")
                self.cancel_current_goal_and_replan()

    def is_path_blocked(self, obstacles):
        if not self.current_path_points:
            return False

        rx, ry, _ = self.get_robot_pose()
        if rx is None:
            return False

        # Dùng robot_radius (không phải clearance) để chỉ huỷ lộ trình khi lối đi
        # thực sự hẹp hơn thân xe, tránh replan liên tục vì nhiễu lidar.
        pts = [(rx, ry)] + self.current_path_points
        for i in range(len(pts) - 1):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            if self.segment_blocked(x1, y1, x2, y2, obstacles, self.robot_radius):
                return True
        return False

    def get_coordinates(self, row, col):
        x, y = self.board.xy(row, col)
        return float(x), float(y)

    @staticmethod
    def normalize_angle(angle):
        """Đưa góc về [-pi, pi] để so sánh hướng không nhảy sai ở biên ±180°."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # 1. Vẽ các điểm (Nodes)
        nodes_marker = Marker()
        nodes_marker.header.frame_id = 'map'
        nodes_marker.header.stamp = self.get_clock().now().to_msg()
        nodes_marker.ns = 'grid_nodes'
        nodes_marker.id = 0
        nodes_marker.type = Marker.SPHERE_LIST
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = 0.15
        nodes_marker.scale.y = 0.15
        nodes_marker.scale.z = 0.15
        nodes_marker.color.r = 0.0
        nodes_marker.color.g = 0.5
        nodes_marker.color.b = 1.0
        nodes_marker.color.a = 1.0

        # 2. Vẽ các đường nối (Edges)
        edges_marker = Marker()
        edges_marker.header.frame_id = 'map'
        edges_marker.header.stamp = self.get_clock().now().to_msg()
        edges_marker.ns = 'grid_edges'
        edges_marker.id = 1
        edges_marker.type = Marker.LINE_LIST
        edges_marker.action = Marker.ADD
        edges_marker.scale.x = 0.02
        edges_marker.color.r = 0.0
        edges_marker.color.g = 0.8
        edges_marker.color.b = 1.0
        edges_marker.color.a = 0.5

        # Diem bi bo (sat vat can) - van ve, mau do, de biet vi sao xe di vong
        blocked_marker = Marker()
        blocked_marker.header = nodes_marker.header
        blocked_marker.ns = 'grid_blocked'
        blocked_marker.id = 4
        blocked_marker.type = Marker.SPHERE_LIST
        blocked_marker.action = Marker.ADD
        blocked_marker.scale.x = blocked_marker.scale.y = blocked_marker.scale.z = 0.08
        blocked_marker.color.r = 1.0
        blocked_marker.color.g = 0.2
        blocked_marker.color.b = 0.2
        blocked_marker.color.a = 1.0

        for row in range(self.rows):
            for col in range(self.cols):
                x, y = self.get_coordinates(row, col)
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                if self.blocked_nodes[row][col]:
                    blocked_marker.points.append(p)
                    continue
                nodes_marker.points.append(p)

                # Nối với điểm bên phải
                if (col < self.cols - 1
                        and not self.blocked_nodes[row][col + 1]
                        and self.edge_key(row, col, row, col + 1) not in self.blocked_edges):
                    p_right = Point()
                    p_right.x, p_right.y = self.get_coordinates(row, col + 1)
                    p_right.z = 0.0
                    edges_marker.points.append(p)
                    edges_marker.points.append(p_right)

                # Nối với điểm phía trên
                if (row < self.rows - 1
                        and not self.blocked_nodes[row + 1][col]
                        and self.edge_key(row, col, row + 1, col) not in self.blocked_edges):
                    p_up = Point()
                    p_up.x, p_up.y = self.get_coordinates(row + 1, col)
                    p_up.z = 0.0
                    edges_marker.points.append(p)
                    edges_marker.points.append(p_up)

        marker_array.markers.append(nodes_marker)
        marker_array.markers.append(edges_marker)
        if blocked_marker.points:
            marker_array.markers.append(blocked_marker)
        else:
            blocked_marker.action = Marker.DELETE
            blocked_marker.points = []
            marker_array.markers.append(blocked_marker)

        # Nhan "hang,cot" (danh so tu 1) canh moi diem - de doi chieu khi nhap
        for row in range(self.rows):
            for col in range(self.cols):
                x, y = self.get_coordinates(row, col)
                t = Marker()
                t.header = nodes_marker.header
                t.ns = 'grid_labels'
                t.id = row * self.cols + col
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = x + 0.12
                t.pose.position.y = y + 0.10
                t.pose.position.z = 0.05
                t.scale.z = 0.09
                t.color.r = t.color.g = t.color.b = 0.85
                t.color.a = 1.0
                t.text = f"{row + 1},{col + 1}"
                marker_array.markers.append(t)

        # 3. Vẽ Target Node (nếu có)
        if self.target_node is not None:
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'target_node'
            target_marker.id = 2
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = self.target_node[0]
            target_marker.pose.position.y = self.target_node[1]
            target_marker.pose.position.z = 0.05
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            target_marker.color.r = 1.0
            target_marker.color.g = 0.5
            target_marker.color.b = 0.0
            target_marker.color.a = 1.0
            marker_array.markers.append(target_marker)

        # 4. Vẽ Path (nếu có)
        if len(self.current_path_points) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = 'planned_path'
            path_marker.id = 3
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.08
            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 0.8
            
            for pt in self.current_path_points:
                p = Point()
                p.x = pt[0]
                p.y = pt[1]
                p.z = 0.02
                path_marker.points.append(p)
                
            marker_array.markers.append(path_marker)
        else:
            # Xóa path cũ
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.ns = 'planned_path'
            path_marker.id = 3
            path_marker.action = Marker.DELETE
            marker_array.markers.append(path_marker)

        self.marker_pub.publish(marker_array)


    def find_path_bfs(self, start_r, start_c, target_r, target_c):
        if self.blocked_nodes[start_r][start_c] or self.blocked_nodes[target_r][target_c]:
            return None
            
        if (start_r, start_c) == (target_r, target_c):
            return []
        # Duong NGAN NHAT (it canh nhat); trong cac duong dai bang nhau chon
        # duong IT CHO RE nhat - moi cho re la mot lan xoay tai cho ~5-8 s tren
        # xe that. BFS tron truoc day hay chon duong zic-zac (4 doan thay vi 2).
        # Trang thai = (diem, huong vua di toi), chi phi = (so canh, so lan re).
        import heapq
        start = (start_r, start_c, None)
        best = {start: (0, 0)}
        parent = {}
        seq = 0                     # chen giua de heap khong phai so sanh huong (None vs tuple)
        heap = [(0, 0, seq, start_r, start_c, None)]
        goal = None
        while heap:
            n_edge, n_turn, _, r, c, d = heapq.heappop(heap)
            if best.get((r, c, d)) != (n_edge, n_turn):
                continue
            if (r, c) == (target_r, target_c):
                goal = (r, c, d)
                break
            for nr, nc in self.board.neighbours(r, c):
                if self.blocked_nodes[nr][nc]:
                    continue
                if self.edge_key(r, c, nr, nc) in self.blocked_edges:
                    continue
                nd = (nr - r, nc - c)
                cost = (n_edge + 1, n_turn + (1 if d is not None and nd != d else 0))
                key = (nr, nc, nd)
                if key not in best or cost < best[key]:
                    best[key] = cost
                    parent[key] = (r, c, d)
                    seq += 1
                    heapq.heappush(heap, (cost[0], cost[1], seq, nr, nc, nd))
        if goal is None:
            return None

        path = []
        curr = goal
        while curr != start:
            path.append((curr[0], curr[1]))
            curr = parent[curr]
        path.reverse()
        return path

    # Xe cach diem xuat phat xa hon muc nay thi them mot doan dua xe ve diem do
    # truoc (bang xy_goal_tolerance cua Nav2: sai so dich 3 cm).
    SNAP_DIST = 0.03
    # Dich CHINH LA o xe dang dung va xe lech tam o duoi muc nay -> coi nhu da toi,
    # KHONG tao doan sua vi tri. Doan vai cm nam sau lung xe lam xe (hoac RPP,
    # lech > 20 do) quay ca vong de di 4 cm - da gap tren xe that 2026-10-10:
    # xe 2 dung yen bi xoay 158 do. Xe dang di thi van dung SNAP_DIST.
    STAY_DIST = 0.10
    STAY_AHEAD = math.radians(30)   # o nam truoc mui trong goc nay -> van chay toi

    def stay_put(self, off, rx, ry, sx, sy):
        """Dich la o xe dang dung, xe lech `off`: True = coi nhu da toi.
        O nam TRUOC mui (vd Nav2 dung som 8 cm vi vat can thoang qua) -> chay
        thang toi, khong can xoay. O ben hong / sau lung va lech <= STAY_DIST ->
        bo qua, khong quay ca vong de nhich vai cm."""
        if off <= self.SNAP_DIST:
            return True
        if off > self.STAY_DIST:
            return False
        _, _, yaw = self.get_robot_pose()
        if yaw is None:
            return True
        ahead = abs(self.normalize_angle(math.atan2(sy - ry, sx - rx) - yaw)) <= self.STAY_AHEAD
        return not ahead

    # Xoay tai cho: so lan xoay bu toi da; thoi gian ra lenh ngan nhat moi lan (s)
    ROT_TRIES = 5
    ROT_MIN_CMD_S = 0.15
    # Xung dai nhat bi ma sat nuot ma van chap nhan (s)
    ROT_T0_MAX = 0.6

    def build_segments(self, nodes):
        """Chuoi diem luoi lien tiep -> cac DOAN THANG nam tren canh.

        Cac canh lien nhau cung huong gop thanh mot doan: di thang 3 o la mot
        lenh, khong dung lai o tung diem. Doi huong thi bat dau doan moi - Nav2
        (Regulated Pure Pursuit) xoay tai cho o dau doan cho thang hang roi moi
        chay, nen xe khong bao gio cat qua o.
        """
        segs = []
        for a, b in zip(nodes, nodes[1:]):
            xa, ya = self.get_coordinates(*a)
            xb, yb = self.get_coordinates(*b)
            if segs:
                x0, y0, x1, y1 = segs[-1]
                la = math.hypot(x1 - x0, y1 - y0)
                lb = math.hypot(xb - xa, yb - ya)
                same_dir = (la > 0 and lb > 0 and
                            abs((x1 - x0) / la - (xb - xa) / lb) < 1e-6 and
                            abs((y1 - y0) / la - (yb - ya) / lb) < 1e-6)
                if same_dir and math.hypot(x1 - xa, y1 - ya) < 1e-6:
                    segs[-1] = (x0, y0, xb, yb)
                    continue
            segs.append((xa, ya, xb, yb))
        return segs

    # Xe cach diem xuat phat trong khoang nay: KHONG them doan "ve diem" rieng
    # ma cho doan dau xuat phat luon tu cho xe dang dung. Doan ve diem ngan
    # theo huong bat ky bat xe xoay - nhich vai cm - xoay lai: "khung khung"
    # luc moi chay.
    MERGE_START_DIST = 0.15

    def send_grid_goal(self, start_row, start_col, target_row, target_col, start_yaw=None,
                       final_heading=None):
        """final_heading: huong (rad, he map) xe phai quay toi sau khi toi dich;
        None = giu huong canh cuoi cung."""
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.abort_route('Không thấy Nav2 (action follow_path) - '
                             'navigation.launch.py đã chạy và "Managed nodes are active" chưa?')
            return

        self.waypoints = []
        self.current_path_points = []
        self.active_segment = None

        path_nodes = self.find_path_bfs(start_row, start_col, target_row, target_col)
        if path_nodes is None:
            self.abort_route('Đích đến bị vật cản chặn và không có đường vòng')
            return
        nodes = [(start_row, start_col)] + path_nodes

        segs = self.build_segments(nodes)
        rx, ry, _ = self.get_robot_pose()
        sx, sy = self.get_coordinates(start_row, start_col)
        off = math.hypot(sx - rx, sy - ry) if rx is not None else 0.0
        if not segs and rx is not None and self.stay_put(off, rx, ry, sx, sy):
            pass                    # dich la o dang dung, xe da trong o
        elif off > self.SNAP_DIST:
            if segs and off <= self.MERGE_START_DIST:
                # Gan diem xuat phat: doan dau bat dau tu CHO XE DUNG
                x0, y0, x1, y1 = segs[0]
                segs[0] = (rx, ry, x1, y1)
            else:
                # Xa (vd bi chan giua canh) hoac dich chinh la diem nay ->
                # doan rieng dua xe ve diem luoi truoc
                segs.insert(0, (rx, ry, sx, sy))

        target_x, target_y = self.get_coordinates(target_row, target_col)
        self.target_node = (target_x, target_y)
        self.waypoints = segs
        # Huong chot o dich: huong nguoi dung chon, khong thi huong doan cuoi
        last = segs[-1] if segs else None
        if final_heading is not None:
            self.final_yaw = final_heading
        elif last and math.hypot(last[2] - last[0], last[3] - last[1]) > 0.1:
            self.final_yaw = math.atan2(last[3] - last[1], last[2] - last[0])
        else:
            self.final_yaw = None

        route = ' -> '.join(f"({r + 1},{c + 1})" for r, c in nodes)
        self.get_logger().info(f'Lộ trình {route}: {len(segs)} đoạn thẳng trên cạnh.')
        self.refresh_path_points()
        self.publish_markers()
        self.send_next_waypoint()

    def refresh_path_points(self):
        """Đường còn phải đi: điểm cuối đoạn đang chạy + điểm cuối các đoạn sau.

        is_path_blocked() nối vị trí xe với các điểm này để xem vật cản mới có
        nằm trên đường không.
        """
        pts = []
        segs = ([self.active_segment] if getattr(self, 'active_segment', None) else []) + self.waypoints
        for s in segs:
            x, y = s[2], s[3]
            if not pts or math.hypot(pts[-1][0] - x, pts[-1][1] - y) > 1e-6:
                pts.append((x, y))
        self.current_path_points = pts

    def make_path(self, x0, y0, x1, y1):
        """Duong thang tu (x0,y0) toi (x1,y1), moi 5 cm mot diem, huong theo doan."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rclpy.time.Time().to_msg()
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(2, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / 0.05)) + 1)
        for k in range(n):
            t = k / (n - 1)
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x0 + (x1 - x0) * t
            p.pose.position.y = y0 + (y1 - y0) * t
            p.pose.orientation.z = math.sin(yaw / 2)
            p.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(p)
        return path

    def cancel_current_goal_and_replan(self):
        # MultiThreadedExecutor cho phep costmap_callback va timer replan chay
        # dong thoi. Khong co khoa nay thi mot vat can se kich hoat nhieu lan
        # huy/replan chong cheo: log hien dau thoi gian dao nguoc va xe bi tinh
        # lai duong 4-5 lan lien tiep truoc khi on dinh.
        with self._state_lock:
            if self.is_canceling:
                return          # da co mot luot huy dang chay roi
            self.is_canceling = True
            handle = self.goal_handle

        self.stop_rotation()        # dang xoay tai cho thi dung luon
        if handle is not None:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.schedule_replan()

    def cancel_done_callback(self, future):
        self.get_logger().info('Đã dừng xe thành công. Đang tính toán đường vòng...')
        self.schedule_replan()

    def schedule_replan(self, delay=1.0):
        """Hẹn giờ replan sau `delay` giây cho xe dừng hẳn.

        Dùng timer thay vì time.sleep() vì hàm này chạy trong callback của executor;
        sleep ở đây sẽ khoá luôn một thread và làm nghẽn costmap_callback.
        """
        if self._replan_timer is not None:
            self.destroy_timer(self._replan_timer)
            self._replan_timer = None
        self._replan_timer = self.create_timer(delay, self.replan_timer_callback)

    def replan_timer_callback(self):
        if self._replan_timer is not None:
            self.destroy_timer(self._replan_timer)
            self._replan_timer = None
        self.replan_after_cancel()

    def replan_after_cancel(self):
        if self.target_node is None:
            # Lộ trình đã bị huỷ hoặc đã tới nơi trong lúc chờ -> không còn gì để replan.
            self.is_canceling = False
            return

        rx, ry, ryaw = self.get_robot_pose()
        if rx is None:
            self.get_logger().warn("Chưa lấy được vị trí xe từ TF, thử replan lại sau 1 giây...")
            self.schedule_replan()
            return

        current_row, current_col = self.get_start_grid(rx, ry)
        target_row, target_col = self.get_closest_grid(self.target_node[0], self.target_node[1])
        self.get_logger().info(
            f"Bắt đầu Replan từ ({self.board.label(current_row, current_col)}) "
            f"đến ({self.board.label(target_row, target_col)})")
        # GIU is_canceling = True cho toi khi Nav2 chap nhan goal moi.
        # Ha co o day se mo cua cho costmap_callback ke tiep kich hoat mot luot
        # huy nua tren goal_handle da chet.
        self.send_grid_goal(current_row, current_col, target_row, target_col, ryaw,
                            final_heading=self.requested_heading)


    def send_next_waypoint(self):
        if not self.waypoints and self.final_yaw is not None:
            # Toi diem cuoi: dau xe phai theo huong canh vua di, lech <= rot_done
            fy, self.final_yaw = self.final_yaw, None
            _, _, yaw = self.get_robot_pose()
            if yaw is not None and abs(self.normalize_angle(fy - yaw)) >= self.rot_done:
                self.start_rotation(fy, finish=True)
                return
        if not self.waypoints:
            self.get_logger().info('Đã đến nơi thành công toàn bộ lộ trình!')
            self.current_path_points = []
            self.active_segment = None
            self.target_node = None
            self.is_canceling = False
            self.publish_markers()
            print(f"\n=== ĐIỀU KHIỂN AGV SA BÀN {self.rows}x{self.cols} ===")
            print("Xe đang chờ lệnh tiếp theo. Hãy nhập trên terminal...")
            return

        x0, y0, x1, y1 = self.waypoints.pop(0)
        self.active_segment = (x0, y0, x1, y1)
        self.refresh_path_points()
        r, c = self.get_closest_grid(x1, y1)
        self.get_logger().info(
            f'Đoạn thẳng ({x0:+.2f}, {y0:+.2f}) -> ({x1:+.2f}, {y1:+.2f}), '
            f'{math.hypot(x1 - x0, y1 - y0):.2f} m, tới ({self.board.label(r, c)})')

        # Dau xe lech huong doan qua nhieu -> tu xoay tai cho truoc (giam toc,
        # bu tre), xong moi giao doan cho Nav2.
        yaw_des = math.atan2(y1 - y0, x1 - x0)
        _, _, yaw = self.get_robot_pose()
        if (yaw is not None and math.hypot(x1 - x0, y1 - y0) > self.SNAP_DIST and
                abs(self.normalize_angle(yaw_des - yaw)) > self.rot_start):
            self.start_rotation(yaw_des)
            return
        self.send_follow_path()

    def send_follow_path(self):
        """Giao THANG doan dang chay cho controller (khong qua planner A*): xe
        bam dung doan thang tren canh sa ban."""
        x0, y0, x1, y1 = self.active_segment
        goal_msg = FollowPath.Goal()
        goal_msg.path = self.make_path(x0, y0, x1, y1)
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        if hasattr(goal_msg, 'progress_checker_id'):
            goal_msg.progress_checker_id = 'progress_checker'

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Trước đây chỉ log rồi return: waypoints và target_node vẫn còn nên
            # vòng chờ trong input_thread treo vĩnh viễn, phải Ctrl-C mới thoát.
            self.abort_route('Nav2 từ chối điểm đến')
            return

        self.goal_handle = goal_handle
        # Goal moi da song -> ket thuc luot huy, cho phep giam sat vat can tro lai.
        self.is_canceling = False
        self.get_logger().info('Điểm đến được chấp nhận, xe đang di chuyển...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self.is_canceling:
            return

        # Trước đây kết quả bị bỏ qua hoàn toàn: Nav2 kẹt hay hết giờ vẫn bị coi là
        # "đã tới" và xe nhảy sang waypoint kế tiếp từ một vị trí sai.
        status = getattr(future.result(), 'status', None)
        if status == GoalStatus.STATUS_CANCELED:
            return  # luồng huỷ đã lo phần replan
        if status is not None and status != GoalStatus.STATUS_SUCCEEDED:
            self.replan_attempts += 1
            if self.replan_attempts > self.max_replan_attempts:
                self.abort_route(f"Nav2 thất bại {self.replan_attempts} lần liên tiếp")
                return
            # Nav2 bao that bai (thuong la "Failed to make progress": xe di cham
            # hon nguong) - KHONG phai bi chan: vat can thi costmap_callback da
            # huy va BFS lai. Nen DI TIEP doan dang do tu cho xe dang dung.
            # Truoc day BFS lai tu diem luoi GAN NHAT - thuong la diem vua roi
            # phia sau - nen xe lui ve diem xuat phat roi di lai, lap mai.
            self.get_logger().warn(
                f'Nav2 không tới được điểm trung gian (status={status}). '
                f'Đi tiếp đoạn đang dở, lần {self.replan_attempts}/{self.max_replan_attempts}...')
            self.goal_handle = None
            self.retry_active_segment()
            return

        self.replan_attempts = 0
        self.get_logger().info('Đã tới điểm trung gian.')
        self.send_next_waypoint()

    def retry_active_segment(self):
        """Gui lai doan dang chay, bat dau tu vi tri xe hien tai, cung diem cuoi.

        Xe dang nam tren canh (di thang tu dau doan) nen doan moi van nam tren
        canh do va huong ve phia truoc - khong lui ve diem cu.
        """
        seg = self.active_segment
        rx, ry, _ = self.get_robot_pose()
        if seg is None or rx is None:
            self.is_canceling = True
            self.schedule_replan()
            return
        x1, y1 = seg[2], seg[3]
        if math.hypot(x1 - rx, y1 - ry) <= self.SNAP_DIST:
            self.send_next_waypoint()          # thuc ra da toi diem cuoi
            return
        self.waypoints.insert(0, (rx, ry, x1, y1))
        self.send_next_waypoint()

    # ------------------------------------------------------------------
    # XOAY TAI CHO o dau doan - tu lam thay vi de Nav2 xoay.
    #
    # RPP / RotationShim (Jazzy) xoay voi toc do KHONG DOI roi cat dot ngot.
    # Lenh toi banh tre ~0.3 s nen xe troi them toc_do x 0.3 s ~ 20-40 do roi
    # xoay nguoc sua, qua lai. O day:
    #   - du doan goc se troi: goc_con_lai - toc_do_quay_do_duoc * rot_lag,
    #     ra lenh dung khi du doan da toi dich (bu tre);
    #   - giam toc theo sqrt(2 * rot_accel * goc_con_lai) nhung khong duoi
    #     rot_wmin: cham qua thi banh ket o ma sat tinh;
    #   - dung han roi do lai, con lech > rot_done thi xoay bu (toi da 3 lan).
    # Lenh di vao cmd_vel_nav nhu lenh cua controller -> van qua
    # velocity_smoother va collision_monitor.
    # ------------------------------------------------------------------
    def cb_odom(self, msg):
        self.odom_w = msg.twist.twist.angular.z

    def cb_diag(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        reason = d.get('estop') or ''
        if reason and not self.esp_estop:
            self.esp_estop = reason
            if self.goal_handle is not None:
                try:
                    self.goal_handle.cancel_goal_async()
                except Exception:       # noqa: BLE001 - van huy lo trinh
                    pass
            self.get_logger().error(
                f'!!! ESP32 ESTOP: {reason} - xe dung han. Kiem tra xe (pin, banh ket, day '
                f'dong co) roi go ESTOP: tat/bat nguon xe, hoac ros2 topic pub --once '
                f'/agv<so xe>/estop_reset std_msgs/msg/Empty')
            self.abort_route(f'ESP32 ESTOP ({reason})')
        elif not reason and self.esp_estop and d.get('event') != 'ESTOP':
            self.get_logger().warn(f'ESP32 da go ESTOP ({self.esp_estop}) - nhap dich moi')
            self.esp_estop = ''

    def cb_imu(self, msg):
        q = msg.orientation
        if abs(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z - 1.0) < 0.1:   # bo goi toan 0
            self.imu_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.gyro_w = msg.angular_velocity.z        # firmware da tru bias
        self.gyro_t = time.monotonic()

    def rot_w(self):
        """Toc do quay that (rad/s) cho bo xoay: gyro /imu/data neu con moi.
        /odom twist KHONG dung duoc: tinh tu do chenh goc giua 2 lan cap nhat
        nen giat 0 / 128 do/s (gyro that 55 do/s) - gai lam bo xoay tuong da
        quay du, dung som 20-40 do."""
        if time.monotonic() - self.gyro_t < 0.2:
            return self.gyro_w
        return self.odom_w

    def publish_w(self, w):
        t = Twist()
        t.angular.z = float(w)
        self.cmd_pub.publish(t)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.rot_hist.append((now, float(w)))
        while self.rot_hist and now - self.rot_hist[0][0] > self.rot_lag:
            self.rot_hist.pop(0)

    def pending_rotation(self):
        """Goc da RA LENH trong rot_lag giay vua qua nhung banh chua kip quay."""
        return sum(w * 0.05 for _, w in self.rot_hist)

    def learn_pulse(self, T, turned):
        """Cap nhat rot_T0 (phan dau xung bi ma sat nuot) tu mot xung dai T
        giay da lam xe quay `turned` rad (duong = dung chieu)."""
        if turned < math.radians(1.0):
            # Xe gan nhu khong nhuc nhich: ca xung bi nuot
            if self.rot_T0 >= self.ROT_T0_MAX - 1e-6:
                # Xung dai toi da van khong du: tang bien do xung
                self.rot_wp = min(self.rot_wmax, self.rot_wp * 1.25)
            self.rot_T0 = min(self.ROT_T0_MAX, max(self.rot_T0, T))
            return
        obs = T - turned / self.rot_wp
        self.rot_T0 = min(self.ROT_T0_MAX, max(0.0, 0.5 * (self.rot_T0 + obs)))

    def start_rotation(self, yaw_des, finish=False):
        self.rot = {'target': yaw_des, 'phase': None, 't': None, 'tries': 0,
                    't0': self.get_clock().now(), 'finish': finish}
        self.get_logger().info(f'Xoay tại chỗ tới hướng {math.degrees(yaw_des):+.0f}°')
        if self.rot_timer is None:
            self.rot_timer = self.create_timer(0.05, self.rotate_tick)

    def stop_rotation(self):
        if self.rot_timer is not None:
            self.destroy_timer(self.rot_timer)
            self.rot_timer = None
        if self.rot is not None:
            self.rot = None
            self.publish_w(0.0)

    def rotate_tick(self):
        r = self.rot
        if r is None:
            return
        now = self.get_clock().now()
        _, _, yaw = self.get_robot_pose()
        if yaw is None:
            self.publish_w(0.0)
            return
        err = self.normalize_angle(r['target'] - yaw)
        if r.get('err0') is None:
            # Bat dau mot lan xoay (lan dau hoac xoay bu)
            r['err0'], r['yaw0'], r['wpeak'], r['t_turn'] = err, yaw, 0.0, now
            if abs(err) <= self.rot_pulse_max:
                # Goc nho: XUNG vong ho. Lenh toi banh tre ~0.3 s nen vong kin
                # khong dung chinh xac duoc goc nho (luc thay xe nhuc nhich da
                # co 0.3 s lenh nam san tren duong ~12-17 do -> vot qua dich,
                # hoac dung som khi xe con ket ma sat tinh -> dung im). Xung
                # w*T thi tong goc quay = tich phan lenh, khong phu thuoc tre;
                # rot_T0 = phan dau xung bi ma sat nuot, hoc dan sau moi xung.
                T = abs(err) / self.rot_wp + self.rot_T0
                r['phase'], r['T'] = 'pulse', T
                r['pulse_end'] = now + Duration(seconds=T)
            else:
                r['phase'] = 'turn'
        r['wpeak'] = max(r['wpeak'], abs(self.rot_w()))
        if r['phase'] == 'pulse':
            if now < r['pulse_end']:
                self.publish_w(math.copysign(self.rot_wp, r['err0']))
            else:
                self.publish_w(0.0)
                r['phase'], r['t'] = 'settle', now
            return
        if r['phase'] == 'turn':
            if self.rot_coast > 0:
                # Goc xe con quay them sau khi ra lenh 0 ~ rot_coast * w DO DUOC
                # (xe that 2026-10-03: 0.36 s, on dinh 0.3-0.45). Dua tren toc
                # do THAT nen dung ca khi banh yeu khong theo kip lenh (lenh 1.3
                # rad/s, xe chi quay 0.86) - tinh theo lenh thi dung som ~20 do.
                pred = err - self.rot_coast * self.rot_w()
            else:
                # Mo hinh theo LENH (coast: 0): lenh trong rot_lag giay qua chac
                # chan toi banh. Dung cho xe ao (khong ma sat). Xe that banh yeu,
                # ma sat tinh nuot lenh ngan -> xoay bu dung im: dung coast > 0.
                pred = err - self.pending_rotation() - self.rot_w() * self.rot_tau
            # Ra lenh it nhat ROT_MIN_CMD_S: xoay bu 3-4 do chi ra lenh mot nhip
            # 50 ms - velocity_smoother chua kip tang toc qua vung chet dong co,
            # xe dung im, lan bu nao cung con nguyen goc lech.
            held = (now - r['t_turn']).nanoseconds * 1e-9
            if (abs(pred) <= self.rot_tol or pred * err < 0) and held >= self.ROT_MIN_CMD_S:
                self.publish_w(0.0)
                r['phase'], r['t'] = 'settle', now
                return
            w = math.sqrt(2.0 * self.rot_accel * abs(pred))
            w = min(self.rot_wmax, max(self.rot_wmin, w))
            self.publish_w(math.copysign(w, pred))
            return
        # settle: cho xe dung han roi danh gia. Doi it nhat rot_lag + 3 tau:
        # vua ra lenh dung thi lenh cu van dang tren duong toi banh, xe co the
        # chua kip nhuc nhich (odom_w = 0) ma chua phai da xong.
        self.publish_w(0.0)
        still_for = (now - r['t']).nanoseconds * 1e-9
        min_wait = self.rot_lag + 3 * self.rot_tau
        if (abs(self.rot_w()) < 0.05 and still_for > min_wait) or still_for > 2.5:
            turned = self.normalize_angle(yaw - r['yaw0'])
            note = f"ω max {math.degrees(r['wpeak']):.0f}°/s"
            if 'T' in r:
                self.learn_pulse(r['T'], turned * math.copysign(1.0, r['err0']))
                note = f"xung {r['T']*1000:.0f} ms, quay {math.degrees(abs(turned)):.1f}°"
            r.setdefault('log', []).append(
                f"{math.degrees(r['err0']):+.0f}°→{math.degrees(err):+.1f}° ({note})")
            r['err0'] = None
            r.pop('T', None)
            if abs(err) >= self.rot_done and r['tries'] < self.ROT_TRIES:
                r['tries'] += 1
                r['phase'] = 'turn'
                return
            took = (now - r['t0']).nanoseconds * 1e-9
            msg = (f'Xoay xong sau {took:.1f} s, còn lệch {math.degrees(err):+.1f}° '
                   f'({r["tries"]} lần xoay bù: {", ".join(r["log"])})')
            if abs(err) >= self.rot_done:
                self.get_logger().error(msg + f' - CHƯA ĐẠT < {math.degrees(self.rot_done):.1f}°')
                self.stop_rotation()
                self.abort_route(f'sai so goc con {math.degrees(abs(err)):.1f}° '
                                 f'>= {math.degrees(self.rot_done):.1f}°')
                return
            self.get_logger().info(msg)
            finish = r['finish']
            self.stop_rotation()
            if finish:
                self.send_next_waypoint()      # het doan -> bao da toi noi
            else:
                self.send_follow_path()

    def abort_route(self, reason):
        """Dừng hẳn lộ trình và trả quyền điều khiển về cho người dùng.

        Cần thiết vì input_thread đang chờ trên `while waypoints or target_node`;
        nếu không dọn trạng thái ở đây thì terminal treo vĩnh viễn.
        """
        self.stop_rotation()
        self.get_logger().error(f'Huỷ lộ trình: {reason}')
        self.waypoints = []
        self.current_path_points = []
        self.target_node = None
        self.goal_handle = None
        self.is_canceling = False
        self.replan_attempts = 0
        self.publish_markers()
        print(f"\n[LỖI] {reason}. Xe đã dừng — hãy nhập lệnh mới.")

# Huong tai dich, theo SA BAN (khong phai la ban that), tinh tu truc sa ban
# (yaw_deg trong grid_*.yaml) - grid_yaw cong them de ra goc tren ban do:
#   Dong = sang phai (cot tang)   Bac = len tren (hang tang)
#   Tay  = sang trai              Nam = xuong duoi
HEADINGS = {'dong': 0.0, 'bac': math.pi / 2, 'tay': math.pi, 'nam': -math.pi / 2}
HEADING_NAMES = {'dong': 'Đông', 'bac': 'Bắc', 'tay': 'Tây', 'nam': 'Nam'}


def parse_heading(text, grid_yaw=0.0):
    """'D'/'Đ'/'dong'/'đông' ... -> (khoa, rad he map). '' -> (None, None). Sai -> ValueError."""
    t = text.strip().lower()
    for a, b in (('đ', 'd'), ('ô', 'o'), ('ơ', 'o'), ('á', 'a'), ('ắ', 'a'), ('â', 'a'),
                 ('ă', 'a'), ('ấ', 'a'), ('ả', 'a'), ('à', 'a'), ('ạ', 'a')):
        t = t.replace(a, b)
    if t == '':
        return None, None
    for key in HEADINGS:
        if t == key or t == key[0]:
            a = HEADINGS[key] + grid_yaw
            return key, math.atan2(math.sin(a), math.cos(a))
    raise ValueError(f"không hiểu hướng '{text}'")


def heading_name(yaw, grid_yaw=0.0):
    """Goc (rad, he map) -> ten huong sa ban gan nhat + do lech."""
    yaw -= grid_yaw
    best = min(HEADINGS, key=lambda k: abs(math.atan2(math.sin(yaw - HEADINGS[k]),
                                                       math.cos(yaw - HEADINGS[k]))))
    d = math.degrees(math.atan2(math.sin(yaw - HEADINGS[best]), math.cos(yaw - HEADINGS[best])))
    return HEADING_NAMES[best], d


def status_line_loop(nav, stop):
    """Dong trang thai CO DINH o dau terminal: goc IMU, vi tri, huong xe.

    Cap nhat 3 lan/giay ke ca luc dang cho nhap. Dong 1 cua man hinh duoc tach
    khoi vung cuon (DECSTBM) nen chu nhap va log cuon ben duoi khong de len no.
    """
    import shutil
    rows = shutil.get_terminal_size().lines
    sys.stdout.write(f"7[2;{rows}r8")      # vung cuon = dong 2..het
    sys.stdout.flush()
    while not stop.is_set():
        parts = []
        if nav.imu_yaw is not None:
            parts.append(f"IMU yaw {math.degrees(nav.imu_yaw):+7.1f}°")
        else:
            parts.append("IMU: chưa có /imu/data")
        rx, ry, ryaw = nav.get_robot_pose()
        if rx is not None:
            name, d = heading_name(ryaw, nav.board.yaw)
            r, c = nav.get_closest_grid(rx, ry)
            parts.append(f"xe ({rx:+.2f}, {ry:+.2f}) hướng {math.degrees(ryaw):+6.1f}° "
                         f"≈ {name} {d:+.0f}°  gần ({r + 1},{c + 1})")
        else:
            parts.append("chưa có vị trí (AMCL)")
        if nav.rot is not None:
            parts.append("đang xoay")
        elif nav.target_node is not None:
            parts.append("đang chạy")
        line = " | ".join(parts)
        sys.stdout.write(f"7[1;1H[2K[7m {line} [0m8")
        sys.stdout.flush()
        stop.wait(0.33)
    sys.stdout.write("7[r8")                  # tra lai vung cuon
    sys.stdout.flush()


def input_thread(navigator):
    print("Đang chờ khởi tạo hệ thống và nhận bản đồ...")
    nav = navigator
    R, C = nav.rows, nav.cols

    timeout = 30  # 30 lần x 0.5s = 15 giây (discovery trong WSL có lúc 8 s)
    while rclpy.ok() and not nav.map_received and timeout > 0:
        time.sleep(0.5)
        timeout -= 1

    if not nav.map_received:
        if nav.real:
            # Truoc day: khong co ban do thi tu dung luoi 10x10 m - tren phong
            # that 6x5 m nghia la gui xe toi diem nam ngoai tuong.
            print("\n[LỖI] Không nhận được /map sau 15 giây. Hãy chạy trước:")
            print("    ros2 launch my_bot_bringup localization.launch.py use_sim_time:=false")
            print("  (hoặc navigation.launch.py use_sim_time:=false) rồi chạy lại.")
            rclpy.shutdown()
            return
        print("\n[CẢNH BÁO] Không nhận được bản đồ sau 15 giây - chạy tiếp "
              "nhưng KHÔNG loại được điểm sát vật cản trên bản đồ.")
        nav.map_received = True
    else:
        # Đợi thêm 1 chút để TF buffer nhận đủ dữ liệu
        time.sleep(1.0)

    while rclpy.ok():
        print(f"\n=== ĐIỀU KHIỂN AGV SA BÀN {R}x{C} ===")

        rx, ry, ryaw = nav.get_robot_pose()
        if rx is not None:
            current_row, current_col = nav.get_start_grid(rx, ry)
            start_yaw = ryaw
            nx, ny = nav.get_coordinates(current_row, current_col)
            off = math.hypot(nx - rx, ny - ry)
            where = (f"Xe gần nhất điểm ({nav.board.label(current_row, current_col)}), "
                     f"cách {off * 100:.0f} cm.")
        else:
            current_row, current_col = nav.board.center_row - 1, nav.board.center_col - 1
            start_yaw = None
            where = "Chưa biết vị trí xe (chưa có TF map -> base_footprint)."

        try:
            print(where)
            target_row_str = input(f"Nhập HÀNG đích (1-{R}, 1 = dưới cùng) hoặc 'q' để thoát: ")
            if target_row_str.strip().lower() == 'q':
                rclpy.shutdown()
                break
            target_row = int(target_row_str) - 1
            target_col = int(input(f"Nhập CỘT đích (1-{C}, 1 = bên trái): ")) - 1

            if not (0 <= target_row < R and 0 <= target_col < C):
                print(f"Lỗi: ngoài sa bàn. Hàng 1-{R}, cột 1-{C}.")
                continue
            if nav.blocked_nodes[target_row][target_col]:
                print(f"Điểm ({nav.board.label(target_row, target_col)}) đang bị chặn "
                      "(sát vật cản). Chọn điểm khác.")
                continue
            while True:
                h = input("Hướng tại đích - Đ(ông)=phải, T(ây)=trái, B(ắc)=lên, "
                          "N(am)=xuống, Enter=giữ hướng đi: ")
                try:
                    hkey, hyaw = parse_heading(h, nav.board.yaw)
                    break
                except ValueError as e:
                    print(f"Lỗi: {e}. Nhập D, T, B, N hoặc Enter.")
            if hkey:
                print(f"-> Tới ({nav.board.label(target_row, target_col)}) rồi quay về "
                      f"{HEADING_NAMES[hkey]} ({math.degrees(hyaw):+.0f}°), sai số ≤ "
                      f"{math.degrees(nav.rot_done):.0f}°.")

            if nav.esp_estop:
                print(f"[LỖI] Xe đang ESTOP ({nav.esp_estop}) - gỡ ESTOP trước rồi nhập lại.")
                continue
            nav.requested_heading = hyaw
            nav.send_grid_goal(current_row, current_col, target_row, target_col, start_yaw,
                               final_heading=hyaw)

            # Tạm thời khóa thread input để xe chạy
            while len(nav.waypoints) > 0 or nav.target_node is not None:
                time.sleep(1)
                if not rclpy.ok():
                    break

        except ValueError:
            print(f"Lỗi: chỉ nhập số nguyên (hàng 1-{R}, cột 1-{C})!")
        except EOFError:
            break

def main(args=None):
    rclpy.init(args=args)

    # --real: chay tren xe that (dung dong ho he thong thay vi /clock cua Gazebo)
    use_sim_time = '--real' not in sys.argv
    print(f"Che do: {'MO PHONG (use_sim_time=true)' if use_sim_time else 'XE THAT (use_sim_time=false)'}")
    # --grid FILE: dung sa ban khac file mac dinh (grid_real.yaml / grid_sim.yaml)
    grid_file = None
    if '--grid' in sys.argv:
        i = sys.argv.index('--grid')
        if i + 1 < len(sys.argv):
            grid_file = sys.argv[i + 1]

    # --decay S: xoa o vat can KET tren local costmap moi S giay (mac dinh tat:
    # moi lan xoa costmap in mot dong INFO ra terminal navigation.launch.py)
    decay = 0.0
    if '--decay' in sys.argv:
        i = sys.argv.index('--decay')
        decay = float(sys.argv[i + 1]) if i + 1 < len(sys.argv) else 1.0

    navigator = GridNavigator(use_sim_time=use_sim_time, grid_file=grid_file,
                              costmap_decay=decay)

    # Chạy vòng lặp spin của ROS trong background thread
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Dong trang thai co dinh o dau terminal (goc IMU, vi tri, huong)
    stop_status = threading.Event()
    status_thread = None
    if sys.stdout.isatty() and '--no-status' not in sys.argv:
        status_thread = threading.Thread(target=status_line_loop,
                                         args=(navigator, stop_status), daemon=True)
        status_thread.start()

    # Thread chính dùng để nhận input liên tục
    try:
        input_thread(navigator)
    finally:
        stop_status.set()
        if status_thread is not None:
            status_thread.join(timeout=1.0)
    
    navigator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
