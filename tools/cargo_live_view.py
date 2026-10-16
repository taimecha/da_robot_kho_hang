#!/usr/bin/env python3
"""Live view of the two real AGV cameras with gated LiDAR + image box sizing.

    source tools/setup_real_env.bash
    python3 tools/cargo_live_view.py

For each car, five valid frames are averaged once and the result is held on
screen. After loading the box, click that car's DA NHAN HANG button to dispatch
it to the height-based delivery cell. Press R to remeasure before dispatch,
S to save images, Q/Esc to close.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import cargo_measure as cm  # noqa: E402
from grid_board import Board, config_path  # noqa: E402

TARGETS = {'agv1': (4, 5), 'agv2': (3, 5)}  # blue box / brown box, 1-based rows and columns
FRAME_AGE = 2.0
PROCESS_PERIOD = 0.7
PARALLEL_TOL_DEG = 3.0
HEADING_TOL_DEG = 1.0
POSITION_TOL_M = 0.15
LIDAR_WIDTH_TOL_M = 0.03
SAMPLE_COUNT = 5
SAMPLE_WINDOW_S = 12.0
HEIGHT_BOUNDARY_MARGIN_M = 0.005  # calibration error near a 24/30.5/37/43.5/50 cm boundary
GREEN_LOWER_MARGIN_M = 0.002  # verified 23.9 cm box: accept up to 2 mm below nominal minimum
MEASURE_KEYS = ('width', 'height', 'z_cam', 'face_yaw_deg')
PANE_W, PANE_H = 340, 410
BUTTON_TOP, BUTTON_BOTTOM = 358, 398
WINDOW_NAME = 'AGV cargo | Click DA NHAN HANG after loading | R: remeasure S: save Q: quit'


def pickup_arg(value):
    try:
        row, col = (int(part) for part in value.split(','))
    except (ValueError, TypeError):
        raise argparse.ArgumentTypeError('Use ROW,COL, for example 3,5') from None
    return row, col


def angle_error_deg(a, b):
    return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


def pose_gate(status, status_age, target_xy, target_yaw):
    """Require a fresh, stopped robot at the pickup node and East heading."""
    if (status is None or status_age > FRAME_AGE or status.get('x') is None
            or status.get('y') is None or status.get('yaw') is None):
        return 'Waiting for robot pose'
    if status.get('state') not in ('idle', 'done'):
        return 'Robot moving / not at rest'
    if math.hypot(status['x'] - target_xy[0], status['y'] - target_xy[1]) > POSITION_TOL_M:
        return 'Robot not at pickup cell'
    if abs(angle_error_deg(status['yaw'], target_yaw)) >= HEADING_TOL_DEG:
        return 'Align robot East (<1 deg)'
    return None


def gate_result(result, status, status_age, target_xy, target_yaw):
    """Return a reason to hide measurements, or None when checks pass."""
    pose_reason = pose_gate(status, status_age, target_xy, target_yaw)
    if pose_reason:
        return pose_reason
    if abs(result['face_yaw_deg']) > PARALLEL_TOL_DEG:
        return f'Box face angle {result["face_yaw_deg"]:+.1f} deg (max 3)'
    if result['lidar_points'] < 5:
        return 'Not enough LiDAR points'
    if abs(result['width'] - result['lidar_width']) > LIDAR_WIDTH_TOL_M:
        return 'Camera / LiDAR width differs >3 cm'
    if result.get('lidar_edge_inset_px', 0.0) > 4.0:
        return 'Camera cut off box edge; wait for clearer image'
    if not (0.0 < result['height'] < 1.0 and 0.0 < result['width'] < 1.0
            and 0.0 < result['z_cam'] < 1.2):
        return 'Box dimension / distance out of range'
    if any('khong thay vach mat tren/mat truoc' in w for w in result['warnings']):
        return 'Top edge uncertain'
    return None


def height_slot(height_m, sorting):
    """Equal-width height bins: lower inclusive, upper exclusive; final upper inclusive."""
    lo, hi = float(sorting['min_m']), float(sorting['max_m'])
    cols = [int(c) for c in sorting['columns']]
    if not (0 < lo < hi and cols and len(set(cols)) == len(cols)):
        raise ValueError('Invalid cargo.yaml height_sort configuration')
    if not math.isfinite(height_m) or height_m < lo or height_m > hi:
        return None
    index = min(int((height_m - lo) / ((hi - lo) / len(cols))), len(cols) - 1)
    return cols[index]


def supervised_height_slot(height_m, sorting):
    """Allow a supervisor to assign the first/last bin within measurement error."""
    slot = height_slot(height_m, sorting)
    if slot is not None:
        return slot
    lo, hi = float(sorting['min_m']), float(sorting['max_m'])
    if lo - HEIGHT_BOUNDARY_MARGIN_M <= height_m < lo:
        return int(sorting['columns'][0])
    if hi < height_m <= hi + HEIGHT_BOUNDARY_MARGIN_M:
        return int(sorting['columns'][-1])
    return None


def safe_height_slot(height_m, sorting):
    """Mark internal bin edges and unverified outer limits for supervisor review."""
    slot = supervised_height_slot(height_m, sorting)
    if slot is None:
        return None
    lo, hi = float(sorting['min_m']), float(sorting['max_m'])
    step = (hi - lo) / len(sorting['columns'])
    if height_m < lo - GREEN_LOWER_MARGIN_M or height_m > hi:
        return None
    if any(abs(height_m - (lo + i * step)) <= HEIGHT_BOUNDARY_MARGIN_M
           for i in range(1, len(sorting['columns']) + 1)):
        return None
    return slot


def average_results(values):
    """Arithmetic mean of exactly five individually validated measurements."""
    if len(values) != SAMPLE_COUNT:
        raise ValueError(f'Need {SAMPLE_COUNT} valid measurements')
    return {key: sum(v[key] for v in values) / SAMPLE_COUNT for key in MEASURE_KEYS}


def stable_result_reason(values):
    """Reject a five-frame set if measured dimensions move too much."""
    limits = {'width': 0.015, 'height': 0.010, 'z_cam': 0.025,
              'face_yaw_deg': 3.0, 'lidar_width': 0.025}
    for key, limit in limits.items():
        samples = [float(v[key]) for v in values]
        if max(samples) - min(samples) > limit:
            return f'Unstable {key}: wait for 5 matching images'
    return None


def deliveries_complete(cars, board, drop_row, now):
    """Close this measurement session only after both confirmed deliveries arrive."""
    for state in cars.values():
        status = state.get('status')
        slot = state.get('slot')
        if not state.get('sent') or slot is None or status is None:
            return False
        if now - state.get('status_at', 0.0) > FRAME_AGE or status.get('state') != 'done':
            return False
        if status.get('x') is None or status.get('y') is None:
            return False
        x, y = board.xy(drop_row - 1, slot - 1)
        if math.hypot(status['x'] - x, status['y'] - y) > 0.03:
            return False
    return bool(cars)


def image_to_bgr(msg):
    if msg.encoding not in ('bgr8', 'rgb8'):
        raise ValueError(f'Unsupported camera encoding: {msg.encoding}')
    frame = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    frame = frame[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else frame


class LiveView(Node):
    def __init__(self, targets=None):
        super().__init__('cargo_live_view')
        self.board = Board.load(config_path(True))
        self.targets = dict(TARGETS if targets is None else targets)
        if set(self.targets) != set(TARGETS) or len(set(self.targets.values())) != len(self.targets):
            raise ValueError('Each AGV needs a different pickup cell')
        for row, col in self.targets.values():
            if not (1 <= row <= self.board.rows and 1 <= col <= self.board.cols):
                raise ValueError(f'Pickup cell ({row},{col}) is outside the grid')
        self.base_cfg = cm.load_cfg()
        self.sorting = self.base_cfg['height_sort']
        self.drop_row = int(self.base_cfg['place']['deliver_row'])
        if not (1 <= self.drop_row <= self.board.rows):
            raise ValueError('cargo.yaml deliver_row is outside the board')
        if any(not 1 <= int(c) <= self.board.cols for c in self.sorting['columns']):
            raise ValueError('cargo.yaml height_sort column is outside the board')
        height_slot(float(self.sorting['min_m']), self.sorting)  # validate bounds
        self.goal_pub = self.create_publisher(String, '/fleet/goal', 10)
        self.result_pub = self.create_publisher(String, '/fleet/cargo_result', 10)
        self.cars = {}
        for car in self.targets:
            self.cars[car] = {
                'cfg': cm.car_cfg(self.base_cfg, car),
                'scans': deque(maxlen=max(10, self.base_cfg['lidar']['scans'])),
                'frame': None, 'frame_at': 0.0, 'frame_seq': 0,
                'last_measured_frame_seq': 0,
                'status': None, 'status_at': 0.0,
                'last_process': 0.0, 'history': deque(maxlen=SAMPLE_COUNT),
                'sample_started': 0.0,
                'accepted': None, 'reported': False, 'slot': None,
                'near_boundary': False,
                'sent': False, 'action_note': '',
                'view': None, 'reason': 'Waiting for camera',
            }
            self.create_subscription(
                LaserScan, f'/{car}/scan', lambda m, c=car: self.scan_cb(c, m),
                qos_profile_sensor_data)
            self.create_subscription(
                Image, f'/{car}/camera/image_raw', lambda m, c=car: self.image_cb(c, m),
                qos_profile_sensor_data)
        self.create_subscription(String, '/fleet/status', self.status_cb, 10)

    def scan_cb(self, car, msg):
        self.cars[car]['scans'].append(
            (time.monotonic(), msg.angle_min, msg.angle_increment, list(msg.ranges)))

    def image_cb(self, car, msg):
        state = self.cars[car]
        try:
            state['frame'] = image_to_bgr(msg)
            state['frame_at'] = time.monotonic()
            state['frame_seq'] += 1
        except ValueError as e:
            state['reason'] = str(e)

    def status_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        car = status.get('name')
        if car in self.cars:
            self.cars[car]['status'] = status
            self.cars[car]['status_at'] = time.monotonic()

    def process(self, car, now):
        s = self.cars[car]
        if s['accepted'] is not None:
            return  # fixed result and annotated image until R is pressed
        if now - s['last_process'] < PROCESS_PERIOD:
            return
        s['last_process'] = now
        s['view'] = s['frame']
        if s['frame'] is None or now - s['frame_at'] > FRAME_AGE:
            s['reason'] = 'Waiting for live camera image'
        elif s['frame_seq'] == s['last_measured_frame_seq']:
            return  # a delayed camera frame must not count twice
        elif len(s['scans']) < s['cfg']['lidar']['scans'] or now - s['scans'][-1][0] > FRAME_AGE:
            s['reason'] = 'Waiting for live LiDAR scans'
        else:
            s['last_measured_frame_seq'] = s['frame_seq']
            scans = [(a, inc, ranges) for _, a, inc, ranges in
                     list(s['scans'])[-s['cfg']['lidar']['scans']:]]
            try:
                result, vis = cm.measure(s['frame'], scans, s['cfg'],
                                         float(os.environ.get('LIDAR_X', s['cfg']['lidar']['x'])))
                s['view'] = vis  # show detected edges while waiting, not only after locking
                r, c = self.targets[car]
                reason = gate_result(result, s['status'], now - s['status_at'],
                                     self.board.xy(r - 1, c - 1), self.board.yaw)
                if reason is None:
                    if not s['history'] or now - s['sample_started'] > SAMPLE_WINDOW_S:
                        s['history'].clear()
                        s['sample_started'] = now
                    s['history'].append(result)
                    if len(s['history']) == SAMPLE_COUNT:
                        stability_reason = stable_result_reason(s['history'])
                        if stability_reason:
                            s['reason'] = stability_reason
                            s['view'] = vis
                            return  # keep a rolling window; a new frame may replace the outlier
                        average = average_results(s['history'])
                        # cargo_measure annotates its last instantaneous value in
                        # the image. Restore that area: only the averaged numbers
                        # in the panel below are the final result.
                        raw = s['frame']
                        if raw.shape[:2] != vis.shape[:2]:
                            raw = cv2.resize(raw, (vis.shape[1], vis.shape[0]))
                        camera = cm.Camera(s['cfg'])
                        und = cv2.undistort(raw, camera.K, camera.D)
                        vis[:38] = und[:38]
                        s['view'] = vis
                        s['accepted'] = average
                        s['slot'] = supervised_height_slot(average['height'], self.sorting)
                        s['near_boundary'] = (s['slot'] is not None and
                                              safe_height_slot(average['height'], self.sorting) is None)
                        s['action_note'] = ''
                        s['reason'] = f'FINAL: mean of {SAMPLE_COUNT} valid images'
                    else:
                        s['reason'] = f'Measuring {len(s["history"])}/{SAMPLE_COUNT}'
                else:
                    s['reason'] = reason
            except (ValueError, cv2.error) as e:
                s['reason'] = str(e).split('\n')[0][:66]
        if s['accepted'] is None and pose_gate(
                s['status'], now - s['status_at'],
                self.board.xy(self.targets[car][0] - 1, self.targets[car][1] - 1),
                self.board.yaw) is not None:
            s['history'].clear()

    def reset(self):
        reset_cars = []
        for car, s in self.cars.items():
            if s['sent']:
                continue  # a dispatched result remains locked
            s['history'].clear()
            s['sample_started'] = 0.0
            s['accepted'] = None
            s['reported'] = False
            s['slot'] = None
            s['near_boundary'] = False
            s['action_note'] = ''
            s['view'] = s['frame']
            s['reason'] = 'Waiting for 5 new valid images'
            s['last_measured_frame_seq'] = s['frame_seq']
            s['last_process'] = 0.0
            reset_cars.append(car)
        return reset_cars

    def confirm(self, car):
        """Supervisor clicked after loading: publish exactly one delivery goal."""
        s = self.cars[car]
        if s['sent']:
            return False, f'{car}: da gui dich giao, khong gui lap'
        if s['accepted'] is None:
            return False, f'{car}: chua chot 5 lan do hop le'
        if s['slot'] is None:
            return False, (f'{car}: cao {s["accepted"]["height"] * 100:.1f} cm nam ngoai dai '
                           f'{self.sorting["min_m"] * 100:.0f}-{self.sorting["max_m"] * 100:.0f} cm')
        r, c = self.targets[car]
        pose_reason = pose_gate(s['status'], time.monotonic() - s['status_at'],
                                self.board.xy(r - 1, c - 1), self.board.yaw)
        if pose_reason:
            return False, f'{car}: {pose_reason}'
        if self.goal_pub.get_subscription_count() == 0:
            return False, 'Chua thay traffic_manager tren /fleet/goal'
        slot = s['slot']
        x, y = self.board.xy(self.drop_row - 1, slot - 1)
        for other, state in self.cars.items():
            if other == car or not state['sent'] or state['slot'] != slot:
                continue
            o = state['status']
            if o is None or o.get('x') is None or o.get('y') is None or \
                    o.get('state') not in ('idle', 'done') or \
                    math.hypot(o['x'] - x, o['y'] - y) < 0.3:
                return False, f'O ({self.drop_row},{slot}) dang duoc {other} su dung'
        goal = f'{list(self.targets).index(car) + 1} {self.drop_row} {slot}'
        self.goal_pub.publish(String(data=goal))
        self.result_pub.publish(String(data=json.dumps({
            'car': car, 'ok': True, 'confirmed': True,
            'pickup_row': r, 'pickup_col': c,
            'deliver_row': self.drop_row, 'deliver_col': slot,
            'height_m': s['accepted']['height'], 'width_m': s['accepted']['width'],
            'distance_m': s['accepted']['z_cam'], 'measurements': SAMPLE_COUNT,
            'near_height_boundary': s.get('near_boundary', False),
        })))
        s['sent'] = True
        note = ' (sat nguong, nguoi giam sat da xac nhan)' if s.get('near_boundary') else ''
        return True, (f'{car}: da nhan hang -> giao toi ({self.drop_row},{slot}), '
                      f'cao {s["accepted"]["height"] * 100:.1f} cm{note}')

    def canvas(self):
        panes = []
        for car in self.targets:
            s = self.cars[car]
            frame = s['view']
            if frame is None:
                frame = np.zeros((240, 320, 3), np.uint8)
            else:
                frame = cv2.resize(frame, (320, 240))
            pane = np.zeros((PANE_H, PANE_W, 3), np.uint8)
            pane[48:288, 10:330] = frame
            idx = car[-1]
            r, c = self.targets[car]
            cv2.putText(pane, f'AGV {idx}  pickup ({r},{c}) EAST', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.57, (255, 255, 255), 1)
            result = s['accepted']
            color = (60, 220, 60) if result else (0, 180, 255)
            if result:
                labels = [f'FINAL AVG 5   DIST {result["z_cam"] * 100:.1f} cm',
                          f'WIDTH {result["width"] * 100:.1f} cm   HEIGHT {result["height"] * 100:.1f} cm']
            else:
                labels = ['DIST --   WIDTH --   HEIGHT --', s['reason'][:42]]
            for i, label in enumerate(labels):
                cv2.putText(pane, label, (10, 312 + i * 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.51, color, 1)
            if s['sent']:
                button = f'DA GUI DEN ({self.drop_row},{s["slot"]})'
                button_color = (80, 80, 80)
            elif s['action_note']:
                button = s['action_note'][:39]
                button_color = (0, 100, 180)
            elif result and s['slot'] is not None:
                if s.get('near_boundary'):
                    button = f'DA NHAN HANG -> ({self.drop_row},{s["slot"]}) SAT NGUONG'
                    button_color = (0, 120, 180)
                else:
                    button = f'DA NHAN HANG  -> ({self.drop_row},{s["slot"]})'
                    button_color = (40, 125, 45)
            elif result:
                button = f'HEIGHT NGOAI {self.sorting["min_m"] * 100:.0f}-{self.sorting["max_m"] * 100:.0f} CM'
                button_color = (0, 100, 180)
            else:
                button = f'CHO DO DU {SAMPLE_COUNT} ANH'
                button_color = (70, 70, 70)
            cv2.rectangle(pane, (10, BUTTON_TOP), (330, BUTTON_BOTTOM), button_color, -1)
            cv2.putText(pane, button, (17, BUTTON_TOP + 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
            panes.append(pane)
        return np.hstack(panes)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--save-dir', default='/tmp/cargo_live', help='folder for S snapshots')
    ap.add_argument('--agv1-pickup', type=pickup_arg, default=TARGETS['agv1'],
                    metavar='ROW,COL', help='pickup cell for AGV 1')
    ap.add_argument('--agv2-pickup', type=pickup_arg, default=TARGETS['agv2'],
                    metavar='ROW,COL', help='pickup cell for AGV 2')
    args = ap.parse_args()
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('No graphical display (DISPLAY/WAYLAND_DISPLAY). Run in VS Code desktop terminal.')
    rclpy.init()
    node = LiveView(targets={'agv1': args.agv1_pickup, 'agv2': args.agv2_pickup})

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONUP or not BUTTON_TOP <= y <= BUTTON_BOTTOM:
            return
        index = x // PANE_W
        if not 0 <= index < len(node.targets):
            return
        car = list(node.targets)[index]
        ok, message = node.confirm(car)
        if not ok:
            node.cars[car]['action_note'] = message.split(':', 1)[-1].strip()
        print(message, flush=True)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if deliveries_complete(node.cars, node.board, node.drop_row, now):
                print('Ca hai xe da toi o giao hang; dong giao dien camera cho lan thu tiep.', flush=True)
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            for car in node.targets:
                node.process(car, now)
                s = node.cars[car]
                result = s['accepted']
                if result and not s['reported']:
                    print(f'{car}: cach {result["z_cam"] * 100:.1f} cm, '
                          f'rong {result["width"] * 100:.1f} cm, '
                          f'cao {result["height"] * 100:.1f} cm, '
                          f'mat lech {result["face_yaw_deg"]:+.1f} do '
                          f'(trung binh {SAMPLE_COUNT} lan; da chot)', flush=True)
                    s['reported'] = True
            cv2.imshow(WINDOW_NAME, node.canvas())
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key == ord('r'):
                reset = node.reset()
                print('Dang do lai 5 lan: ' + (', '.join(reset) if reset else 'khong co xe nao chua giao'),
                      flush=True)
            if key == ord('s'):
                out = Path(args.save_dir)
                out.mkdir(parents=True, exist_ok=True)
                stamp = time.strftime('%Y%m%d_%H%M%S')
                for car in node.targets:
                    frame = node.cars[car]['view']
                    if frame is not None:
                        cv2.imwrite(str(out / f'{car}_{stamp}.png'), frame)
                cv2.imwrite(str(out / f'comparison_{stamp}.png'), node.canvas())
                print(f'Da luu anh vao {out}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
