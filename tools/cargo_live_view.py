#!/usr/bin/env python3
"""Live view of the two real AGV cameras with gated LiDAR + image box sizing.

    source tools/setup_real_env.bash
    python3 tools/cargo_live_view.py

For each car, five valid frames are averaged once and the result is held on
screen. Press R to measure both cars again, S to save images, Q/Esc to close.
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

TARGETS = {'agv1': (3, 5), 'agv2': (4, 5)}  # 1-based rows and columns
FRAME_AGE = 2.0
PROCESS_PERIOD = 0.7
PARALLEL_TOL_DEG = 3.0
HEADING_TOL_DEG = 1.0
POSITION_TOL_M = 0.15
LIDAR_WIDTH_TOL_M = 0.03
SAMPLE_COUNT = 5
MEASURE_KEYS = ('width', 'height', 'z_cam', 'face_yaw_deg')


def angle_error_deg(a, b):
    return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


def gate_result(result, status, status_age, target_xy, target_yaw):
    """Return a reason to hide measurements, or None when checks pass."""
    if (status is None or status_age > FRAME_AGE or status.get('x') is None
            or status.get('y') is None or status.get('yaw') is None):
        return 'Waiting for robot pose'
    if status.get('state') not in ('idle', 'done'):
        return 'Robot moving / not at rest'
    if math.hypot(status['x'] - target_xy[0], status['y'] - target_xy[1]) > POSITION_TOL_M:
        return 'Robot not at pickup cell'
    if abs(angle_error_deg(status['yaw'], target_yaw)) >= HEADING_TOL_DEG:
        return 'Align robot East (<1 deg)'
    if abs(result['face_yaw_deg']) > PARALLEL_TOL_DEG:
        return f'Box face angle {result["face_yaw_deg"]:+.1f} deg (max 3)'
    if result['lidar_points'] < 5:
        return 'Not enough LiDAR points'
    if abs(result['width'] - result['lidar_width']) > LIDAR_WIDTH_TOL_M:
        return 'Camera / LiDAR width differs >3 cm'
    if not (0.0 < result['height'] < 1.0 and 0.0 < result['width'] < 1.0
            and 0.0 < result['z_cam'] < 1.2):
        return 'Box dimension / distance out of range'
    if any('khong thay vach mat tren/mat truoc' in w for w in result['warnings']):
        return 'Top edge uncertain'
    return None


def average_results(values):
    """Arithmetic mean of exactly five individually validated measurements."""
    if len(values) != SAMPLE_COUNT:
        raise ValueError(f'Need {SAMPLE_COUNT} valid measurements')
    return {key: sum(v[key] for v in values) / SAMPLE_COUNT for key in MEASURE_KEYS}


def image_to_bgr(msg):
    if msg.encoding not in ('bgr8', 'rgb8'):
        raise ValueError(f'Unsupported camera encoding: {msg.encoding}')
    frame = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    frame = frame[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else frame


class LiveView(Node):
    def __init__(self):
        super().__init__('cargo_live_view')
        self.board = Board.load(config_path(True))
        self.base_cfg = cm.load_cfg()
        self.cars = {}
        for car in TARGETS:
            self.cars[car] = {
                'cfg': cm.car_cfg(self.base_cfg, car),
                'scans': deque(maxlen=max(10, self.base_cfg['lidar']['scans'])),
                'frame': None, 'frame_at': 0.0, 'frame_seq': 0,
                'last_measured_frame_seq': 0,
                'status': None, 'status_at': 0.0,
                'last_process': 0.0, 'history': deque(maxlen=SAMPLE_COUNT),
                'accepted': None, 'reported': False,
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
                r, c = TARGETS[car]
                reason = gate_result(result, s['status'], now - s['status_at'],
                                     self.board.xy(r - 1, c - 1), self.board.yaw)
                if reason is None:
                    s['history'].append(result)
                    if len(s['history']) == SAMPLE_COUNT:
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
                        s['reason'] = f'FINAL: mean of {SAMPLE_COUNT} valid images'
                    else:
                        s['reason'] = f'Measuring {len(s["history"])}/{SAMPLE_COUNT}'
                else:
                    s['reason'] = reason
            except (ValueError, cv2.error) as e:
                s['reason'] = str(e).split('\n')[0][:66]
        if s['accepted'] is None and not s['reason'].startswith('Measuring'):
            s['history'].clear()

    def reset(self):
        for s in self.cars.values():
            s['history'].clear()
            s['accepted'] = None
            s['reported'] = False
            s['view'] = s['frame']
            s['reason'] = 'Waiting for 5 new valid images'
            s['last_measured_frame_seq'] = s['frame_seq']
            s['last_process'] = 0.0

    def canvas(self):
        panes = []
        for car in TARGETS:
            s = self.cars[car]
            frame = s['view']
            if frame is None:
                frame = np.zeros((240, 320, 3), np.uint8)
            else:
                frame = cv2.resize(frame, (320, 240))
            pane = np.zeros((350, 340, 3), np.uint8)
            pane[48:288, 10:330] = frame
            idx = car[-1]
            r, c = TARGETS[car]
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
            panes.append(pane)
        return np.hstack(panes)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--save-dir', default='/tmp/cargo_live', help='folder for S snapshots')
    args = ap.parse_args()
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('No graphical display (DISPLAY/WAYLAND_DISPLAY). Run in VS Code desktop terminal.')
    rclpy.init()
    node = LiveView()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            for car in TARGETS:
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
            cv2.imshow('AGV cargo measurement | R: remeasure S: save Q: quit', node.canvas())
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key == ord('r'):
                node.reset()
                print('Da xoa ket qua; dang do lai 5 lan moi cho moi xe', flush=True)
            if key == ord('s'):
                out = Path(args.save_dir)
                out.mkdir(parents=True, exist_ok=True)
                stamp = time.strftime('%Y%m%d_%H%M%S')
                for car in TARGETS:
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
