#!/usr/bin/env python3
"""Live view of the two real AGV cameras with gated LiDAR + image box sizing.

    source tools/setup_real_env.bash
    python3 tools/cargo_live_view.py

Click BAT DAU to reach the observation cell (one step before pickup). Five
valid frames lock the class and dispatch pickup. Only after arrival and manual
loading confirmation is delivery dispatched. After unloading, click DA DO HANG
to record it, then QUAY LAI for another independent cycle. R: remeasure;
S: save; Q/Esc: close. Never run cargo_task.py at the same time.
"""
import argparse
from collections import deque
import json
import math
import os
import re
from pathlib import Path
import sys
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from rcl_interfaces.msg import Log
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import cargo_measure as cm  # noqa: E402
from grid_board import Board, config_path  # noqa: E402

TARGETS = {'agv1': (4, 5), 'agv2': (3, 5)}  # pickup cells only; cargo class is not tied to car
FRAME_AGE = 2.0
PROCESS_PERIOD = 0.35  # ESP32-CAM supplies 3 fresh frames/s
PARALLEL_TOL_DEG = 15.0  # ray-plane intersection already compensates moderate face yaw
HEADING_TOL_DEG = 2.5  # matches grid_real.yaml rotate.done_deg; user requirement <3 deg
POSITION_TOL_M = 0.15
LIDAR_WIDTH_TOL_M = 0.03
SAMPLE_COUNT = 5
SAMPLE_WINDOW_S = 12.0
SUCCESS_HOLD_S = 0.0  # retained CLI compatibility; window stays open for more trips
WORKFLOW_POSITION_TOL_M = 0.02
LEG_TIMEOUT_S = 300.0
HEIGHT_BOUNDARY_MARGIN_M = 0.005  # calibration error near a 24/30.5/37/43.5/50 cm boundary
GREEN_LOWER_MARGIN_M = 0.002  # verified 23.9 cm box: accept up to 2 mm below nominal minimum
MEASURE_KEYS = ('width', 'height', 'z_cam', 'face_yaw_deg')
PANE_W, PANE_H = 340, 410
BUTTON_TOP, BUTTON_BOTTOM = 358, 398
WINDOW_NAME = 'AGV cargo | Observe -> pickup -> confirm -> deliver -> repeat | R: remeasure Q: quit'


def emit_cargo_event(node, car, event, now=None, **details):
    """Record comparable workflow milestones without changing robot commands."""
    if getattr(node, 'event_log_path', None) is None and getattr(node, 'event_pub', None) is None:
        return
    moment = time.monotonic() if now is None else now
    state = node.cars[car]
    record = dict(run_id=node.run_id, car=car, cycle=state.get('cycle'),
                  event=event, t_monotonic=moment, t_unix=time.time(), **details)
    data = json.dumps(record, ensure_ascii=False, allow_nan=False)
    if node.event_log_path is not None:
        with open(node.event_log_path, 'a', encoding='utf-8') as stream:
            stream.write(data + '\n')
    if node.event_pub is not None:
        node.event_pub.publish(String(data=data))


def unfinished_observation(path, car):
    """Find the last started cycle that has not reached its observation cell."""
    pending = None
    try:
        with open(path, encoding='utf-8') as stream:
            for line in stream:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if event.get('car') != car:
                    continue
                if event.get('event') == 'started':
                    pending = event
                elif (pending is not None and
                      event.get('run_id') == pending.get('run_id') and
                      event.get('cycle') == pending.get('cycle') and
                      event.get('event') == 'observation_arrived'):
                    pending = None
    except FileNotFoundError:
        return None
    return pending


def observation_cell(pickup, heading, board):
    """One grid step behind pickup, along the configured facing direction."""
    from grid_navigator import parse_heading
    yaw = parse_heading(heading, 0.0)[1]
    dc, dr = round(math.cos(yaw)), round(math.sin(yaw))
    cell = pickup[0] - dr, pickup[1] - dc
    if not (1 <= cell[0] <= board.rows and 1 <= cell[1] <= board.cols):
        raise ValueError(f'Observation cell {cell} is outside the grid')
    return cell


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
    if not all(math.isfinite(status[k]) for k in ('x', 'y', 'yaw')):
        return 'Waiting for valid robot pose'
    if status.get('state') not in ('idle', 'done'):
        return 'Robot moving / not at rest'
    if math.hypot(status['x'] - target_xy[0], status['y'] - target_xy[1]) > POSITION_TOL_M:
        return 'Robot not at pickup cell'
    if abs(angle_error_deg(status['yaw'], target_yaw)) >= HEADING_TOL_DEG:
        return f'Align robot East (<{HEADING_TOL_DEG:g} deg)'
    return None


def gate_result(result, status, status_age, target_xy, target_yaw):
    """Return a reason to hide measurements, or None when checks pass."""
    numeric = ('width','height','z_cam','face_yaw_deg','lidar_width')
    if not all(math.isfinite(result.get(k,float('nan'))) for k in numeric):
        return 'Invalid box measurement'
    if not math.isfinite(result.get('face_rms_m',0.0)):
        return 'Invalid LiDAR face fit'
    pose_reason = pose_gate(status, status_age, target_xy, target_yaw)
    if pose_reason:
        return pose_reason
    if abs(result['face_yaw_deg']) > PARALLEL_TOL_DEG:
        return f'Box too oblique ({result["face_yaw_deg"]:+.1f} deg, max 15)'
    if result.get('face_rms_m', 0.0) > .012:
        return 'LiDAR face fit too noisy'
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


def delivery_arrived(state, board, drop_row, now):
    """Require a fresh successful stop at the assigned delivery cell."""
    status = state.get('status')
    slot = state.get('slot')
    if not state.get('sent') or slot is None or status is None:
        return False
    if now - state.get('status_at', 0.0) > FRAME_AGE or status.get('state') != 'done':
        return False
    pickup_route_id = state.get('pickup_route_id')
    if pickup_route_id is not None and status.get('route_id') == pickup_route_id:
        return False
    if status.get('x') is None or status.get('y') is None:
        return False
    x, y = board.xy(drop_row - 1, slot - 1)
    return (math.hypot(status['x'] - x, status['y'] - y) <= WORKFLOW_POSITION_TOL_M
            and (state.get('delivery_yaw') is None or
                 abs(angle_error_deg(status.get('yaw', float('nan')),
                                     state['delivery_yaw'])) < HEADING_TOL_DEG))


def deliveries_complete(cars, board, drop_row, now):
    """Both confirmed deliveries reached their assigned cells."""
    return bool(cars) and all(delivery_arrived(s, board, drop_row, now)
                              for s in cars.values())


def image_to_bgr(msg):
    if msg.encoding not in ('bgr8', 'rgb8'):
        raise ValueError(f'Unsupported camera encoding: {msg.encoding}')
    frame = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    frame = frame[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else frame


def display_frame(state):
    """Live camera in every phase; edge overlays only belong to their own frame.

    Locking a measurement locks numbers, never the live video. In particular,
    don't show the observation snapshot once the car approaches or loads cargo.
    """
    if (state['phase'] == 'measuring' and state['accepted'] is None and
            state.get('view') is not None and
            state.get('last_measured_frame_seq') == state.get('frame_seq')):
        return state['view']
    return state['frame']


def navigation_error(state, message):
    """Keep the actual failure visible instead of the old camera-wait label."""
    match = re.search(r'sai so vi tri.*?(\d+(?:\.\d+)?) cm', message)
    reason = f'Position error {match[1]} cm > 2 cm' if match else message
    if state.get('phase') != 'error':
        state['failed_phase'] = state.get('phase')
    state.update(phase='error', action_note=message, reason=reason)


class LiveView(Node):
    def __init__(self, targets=None, event_log_path=None, sim=False):
        super().__init__('cargo_live_view')
        self.run_id = f'{time.strftime("%Y%m%dT%H%M%S")}-{os.getpid()}-{uuid.uuid4().hex[:8]}'
        self.event_log_path = Path(event_log_path) if event_log_path else None
        if self.event_log_path is not None:
            self.event_log_path.parent.mkdir(parents=True, exist_ok=True)
        self.event_pub = self.create_publisher(String, '/fleet/cargo_event', 10)
        self.board = Board.load(config_path(True))
        self.targets = dict(TARGETS if targets is None else targets)
        if (not self.targets or not set(self.targets).issubset(TARGETS) or
                len(set(self.targets.values())) != len(self.targets)):
            raise ValueError('Each AGV needs a different pickup cell')
        for row, col in self.targets.values():
            if not (1 <= row <= self.board.rows and 1 <= col <= self.board.cols):
                raise ValueError(f'Pickup cell ({row},{col}) is outside the grid')
        self.base_cfg = cm.load_cfg()
        from grid_navigator import parse_heading
        self.pick_heading = self.base_cfg['place']['pickup_heading']
        self.pick_yaw = parse_heading(self.pick_heading, self.board.yaw)[1]
        self.observations = {car: observation_cell(cell, self.pick_heading, self.board)
                             for car, cell in self.targets.items()}
        all_cells = list(self.targets.values()) + list(self.observations.values())
        if len(set(all_cells)) != len(all_cells):
            raise ValueError('Pickup and observation cells must not overlap across cars')
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
            car_config = cm.car_cfg(self.base_cfg, car, sim=sim)
            if sim:
                # fleet_fake_robot publishes scan from the axle centre at 7.5 cm.
                car_config['lidar'] = dict(car_config['lidar'], x=0.0, z=0.075)
            self.cars[car] = {
                'cfg': car_config,
                'scans': deque(maxlen=max(10, self.base_cfg['lidar']['scans'])),
                'frame': None, 'frame_at': 0.0, 'frame_seq': 0,
                'last_measured_frame_seq': 0,
                'status': None, 'status_at': 0.0,
                'last_process': 0.0, 'history': deque(maxlen=SAMPLE_COUNT),
                'sample_started': 0.0,
                'accepted': None, 'candidate': None, 'candidate_at': 0.,
                'reported': False, 'slot': None,
                'near_boundary': False,
                'sent': False, 'pickup_route_id': None,
                'delivered_at': None, 'unloaded_at': None, 'action_note': '',
                'phase': 'ready', 'leg_at': 0.0, 'leg_route_id': None,
                'cycle': 0,
                'view': None, 'reason': 'Waiting for camera',
            }
            self.create_subscription(
                LaserScan, f'/{car}/scan', lambda m, c=car: self.scan_cb(c, m),
                qos_profile_sensor_data)
            self.create_subscription(
                Image, f'/{car}/camera/image_raw', lambda m, c=car: self.image_cb(c, m),
                qos_profile_sensor_data)
        self.create_subscription(String, '/fleet/status', self.status_cb, 10)
        self.create_subscription(Log, '/rosout', self.route_log, 50)

    def route_log(self, msg):
        if msg.level < 30:
            return
        for car, s in self.cars.items():
            terminal = (msg.name == 'traffic_manager' and
                        (msg.level >= 40 or msg.msg.startswith(f'[{car}] dung:')))
            precision = (msg.name == f'{car}.fleet_agent' and msg.level >= 40 and
                         'sai so vi tri tai dich' in msg.msg)
            if (s['phase'] in ('to_stage', 'to_observe', 'to_pickup', 'delivering') and
                    msg.msg.startswith(f'[{car}]') and (terminal or precision)):
                navigation_error(s, msg.msg)

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
        if s['phase'] != 'measuring':
            return
        if s['accepted'] is not None:
            return  # keep measured numbers; canvas still reads the live camera
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
                r, c = self.observations[car]
                reason = gate_result(result, s['status'], now - s['status_at'],
                                     self.board.xy(r - 1, c - 1), self.pick_yaw)
                if reason is None and not self.at_cell(car, (r, c), now):
                    reason = 'Observation position error >2 cm'
                if reason is None:
                    s['candidate'], s['candidate_at'] = result, now
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
                        emit_cargo_event(self, car, 'measured', now, width_m=average['width'],
                                         height_m=average['height'], deliver_col=s['slot'])
                        if s['slot'] is not None:
                            ok, note = self.send_leg(car, self.targets[car], 'to_pickup', now)
                            if not ok:
                                s['action_note'] = note
                    else:
                        s['reason'] = f'Measuring {len(s["history"])}/{SAMPLE_COUNT}'
                else:
                    s['reason'] = reason
            except (ValueError, cv2.error) as e:
                s['reason'] = str(e).split('\n')[0][:66]
        if s['accepted'] is None and pose_gate(
                s['status'], now - s['status_at'],
                self.board.xy(self.observations[car][0] - 1, self.observations[car][1] - 1),
                self.pick_yaw) is not None:
            s['history'].clear()
            s['candidate'] = None

    def at_cell(self, car, cell, now, heading=True, new_route=False):
        s = self.cars[car]
        status = s['status']
        if pose_gate(status, now - s['status_at'], self.board.xy(cell[0]-1, cell[1]-1),
                     self.pick_yaw if heading else (status or {}).get('yaw', 0.0)):
            return False
        x, y = self.board.xy(cell[0]-1, cell[1]-1)
        if math.hypot(status['x']-x, status['y']-y) > WORKFLOW_POSITION_TOL_M:
            return False
        return not new_route or (status.get('state') == 'done' and
                                status.get('route_id') is not None and
                                status['route_id'] != s['leg_route_id'])

    def send_leg(self, car, cell, phase, now):
        s = self.cars[car]
        if self.goal_pub.get_subscription_count() == 0:
            return False, 'Chua thay traffic_manager tren /fleet/goal'
        if not s['status'] or now - s['status_at'] > FRAME_AGE:
            return False, 'Waiting for fresh robot pose'
        if s['status'].get('state') not in ('idle', 'done'):
            return False, 'Robot must be stopped before a new leg'
        s.update(phase=phase, leg_at=now, leg_route_id=s['status'].get('route_id'),
                 action_note='')
        # Only observation/pickup need the camera facing the box. At delivery
        # preserve the final route direction; an unnecessary final pivot can
        # displace the axle by centimetres and spoil an otherwise good stop.
        heading = '' if phase in ('to_stage', 'delivering') else self.pick_heading
        if phase == 'delivering':
            s['delivery_yaw'] = None
        self.goal_pub.publish(String(data=f'{car} {cell[0]} {cell[1]} {heading}'.strip()))
        return True, f'{car}: -> {cell}'

    def advance(self, car, now):
        s = self.cars[car]
        phase = s['phase']
        if phase not in ('to_stage', 'to_observe', 'to_pickup', 'delivering'):
            return
        status = s['status'] or {}
        if status.get('state') in ('estop', 'precision_failed'):
            navigation_error(s, status.get('error') or status.get('estop') or status['state'])
            return
        if now - s['leg_at'] > LEG_TIMEOUT_S:
            navigation_error(s, 'Navigation timeout - check route and robot pose')
            return
        if phase == 'delivering':
            return
        if phase == 'to_stage':
            stage = observation_cell(self.observations[car], self.pick_heading, self.board)
            if self.at_cell(car, stage, now, heading=False, new_route=True):
                emit_cargo_event(self, car, 'recovery_stage_arrived', now,
                                 row=stage[0], col=stage[1])
                ok, note = self.send_leg(car, self.observations[car], 'to_observe', now)
                if ok:
                    s['reason'] = 'Dang vao lai o quan sat tu phia Tay'
                    emit_cargo_event(self, car, 'recovery_reapproach_sent', now,
                                     row=self.observations[car][0],
                                     col=self.observations[car][1])
                else:
                    navigation_error(s, note)
            return
        cell = self.observations[car] if phase == 'to_observe' else self.targets[car]
        if self.at_cell(car, cell, now, new_route=True):
            s.update(phase='measuring' if phase == 'to_observe' else 'loading', action_note='')
            emit_cargo_event(self, car,
                             'observation_arrived' if phase == 'to_observe' else 'pickup_arrived',
                             now, row=cell[0], col=cell[1])
            s['history'].clear()
            if phase == 'to_observe':
                s['scans'].clear()  # never fit cargo using scans taken while driving
            s['last_measured_frame_seq'] = s['frame_seq']

    def start_cycle(self, car, now):
        s = self.cars[car]
        if s['phase'] not in ('ready', 'unloaded'):
            return False, f'{car}: dang thuc hien chu ky'
        ok, note = self.send_leg(car, self.observations[car], 'to_observe', now)
        if not ok:
            return ok, note
        s.update(accepted=None, candidate=None, candidate_at=0., reported=False, slot=None, sent=False,
                 resuming_delivery=False, failed_phase=None,
                 pickup_route_id=None, delivered_at=None, unloaded_at=None, near_boundary=False,
                 sample_started=0.0, cycle=s['cycle']+1, view=s['frame'])
        s['history'].clear()
        emit_cargo_event(self, car, 'started', now,
                         observation_row=self.observations[car][0],
                         observation_col=self.observations[car][1])
        return True, note

    def action(self, car):
        s = self.cars[car]
        now = time.monotonic()
        if s['phase'] == 'delivered':
            s['unloaded_at'] = now
            emit_cargo_event(self, car, 'unloaded_confirmed', now,
                             deliver_row=self.drop_row, deliver_col=s.get('slot'))
            s['phase'] = 'unloaded'
            return True, f'{car}: da xac nhan do hang; bam QUAY LAI khi san sang'
        if s['phase'] in ('ready', 'unloaded'):
            return self.start_cycle(car, now)
        if s['phase'] == 'measuring' and s['accepted'] and s['slot'] is not None:
            return self.send_leg(car, self.targets[car], 'to_pickup', now)
        if s['phase'] == 'delivery_ready':
            s['pickup_route_id'] = (s.get('status') or {}).get('route_id')
            return self.send_leg(car, (self.drop_row, s['slot']), 'delivering', now)
        if s['phase'] == 'error':
            phase = s.get('failed_phase')
            if phase == 'delivering' and s.get('sent') and s.get('slot') is not None:
                s['pickup_route_id'] = (s.get('status') or {}).get('route_id')
                return self.send_leg(car, (self.drop_row,s['slot']), phase, now)
            if phase in ('to_observe', 'to_stage'):
                stage = observation_cell(self.observations[car], self.pick_heading, self.board)
                ok, note = self.send_leg(car, stage, 'to_stage', now)
                if ok:
                    s['reason'] = f'Dang ve o ({stage[0]},{stage[1]}) de vao lai o do'
                    emit_cargo_event(self, car, 'recovery_stage_sent', now,
                                     row=stage[0], col=stage[1])
                return ok, note
            if phase == 'to_pickup':
                return self.send_leg(car, self.targets[car], phase, now)
            return False, 'Khong co chang bi loi de thu lai'
        return self.confirm(car)

    def reset(self):
        reset_cars = []
        for car, s in self.cars.items():
            if s['phase'] != 'measuring':
                continue  # never clear cargo while approaching, loading or delivering
            s['history'].clear()
            s['sample_started'] = 0.0
            s['accepted'] = None
            s['candidate'] = None
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
        if s.get('phase') != 'loading':
            return False, f'{car}: chua dung tai o lay hang de xac nhan'
        if s['sent']:
            return False, f'{car}: da gui dich giao, khong gui lap'
        if s['accepted'] is None:
            return False, f'{car}: chua chot 5 lan do hop le'
        if s['slot'] is None:
            return False, (f'{car}: cao {s["accepted"]["height"] * 100:.1f} cm nam ngoai dai '
                           f'{self.sorting["min_m"] * 100:.0f}-{self.sorting["max_m"] * 100:.0f} cm')
        r, c = self.targets[car]
        pose_reason = pose_gate(s['status'], time.monotonic() - s['status_at'],
                                self.board.xy(r - 1, c - 1), self.pick_yaw)
        if pose_reason:
            return False, f'{car}: {pose_reason}'
        if not self.at_cell(car, (r, c), time.monotonic()):
            return False, f'{car}: can dung cach tam o lay <= 2 cm'
        if self.goal_pub.get_subscription_count() == 0:
            return False, 'Chua thay traffic_manager tren /fleet/goal'
        slot = s['slot']
        if slot != supervised_height_slot(s['accepted']['height'], self.sorting):
            return False, f'{car}: loai hang khong khop o giao; can do lai'
        x, y = self.board.xy(self.drop_row - 1, slot - 1)
        for other, state in self.cars.items():
            if other == car:
                continue
            o = state['status']
            reserved = state.get('sent') and state.get('slot') == slot
            occupied = (o and o.get('x') is not None and o.get('y') is not None and
                        math.hypot(o['x'] - x, o['y'] - y) < 0.3)
            if occupied or (reserved and (state.get('delivered_at') is None or
                                         time.monotonic()-state.get('status_at', 0) > FRAME_AGE)):
                return False, f'O ({self.drop_row},{slot}) dang duoc {other} su dung'
        goal = f'{car} {self.drop_row} {slot}'
        s['pickup_route_id'] = s['status'].get('route_id')
        self.goal_pub.publish(String(data=goal))
        self.result_pub.publish(String(data=json.dumps({
            'car': car, 'ok': True, 'confirmed': True,
            'cycle': s.get('cycle', 1),
            'pickup_row': r, 'pickup_col': c,
            'deliver_row': self.drop_row, 'deliver_col': slot,
            'height_m': s['accepted']['height'], 'width_m': s['accepted']['width'],
            'distance_m': s['accepted']['z_cam'], 'measurements': SAMPLE_COUNT,
            'near_height_boundary': s.get('near_boundary', False),
        })))
        s['sent'] = True
        s['delivery_yaw'] = None
        s['phase'], s['leg_at'] = 'delivering', time.monotonic()
        emit_cargo_event(self, car, 'loaded_confirmed', s['leg_at'],
                         pickup_row=r, pickup_col=c,
                         deliver_row=self.drop_row, deliver_col=slot)
        note = ' (sat nguong, nguoi giam sat da xac nhan)' if s.get('near_boundary') else ''
        return True, (f'{car}: da nhan hang -> giao toi ({self.drop_row},{slot}), '
                      f'cao {s["accepted"]["height"] * 100:.1f} cm{note}')

    def record_delivery(self, car, now):
        """Emit one verifiable arrival event and keep its result on screen."""
        s = self.cars[car]
        if (s.get('phase', 'delivering') != 'delivering' or
                s.get('delivered_at') is not None or
                (s.get('accepted') is None and not s.get('resuming_delivery')) or
                not delivery_arrived(s, self.board, self.drop_row, now)):
            return False
        s['delivered_at'] = now
        s['phase'] = 'delivered'
        status = s['status']
        self.result_pub.publish(String(data=json.dumps({
            'event': 'delivered', 'car': car, 'ok': True, 'confirmed': True,
            'cycle': None if s.get('resuming_delivery') else s.get('cycle', 1),
            'delivered': True, 'deliver_row': self.drop_row, 'deliver_col': s['slot'],
            'height_m': (s.get('accepted') or {}).get('height'),
            'width_m': (s.get('accepted') or {}).get('width'),
            'distance_m': (s.get('accepted') or {}).get('z_cam'),
            'resumed_delivery': s.get('resuming_delivery', False),
            'x': status['x'], 'y': status['y'], 'yaw': status.get('yaw'),
            'route_id': status.get('route_id'),
            't_unix': time.time(),
        })))
        emit_cargo_event(self, car, 'delivery_arrived', now,
                         deliver_row=self.drop_row, deliver_col=s['slot'],
                         resumed_delivery=s.get('resuming_delivery', False))
        return True

    def canvas(self):
        panes = []
        now = time.monotonic()
        for car in self.targets:
            s = self.cars[car]
            frame = display_frame(s)
            if frame is None:
                frame = np.zeros((240, 320, 3), np.uint8)
            else:
                frame = cv2.resize(frame, (320, 240))
            pane = np.zeros((PANE_H, PANE_W, 3), np.uint8)
            pane[48:288, 10:330] = frame
            if s['frame'] is None or now - s['frame_at'] > FRAME_AGE:
                cv2.rectangle(pane, (10, 48), (330, 76), (0, 0, 150), -1)
                age = now - s['frame_at']
                label = f'CAMERA STALE {age:.1f}s' if s['frame'] is not None else 'WAITING FOR CAMERA'
                cv2.putText(pane, label, (16, 68), cv2.FONT_HERSHEY_SIMPLEX,
                            0.48, (255, 255, 255), 1)
            idx = car[-1]
            r, c = self.targets[car]
            title = (f'AGV {idx} DA GIAO HANG THANH CONG ({self.drop_row},{s["slot"]})'
                     if s.get('delivered_at') is not None else
                     f'AGV {idx}  pickup ({r},{c}) {self.pick_heading.upper()}')
            cv2.putText(pane, title, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            result = s['accepted']
            color = (60, 220, 60) if result else (0, 180, 255)
            if s['phase'] == 'error':
                labels = ['LOI DIEU HUONG', s['reason'][:42]]
                color = (0, 180, 255)
            elif (result is None and s['phase']=='measuring' and s.get('candidate') and
                  now-s.get('candidate_at',0.)<=FRAME_AGE):
                candidate=s['candidate']
                labels=[f'TAM {len(s["history"])}/{SAMPLE_COUNT}   DIST {candidate["z_cam"]*100:.1f} cm',
                        f'RONG {candidate["width"]*100:.1f} cm  CAO {candidate["height"]*100:.1f} cm']
            elif s.get('resuming_delivery') and result is None:
                labels = [f'TIEP TUC CHUYEN GIAO ({self.drop_row},{s["slot"]})',
                          'KHONG DO LAI HANG TREN XE']
            elif result:
                labels = [f'FINAL AVG 5   DIST {result["z_cam"] * 100:.1f} cm',
                          f'WIDTH {result["width"] * 100:.1f} cm   HEIGHT {result["height"] * 100:.1f} cm']
            else:
                labels = ['DIST --   WIDTH --   HEIGHT --', s['reason'][:42]]
            for i, label in enumerate(labels):
                cv2.putText(pane, label, (10, 312 + i * 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.51, color, 1)
            if s['phase'] == 'delivered':
                button = 'DA DO HANG (XAC NHAN)'
                button_color = (40, 125, 45)
            elif s['phase'] == 'unloaded':
                button = 'QUAY LAI LAY TIEP'
                button_color = (40, 125, 45)
            elif s['phase'] == 'delivery_ready':
                button = f'TIEP TUC GIAO HANG -> ({self.drop_row},{s["slot"]})'
                button_color = (40, 125, 45)
            elif s['phase'] == 'ready':
                button = f'BAT DAU -> O QUAN SAT {self.observations[car]}'
                button_color = (40, 125, 45)
            elif s['phase'] in ('to_stage', 'to_observe', 'to_pickup', 'error'):
                button = {'to_observe': 'DANG TOI O QUAN SAT',
                          'to_stage': 'DANG VE O DUNG TRUOC DE CAN LAI',
                          'to_pickup': 'DA NHAN DIEN - DANG TOI O LAY',
                          'error': 'THU LAI CHANG DANG LOI'}[s['phase']]
                button_color = (0, 100, 180)
            elif s['sent']:
                button = f'DA GUI DEN ({self.drop_row},{s["slot"]})'
                button_color = (80, 80, 80)
            elif s['action_note']:
                button = s['action_note'][:39]
                button_color = (0, 100, 180)
            elif result and s['slot'] is not None and s['phase'] == 'measuring':
                button = 'DA NHAN DIEN - BAM DE TOI O LAY'
                button_color = (40, 125, 45)
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
    ap.add_argument('--robots', nargs='+', choices=tuple(TARGETS), default=list(TARGETS),
                    help='active robots, e.g. --robots agv2')
    ap.add_argument('--resume-delivery', metavar='CAR,COLUMN',
                    help='resume a previously loaded cargo job, e.g. agv2,3; click to dispatch')
    ap.add_argument('--resume-observation-error', action='store_true',
                    help='one robot only: reopen an interrupted observation failure; '
                         'Retry first drives to the preceding grid cell')
    ap.add_argument('--dispatch-resumed', action='store_true',
                    help='dispatch the explicitly resumed, already confirmed delivery once pose is fresh')
    ap.add_argument('--save-dir', default='/tmp/cargo_live', help='folder for S snapshots')
    ap.add_argument('--event-log', help='JSONL file for pickup/delivery timing events')
    ap.add_argument('--sim', action='store_true',
                    help='use virtual-camera calibration from cargo.yaml')
    ap.add_argument('--agv1-pickup', type=pickup_arg, default=TARGETS['agv1'],
                    metavar='ROW,COL', help='pickup cell for AGV 1')
    ap.add_argument('--agv2-pickup', type=pickup_arg, default=TARGETS['agv2'],
                    metavar='ROW,COL', help='pickup cell for AGV 2')
    ap.add_argument('--success-hold-s', type=float, default=SUCCESS_HOLD_S,
                    help='deprecated; window stays open until Q or close')
    args = ap.parse_args()
    if args.resume_observation_error and (len(args.robots) != 1 or args.resume_delivery):
        ap.error('--resume-observation-error requires exactly one --robots car '
                 'and cannot be combined with --resume-delivery')
    if args.dispatch_resumed and not args.resume_delivery:
        ap.error('--dispatch-resumed requires --resume-delivery')
    if args.success_hold_s < 0:
        ap.error('--success-hold-s must be >= 0')
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('No graphical display (DISPLAY/WAYLAND_DISPLAY). Run in VS Code desktop terminal.')
    rclpy.init()
    pickups = {'agv1': args.agv1_pickup, 'agv2': args.agv2_pickup}
    event_log = args.event_log or str(Path(args.save_dir) / 'cargo_events.jsonl')
    node = LiveView(targets={car: pickups[car] for car in args.robots},
                    event_log_path=event_log, sim=args.sim)
    print(f'Cargo timing events: {event_log}', flush=True)

    if args.resume_observation_error:
        car = args.robots[0]
        previous = unfinished_observation(event_log, car)
        if previous is None:
            node.destroy_node()
            rclpy.shutdown()
            ap.error('Khong tim thay luot quan sat dang loi trong --event-log')
        state = node.cars[car]
        node.run_id = previous['run_id']
        state.update(phase='error', failed_phase='to_observe', cycle=previous['cycle'],
                     reason='Bam THU LAI: ve o dung truoc roi vao lai o quan sat')
        emit_cargo_event(node, car, 'observation_retry_resumed')
        print(f'{car}: THU LAI se ve o dung truoc '
              f'{observation_cell(node.observations[car], node.pick_heading, node.board)}',
              flush=True)

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONUP or not BUTTON_TOP <= y <= BUTTON_BOTTOM:
            return
        index = x // PANE_W
        if not 0 <= index < len(node.targets):
            return
        car = list(node.targets)[index]
        ok, message = node.action(car)
        if not ok:
            node.cars[car]['action_note'] = message.split(':', 1)[-1].strip()
        print(message, flush=True)

    if args.resume_delivery:
        try:
            car, column = args.resume_delivery.split(',')
            column = int(column)
            if car not in node.cars or column not in node.sorting['columns']:
                raise ValueError('xe/loai hang khong hop le')
        except (ValueError, TypeError):
            node.destroy_node()
            rclpy.shutdown()
            ap.error('--resume-delivery requires an active car and valid column: agv2,3')
        node.cars[car].update(phase='delivery_ready', slot=column, sent=True,
                              resuming_delivery=True, delivery_yaw=None,
                              reason='Bam de tiep tuc giao hang da xac nhan')

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            for car in node.targets:
                state = node.cars[car]
                if args.dispatch_resumed and state['phase'] == 'delivery_ready':
                    # This flag is explicit authorization for the previously
                    # confirmed job only. Phase changes prevent repeat sends.
                    ok, message = node.action(car)
                    if ok:
                        print(message, flush=True)
                node.advance(car, now)
                if node.record_delivery(car, now):
                    slot = node.cars[car]['slot']
                    print(f'{car}: DA GIAO HANG THANH CONG tai ({node.drop_row},{slot})', flush=True)
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
                    frame = display_frame(node.cars[car])
                    if frame is not None:
                        cv2.imwrite(str(out / f'{car}_{stamp}.png'), frame)
                cv2.imwrite(str(out / f'comparison_{stamp}.png'), node.canvas())
                print(f'Da luu anh vao {out}', flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
