#!/usr/bin/env python3
"""Measure AGV1 wheel-speed closed-loop response during a short straight run.

Run with the real robot on a clear floor, with only micro_ros_agent active:
    source tools/setup_real_env.bash
    ROS_DOMAIN_ID=0 python3 tools/pid_field_test.py

The script records /agv1/joint_states (~50 Hz) and /agv1/diag (~1 Hz),
publishes 0.12 m/s for 5 s, and publishes zero in a finally block.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
WHEEL_RADIUS_FIRMWARE_M = 0.03225
WHEEL_RADIUS_ODOM_M = 0.03281


class Capture(Node):
    def __init__(self):
        super().__init__('agv1_pid_field_test')
        self.t0 = time.monotonic()
        self.js_at = 0.0
        self.diag_at = 0.0
        self.diag = None
        self.rows = []
        self.diag_rows = []
        self.phase = 'baseline'
        self.cmd_v = 0.0
        self.pub = self.create_publisher(Twist, '/agv1/cmd_vel', 10)
        self.create_subscription(JointState, '/agv1/joint_states', self.on_js,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/agv1/diag', self.on_diag,
                                 qos_profile_sensor_data)

    def on_js(self, msg):
        if len(msg.velocity) < 2 or len(msg.position) < 2:
            return
        now = time.monotonic()
        self.js_at = now
        k = 60.0 / (2.0 * math.pi)
        self.rows.append({
            't_s': now - self.t0, 'phase': self.phase,
            'command_mps': self.cmd_v,
            'target_rpm': self.cmd_v * k / WHEEL_RADIUS_FIRMWARE_M,
            'left_rpm': msg.velocity[0] * k,
            'right_rpm': msg.velocity[1] * k,
            'left_pos_rad': msg.position[0],
            'right_pos_rad': msg.position[1],
        })

    def on_diag(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('xe') != 1:
            return
        now = time.monotonic()
        self.diag_at = now
        self.diag = data
        self.diag_rows.append({
            't_s': now - self.t0, 'phase': self.phase,
            'left_pwm': (data.get('pwm') or [None, None])[0],
            'right_pwm': (data.get('pwm') or [None, None])[1],
            'left_target_rpm': (data.get('tgt') or [None, None])[0],
            'right_target_rpm': (data.get('tgt') or [None, None])[1],
            'estop': data.get('estop', ''),
        })

    def command(self, speed):
        self.cmd_v = speed
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)

    def guard(self):
        now = time.monotonic()
        if now - self.js_at > 0.35:
            raise RuntimeError('Mat du lieu encoder /agv1/joint_states > 0,35 s')
        if now - self.diag_at > 2.5:
            raise RuntimeError('Mat du lieu chan doan /agv1/diag > 2,5 s')
        if self.diag.get('estop'):
            raise RuntimeError('ESP32 ESTOP: ' + str(self.diag['estop']))
        if self.phase == 'running' and self.rows:
            last = self.rows[-1]
            if min(last['left_rpm'], last['right_rpm']) < -3.0:
                raise RuntimeError('Encoder bao banh quay nguoc')
            if max(last['left_rpm'], last['right_rpm']) > max(52.0, last['target_rpm'] * 1.3):
                raise RuntimeError('RPM vuot nguong an toan; dung phep thu')


def phase(node, label, speed, duration, distance_limit=None):
    node.phase = label
    start = time.monotonic()
    next_pub = start
    first_pos = None
    while time.monotonic() - start < duration:
        now = time.monotonic()
        if now >= next_pub:
            node.command(speed)
            next_pub = now + 0.05
        rclpy.spin_once(node, timeout_sec=0.015)
        node.guard()
        if distance_limit is not None and node.rows:
            row = node.rows[-1]
            pos = (row['left_pos_rad'] + row['right_pos_rad']) / 2.0
            if first_pos is None:
                first_pos = pos
            if abs(pos - first_pos) * WHEEL_RADIUS_ODOM_M > distance_limit:
                raise RuntimeError('Da dat gioi han quang duong encoder')


def save_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def summarize(node, speed, duration, start, stop, result):
    steady = [r for r in node.rows
              if r['phase'] == 'running' and stop - 2.0 <= r['t_s'] < stop]
    target = speed * 60.0 / (2.0 * math.pi * WHEEL_RADIUS_FIRMWARE_M)
    metrics = {'status': result, 'command_mps': speed,
               'command_duration_s': duration, 'target_rpm': target,
               'joint_samples': len(node.rows), 'diag_samples': len(node.diag_rows),
               'step_start_s': start, 'step_end_s': stop,
               'actual_motion_s': max(0.0, stop - start) if start is not None else 0.0}
    if steady:
        for side in ('left', 'right'):
            measured = np.array([r[f'{side}_rpm'] for r in steady])
            metrics[f'{side}_mean_rpm'] = float(measured.mean())
            metrics[f'{side}_mae_rpm'] = float(np.abs(measured - target).mean())
            metrics[f'{side}_mean_error_pct'] = float(100 * (measured.mean() - target) / target)
        robot = np.array([(r['left_rpm'] + r['right_rpm']) / 2.0 for r in steady])
        metrics['robot_mean_rpm'] = float(robot.mean())
        metrics['robot_mae_rpm'] = float(np.abs(robot - target).mean())
        metrics['robot_mean_error_pct'] = float(100 * (robot.mean() - target) / target)
    # Xac lap: tung banh nam trong ±5% muc dat va duy tri den khi ngung lenh.
    # Chi bao cao khi co it nhat 0,5 s du lieu lien tiep trong dai.
    running = [r for r in node.rows if r['phase'] == 'running' and start <= r['t_s'] < stop]
    if running and target > 0:
        for name, get_rpm in (
            ('left', lambda r: r['left_rpm']),
            ('right', lambda r: r['right_rpm']),
            ('robot', lambda r: (r['left_rpm'] + r['right_rpm']) / 2.0),
        ):
            last_outside = max((i for i, r in enumerate(running)
                                if abs(get_rpm(r) - target) > 0.05 * target), default=-1)
            settled_at = last_outside + 1
            if settled_at < len(running) and stop - running[settled_at]['t_s'] >= 0.5:
                metrics[f'{name}_settling_5pct_s'] = float(
                    running[settled_at]['t_s'] - start)
            else:
                metrics[f'{name}_settling_5pct_s'] = None
    if node.rows:
        initial = node.rows[0]
        final = node.rows[-1]
        metrics['wheel_odom_distance_m'] = float(
            ((final['left_pos_rad'] - initial['left_pos_rad']) +
             (final['right_pos_rad'] - initial['right_pos_rad'])) *
            WHEEL_RADIUS_ODOM_M / 2.0)
    return metrics


def plot(path, rows, diag_rows, summary):
    t = np.array([r['t_s'] for r in rows])
    fig, (ax, pwm_ax) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                     gridspec_kw={'height_ratios': [3, 1.35]})
    ax.plot(t, [r['target_rpm'] for r in rows], color='black', linewidth=1.8,
            linestyle='--', label='RPM đặt (từ lệnh vận tốc)')
    ax.plot(t, [r['left_rpm'] for r in rows], color='#1765bd', linewidth=1.3,
            label='Bánh trái — encoder')
    ax.plot(t, [r['right_rpm'] for r in rows], color='#dc6830', linewidth=1.3,
            label='Bánh phải — encoder')
    ax.plot(t, [(r['left_rpm'] + r['right_rpm']) / 2.0 for r in rows],
            color='#16814a', linewidth=2.0,
            label='Xe — trung bình hai bánh')
    ax.set_ylabel('Tốc độ bánh (vòng/phút)')
    if abs(summary['target_rpm'] - 40.0) < 0.05:
        ax.axhspan(38.0, 42.0, color='#16814a', alpha=0.09,
                   label='Dải xác lập ±5%')
    ax.legend(loc='upper right', ncol=2, fontsize=9)
    ax.grid(alpha=0.25)
    condition = summary.get('condition', 'trên sàn')
    ax.set_title(f'AGV1 — đáp ứng điều khiển tốc độ hai bánh {condition}')

    if diag_rows:
        dt = [r['t_s'] for r in diag_rows]
        pwm_ax.scatter(dt, [r['left_pwm'] for r in diag_rows], marker='o',
                       color='#1765bd', label='PWM trái (mẫu 1 Hz)')
        pwm_ax.scatter(dt, [r['right_pwm'] for r in diag_rows], marker='x',
                       color='#dc6830', label='PWM phải (mẫu 1 Hz)')
    pwm_ax.set_ylabel('PWM (0–255)')
    pwm_ax.set_xlabel('Thời gian từ khi bắt đầu ghi (s)')
    pwm_ax.legend(loc='upper right', fontsize=9)
    pwm_ax.grid(alpha=0.25)
    for axis in (ax, pwm_ax):
        axis.axvline(summary['step_start_s'], color='#555', alpha=0.65,
                     linestyle=':')
        axis.axvline(summary['step_end_s'], color='#555', alpha=0.65,
                     linestyle=':')
    info = (f"Lệnh {summary['command_mps']:.3f} m/s trong "
            f"{summary['command_duration_s']:.1f} s; "
            f"RPM đặt {summary['target_rpm']:.1f}; "
            f"quãng đường encoder {summary.get('wheel_odom_distance_m', float('nan')):.2f} m. "
            "Firmware có giới hạn tăng tốc nội bộ.")
    fig.text(0.5, 0.008, info, ha='center', fontsize=9)
    fig.tight_layout(rect=(0, 0.025, 1, 1))
    fig.savefig(path.with_suffix('.png'), dpi=220)
    fig.savefig(path.with_suffix('.pdf'))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.12)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--out', type=Path,
                        default=ROOT / 'docs/figures/pid_real_agv1')
    parser.add_argument('--condition', choices=('floor', 'stand'), default='floor',
                        help='Dieu kien thu de gan nhan dung tren do thi')
    args = parser.parse_args()
    if not 0.05 <= args.speed <= 0.15 or not 1.0 <= args.duration <= 5.0:
        parser.error('Cho phep 0,05–0,15 m/s va 1–5 s')
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Capture()
    result = 'completed'
    start = stop = None
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (time.monotonic() - node.js_at < 0.2 and
                    time.monotonic() - node.diag_at < 2.0 and
                    node.pub.get_subscription_count() >= 1):
                break
        else:
            raise RuntimeError('Chua nhan duoc encoder + diag va subscriber cmd_vel')
        node.guard()
        publishers = node.get_publishers_info_by_topic('/agv1/cmd_vel')
        if len(publishers) > 1:
            raise RuntimeError('Co lenh cmd_vel khac dang chay; khong thu song song')
        print('AGV1 san sang: encoder va diag moi, khong ESTOP. Bat dau trong 1 s...')
        phase(node, 'baseline', 0.0, 1.0)
        start = time.monotonic() - node.t0
        print(f'Chay thang {args.speed:.2f} m/s trong {args.duration:.1f} s')
        phase(node, 'running', args.speed, args.duration,
              distance_limit=args.speed * args.duration + 0.12)
    except (KeyboardInterrupt, RuntimeError) as exc:
        result = 'stopped: ' + str(exc)
        print(result)
    finally:
        stop = time.monotonic() - node.t0
        for _ in range(12):
            node.command(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            if node.js_at and node.diag_at:
                phase(node, 'after_stop', 0.0, 1.8)
        except RuntimeError as exc:
            result = 'stopped: ' + str(exc)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = args.out / f'agv1_pid_{stamp}'
        save_csv(base.with_name(base.name + '_joint.csv'), node.rows)
        save_csv(base.with_name(base.name + '_diag.csv'), node.diag_rows)
        summary = summarize(node, args.speed, args.duration, start, stop, result)
        summary['condition'] = 'trên giá kê' if args.condition == 'stand' else 'trên sàn'
        base.with_name(base.name + '_summary.json').write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
        if len(node.rows) >= 10 and start is not None:
            plot(base, node.rows, node.diag_rows, summary)
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        print(f'Du lieu va do thi: {base.parent}/{base.name}*')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
