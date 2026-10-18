#!/usr/bin/env python3
"""Drive AGV1 straight about 2 m by wheel encoder, then command a stop."""
import json
import math
import time
from pathlib import Path

import rclpy

from pid_field_test import Capture, WHEEL_RADIUS_ODOM_M, save_csv

TARGET_M = 2.0
TARGET_RPM = 40.0
SPEED_MPS = TARGET_RPM * 2 * math.pi * 0.03225 / 60.0
MAX_RUN_S = 25.0
OUT = Path(__file__).resolve().parents[1] / 'docs/figures/pid_real_agv1'


def wheel_distance(start, row):
    return ((row['left_pos_rad'] - start['left_pos_rad']) +
            (row['right_pos_rad'] - start['right_pos_rad'])) * WHEEL_RADIUS_ODOM_M / 2.0


def main():
    rclpy.init()
    node = Capture()
    result = 'completed'
    start_row = None
    commanded_at = None
    stopped_at = None
    reached = 0.0
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (time.monotonic() - node.js_at < 0.2 and
                    time.monotonic() - node.diag_at < 2.0 and
                    node.pub.get_subscription_count() >= 1):
                break
        else:
            raise RuntimeError('Khong co encoder/diag moi hoac ESP32 chua nhan cmd_vel')
        node.guard()
        if len(node.get_publishers_info_by_topic('/agv1/cmd_vel')) > 1:
            raise RuntimeError('Co nguon lenh cmd_vel khac dang chay')
        if abs(node.rows[-1]['left_rpm']) > 1 or abs(node.rows[-1]['right_rpm']) > 1:
            raise RuntimeError('Xe chua dung yen')
        start_row = node.rows[-1].copy()
        node.phase = 'running'
        commanded_at = time.monotonic()
        next_pub = commanded_at
        print(f'AGV1 chay thang {TARGET_RPM:.0f} RPM, dung khi encoder dat {TARGET_M:.2f} m', flush=True)
        while True:
            now = time.monotonic()
            if now >= next_pub:
                node.command(SPEED_MPS)
                next_pub = now + 0.05
            rclpy.spin_once(node, timeout_sec=0.015)
            node.guard()
            reached = wheel_distance(start_row, node.rows[-1])
            if reached >= TARGET_M:
                break
            if reached < -0.03:
                raise RuntimeError('Encoder bao xe di nguoc')
            if now - commanded_at > MAX_RUN_S:
                raise RuntimeError('Qua gioi han thoi gian 25 s')
    except (KeyboardInterrupt, RuntimeError) as exc:
        result = 'stopped: ' + str(exc)
        print(result, flush=True)
    finally:
        stopped_at = time.monotonic()
        node.phase = 'after_stop'
        for _ in range(12):
            node.command(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        until = time.monotonic() + 2.0
        while time.monotonic() < until:
            node.command(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        final_distance = wheel_distance(start_row, node.rows[-1]) if start_row and node.rows else 0.0
        stopped = bool(node.rows and
                       abs(node.rows[-1]['left_rpm']) < 1 and
                       abs(node.rows[-1]['right_rpm']) < 1)
        OUT.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        base = OUT / f'agv1_drive_2m_{stamp}'
        save_csv(base.with_name(base.name + '_joint.csv'), node.rows)
        save_csv(base.with_name(base.name + '_diag.csv'), node.diag_rows)
        summary = {'status': result, 'target_distance_m': TARGET_M,
                   'encoder_distance_m': final_distance, 'speed_setpoint_rpm': TARGET_RPM,
                   'command_duration_s': stopped_at - commanded_at if commanded_at else 0.0,
                   'stopped_below_1rpm': stopped, 'estop': (node.diag or {}).get('estop', '')}
        base.with_name(base.name + '_summary.json').write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
        print(f'Du lieu: {base}*', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
