#!/usr/bin/env python3
"""Reject transient LiDAR specks at rest and in motion.

Near narrow returns need two previous confirmations; broad near returns need
one. Beyond the near zone, isolated single rays are rejected even if they
repeat; two-ray clusters need temporal confirmation, while larger clusters
pass at once. Previous scans are compared in odom coordinates when available,
so robot motion does not disable filtering. On the first scan after startup
or a gap, near returns pass through but far single rays are still rejected.
"""
import math
import time
from collections import deque

import numpy as np
from scipy.spatial import cKDTree


def filter_ranges(ranges, range_min, range_max, angle_increment,
                  max_jump=0.07, relative_jump=0.015):
    """Spatial-only helper: discard rays without a compatible neighbour."""
    values = list(ranges)
    count = len(values)
    if count < 3:
        return values, 0
    arr = np.asarray(values, dtype=float)
    valid = np.isfinite(arr) & (arr >= range_min) & (arr <= range_max)
    circular = abs(count * abs(angle_increment) - 2 * math.pi) < abs(angle_increment) * 1.5
    sizes = cluster_sizes(arr, valid, max_jump, relative_jump, circular)
    result = values.copy()
    isolated = np.flatnonzero(valid & (sizes == 1))
    for i in isolated:
        result[i] = float('nan')
    return result, len(isolated)


def closest_pose(samples, stamp, max_offset=0.08):
    """Use odometry acquired at the scan time, not at Wi-Fi arrival time."""
    if not samples or stamp <= 0:
        return None
    sample = min(samples, key=lambda s: abs(s[0] - stamp))
    return sample[1:] if abs(sample[0] - stamp) <= max_offset else None


def cluster_sizes(values, valid, max_jump, relative_jump, circular):
    """Count contiguous rays at compatible ranges, including the 360° seam."""
    sizes = np.zeros(len(values), dtype=np.int32)
    runs = []
    start = None

    def connected(i, j):
        return (valid[i] and valid[j] and
                abs(values[i] - values[j]) <=
                max_jump + relative_jump * min(values[i], values[j]))

    for i in range(len(values)):
        if not valid[i]:
            if start is not None:
                runs.append((start, i))
                start = None
        elif start is None:
            start = i
        elif not connected(i - 1, i):
            runs.append((start, i))
            start = i
    if start is not None:
        runs.append((start, len(values)))
    for start, end in runs:
        sizes[start:end] = end - start
    if (circular and len(runs) > 1 and runs[0][0] == 0 and
            runs[-1][1] == len(values) and connected(len(values) - 1, 0)):
        first, last = runs[0], runs[-1]
        sizes[first[0]:first[1]] = sizes[last[0]:last[1]] = (
            first[1] - first[0] + last[1] - last[0])
    return sizes


class ScanHistoryFilter:
    def __init__(self, history_size=5, max_gap=0.5, max_jump=0.07,
                 relative_jump=0.015, near_cutoff=0.35, near_time=0.38,
                 near_cluster_rays=6, far_cluster_rays=3,
                 world_tolerance=0.06, lidar_x=0.014):
        self.history = deque(maxlen=history_size)
        self.max_gap = max_gap
        self.max_jump = max_jump
        self.relative_jump = relative_jump
        self.near_cutoff = near_cutoff
        self.near_time = near_time
        self.near_cluster_rays = near_cluster_rays
        self.far_cluster_rays = far_cluster_rays
        self.world_tolerance = world_tolerance
        self.lidar_x = lidar_x
        self.last_time = None
        self.geometry = None

    def world_points(self, values, valid, angle_min, angle_increment, pose):
        if pose is None:
            return None
        x, y, yaw = pose
        a = angle_min + np.arange(len(values)) * angle_increment + yaw
        rr = np.asarray(values)
        px = x + self.lidar_x * math.cos(yaw) + rr[valid] * np.cos(a[valid])
        py = y + self.lidar_x * math.sin(yaw) + rr[valid] * np.sin(a[valid])
        out = np.full((len(values), 2), np.nan)
        out[valid] = np.column_stack((px, py))
        return out

    def process(self, ranges, range_min, range_max, angle_min, angle_increment,
                moving=False, now=None, pose=None):
        now = time.monotonic() if now is None else now
        geometry = (len(ranges), angle_min, angle_increment)
        circular = abs(len(ranges) * abs(angle_increment) - 2 * math.pi) < abs(angle_increment) * 1.5
        if (self.last_time is None or now < self.last_time
                or now - self.last_time > self.max_gap
                or geometry != self.geometry):
            self.history.clear()
        self.last_time = now
        self.geometry = geometry
        source = list(ranges)
        arr = np.asarray(source, dtype=float)
        valid = np.isfinite(arr) & (arr >= range_min) & (arr <= range_max)
        sizes = cluster_sizes(arr, valid, self.max_jump, self.relative_jump, circular)
        points = self.world_points(arr, valid, angle_min, angle_increment, pose)
        far_single = valid & (arr >= self.near_cutoff) & (sizes == 1)
        if not self.history:
            result = source.copy()
            # No temporal history yet, but a single ray is not a large
            # obstacle even on the first scan. With no odom during motion,
            # preserve the whole raw scan as a safety fallback.
            if not moving or points is not None:
                for i in np.flatnonzero(far_single):
                    result[i] = float('nan')
            removed = int(np.count_nonzero(valid & np.isnan(result)))
        else:
            result = source.copy()
            near = valid & (arr < self.near_cutoff)
            far_pair = valid & ~near & (sizes >= 2) & (sizes < self.far_cluster_rays)
            candidates = near | far_pair
            confirmations = np.zeros(len(source), dtype=np.uint8)
            needed = np.ones(len(source), dtype=np.uint8)
            needed[near & (sizes < self.near_cluster_rays)] = 2
            # Without time-aligned odom, scan-only matching is unsafe during
            # motion: pass the whole scan through until pose is available.
            if moving and points is None:
                confirmations[candidates] = needed[candidates]
                far_single[:] = False
            for old in reversed(self.history):
                age = now - old['time']
                if age > self.max_gap:
                    break
                check = candidates & (confirmations < needed)
                if age > self.near_time:
                    check &= ~near
                idx = np.flatnonzero(check)
                if not len(idx):
                    continue
                if points is not None and old['points'] is not None:
                    for require_cluster in (False, True):
                        matching = idx[far_pair[idx] == require_cluster]
                        if not len(matching):
                            continue
                        eligible = old['valid'] & ((old['sizes'] >= 2) if require_cluster else True)
                        previous = old['points'][eligible]
                        if not len(previous):
                            continue
                        distances, _ = cKDTree(previous).query(points[matching])
                        confirmations[matching] += (
                            distances <= self.world_tolerance + 0.02 * arr[matching])
                elif not moving:
                    # Stationary fallback before odom is ready.
                    old_values = old['ranges']
                    for i in idx:
                        for j in (i - 1, i, i + 1):
                            if not 0 <= j < len(source):
                                if not circular:
                                    continue
                                j %= len(source)
                            previous = old_values[j]
                            if (not far_pair[i] or old['sizes'][j] >= 2) and (
                                    math.isfinite(previous) and range_min <= previous <= range_max
                                    and abs(arr[i] - previous) <= self.max_jump
                                    + self.relative_jump * min(arr[i], previous)):
                                confirmations[i] += 1
                                break
            supported = confirmations >= needed
            for i in np.flatnonzero(far_single | (candidates & ~supported)):
                result[i] = float('nan')
            removed = int(np.count_nonzero(valid & np.isnan(result)))
        # Keep raw history so a real narrow cluster can be confirmed later.
        self.history.append({'ranges': source, 'valid': valid,
                             'sizes': sizes, 'points': points, 'time': now})
        return result, removed


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry

    class ScanFilter(Node):
        def __init__(self):
            super().__init__('scan_filter')
            self.declare_parameter('enabled', True)
            self.declare_parameter('max_jump', 0.07)
            self.declare_parameter('relative_jump', 0.015)
            self.declare_parameter('history_size', 5)
            self.declare_parameter('max_gap', 0.5)
            self.declare_parameter('near_cutoff', 0.35)
            self.declare_parameter('near_time', 0.38)
            self.declare_parameter('near_cluster_rays', 6)
            self.declare_parameter('far_cluster_rays', 3)
            self.declare_parameter('world_tolerance', 0.06)
            self.jump = float(self.get_parameter('max_jump').value)
            self.relative = float(self.get_parameter('relative_jump').value)
            if not math.isfinite(self.jump) or not math.isfinite(self.relative) or min(self.jump, self.relative) < 0:
                raise ValueError('Filter thresholds must be finite and nonnegative')
            self.filter = ScanHistoryFilter(
                history_size=int(self.get_parameter('history_size').value),
                max_gap=float(self.get_parameter('max_gap').value),
                max_jump=self.jump, relative_jump=self.relative,
                near_cutoff=float(self.get_parameter('near_cutoff').value),
                near_time=float(self.get_parameter('near_time').value),
                near_cluster_rays=int(self.get_parameter('near_cluster_rays').value),
                far_cluster_rays=int(self.get_parameter('far_cluster_rays').value),
                world_tolerance=float(self.get_parameter('world_tolerance').value))
            self.moving_until = 0.0
            self.odom_history = deque(maxlen=100)
            self.pub = self.create_publisher(LaserScan, 'scan_filtered', qos_profile_sensor_data)
            self.create_subscription(LaserScan, 'scan', self.callback, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
            self.scans = self.removed = 0
            self.create_timer(10.0, self.report)

        def callback(self, msg):
            removed = 0
            if self.get_parameter('enabled').value:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                ranges, removed = self.filter.process(
                    msg.ranges, msg.range_min, msg.range_max, msg.angle_min,
                    msg.angle_increment, moving=time.monotonic() < self.moving_until,
                    now=stamp if stamp > 0 else None,
                    pose=closest_pose(self.odom_history, stamp))
                msg.ranges = ranges
            self.pub.publish(msg)
            self.scans += 1
            self.removed += removed

        def odom_callback(self, msg):
            twist = msg.twist.twist
            p = msg.pose.pose
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.odom_history.append((stamp, p.position.x, p.position.y, yaw))
            if abs(twist.linear.x) > 0.02 or abs(twist.angular.z) > 0.05:
                # Includes deceleration and small packet gaps. Motion only
                # controls the odom-unavailable fallback, not filtering.
                self.moving_until = time.monotonic() + 0.6

        def report(self):
            self.get_logger().info(f'{self.scans} scans; removed {self.removed} transient rays / 10 s')
            self.scans = self.removed = 0

    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
