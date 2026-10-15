#!/usr/bin/env python3
"""Remove transient single rays while preserving persistent narrow objects.

The first scan and scans while moving pass through. Only isolated returns that
have not appeared at a similar angle and range in recent scans are removed.
"""
import math
import time
from collections import deque


def filter_ranges(ranges, range_min, range_max, angle_increment,
                  max_jump=0.15, relative_jump=0.02, history=()):
    values = list(ranges)
    count = len(values)
    if count < 3:
        return values, 0
    valid = [math.isfinite(r) and range_min <= r <= range_max for r in values]
    circular = abs(count * abs(angle_increment) - 2 * math.pi) < abs(angle_increment) * 1.5
    result = values.copy()
    removed = 0
    for i, r in enumerate(values):
        if not valid[i]:
            continue
        supported = False
        for j in (i - 1, i + 1):
            if not 0 <= j < count:
                if not circular:
                    continue
                j %= count
            if valid[j] and abs(r - values[j]) <= max_jump + relative_jump * min(r, values[j]):
                supported = True
                break
        if not supported:
            for old in history:
                if len(old) != count:
                    continue
                for offset in (-1, 0, 1):
                    j = i + offset
                    if not 0 <= j < count:
                        if not circular:
                            continue
                        j %= count
                    previous = old[j]
                    if (math.isfinite(previous) and range_min <= previous <= range_max
                            and abs(r - previous) <= max_jump + relative_jump * min(r, previous)):
                        supported = True
                        break
                if supported:
                    break
        if not supported:
            result[i] = float('nan')
            removed += 1
    return result, removed


class ScanHistoryFilter:
    def __init__(self, history_size=5, max_gap=0.5, max_jump=0.15,
                 relative_jump=0.02):
        self.history = deque(maxlen=history_size)
        self.max_gap = max_gap
        self.max_jump = max_jump
        self.relative_jump = relative_jump
        self.last_time = None
        self.geometry = None

    def process(self, ranges, range_min, range_max, angle_min, angle_increment,
                moving=False, now=None):
        now = time.monotonic() if now is None else now
        geometry = (len(ranges), angle_min, angle_increment)
        if (self.last_time is None or now - self.last_time > self.max_gap
                or geometry != self.geometry or moving):
            self.history.clear()
        self.last_time = now
        self.geometry = geometry
        source = list(ranges)
        if moving or not self.history:
            result, removed = source, 0
        else:
            result, removed = filter_ranges(source, range_min, range_max,
                                            angle_increment, self.max_jump,
                                            self.relative_jump, self.history)
        # Store raw data. Otherwise a legitimate narrow object disappears
        # permanently after its first filtered scan.
        self.history.append(source)
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
            self.declare_parameter('max_jump', 0.15)
            self.declare_parameter('relative_jump', 0.02)
            self.declare_parameter('history_size', 5)
            self.declare_parameter('max_gap', 0.5)
            self.jump = float(self.get_parameter('max_jump').value)
            self.relative = float(self.get_parameter('relative_jump').value)
            if not math.isfinite(self.jump) or not math.isfinite(self.relative) or min(self.jump, self.relative) < 0:
                raise ValueError('Filter thresholds must be finite and nonnegative')
            self.filter = ScanHistoryFilter(
                history_size=int(self.get_parameter('history_size').value),
                max_gap=float(self.get_parameter('max_gap').value),
                max_jump=self.jump, relative_jump=self.relative)
            self.moving_until = 0.0
            self.pub = self.create_publisher(LaserScan, 'scan_filtered', qos_profile_sensor_data)
            self.create_subscription(LaserScan, 'scan', self.callback, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
            self.scans = self.removed = 0
            self.create_timer(10.0, self.report)

        def callback(self, msg):
            removed = 0
            if self.get_parameter('enabled').value:
                ranges, removed = self.filter.process(
                    msg.ranges, msg.range_min, msg.range_max, msg.angle_min,
                    msg.angle_increment, moving=time.monotonic() < self.moving_until)
                msg.ranges = ranges
            self.pub.publish(msg)
            self.scans += 1
            self.removed += removed

        def odom_callback(self, msg):
            twist = msg.twist.twist
            if abs(twist.linear.x) > 0.02 or abs(twist.angular.z) > 0.05:
                # Includes deceleration and small packet gaps. During motion a
                # one-ray object must remain visible to collision protection.
                self.moving_until = time.monotonic() + 0.6

        def report(self):
            self.get_logger().info(f'{self.scans} scans; removed {self.removed} isolated rays / 10 s')
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
