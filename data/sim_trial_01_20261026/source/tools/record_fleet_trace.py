#!/usr/bin/env python3
"""Record passive two-car fleet status and route commands for report figures.

Example: ROS_DOMAIN_ID=87 python3 tools/record_fleet_trace.py RUN/fleet_trace.jsonl
Stop with Ctrl+C. This node never publishes a goal or velocity command.
"""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Trace(Node):
    def __init__(self, path):
        super().__init__('fleet_trace_recorder')
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open('w', encoding='utf-8', buffering=1)
        self.last_sample = {}
        self.create_subscription(String, '/fleet/status', self.status_cb, 50)
        for car in ('agv1', 'agv2'):
            self.create_subscription(
                String, f'/fleet/{car}/cmd',
                lambda msg, name=car: self.command_cb(name, msg), 20)

    def write(self, kind, data):
        self.file.write(json.dumps({
            'kind': kind, 't_wall': time.time(), 't_monotonic': time.monotonic(),
            **data,
        }, ensure_ascii=False) + '\n')

    def status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        name = data.get('name')
        if name not in ('agv1', 'agv2'):
            return
        now = time.monotonic()
        if now - self.last_sample.get(name, 0) >= 0.2:
            self.last_sample[name] = now
            self.write('status', data)

    def command_cb(self, name, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self.write('command', {'name': name, 'command': data})

    def close(self):
        self.file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='JSONL output path')
    args = parser.parse_args()
    rclpy.init()
    node = Trace(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
