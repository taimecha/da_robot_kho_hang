#!/usr/bin/env python3
"""Record reproducible two-AGV fleet scenarios on ROS_DOMAIN_ID=87.

Run after ``tools/fleet_sim.sh up`` and ``traffic_manager.py agv1 agv2``.
The JSON records actual /fleet/status and /fleet/<name>/cmd messages; it does
not calculate or invent a robot trajectory. Gazebo is optional.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grid_board import Board, config_path  # noqa: E402


class ResearchRun(Node):
    def __init__(self):
        super().__init__('fleet_research_recorder')
        self.board = Board.load(config_path(True))
        self.data = {'metadata': {
            'generated_at': time.strftime('%Y-%m-%d %H:%M:%S %z'),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'grid_spacing_m': self.board.spacing,
            'position_source': '/fleet/status (fake robot odometry, map=odom)',
            'scenarios': []}, 'samples': [], 'commands': [], 'events': []}
        self.latest = {}
        self.last_sample = {}
        self.last_cmd = {}
        self.scene = 'setup'
        self.goal_pub = self.create_publisher(String, '/fleet/goal', 10)
        self.obst_pub = self.create_publisher(String, '/sim/obstacles', 10)
        self.create_subscription(String, '/fleet/status', self.status_cb, 50)
        for n in ('agv1', 'agv2'):
            self.create_subscription(String, f'/fleet/{n}/cmd',
                                     lambda m, name=n: self.cmd_cb(name, m), 20)

    def status_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        name = d.get('name')
        if name not in ('agv1', 'agv2'):
            return
        self.latest[name] = d
        now = time.time()
        if now - self.last_sample.get(name, 0) >= 0.2:
            self.data['samples'].append({'t': now, 'scene': self.scene, **d})
            self.last_sample[name] = now

    def cmd_cb(self, name, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        sig = (d.get('route_id'), d.get('upto'), tuple(map(tuple, d.get('nodes', []))))
        if sig != self.last_cmd.get(name):
            self.data['commands'].append({'t': time.time(), 'scene': self.scene,
                                          'name': name, **d})
            self.last_cmd[name] = sig

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.1, end - time.monotonic()))

    def event(self, label, **details):
        row = {'t': time.time(), 'scene': self.scene, 'event': label, **details}
        self.data['events'].append(row)
        print(f"[{self.scene}] {label}: {details}", flush=True)

    def goal(self, text):
        self.event('goal', text=text)
        self.goal_pub.publish(String(data=text))

    def obstacle(self, nodes):
        val = ';'.join(f'{x:.3f},{y:.3f}' for r, c in nodes
                       for x, y in [self.board.xy(r - 1, c - 1)])
        self.event('obstacle', nodes=nodes, data=val)
        self.obst_pub.publish(String(data=val))

    def at(self, name, row, col, radius=0.07):
        d = self.latest.get(name, {})
        if d.get('x') is None or d.get('y') is None:
            return False
        x, y = self.board.xy(row - 1, col - 1)
        return math.hypot(d['x'] - x, d['y'] - y) <= radius

    def wait_at(self, goals, timeout=180, stable=2.0):
        start = time.monotonic()
        good_since = None
        while time.monotonic() - start < timeout:
            self.spin_for(0.2)
            good = all(self.at(name, *rc) for name, rc in goals.items())
            if good and good_since is None:
                good_since = time.monotonic()
            if not good:
                good_since = None
            if good_since and time.monotonic() - good_since >= stable:
                self.event('arrived', goals=goals,
                           states={n: self.latest[n].get('state') for n in goals})
                return True
        self.event('timeout', goals=goals,
                   positions={n: {k: d.get(k) for k in ('x', 'y', 'state')}
                              for n, d in self.latest.items()})
        return False

    def save(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2))
        print(f'Saved {path}', flush=True)


def main():
    rclpy.init()
    node = ResearchRun()
    path = Path(sys.argv[1] if len(sys.argv) > 1 else
                'docs/figures/mo_phong_nckh/du_lieu_mo_phong.json')
    try:
        node.spin_for(3)
        if set(node.latest) != {'agv1', 'agv2'}:
            raise RuntimeError(f'Chưa nhận trạng thái đủ hai xe: {list(node.latest)}')

        node.scene = 'doi_cho'
        node.event('start', start={'agv1': [3, 2], 'agv2': [3, 4]})
        node.goal('pair 1 3 4 d ; 2 3 2 t')
        ok = node.wait_at({'agv1': (3, 4), 'agv2': (3, 2)}, timeout=180)
        node.data['metadata']['scenarios'].append({'name': node.scene, 'arrived': ok})

        node.scene = 'chuan_bi_giao_cat'
        node.goal('1 2 3 b')
        ok = node.wait_at({'agv1': (2, 3)}, timeout=120)
        node.data['metadata']['scenarios'].append({'name': node.scene, 'arrived': ok})

        node.scene = 'giao_cat'
        node.event('start', start={'agv1': [2, 3], 'agv2': [3, 2]})
        node.goal('pair 1 4 3 b ; 2 3 4 d')
        ok = node.wait_at({'agv1': (4, 3), 'agv2': (3, 4)}, timeout=160)
        node.data['metadata']['scenarios'].append({'name': node.scene, 'arrived': ok})

        node.scene = 'chuan_bi_vat_can'
        node.goal('pair 1 3 1 d ; 2 5 5 d')
        ok = node.wait_at({'agv1': (3, 1), 'agv2': (5, 5)}, timeout=160)
        node.data['metadata']['scenarios'].append({'name': node.scene, 'arrived': ok})

        node.scene = 'vat_can_tinh'
        node.obstacle([(3, 3)])
        node.spin_for(5)
        node.goal('1 3 5 d')
        ok = node.wait_at({'agv1': (3, 5)}, timeout=160)
        node.data['metadata']['scenarios'].append({'name': node.scene, 'arrived': ok})

        node.scene = 'chuan_bi_vat_can_dong'
        node.obstacle([])
        node.spin_for(3)
        node.goal('1 3 1 t')
        ok = node.wait_at({'agv1': (3, 1)}, timeout=160)
        node.data['metadata']['scenarios'].append({'name': node.scene, 'arrived': ok})

        node.scene = 'vat_can_dong'
        node.event('start', start={'agv1': [3, 1], 'agv2': [5, 5]})
        node.goal('1 3 5 d')
        start = time.monotonic()
        while not node.at('agv1', 3, 2, radius=0.18) and time.monotonic()-start < 50:
            node.spin_for(0.1)
        node.obstacle([(3, 3)])
        node.spin_for(10)
        node.obstacle([])
        ok = node.wait_at({'agv1': (3, 5)}, timeout=160)
        node.data['metadata']['scenarios'].append({'name': node.scene, 'arrived': ok})
    finally:
        node.save(path)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
