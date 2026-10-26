#!/usr/bin/env python3
"""Gop cay TF cua cac xe vao /tf chung de MOT cua so RViz thay ca doi xe.

    python3 tools/fleet_viz.py agv1 agv2      # fleet_real.sh / fleet_sim.sh tu chay

Moi xe co cay TF rieng (/agvN/tf, /agvN/tf_static) voi cung ten frame (odom,
base_footprint, lidar_link...), nen RViz khong doc duoc hai cay mot luc. Node nay:
  - /agvN/tf, /agvN/tf_static -> /tf, /tf_static, doi ten frame thanh "agvN/<frame>"
    (rieng "map" giu nguyen - dung chung);
  - /agvN/scan -> /fleet_viz/agvN/scan voi frame_id "agvN/lidar_link".
Che do nhieu xe khong ai dung /tf goc (moi xe da remap sang /agvN/tf) nen khong dam.
RViz: tools/fleet.rviz (RobotModel dung TF Prefix "agvN").
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PolygonStamped
from map_msgs.msg import OccupancyGridUpdate

SHARED = ('map',)
STATIC_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


class FleetViz(Node):
    def __init__(self, cars, nav=False):
        super().__init__('fleet_viz')
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.static_pub = self.create_publisher(TFMessage, '/tf_static', STATIC_QOS)
        self.static = {}                    # (car, child) -> TransformStamped da doi ten
        self.scan_pubs = {}
        self.nav_pubs = {}
        for car in cars:
            self.create_subscription(TFMessage, f'/{car}/tf',
                                     lambda m, c=car: self.cb_tf(c, m), 100)
            self.create_subscription(TFMessage, f'/{car}/tf_static',
                                     lambda m, c=car: self.cb_static(c, m), STATIC_QOS)
            self.scan_pubs[car] = self.create_publisher(LaserScan, f'/fleet_viz/{car}/scan',
                                                        5 if nav else qos_profile_sensor_data)
            self.create_subscription(LaserScan, f'/{car}/scan',
                                     lambda m, c=car: self.cb_scan(c, m), qos_profile_sensor_data)
            if nav:
                topics=[('received_global_plan',Path,10),('local_plan',Path,10)]
                for costmap in ('local_costmap','global_costmap'):
                    topics.extend([(costmap+'/costmap',OccupancyGrid,STATIC_QOS),
                                   (costmap+'/costmap_updates',OccupancyGridUpdate,10),
                                   (costmap+'/published_footprint',PolygonStamped,10)])
                for topic,kind,qos in topics:
                    key=(car,topic)
                    self.nav_pubs[key]=self.create_publisher(kind,f'/fleet_viz/{car}/{topic}',qos)
                    self.create_subscription(kind,f'/{car}/{topic}',
                        lambda m,c=car,t=topic:self.cb_nav(c,t,m),qos)
        self.get_logger().info(f'Gop TF {", ".join(cars)} -> /tf (frame agvN/...), '
                               f'scan -> /fleet_viz/agvN/scan')

    @staticmethod
    def rename(car, frame):
        f = frame.lstrip('/')
        return f if f in SHARED or not f else f'{car}/{f}'

    def renamed(self, car, msg):
        for t in msg.transforms:
            t.header.frame_id = self.rename(car, t.header.frame_id)
            t.child_frame_id = self.rename(car, t.child_frame_id)
        return msg

    def cb_tf(self, car, msg):
        self.tf_pub.publish(self.renamed(car, msg))

    def cb_static(self, car, msg):
        for t in self.renamed(car, msg).transforms:
            self.static[(car, t.child_frame_id)] = t
        self.static_pub.publish(TFMessage(transforms=list(self.static.values())))

    def cb_scan(self, car, msg):
        msg.header.frame_id = self.rename(car, msg.header.frame_id)
        self.scan_pubs[car].publish(msg)

    def cb_nav(self, car, topic, msg):
        msg.header.frame_id=self.rename(car,msg.header.frame_id)
        if isinstance(msg,Path):
            for pose in msg.poses:pose.header.frame_id=self.rename(car,pose.header.frame_id)
        self.nav_pubs[(car,topic)].publish(msg)


def main():
    rclpy.init(args=sys.argv)
    args = remove_ros_args(sys.argv)[1:]
    nav='--nav' in args
    cars=[a for a in args if a!='--nav'] or ['agv1','agv2']
    node = FleetViz(cars,nav=nav)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
