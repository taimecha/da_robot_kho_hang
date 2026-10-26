#!/usr/bin/env python3
"""Drive one simulated scanner, then save slam_toolbox's LiDAR-derived /map."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from grid_board import Board


class Mapper(Node):
    def __init__(self, folder):
        super().__init__('warehouse_lidar_mapping')
        self.folder=folder
        board=Board.load(folder/'grid.yaml')
        self.waypoints=[board.xy(*node) for node in ((1,3),(3,3),(3,0),(1,0))]
        self.pose=None;self.map=None;self.scans=0;self.map_updates=0
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.trace=open(folder/'mapping_trajectory.csv','w',newline='')
        self.writer=csv.writer(self.trace)
        self.writer.writerow(('t_s','x_m','y_m','yaw_rad','scan_count','map_updates'))
        self.started=time.monotonic()

    def on_odom(self,m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        self.pose=(p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))

    def on_scan(self,m):
        self.scans+=1

    def on_map(self,m):
        if m.info.width and m.info.height:
            self.map=m;self.map_updates+=1

    def stop(self):
        self.pub.publish(Twist())

    def run(self):
        deadline=time.monotonic()+150
        index=0
        while rclpy.ok() and time.monotonic()<deadline and index<len(self.waypoints):
            rclpy.spin_once(self,timeout_sec=.1)
            if self.pose is None:continue
            x,y,yaw=self.pose;gx,gy=self.waypoints[index]
            dist=math.hypot(gx-x,gy-y)
            if dist<.075:
                index+=1;self.stop();continue
            angle=math.atan2(gy-y,gx-x)
            error=math.atan2(math.sin(angle-yaw),math.cos(angle-yaw))
            cmd=Twist()
            if abs(error)>.12:
                cmd.angular.z=math.copysign(min(.8,max(.15,abs(error)*1.8)),error)
            else:
                cmd.linear.x=min(.22,max(.07,dist*.8))
                cmd.angular.z=max(-.3,min(.3,error*1.5))
            self.pub.publish(cmd)
            self.writer.writerow((round(time.monotonic()-self.started,3),x,y,yaw,
                                  self.scans,self.map_updates))
        self.stop()
        if index<len(self.waypoints):raise RuntimeError(f'mapping traversal timed out at waypoint {index+1}')
        end=time.monotonic()+6
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.1)
        if self.map is None or self.scans<20 or self.map_updates<2:
            raise RuntimeError('slam_toolbox did not produce a usable LiDAR map')
        self.save_map()

    def save_map(self):
        m=self.map
        cells=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        pixels=np.where(cells<0,205,np.where(cells>=65,0,254)).astype(np.uint8)
        image=np.flipud(pixels)
        (self.folder/'map.pgm').write_bytes(
            f'P5\n{m.info.width} {m.info.height}\n255\n'.encode()+image.tobytes())
        q=m.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if abs(yaw)>.01:raise RuntimeError(f'unsupported rotated SLAM map origin: {yaw}')
        (self.folder/'map.yaml').write_text(yaml.safe_dump(dict(
            image='map.pgm',resolution=float(m.info.resolution),
            origin=[float(m.info.origin.position.x),float(m.info.origin.position.y),0.],
            negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')))
        (self.folder/'mapping.json').write_text(json.dumps(dict(
            source='slam_toolbox /map from virtual LaserScan',scan_count=self.scans,
            map_updates=self.map_updates,width=m.info.width,height=m.info.height,
            resolution_m=m.info.resolution,known_cells=int(np.count_nonzero(cells>=0)),
            occupied_cells=int(np.count_nonzero(cells>=65)),
            waypoints_m=self.waypoints),indent=2))
        self.trace.flush()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True)
    a=ap.parse_args();rclpy.init();node=Mapper(Path(a.out))
    try:node.run()
    finally:
        node.stop();node.trace.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
