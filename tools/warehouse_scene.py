#!/usr/bin/env python3
"""Three independent feeders and an RGB/LiDAR cargo workflow on real Nav2.

Kinematic simulation, NOT Gazebo contact/dynamics or a physical accuracy test.
Only the scene knows truth. measure() receives RGB + nearest-return scans only.
"""
import argparse
import copy
import csv
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import signal
from pathlib import Path
import sys
import time

import cv2
import numpy as np
cv2.setNumThreads(1)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from grid_board import Board
from cargo_measure import measure
from grid_navigator import parse_heading
from delivery_queue import DeliveryQueue, bay_exit_clear, column_controls
from warehouse_queue import (Lane, COLORS, camera_config, classify_height, renderer,
                             parse_orders, conveyor_geometry, DeliveryOutfeed, TRAY_Z, DECK_Z,
                             FRONT_GAP)
from warehouse_world import GazeboMirror
from warehouse_telemetry import TelemetryGuard

STATES = dict(waiting='CHO BAT DAU', observe='TOI O QUAN SAT', measure='DO KIEN DAU HANG',
              inspection_yield='NHUONG LOI QUAN SAT', inspection_wait='DO LOI CAMERA THONG',
              pickup='TOI O LAY HANG', loading='BAN DAY DANG XEP HANG',
              wait_bay='CHO O GIAO HANG', deliver='DANG GIAO HANG',
              staging='TOI VUNG XEP HANG', queued='XEP HANG CHO GIAO',
              convoy='NOI DUOI TREN COT GIAO',
              approach='VAO TRAM GIAO', exiting='DA GIAO - DANG ROI TRAM',
              parking='VE O DO XE',
              round_loaded='DA NAP - CHO DU 3 XE', round_waiting='CHO DOT KE TIEP',
              round_return='VE VUNG DOI DOT KE TIEP',
              unload='DANG CHUYEN HANG LEN BANG TAI', returning='QUAY LAI LAY HANG',
              dock_wait='CHO BANG TAI NHAN HANG',
              finished='DA GIAO HET HANG', error='LOI - XE DA DUNG')


class Warehouse(Node):
    def __init__(self, args):
        super().__init__('warehouse_scene')
        self.args=args; self.out=Path(args.out)
        self.board=Board.load(self.out/'grid.yaml')
        orders=parse_orders(args.orders,args.items)
        self.lanes={n:Lane(n,i+2,self.board.xy(i+1,4),orders[n]) for i,n in enumerate(COLORS)}
        self.phase={n:'waiting' for n in self.lanes}
        self.pose={}; self.vel={}; self.status={}; self.scans={n:deque(maxlen=5) for n in self.lanes}
        self.pose_history={n:deque(maxlen=100) for n in self.lanes}
        self.localization_errors={n:[] for n in self.lanes}
        self.localization_file=None
        if self.board.cfg.get('lidar_map'):
            self.localization_file=open(self.out/'localization.csv','w',newline='',buffering=1)
            self.localization_writer=csv.writer(self.localization_file)
            self.localization_writer.writerow(('robot','t_s','amcl_x_m','amcl_y_m',
                                               'reference_x_m','reference_y_m','position_error_m'))
        self.scan_t={}; self.pose_t={}; self.status_t={}
        self.images={}; self.annotations={}; self.errors={}; self.samples={n:[] for n in self.lanes}
        self.locked={}; self.delivered={n:[] for n in self.lanes}; self.bays={}
        self.delivery=DeliveryQueue()
        self.column_queues=bool(self.board.cfg.get('column_queues'))
        self.queue_entries={};self.queue_entered=set()
        self.column_state={}
        self.column_orders=set();self.column_reported=set()
        self.pending_goals={}; self.active_goals={}; self.deadlines={}; self.last_goal_send={}
        self.last_frame=0.; self.worker=ThreadPoolExecutor(max_workers=1); self.future=None
        self.rgb=renderer(); self.mirror=GazeboMirror() if not args.no_gazebo else None
        self.mirror_worker=ThreadPoolExecutor(max_workers=1) if self.mirror else None
        self.mirror_future=None
        self.telemetry=TelemetryGuard();self.telemetry_state='healthy'
        self.pause_started=None;self.frame_epoch=0;self.future_epoch=0
        self.started=False; self.start_t=time.monotonic(); self.done=False; self.failed=False
        self.round_number=1
        self.telemetry_ready=False
        self.min_separation=float('inf'); self.close_pairs=set(); self.completed_events=[]
        self.outfeed=DeliveryOutfeed(self.board)
        self.last_success={}
        self.inspection_blocked_since={};self.inspection_blocker={};self.inspection_wait_since={}
        self.unload_start={}
        self.all_events=[]; self.dropped=self.outfeed.items; self.last_ui=0.; self.last_pose_send=0.
        self.event_file=open(self.out/'events.jsonl','w',buffering=1)
        self.pose_file=open(self.out/'trajectories.csv','w',buffering=1)
        self.pose_writer=csv.writer(self.pose_file)
        self.pose_writer.writerow(['t_s','robot','x_m','y_m','yaw_rad','phase','payload'])
        self.goal_pub=self.create_publisher(String,'/fleet/goal',10)
        self.service_pub=self.create_publisher(String,'/warehouse/service_holds',10)
        self.queue_pub=self.create_publisher(String,'/warehouse/delivery_queues',10)
        self.box_pub=self.create_publisher(String,'/sim/boxes',10)
        self.state_pub=self.create_publisher(String,'/warehouse/state',10)
        self.image_pubs={n:self.create_publisher(Image,f'/{n}/camera/image_raw',qos_profile_sensor_data) for n in self.lanes}
        self.create_subscription(String,'/fleet/status',self.on_status,50)
        self.create_subscription(String,'/warehouse/control',lambda m:self.control(m.data),10)
        for n in self.lanes:
            self.create_subscription(Odometry,f'/{n}/odom',lambda m,n=n:self.on_pose(n,m),1)
            self.create_subscription(LaserScan,f'/{n}/scan',lambda m,n=n:self.on_scan(n,m),qos_profile_sensor_data)
            if self.localization_file:
                self.create_subscription(PoseWithCovarianceStamped,f'/{n}/amcl_pose',
                                         lambda m,n=n:self.on_amcl(n,m),10)
        if not args.headless:
            cv2.namedWindow('Warehouse - 3 AGV / RGB + LiDAR / SPACE: start / Q: stop',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Warehouse - 3 AGV / RGB + LiDAR / SPACE: start / Q: stop',1200,640)
        self.create_timer(.05,self.tick)
        self.event('scene_ready', queues={n:l.order for n,l in self.lanes.items()},
                   delivery_mode=args.delivery_mode, sync_rounds=args.sync_rounds,
                   delivery_flow=self.board.cfg.get('delivery_flow','east'),column_queues=self.column_queues)

    def event(self, event_name, **data):
        e=dict(t_s=round(time.monotonic()-self.start_t,3),event=event_name,**data)
        self.all_events.append(e)
        self.event_file.write(json.dumps(e)+'\n')
        self.get_logger().info(json.dumps(e))

    def on_pose(self,n,m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        self.pose[n]=(p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
        self.vel[n]=(m.twist.twist.linear.x,m.twist.twist.angular.z)
        source=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        age=max(0.,self.get_clock().now().nanoseconds*1e-9-source) if source else 0.
        self.pose_t[n]=time.monotonic()-age
        self.pose_history[n].append((source,p.x,p.y))

    def on_amcl(self,n,m):
        history=self.pose_history[n]
        if not history:return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        reference=min(history,key=lambda p:abs(p[0]-stamp))
        if abs(reference[0]-stamp)>.3:return
        x,y=m.pose.pose.position.x,m.pose.pose.position.y
        error=math.hypot(x-reference[1],y-reference[2])
        self.localization_errors[n].append(error)
        self.localization_writer.writerow((n,round(stamp,3),x,y,reference[1],reference[2],error))

    def on_scan(self,n,m):
        self.scans[n].append((m.angle_min,m.angle_increment,list(m.ranges)))
        self.scan_t[n]=time.monotonic()

    def on_status(self,m):
        d=json.loads(m.data); n=d.get('name')
        if n not in self.lanes:return
        self.status[n]=d; self.status_t[n]=time.monotonic()-max(0.,time.time()-d.get('t',time.time()))
        if d.get('state') in ('precision_failed','estop') and self.started and not self.failed:
            self.fail(n,d.get('error') or d['state'])

    def control(self, command):
        if command=='start' and not self.started:
            if not all(n in self.pose and n in self.status for n in self.lanes):
                self.get_logger().warn('Waiting for all three robot poses and agents');return
            self.started=True; self.event('mission_started')
            if self.args.sync_rounds:self.event('round_started',round=self.round_number)
            for n,l in self.lanes.items():self.go(n,'observe',l.row,4,'d')
        elif command=='stop':
            self.fail('all','operator stop')

    def go(self,n,phase,row,col,heading=''):
        command=f'{n} {row} {col} {heading}'.strip()
        self.phase[n]=phase
        self.active_goals[n]=(row,col,heading)
        # Wait for manager acknowledgement before accepting an old done status.
        old=self.status.get(n,{}).get('route_id')
        self.pending_goals[n]=dict(command=command,old_route=old,t=time.monotonic(),ack=False)
        self.deadlines[n]=time.monotonic()+180
        self.last_goal_send[n]=time.monotonic()
        self.goal_pub.publish(String(data=command))
        self.event('goal_sent',robot=n,phase=phase,row=row,col=col,heading=heading)

    def reached(self,n):
        if n not in self.pending_goals:return False
        pending=self.pending_goals[n]; d=self.status.get(n,{})
        row,col,h=self.active_goals[n]
        if d.get('route_mode') in ('yield','yield_wait'):
            # Let the traffic manager finish the coordinated escape. Reissuing
            # the requested goal here would cancel it before the pivot ends.
            return False
        matches=d.get('route_goal')==[row-1,col-1]
        if d.get('route_id') != pending['old_route'] and matches:
            pending['ack']=True
        # A temporary yield is not an acknowledgement of the requested goal.
        # If the manager later cancels a route, retain/reissue the scene intent.
        if pending['ack'] and not matches and d.get('route_mode') not in ('yield','yield_wait'):
            pending['ack']=False;pending['old_route']=d.get('route_id')
            self.event('goal_intent_lost',robot=n,command=pending['command'])
        if not pending['ack']:
            # DDS discovery/start-up can drop an early request. Retry until ACK,
            # never reissue an already accepted route.
            if time.monotonic()-self.last_goal_send[n]>3:
                self.goal_pub.publish(String(data=pending['command']));self.last_goal_send[n]=time.monotonic()
            return False
        x,y=self.board.xy(row-1,col-1)
        # In the LiDAR/AMCL mode, the controller and dispatcher use the
        # estimated map pose. Ground-truth odometry remains an audit signal;
        # requiring it to pass the controller's 2 cm test would deadlock a
        # robot that has already reported a successful precision stop.
        if self.board.cfg.get('lidar_map'):
            if d.get('x') is None or d.get('y') is None or d.get('yaw') is None:return False
            p=(d['x'],d['y'],d['yaw'])
        else:p=self.pose[n]
        heading=parse_heading(h,self.board.yaw)[1] if h else p[2]
        error=math.atan2(math.sin(p[2]-heading),math.cos(p[2]-heading))
        tolerance=float(self.board.cfg.get('position_tolerance_m',.02))+.001
        # The agent checked final heading before reporting done. A second
        # comparison against a later AMCL sample can reject a valid stop on
        # the 2.5-degree boundary and leave the scene waiting forever.
        heading_ok=(self.board.cfg.get('lidar_map') or abs(error)<math.radians(2.5))
        okay=(d.get('state')=='done' and math.hypot(p[0]-x,p[1]-y)<=tolerance and
              heading_ok)
        if okay:
            del self.pending_goals[n]
            extra={}
            if self.board.cfg.get('lidar_map'):
                extra['physical_error_m']=math.hypot(self.pose[n][0]-x,self.pose[n][1]-y)
            self.event('goal_reached',robot=n,phase=self.phase[n],row=row,col=col,**extra)
        return okay

    def fail(self,n,reason):
        if self.failed:return
        self.failed=True
        self.goal_pub.publish(String(data='stop'))
        for name in self.phase:self.phase[name]='error'
        self.errors[n]=reason; self.event('mission_failed',robot=n,reason=reason)
        self.report()

    def supervise_telemetry(self,now):
        if not self.started or self.failed or self.done:return
        ages={n:(max(0.,now-self.pose_t.get(n,0)),max(0.,now-self.status_t.get(n,0)))
              for n in self.lanes}
        state=self.telemetry.step(now,ages)
        if state!=self.telemetry_state:
            self.event('telemetry_'+state,ages_s={n:list(a) for n,a in ages.items()})
            self.telemetry_state=state
        if state=='failed':
            self.fail('all','telemetry unavailable for 10 s or could not recover within 15 s');return
        if self.telemetry.paused and self.pause_started is None:
            self.pause_started=now;self.frame_epoch+=1
            for n in self.lanes:self.samples[n]=[]
            # Clear movement commands, retain scene goals, FIFO and bay leases.
            self.goal_pub.publish(String(data='stop'))
        elif state=='resumed':
            duration=now-self.pause_started;self.pause_started=None;self.frame_epoch+=1
            for n,l in self.lanes.items():
                if n in self.deadlines:self.deadlines[n]+=duration
                if l.stage in ('loading','feeding'):l.started+=duration
            self.outfeed.shift_time(duration)
            for n,pending in self.pending_goals.items():
                pending.update(old_route=self.status.get(n,{}).get('route_id'),ack=False,t=now)
                self.last_goal_send[n]=now
                self.goal_pub.publish(String(data=pending['command']))
                self.event('navigation_resumed',robot=n,command=pending['command'],pause_s=duration)

    def work_frames(self, jobs):
        output=[]
        for n,pose,boxes,scans,eligible in jobs:
            self.rgb.boxes=boxes; image=self.rgb.render(pose)
            result=vis=error=None
            if eligible:
                try:result,vis=measure(image,scans,camera_config())
                except ValueError as e:error=str(e)
            output.append((n,image,result,vis,error))
        return output

    def frames(self,now):
        if self.future and self.future.done():
            for n,img,result,vis,error in self.future.result():
                self.images[n]=img
                # The local UI needs no DDS image stream. Avoid serializing
                # three large RGB messages when there is no external viewer.
                if self.image_pubs[n].get_subscription_count():
                    m=Image();m.header.stamp=self.get_clock().now().to_msg()
                    m.header.frame_id='camera_link';m.height,m.width=img.shape[:2]
                    m.encoding='bgr8';m.step=m.width*3;m.data=img.tobytes()
                    self.image_pubs[n].publish(m)
                if (self.phase[n]!='measure' or self.telemetry.paused or
                        self.observation_blocker(n) or
                        self.future_epoch!=self.frame_epoch):continue
                if error:self.errors[n]=error;self.samples[n]=[]
                if result:
                    self.errors[n]='';self.annotations[n]=vis
                    samples=self.samples[n]; samples.append(result)
                    if len(samples)>=5:
                        vals=np.array([[a['width'],a['height']] for a in samples[-5:]])
                        if np.ptp(vals,axis=0).max()>.012:
                            self.samples[n]=samples[-4:];continue
                        w,h=vals.mean(0)
                        try:kind=classify_height(h)
                        except ValueError as e:self.errors[n]=str(e);self.samples[n]=[];continue
                        locked=dict(width_m=float(w),height_m=float(h),kind=kind,
                                    camera_distance_m=float(np.mean([a['z_cam'] for a in samples[-5:]])))
                        self.locked[n]=locked
                        # Truth is accessed here ONLY for a post-detection audit.
                        front=self.lanes[n].front
                        self.event('vision_locked',robot=n,package=front['id'],
                                   **locked,truth_kind=front['kind'],truth_width_m=front['w'],truth_height_m=front['h'])
                        cv2.imwrite(str(self.out/f"{front['id']}_measured.png"),vis)
                        if kind!=front['kind'] or abs(w-front['w'])>.02 or abs(h-front['h'])>.02:
                            self.fail(n,'vision audit: wrong front package measurement');continue
                        self.go(n,'pickup',self.lanes[n].row,5,'d')
            self.future=None
        if self.future is None and now-self.last_frame>.45 and len(self.pose)==3:
            boxes=[dict(b) for l in self.lanes.values() for b in l.items]
            jobs=[]
            for n,p in self.pose.items():
                v,w=self.vel.get(n,(1,1))
                eligible=(not self.telemetry.paused and self.phase[n]=='measure' and abs(v)<.005 and abs(w)<.015 and
                          len(self.scans[n])>=5 and now-self.scan_t.get(n,0)<.5 and
                          not self.observation_blocker(n) and
                          self.lanes[n].stage=='ready')
                jobs.append((n,p,copy.deepcopy(boxes),list(self.scans[n]),eligible))
            self.future_epoch=self.frame_epoch
            self.future=self.worker.submit(self.work_frames,jobs);self.last_frame=now

    def mechanics(self,now):
        if not self.telemetry.paused and not self.failed:
            for package in self.outfeed.update(now):self.event('outbound_cleared',package=package)
        for n,l in self.lanes.items():
            if n not in self.pose:continue
            event=l.update(now,self.pose[n]) if not self.telemetry.paused and not self.failed else None
            if event:
                self.event(event,robot=n,front=l.front['id'] if l.front else None,
                           distance_from_pickup_m=math.dist(self.pose[n][:2],l.pickup))
            if event=='loaded':
                self.phase[n]='round_loaded' if self.args.sync_rounds else 'wait_bay'
            if self.phase[n]=='unload' and l.payload:
                motion_now=self.pause_started if self.telemetry.paused else now
                u=min(1.,max(0.,1-(self.deadlines[n]-motion_now)/1.5))
                smooth=u*u*(3-2*u)
                start=self.unload_start[n];x,y=self.outfeed.entry(self.locked[n]['kind'])
                l.payload.update(x=start[0]+(x-start[0])*smooth,
                                 y=start[1]+(y-start[1])*smooth,
                                 z=TRAY_Z+(DECK_Z-TRAY_Z)*smooth,yaw=self.board.yaw)
        boxes=[dict(b) for l in self.lanes.values() for b in l.items]
        boxes += [dict(l.payload) for l in self.lanes.values() if l.payload]
        boxes += self.dropped
        self.box_pub.publish(String(data=json.dumps(boxes)))
        if (self.mirror and now-self.last_pose_send>.20 and
                (self.mirror_future is None or self.mirror_future.done())):
            if self.mirror_future is not None:
                try:self.mirror_future.result()
                except Exception as e:self.get_logger().warn('Gazebo visual update: '+str(e))
            poses={}
            motion_now=self.pause_started if self.telemetry.paused and self.pause_started is not None else now
            # A tiny pose nudge refreshes old Gazebo GUI clients after reconnect.
            epsilon=.0001*(int(now)%2)
            for n,l in self.lanes.items():
                if n not in self.pose:continue
                x,y,th=self.pose[n]
                poses[n]=(x,y,epsilon,0.,th)
                poses[n+'_tray']=(x,y,.154+epsilon,0.,th)
                px,py=l.pickup
                for part,coords in l.mechanism(motion_now).items():
                    coords=list(coords);coords[2]+=epsilon
                    poses[n+'_'+part]=tuple(coords+[0.,0.] if len(coords)==3 else coords)
                for j in range(conveyor_geometry(len(l.order))[2]) if l.stage=='feeding' else ():
                    if .36+j*.08<.70:continue  # dedicated lift table at the nose
                    roll=math.pi/2
                    spin=(motion_now-l.started)*6 if l.stage=='feeding' else 0.
                    poses[f'{n}_roller_{j}']=(px+.36+j*.08,py,.044+epsilon,roll,spin,0.)
            for b in boxes:
                poses[b['id']]=(b['x'],b['y'],b.get('z',0.)+epsilon,0.,b['yaw'])
            self.mirror_future=self.mirror_worker.submit(self.mirror.send,poses);self.last_pose_send=now

    def unavailable_nodes(self,me):
        nodes=set()
        for n,p in self.pose.items():
            if n==me:continue
            for r,c in self.board.nodes():
                if math.dist(p[:2],self.board.xy(r,c))<self.board.spacing/2+self.board.robot_radius+.02:
                    nodes.add((r+1,c+1))
            if n in self.pending_goals:
                same_column=(getattr(self,'column_queues',False) and
                             self.delivery.kind.get(me) is not None and
                             self.delivery.kind.get(n)==self.delivery.kind.get(me) and
                             self.active_goals[n][:2]==(1,self.delivery.kind[me]))
                if not same_column:nodes.add(self.active_goals[n][:2])
        nodes.update(node for n,node in self.delivery.staging.items() if n!=me)
        return nodes

    def observation_blocker(self,n,station=None):
        """Find a robot between an observation station and its front package.

        Use the agents' estimated map poses, as the real workflow would; the
        physical odometry pose is reserved for simulation audits.
        """
        if not self.board.cfg.get('lidar_map') or not self.lanes[n].front:return None
        p=station or self.status.get(n,{})
        if isinstance(p,dict):
            if p.get('x') is None or p.get('y') is None:return None
            p=(p['x'],p['y'])
        fx,fy=self.lanes[n].pickup
        fx+=FRONT_GAP*math.cos(self.board.yaw)
        fy+=FRONT_GAP*math.sin(self.board.yaw)
        ex,ey=math.cos(self.board.yaw),math.sin(self.board.yaw)
        front_along=(fx-p[0])*ex+(fy-p[1])*ey
        if front_along<.4:return None
        for other,d in self.status.items():
            if other==n or d.get('x') is None or d.get('y') is None:continue
            dx,dy=d['x']-p[0],d['y']-p[1]
            along=dx*ex+dy*ey
            lateral=abs(-dx*ey+dy*ex)
            if .15<along<front_along-.1 and lateral<.23:return other
        return None

    def dispatch_delivery(self,n,now):
        if getattr(self,'column_queues',False):
            return self.dispatch_column_delivery(n,now)
        kind=self.locked[n]['kind']
        q=self.delivery
        if q.enqueue(n,kind,now):
            self.event('delivery_queued',robot=n,slot=kind,rank=q.rank(n))
        if self.phase[n]=='staging':
            # Finish this short leg before switching destination. Replacing
            # it halfway along an edge makes the manager plan from the node
            # behind the robot and sends it back there for an extra pivot.
            if not self.reached(n):return
            self.phase[n]='queued'
            self.event('staging_arrived',robot=n,slot=kind,rank=q.rank(n),node=q.staging[n])
        unavailable=self.unavailable_nodes(n)
        if q.can_admit(n) and (1,kind) not in unavailable:
            q.admit(n)
            self.event('delivery_admitted',robot=n,slot=kind,wait_s=now-q.joined[n])
            # One route reaches the bay through its configured final edge.
            # Row 2 is a transit node, not a second precision-stop goal.
            self.go(n,'deliver',1,kind,'n')
            return
        # FIFO advances when the bay becomes available. Moving between waiting
        # slots while the owner unloads adds travel, pivots and route changes
        # without allowing an earlier delivery.
        if self.phase[n]=='queued':return
        old=q.staging.get(n)
        # Never park a waiter in the compulsory row-2 entrance or in the
        # approaching owner's exit/entry corridor.
        staging_blocked=set(unavailable)|{(2,c) for c in range(1,self.board.cols+1)}
        owner=q.owners.get(kind)
        if owner and self.phase.get(owner) in ('deliver','approach') and owner in self.pose:
            owner_row=self.board.closest(*self.pose[owner][:2])[0]+1
            # Keep the owner's horizontal exit and vertical bay approach
            # clear, without excluding every column of all preceding rows.
            staging_blocked|={(owner_row,c) for c in range(1,self.board.cols+1)}
            staging_blocked|={(r,kind) for r in range(3,min(owner_row,self.board.rows)+1)}
        node=q.assign_staging(n,self.board,staging_blocked,pose=self.pose[n])
        if node is not None and (self.phase[n]=='wait_bay' or node!=old):
            self.event('staging_assigned',robot=n,slot=kind,rank=q.rank(n),node=node)
            self.go(n,'staging',*node,'')

    def register_column_orders(self,now):
        # Classification finishes on different ticks. Keep an unstarted
        # column spatially ordered across ALL those ticks, not only within
        # each batch of new camera results. Freeze it at first departure.
        candidates=[n for n in self.lanes if self.phase[n] in ('pickup','loading','wait_bay')
                    and n in self.locked and n not in self.delivery.kind]
        candidates.sort(key=lambda n:(self.locked[n]['kind'],self.board.closest(*self.pose[n][:2])[0],n))
        for n in candidates:
            kind=self.locked[n]['kind']
            self.delivery.enqueue(n,kind,now)
            self.event('delivery_intent',robot=n,slot=kind)
            if kind in self.column_orders:
                self.column_reported.add(n)
                self.event('delivery_queued',robot=n,slot=kind,rank=self.delivery.rank(n),registration='tail_append')
        for kind,waiting in self.delivery.waiting.items():
            if kind in self.column_orders:continue
            target=self.board.xy(0,kind-1)
            ordered=sorted(waiting,key=lambda n:(self.board.closest(*self.pose[n][:2])[0],
                            math.dist(self.pose[n][:2],target),n))
            self.delivery.waiting[kind]=deque(ordered)

    def commit_column_order(self,kind):
        if kind in self.column_orders:return
        self.column_orders.add(kind)
        for n in self.delivery.waiting[kind]:
            self.column_reported.add(n)
            self.event('delivery_queued',robot=n,slot=kind,rank=self.delivery.rank(n),registration='column_departure')

    def publish_column_controls(self):
        if not self.column_queues:return
        controls=column_controls(self.delivery,self.board,self.pose,self.queue_entries,self.queue_entered)
        for n,d in controls.items():
            if d!=self.column_state.get(n):self.event('column_permission',robot=n,**d)
        self.column_state=controls
        self.queue_pub.publish(String(data=json.dumps(controls)))
        return controls

    def dispatch_column_delivery(self,n,now):
        q=self.delivery;kind=self.locked[n]['kind']
        self.commit_column_order(kind)
        if q.can_admit(n) and (1,kind) not in self.unavailable_nodes(n):
            q.admit(n)
            self.event('delivery_admitted',robot=n,slot=kind,wait_s=now-q.joined[n])
            if self.phase[n]=='convoy':
                self.phase[n]='deliver'
                # Keep the exact route and action; only open its final edge.
                self.publish_column_controls()
                return
        if n not in self.queue_entries:
            controls=self.publish_column_controls()
            self.queue_entries[n]=controls[n]['entry_row']
            self.event('column_join',robot=n,slot=kind,entry_row=self.queue_entries[n])
            self.publish_column_controls()
            phase='deliver' if q.owners.get(kind)==n else 'convoy'
            self.go(n,phase,1,kind,'n')
        elif self.phase[n]=='convoy':
            self.reached(n)  # acknowledge/retry the same route, never a staging goal

    def mission(self,now):
        if not self.started or self.failed or self.done or self.telemetry.paused:return
        if self.column_queues:self.register_column_orders(now)
        if self.args.sync_rounds and all(p=='round_loaded' for p in self.phase.values()):
            # Register the whole observed cohort before anyone is admitted.
            # The schedule never supplies class IDs to the camera detector.
            self.event('round_ready',round=self.round_number,
                       kinds={n:self.locked[n]['kind'] for n in self.lanes},
                       packages={n:l.payload['id'] for n,l in self.lanes.items()})
            for n in self.lanes:
                self.phase[n]='wait_bay'
                if self.args.delivery_mode=='staged':
                    kind=self.locked[n]['kind']
                    if self.delivery.enqueue(n,kind,now):
                        self.event('delivery_queued',robot=n,slot=kind,rank=self.delivery.rank(n))
        for n,l in self.lanes.items():
            ph=self.phase[n]
            if self.args.delivery_mode=='staged':
                if ph not in ('wait_bay','staging','queued'):
                    self.delivery.release_staging_when_clear(n,self.pose[n],self.board)
                if ph in ('exiting','returning','parking','round_return') and n in self.delivery.kind:
                    kind=self.delivery.kind[n]
                    if bay_exit_clear(self.pose[n],self.board,kind):
                        self.delivery.release(n)
                        self.queue_entries.pop(n,None);self.queue_entered.discard(n)
                        self.column_reported.discard(n)
                        if not self.delivery.waiting[kind] and kind not in self.delivery.owners:
                            self.column_orders.discard(kind)
                        self.event('bay_released',robot=n,slot=kind,
                                   distance_m=math.dist(self.pose[n][:2],self.board.xy(0,kind-1)))
            if n in self.pending_goals and now>self.deadlines[n]:
                self.fail(n,f'navigation timeout: {ph}');return
            if ph in ('observe','returning') and self.reached(n):
                # Release a shared delivery bay only AFTER physically leaving it.
                for k,owner in list(self.bays.items()):
                    if owner==n:del self.bays[k]
                if l.front:
                    self.begin_measure(n,now)
                else:self.phase[n]='finished'
            elif ph=='measure':
                blocker=self.observation_blocker(n)
                if blocker:
                    if n not in self.inspection_blocked_since:
                        self.inspection_blocked_since[n]=now
                        self.event('inspection_occluded',robot=n,blocker=blocker)
                    elif now-self.inspection_blocked_since[n]>2:
                        self.inspection_blocker[n]=blocker
                        self.samples[n]=[];self.frame_epoch+=1
                        yield_row=l.row+1 if l.row<self.board.rows else l.row-1
                        self.go(n,'inspection_yield',yield_row,4)
                        self.event('inspection_yield',robot=n,blocker=blocker,row=yield_row,col=4)
                        self.inspection_blocked_since.pop(n,None)
                else:self.inspection_blocked_since.pop(n,None)
                if self.phase[n]=='measure' and now>self.deadlines[n]:
                    self.fail(n,'could not isolate/measure front package: '+self.errors.get(n,''));return
            elif ph=='inspection_yield' and self.reached(n):
                self.phase[n]='inspection_wait';self.inspection_wait_since[n]=now
            elif ph=='inspection_wait':
                station=self.board.xy(l.row-1,3)
                if not self.observation_blocker(n,station):
                    self.event('inspection_clear',robot=n,blocker=self.inspection_blocker.pop(n,None))
                    self.inspection_wait_since.pop(n,None)
                    self.go(n,'observe',l.row,4,'d')
                elif now-self.inspection_wait_since[n]>90:
                    self.fail(n,'observation blocked for more than 90 s');return
            elif ph=='pickup' and self.reached(n):
                l.load(now);self.phase[n]='loading';self.event('load_started',robot=n,package=l.payload['id'])
            elif ph in ('wait_bay','staging','queued','convoy') and self.args.delivery_mode=='staged':
                self.dispatch_delivery(n,now)
            elif ph=='wait_bay':
                kind=self.locked[n]['kind']
                if kind not in self.bays:
                    self.bays[kind]=n;self.go(n,'deliver',1,kind)
            elif ph=='approach' and self.reached(n):
                self.go(n,'deliver',1,self.locked[n]['kind'],'n')
            elif ph=='deliver' and self.reached(n):
                self.phase[n]='dock_wait'
                self.event('delivery_arrived',robot=n,package=l.payload['id'],slot=self.locked[n]['kind'])
                self.begin_unload(n,now)
            elif ph=='dock_wait':self.begin_unload(n,now)
            elif ph=='unload' and now>=self.deadlines[n]:
                b=l.unload();kind=self.locked[n]['kind']
                self.outfeed.accept(b,kind,now)
                self.delivered[n].append(b['id'])
                self.last_success[n]=(now,kind)
                self.event('delivery_success',robot=n,package=b['id'],slot=kind,
                           receiving_position_m=[b['x'],b['y'],b['z']])
                if self.args.delivery_mode=='serial':
                    self.go(n,'returning',l.row,4,'d')
                elif self.column_queues:
                    if self.board.cfg.get('delivery_flow')=='east':
                        self.go(n,'exiting',2,self.board.cols,'b')
                    else:
                        # One goal back to work. The planner chooses the exit
                        # direction; there is no compulsory adjacent waypoint.
                        self.return_after_delivery(n)
                elif self.board.cfg.get('delivery_flow')=='east':
                    self.go(n,'exiting',1,kind+1,'d')
                else:self.return_after_delivery(n)
            elif ph=='exiting' and self.reached(n):
                if n in self.delivery.kind:
                    self.fail(n,'exit reached without safely clearing delivery bay');return
                if self.column_queues:
                    self.return_after_delivery(n)
                    continue
                if l.front:
                    if self.args.sync_rounds:
                        # Waiting at observation (2,4) would block the T4 entry.
                        # Completed robots wait off the entry row instead.
                        self.go(n,'round_return',5,list(self.lanes).index(n)+1,'d')
                    else:self.go(n,'returning',l.row,4,'d')
                else:
                    parking=(5,list(self.lanes).index(n)+1) if self.args.sync_rounds else (l.row,1)
                    self.go(n,'parking',*parking,'' if self.board.cfg.get('lidar_map') else 'd')
            elif ph=='round_return' and self.reached(n):
                self.phase[n]='round_waiting'
            elif ph=='parking' and self.reached(n):
                self.phase[n]='finished'
        if self.args.sync_rounds and all(p=='round_waiting' for p in self.phase.values()):
            self.event('round_completed',round=self.round_number)
            self.round_number+=1
            self.event('round_started',round=self.round_number)
            for n,l in self.lanes.items():self.go(n,'observe',l.row,4,'d')
        if all(p=='finished' for p in self.phase.values()) and all(b['outbound_cleared'] for b in self.dropped):
            if self.args.sync_rounds:self.event('round_completed',round=self.round_number)
            self.done=True;self.event('mission_complete',deliveries=sum(map(len,self.delivered.values())))
            self.report()

    def begin_measure(self,n,now):
        self.phase[n]='measure';self.deadlines[n]=now+45;self.samples[n]=[]
        self.locked.pop(n,None);self.annotations.pop(n,None)

    def begin_unload(self,n,now):
        l=self.lanes[n]
        if not self.outfeed.can_accept(self.locked[n]['kind'],l.payload['d']):return
        self.phase[n]='unload';self.deadlines[n]=now+1.5
        self.unload_start[n]=self.pose[n][:2]
        self.event('unload_started',robot=n,package=l.payload['id'])

    def return_after_delivery(self,n):
        l=self.lanes[n]
        if l.front:
            if self.args.sync_rounds:self.go(n,'round_return',5,list(self.lanes).index(n)+1,'d')
            else:self.go(n,'returning',l.row,4,'d')
        else:self.go(n,'parking',5,list(self.lanes).index(n)+1,
                     '' if self.board.cfg.get('lidar_map') else 'd')

    def publish_service_holds(self):
        holds={}
        for n,phase in self.phase.items():
            protected=phase in ('measure','loading','unload','dock_wait','round_loaded')
            if phase in ('pickup','deliver') and n in self.active_goals and n in self.pose:
                r,c,_=self.active_goals[n]
                protected=math.dist(self.pose[n][:2],self.board.xy(r-1,c-1))<.35
            if protected and not self.failed:holds[n]=time.time()+3
        self.service_pub.publish(String(data=json.dumps(holds)))

    def report(self):
        locks=[e for e in self.all_events if e['event']=='vision_locked']
        self.completed_events=locks
        elapsed=next((e['t_s'] for e in self.all_events if e['event']=='mission_complete'),
                     time.monotonic()-self.start_t)
        report=dict(simulation='kinematic robots + Nav2 + RGB/analytic LiDAR; no contact dynamics',
                    complete=self.done,failed=self.failed,elapsed_s=elapsed,
                    delivery_mode=self.args.delivery_mode,
                    delivered=self.delivered,min_robot_center_separation_m=self.min_separation,
                    proximity_violations=len(self.close_pairs),
                    vision_measurements=locks,events=len(self.all_events))
        if self.localization_file:
            report['localization']={n:dict(samples=len(values),
                median_error_m=float(np.median(values)) if values else None,
                max_error_m=max(values) if values else None)
                for n,values in self.localization_errors.items()}
        (self.out/'summary.json').write_text(json.dumps(report,indent=2))

    def dashboard(self):
        panels=[]
        for n,l in self.lanes.items():
            tile=np.full((620,400,3),(27,31,38),np.uint8)
            def text(s,y,color=(220,225,230),size=.53):
                cv2.putText(tile,s,(12,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)
            rgb=tuple(int(float(c)*255) for c in COLORS[n].split()[::-1])
            text(n.upper()+f' | lane {l.row-1} | pickup ({l.row},5)',30,rgb,.67)
            image=self.annotations.get(n) if n in self.locked else self.images.get(n)
            # Always update the live camera while moving; annotated lock below.
            if self.phase[n] not in ('measure','pickup','loading','wait_bay'):
                image=self.images.get(n)
            if image is not None:tile[48:348]=cv2.resize(image,(400,300))
            label='TAM DUNG - CHO DU LIEU ON DINH' if self.telemetry.paused and not self.failed else STATES[self.phase[n]]
            text(label,380,(60,190,255) if self.telemetry.paused else (100,230,140),.49)
            text('FIFO: '+' > '.join(f'T{b["kind"]}' for b in l.items),410)
            rank=self.delivery.rank(n)
            slot=self.delivery.staging.get(n)
            if rank==0:
                text('TRAM GIAO: DA DUOC CAP QUYEN',438)
            elif rank is not None:
                if self.column_queues and n in self.column_state:
                    gate=self.column_state[n]['gate_row']
                    text(f'NOI DUOI #{rank} | COT {self.delivery.kind[n]} | HANG {gate}',438,size=.48)
                else:text(f'CHO GIAO #{rank}'+(f' | o {slot}' if slot else ''),438)
            else:text('Feeder: '+l.stage,438)
            if n in self.locked:
                m=self.locked[n]
                text(f'AVG 5 | W {m["width_m"]*100:.1f}  H {m["height_m"]*100:.1f} cm',468)
                text(f'DIST {m["camera_distance_m"]*100:.1f} cm | T{m["kind"]} -> (1,{m["kind"]})',496)
            else:text(f'Valid frames: {len(self.samples[n])}/5',468)
            text(f'Delivered {len(self.delivered[n])}/{len(l.order)}',527,rgb)
            if self.errors.get(n):text(self.errors[n][:46],557,(60,165,255),.43)
            elif n in self.last_success and time.monotonic()-self.last_success[n][0]<6:
                text(f'DA GIAO THANH CONG -> (1,{self.last_success[n][1]})',557,(100,230,140),.46)
            total=sum(map(len,self.delivered.values()));expected=sum(len(a.order) for a in self.lanes.values())
            footer=(f'ROUND {self.round_number}/{len(l.order)} | SPACE start | Q stop'
                    if self.args.sync_rounds else f'TOTAL {total}/{expected} | SPACE start | Q stop')
            text(footer,601,(160,170,180),.43)
            panels.append(tile)
        canvas=np.hstack(panels)
        cv2.imwrite(str(self.out/'dashboard_latest.jpg'),canvas)
        if self.done or self.failed:cv2.imwrite(str(self.out/'dashboard_final.png'),canvas)
        if not self.args.headless:
            cv2.imshow('Warehouse - 3 AGV / RGB + LiDAR / SPACE: start / Q: stop',canvas)
            key=cv2.waitKey(1)&255
            if key==32:self.control('start')
            elif key in (ord('q'),27):self.control('stop')

    def tick(self):
        now=time.monotonic()
        try:
            self.publish_service_holds()
            if not self.telemetry_ready and len(self.pose)==3 and len(self.status)==3:
                self.telemetry_ready=True;self.event('telemetry_ready')
            if self.args.autostart and not self.started and len(self.status)==3 and now-self.start_t>8:
                self.control('start')
            self.supervise_telemetry(now)
            if len(self.pose)==3 and not self.telemetry.paused and not self.failed:
                names=list(self.pose)
                for i,a in enumerate(names):
                    for b in names[i+1:]:
                        d=math.dist(self.pose[a][:2],self.pose[b][:2])
                        self.min_separation=min(self.min_separation,d)
                        if d<.26 and (a,b) not in self.close_pairs:
                            self.close_pairs.add((a,b));self.fail(a,f'robot proximity < 26 cm: {a}/{b}');return
            self.mechanics(now);self.mission(now);self.frames(now)
            self.publish_column_controls()
            if now-self.last_ui>.5:
                self.dashboard();self.last_ui=now
                for n,p in self.pose.items():
                    payload=self.lanes[n].payload
                    self.pose_writer.writerow([round(now-self.start_t,3),n,*p,self.phase[n],
                                               payload['id'] if payload else ''])
                self.state_pub.publish(String(data=json.dumps(dict(phase=self.phase,done=self.done,failed=self.failed))))
        except Exception as e:
            self.get_logger().error(repr(e))
            if not self.failed:self.fail('scene',repr(e))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True)
    ap.add_argument('--headless',action='store_true');ap.add_argument('--autostart',action='store_true')
    ap.add_argument('--no-gazebo',action='store_true');ap.add_argument('--items',type=int,choices=range(1,5),default=0)
    ap.add_argument('--orders',default='')
    ap.add_argument('--delivery-mode',choices=('staged','serial'),default='staged')
    ap.add_argument('--sync-rounds',action='store_true')
    args=ap.parse_args();rclpy.init();node=Warehouse(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        if rclpy.ok():node.goal_pub.publish(String(data='stop'))
        if not node.done and not node.failed:node.event('mission_interrupted')
        node.report()
        if node.mirror_worker:node.mirror_worker.shutdown(wait=False,cancel_futures=True)
        node.worker.shutdown(wait=True);node.event_file.close();node.pose_file.close()
        if node.localization_file:node.localization_file.close()
        cv2.destroyAllWindows();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
