#!/usr/bin/env python3
"""Own-process-group lifecycle, one isolated simulation per laptop."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
import numpy as np
WS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WS))
from grid_board import Board
from warehouse_queue import Lane, COLORS, parse_orders, DEMO_15, DEMO_30, conveyor_geometry
from warehouse_world import build_world
from warehouse_rviz import prepare_rviz, rviz_environment
from warehouse_landmarks import LANDMARKS
LOCK=Path('/tmp/warehouse_queue_sim_owner.json')


def identity(pid):
    try:return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except (FileNotFoundError,ProcessLookupError):return None


def owner():
    try:
        d=json.loads(LOCK.read_text())
        return d if identity(d['pid'])==d['start'] else None
    except (FileNotFoundError,ValueError,KeyError):return None


def prepare(out,items,orders=None,delivery_mode='staged',sync_rounds=False,
            delivery_flow='flexible',lidar_map=False):
    out.mkdir(parents=True,exist_ok=True)
    grid=yaml.safe_load((WS/'src/my_bot_bringup/config/grid_real.yaml').read_text())
    grid.update(center_x=0.,center_y=0.)
    grid['rotate'].update(min_speed=.04,max_speed=1.,decel=3.,gain=2.2,
                          brake_delay=.05,settle_s=.25,timeout_s=18.)
    grid['delivery_flow']=delivery_flow
    grid['column_queues']=delivery_mode=='staged'
    if lidar_map:
        grid['lidar_map']=True
        # AMCL's observed localization excursions approach 3 cm. The
        # simulation accepts a 3 cm map-frame stop and audits the physical
        # pickup/delivery error separately; the real robot keeps 2 cm.
        grid['position_tolerance_m']=.03
    # Estimated motion/congestion costs in seconds, only for the simulator.
    # These are planning weights, not experimental performance results.
    grid['route_time_costs']=dict(linear_speed_mps=.22,turn_start_s=1.8,quarter_turn_s=1.6,
        route_node_wait_s=.4,occupied_wait_s=3.,goal_wait_s=3.,opposite_wait_s=6.)
    # Scan returns from another simulated robot can remain on a local costmap
    # after that robot moves. Its recent poses are already tracked by /fleet/status.
    grid['robot_echo_history_s']=3.0
    if delivery_mode=='staged':
        grid['goal_approaches']=[[[1,c],[2,c]] for c in range(1,5)]
    if delivery_mode=='staged' and delivery_flow=='east':
        # Service row: enter from row 2, leave EAST, return north at column 5.
        # No waiting car may block this one-way exit corridor.
        grid['forbidden_directions']=[[[1,c+1],[1,c]] for c in range(1,5)]
        grid['forbidden_directions'] += [[[1,c],[2,c]] for c in range(1,5)]
    (out/'grid.yaml').write_text(yaml.safe_dump(grid))
    # A known empty room; conveyors are sensed through the nearest-return scan.
    orders=orders or parse_orders('',items)
    spans=[conveyor_geometry(len(v)) for v in orders.values()]
    right=max(5.5,1.2+max(center+length/2 for center,length,_ in spans)+.6)
    width=math.ceil((right+2.5)/.05)
    origin_y=min(-2.5,-1.2-2.1)
    height=math.ceil((2.5-origin_y)/.05)
    pixels=np.full((height,width),254,np.uint8);pixels[:2]=0;pixels[-2:]=0;pixels[:,:2]=0;pixels[:,-2:]=0
    if lidar_map:
        for x,y,sx,sy in LANDMARKS:
            left=max(0,int((x-sx/2+2.5)/.05));right_edge=min(width,int((x+sx/2+2.5)/.05)+1)
            low=max(0,int((y-sy/2-origin_y)/.05));high=min(height,int((y+sy/2-origin_y)/.05)+1)
            pixels[height-high:height-low,left:right_edge]=0
    (out/'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels.tobytes())
    (out/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,
        origin=[-2.5,origin_y,0.],negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')))
    board=Board.load(out/'grid.yaml')
    orders=orders or parse_orders('',items)
    lanes={n:Lane(n,i+2,board.xy(i+1,4),orders[n]) for i,n in enumerate(COLORS)}
    (out/'scenario.json').write_text(json.dumps(dict(orders=orders,delivery_mode=delivery_mode,
                                                   sync_rounds=sync_rounds,
                                                   delivery_flow=delivery_flow,
                                                   height_boundaries_cm=[22,29,36,43,50]),indent=2))
    build_world(board,lanes,out/'warehouse.sdf')
    prepare_rviz(out,WS/'tools/fleet.rviz')
    nav=yaml.safe_load((WS/'src/my_bot_bringup/config/navigation.yaml').read_text())
    def visit(x,n):
        if isinstance(x,dict):
            result={k:visit(v,n) for k,v in x.items()}
            if x.get('plugin')=='nav2_costmap_2d::StaticLayer':result['map_topic']='/map'
            if x.get('data_type')=='LaserScan':result['inf_is_valid']=True
            return result
        if isinstance(x,list):return [visit(v,n) for v in x]
        if isinstance(x,str):
            return {'/scan':f'/{n}/scan','/odom':f'/{n}/odom',
                    '/local_costmap/published_footprint':f'/{n}/local_costmap/published_footprint'}.get(x,x)
        return x
    for n in COLORS:
        cfg=visit(nav,n)
        # Virtual drivetrain parameters only; real-robot tuning is untouched.
        for k in ('FollowPath','PrecisionFollowPath'):
            cfg['controller_server']['ros__parameters'][k]['desired_linear_vel']=.22
        (out/f'navigation_{n}.yaml').write_text(yaml.safe_dump(cfg))
    return board


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('action',choices=('up','down','status','vision-check'),nargs='?',default='up')
    ap.add_argument('--headless',action='store_true');ap.add_argument('--autostart',action='store_true')
    ap.add_argument('--no-gazebo',action='store_true');ap.add_argument('--items',type=int,choices=range(1,5),default=0)
    ap.add_argument('--no-rviz',action='store_true',help='Do not open the RViz window')
    ap.add_argument('--out',default='')
    ap.add_argument('--orders',default='',help='Example: agv1:2;agv2:2;agv3:2')
    ap.add_argument('--delivery-mode',choices=('staged','serial'),default='staged')
    ap.add_argument('--scenario',choices=('mixed','demo15','demo30'),default='mixed')
    ap.add_argument('--sync-rounds',action='store_true')
    ap.add_argument('--delivery-flow',choices=('flexible','east'),default='flexible')
    ap.add_argument('--lidar-map',action='store_true',
                    help='Build /map from virtual LiDAR with slam_toolbox, then localize with AMCL')
    ap.add_argument('--exit-on-complete',action='store_true')
    a=ap.parse_args(); current=owner()
    if a.action=='down':
        if not current:print('Simulation is not running.');return
        try:os.kill(current['pid'],signal.SIGINT)
        except ProcessLookupError:pass
        print('Stopping this simulation only.',flush=True)
        for _ in range(100):
            if owner() is None:
                print('Simulation stopped.');return
            time.sleep(.1)
        print('Shutdown still in progress; wait before restarting.',file=sys.stderr)
        return 1
    if a.action=='status':print(json.dumps(current or {'running':False},indent=2));return
    if a.action=='vision-check':
        from warehouse_vision_check import run
        raise SystemExit(0 if run(a.out or '/tmp/warehouse_vision') else 1)
    if current:raise SystemExit(f"Already running: {current['out']}. Stop with tools/warehouse_sim.sh down")
    if os.environ.get('ROS_DOMAIN_ID')!='88':raise SystemExit('Use tools/warehouse_sim.sh (isolated ROS domain 88)')
    out=Path(a.out or str(Path.home()/'agv_logs'/time.strftime('warehouse_3agv_%Y%m%d_%H%M%S'))).resolve()
    if (out/'events.jsonl').exists():
        raise SystemExit('Output already contains a simulation run; choose a new --out directory.')
    if a.scenario in ('demo15','demo30'):
        if a.orders or a.items:ap.error(f'--scenario {a.scenario} already defines its packages; omit --items/--orders')
        scenario=DEMO_30 if a.scenario=='demo30' else DEMO_15
        a.orders=';'.join(n+':'+','.join(map(str,kinds)) for n,kinds in scenario.items())
    try:orders=parse_orders(a.orders,a.items)
    except ValueError as e:ap.error(str(e))
    if a.sync_rounds and len({len(v) for v in orders.values()})!=1:
        ap.error('--sync-rounds requires equal package counts on all three lanes')
    if a.sync_rounds and a.delivery_mode!='staged':
        ap.error('synchronized rounds require --delivery-mode staged')
    board=prepare(out,a.items,orders,a.delivery_mode,a.sync_rounds,a.delivery_flow,a.lidar_map)
    if a.lidar_map:
        # The geometric room is sensor truth only. Nav2 never receives this
        # file; its map will be saved from slam_toolbox's /map topic.
        (out/'map.pgm').rename(out/'truth_map.pgm')
        truth=yaml.safe_load((out/'map.yaml').read_text())
        truth['image']='truth_map.pgm'
        (out/'truth_map.yaml').write_text(yaml.safe_dump(truth))
        (out/'map.yaml').unlink()
    LOCK.write_text(json.dumps(dict(pid=os.getpid(),start=identity(os.getpid()),out=str(out))))
    children=[];logs=[];owned={}
    def remember_descendants():
        # Gazebo may create new process groups; a killpg(wrapper) is insufficient.
        # Remember descendants while their parent exists, with Linux start time
        # to prevent signalling an unrelated process after PID reuse.
        processes={}
        for f in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields=f.read_text().rsplit(')',1)[1].split()
                processes[int(f.parent.name)]=(int(fields[1]),fields[19])
            except (OSError,ValueError,IndexError):continue
        known={os.getpid(),*owned}
        for _ in range(10):
            added=False
            for pid,(parent,start) in processes.items():
                if parent in known and pid not in known:
                    known.add(pid);owned[pid]=start;added=True
            if not added:break
    def spawn(name,cmd,env=None):
        log=open(out/(name+'.log'),'w');logs.append(log)
        p=subprocess.Popen(cmd,cwd=WS,env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
        children.append((name,p));owned[p.pid]=identity(p.pid);return p
    def stop_child(name,p):
        try:os.killpg(p.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            p.wait()
        children.remove((name,p))
    def wait_nav(p):
        path=out/'nav_fleet.log'
        active=set()
        for attempt_second in range(180):
            remember_descendants()
            if p.poll() is not None:raise RuntimeError(f'Nav2 exited; see {path}')
            output=path.read_text()
            for n in COLORS:
                if n not in active and any(n in line and 'Managed nodes are active' in line
                                           for line in output.splitlines()):
                    active.add(n);print(f'Nav2 {n}: active',flush=True)
            if len(active)==len(COLORS):return
            if 'Failed to bring up all requested nodes' in output:
                raise RuntimeError(f'Nav2 lifecycle failed; see {path}')
            if attempt_second>=20 and 'process started' not in output:
                raise RuntimeError(f'Nav2 launch did not spawn nodes; see {path}')
            time.sleep(1)
        raise RuntimeError(f'Nav2 did not activate ({sorted(active)} active); see {path}')
    def wait_one_nav(name,p):
        path=out/f'nav_{name}.log'
        for second in range(90):
            remember_descendants()
            if p.poll() is not None:return False
            output=path.read_text()
            if 'Managed nodes are active' in output:return True
            if 'Failed to bring up all requested nodes' in output:return False
            if second>=10 and 'failed to send response to' in output:
                # The component load client has timed out. Recreate this
                # Nav2 container instead of waiting on a dead launch.
                return False
            time.sleep(1)
        return False
    def on_signal(sig,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,on_signal);signal.signal(signal.SIGTERM,on_signal)
    result=0
    try:
        print('Logs and evidence:',out,flush=True)
        if a.lidar_map:
            from warehouse_grid_from_map import place_grid
            x,y=board.xy(1,0)
            sensor_env=dict(os.environ,FAKE_MAP=str(out/'truth_map.yaml'),
                            FAKE_DELAY='.02',FAKE_TAU='.06',FAKE_DEAD_V='0',
                            FAKE_DEAD_W='0',FAKE_LIDAR_Z='.19',FAKE_RAYS='720',
                            FAKE_LIDAR_NOISE='0.001')
            mapper_robot=spawn('mapping_robot',[
                sys.executable,str(WS/'tools/fleet_fake_robot.py'),str(x),str(y),'0',
                '--no-map-tf'],sensor_env)
            slam=yaml.safe_load((WS/'src/my_bot_bringup/config/slam_async.yaml').read_text())
            slam['slam_toolbox']['ros__parameters']['use_sim_time']=False
            (out/'slam.yaml').write_text(yaml.safe_dump(slam))
            slam_process=spawn('slam',['ros2','launch','slam_toolbox','online_async_launch.py',
                                       f'slam_params_file:={out}/slam.yaml','use_sim_time:=false'])
            mapping=spawn('mapping',[sys.executable,str(WS/'tools/warehouse_lidar_mapping.py'),
                                      '--out',str(out)])
            while mapping.poll() is None:
                remember_descendants()
                if mapper_robot.poll() is not None or slam_process.poll() is not None:
                    raise RuntimeError('LiDAR mapping stopped early; see mapping_robot.log and slam.log')
                time.sleep(1)
            if mapping.returncode:
                raise RuntimeError('LiDAR mapping failed; see '+str(out/'mapping.log'))
            stop_child('mapping',mapping)
            stop_child('slam',slam_process)
            stop_child('mapping_robot',mapper_robot)
            board,alignment=place_grid(out/'map.yaml',out/'grid.yaml')
            lanes={n:Lane(n,i+2,board.xy(i+1,4),orders[n]) for i,n in enumerate(COLORS)}
            build_world(board,lanes,out/'warehouse.sdf')
            prepare_rviz(out,WS/'tools/fleet.rviz')
            print('LiDAR map saved; grid aligned to observed walls:',alignment,flush=True)
        spawn('map',['ros2','run','nav2_map_server','map_server','--ros-args','-p',f'yaml_filename:={out}/map.yaml'])
        spawn('map_lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
                               '-r','__node:=warehouse_map_lifecycle','-p',"node_names:=['map_server']",'-p','autostart:=true'])
        env=dict(os.environ,FAKE_MAP=str(out/('truth_map.yaml' if a.lidar_map else 'map.yaml')),
                 FAKE_DELAY='.02',FAKE_TAU='.06',
                 FAKE_DEAD_V='0',FAKE_DEAD_W='0',FAKE_LIDAR_Z='.19',FAKE_RAYS='720',FAKE_LIDAR_NOISE='0.001')
        for i,n in enumerate(COLORS):
            x,y=board.xy(i+1,0)
            others=','.join(f'/{o}/odom' for o in COLORS if o!=n)
            cmd=[sys.executable,str(WS/'tools/fleet_fake_robot.py'),str(x),str(y),'0',
                '--other',others]
            if a.lidar_map:cmd.append('--no-map-tf')
            cmd+=['--ros-args','-r',f'__ns:=/{n}','-r',f'/tf:=/{n}/tf',
                  '-r',f'/tf_static:=/{n}/tf_static']
            spawn('fake_'+n,cmd,env)
        if a.lidar_map:
            amcl=spawn('amcl',['ros2','launch',str(WS/'tools/warehouse_amcl.launch.py'),
                               f'config_dir:={out}'])
            for _ in range(90):
                remember_descendants()
                if amcl.poll() is not None:raise RuntimeError('AMCL exited; see '+str(out/'amcl.log'))
                if (out/'amcl.log').read_text().count('Managed nodes are active')>=3:break
                time.sleep(1)
            else:raise RuntimeError('Three AMCL instances did not activate; see '+str(out/'amcl.log'))
        print('Activating three Nav2 containers...',flush=True)
        if a.lidar_map:
            # AMCL adds three DDS participants. Starting one container at a
            # time avoids concurrent component-load service timeouts.
            for n in COLORS:
                for attempt in range(4):
                    p=spawn('nav_'+n,['ros2','launch',str(WS/'tools/fleet_nav.launch.py'),
                                     f'namespace:={n}',f'params_file:={out}/navigation_{n}.yaml'])
                    if wait_one_nav(n,p):
                        print(f'Nav2 {n}: active',flush=True)
                        break
                    stop_child('nav_'+n,p)
                else:raise RuntimeError(f'Nav2 {n} did not activate; see {out}/nav_{n}.log')
        else:
            p=spawn('nav_fleet',['ros2','launch',str(WS/'tools/warehouse_nav.launch.py'),
                                 f'config_dir:={out}'])
            wait_nav(p)
        for n in COLORS:
            spawn('agent_'+n,[sys.executable,str(WS/'fleet_agent.py'),n,'--grid',str(out/'grid.yaml'),
                 '--ros-args','-r',f'__ns:=/{n}','-r',f'/tf:=/{n}/tf','-r',f'/tf_static:=/{n}/tf_static'])
        spawn('traffic',[sys.executable,str(WS/'traffic_manager.py'),*COLORS,'--grid',str(out/'grid.yaml'),
                         '--no-stdin','--service-holds-topic','/warehouse/service_holds',
                         '--delivery-queues-topic','/warehouse/delivery_queues'])
        if not a.headless and not a.no_rviz:
            spawn('fleet_viz',[sys.executable,str(WS/'tools/fleet_viz.py'),*COLORS,'--nav'])
            spawn('rviz',['rviz2','-d',str(out/'warehouse.rviz')],rviz_environment(out))
        if not a.no_gazebo:
            spawn('gazebo',['gz','sim','-r','-s',str(out/'warehouse.sdf')])
            if not a.headless:
                spawn('gazebo_gui',['gz','sim','-g','--gui-config',str(out/'gui.config')])
        cmd=[sys.executable,str(WS/'tools/warehouse_scene.py'),'--out',str(out)]
        for key in ('headless','autostart','no_gazebo'):
            if getattr(a,key):cmd.append('--'+key.replace('_','-'))
        if a.items:cmd+=['--items',str(a.items)]
        cmd+=['--orders',a.orders,'--delivery-mode',a.delivery_mode]
        if a.sync_rounds:cmd.append('--sync-rounds')
        scene=spawn('scene',cmd)
        for _ in range(45):
            remember_descendants()
            if scene.poll() is not None:raise RuntimeError('Scene exited; see '+str(out/'scene.log'))
            events=out/'events.jsonl'
            if events.exists() and '"event": "telemetry_ready"' in events.read_text():break
            time.sleep(1)
        else:raise RuntimeError('Missing three robot telemetry streams; see '+str(out/'scene.log'))
        print('Ready. SPACE in camera window starts all 3 queues. Ctrl+C stops simulation.',flush=True)
        while True:
            time.sleep(1)
            remember_descendants()
            if a.exit_on_complete and (out/'summary.json').exists():
                try:summary=json.loads((out/'summary.json').read_text())
                except json.JSONDecodeError:summary={}
                if summary.get('failed'):raise RuntimeError('Mission failed; see '+str(out/'scene.log'))
                if summary.get('complete'):break
            for name,p in children:
                if p.poll() is not None:raise RuntimeError(f'{name} exited ({p.returncode}); see {out}/{name}.log')
    except KeyboardInterrupt:pass
    except RuntimeError as e:
        print(str(e),file=sys.stderr)
        result=1
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        remember_descendants()
        # Only process groups created by this launcher. No global pkill patterns.
        for name,p in reversed(children):
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for pid,start in owned.items():
            if identity(pid)==start:
                try:os.kill(pid,signal.SIGINT)
                except ProcessLookupError:pass
        time.sleep(2)
        for name,p in reversed(children):
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            p.wait()
        for pid,start in owned.items():
            if identity(pid)==start:
                try:os.kill(pid,signal.SIGKILL)
                except ProcessLookupError:pass
        for log in logs:log.close()
        LOCK.unlink(missing_ok=True)
        print('Simulation stopped. Evidence kept at',out,flush=True)
    return result


if __name__=='__main__':raise SystemExit(main())
