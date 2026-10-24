#!/usr/bin/env python3
"""Figures and invariant audit from a recorded simulation, never invented data."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from grid_board import Board
sys.path.insert(0,str(Path(__file__).resolve().parent))
from warehouse_queue import round_coverage, conveyor_geometry, DECK_Z


def report(folder):
    folder=Path(folder)
    events=[json.loads(s) for s in (folder/'events.jsonl').read_text().splitlines()]
    column_mode=any(e['event']=='column_join' for e in events)
    last_kind={}; picked={}; delivered={}; loads={}; loaded_at={}; issues=[];cleared=set()
    bay_owner={}; request_queue={}; handoffs=[]; last_release={}; arrivals={}; queued_at={}
    for e in events:
        n=e.get('robot');kind=e['event']
        if kind=='vision_locked':
            last_kind[n]=e['kind']
            if e['kind']!=e['truth_kind']:issues.append('wrong class: '+e['package'])
        elif kind=='load_started':
            picked.setdefault(n,[]).append(e['package'])
            loads[n]=False
            loaded_at.pop(n,None)
        elif kind=='loaded':loads[n]=True;loaded_at[n]=e['t_s']
        elif kind=='feed_started':
            if not loads.get(n) or e['distance_from_pickup_m']<=.42:
                issues.append('feeder advanced before loaded/departed: '+str(e))
        elif kind=='delivery_success':
            delivered.setdefault(n,[]).append(e['package'])
            if e['slot']!=last_kind[n]:issues.append('wrong destination: '+e['package'])
            if 'receiving_position_m' in e and abs(e['receiving_position_m'][2]-DECK_Z)>1e-6:
                issues.append('carton stacked instead of received on belt: '+e['package'])
        elif kind=='outbound_cleared':
            if e['package'] in cleared:issues.append('outbound clearance recorded twice: '+e['package'])
            if not any(e['package'] in ids for ids in delivered.values()):
                issues.append('outbound clearance before delivery: '+e['package'])
            cleared.add(e['package'])
        elif kind=='delivery_queued':
            request_queue.setdefault(e['slot'],[]).append(n)
            queued_at[n]=e['t_s']
        elif kind=='delivery_admitted':
            if column_mode and not loads.get(n):issues.append('unloaded vehicle admitted to delivery bay: '+n)
            slot=e['slot'];waiting=request_queue.setdefault(slot,[])
            if bay_owner.get(slot):issues.append('two owners of delivery bay: '+str(slot))
            if not waiting or waiting.pop(0)!=n:issues.append('delivery queue overtaken: '+str(slot))
            bay_owner[slot]=n
            if slot in last_release:
                ready=max(queued_at[n],loaded_at.get(n,queued_at[n])) if column_mode else queued_at[n]
                handoffs.append(dict(slot=slot,previous=last_release[slot]['robot'],next=n,
                                     delay_s=round(e['t_s']-last_release[slot]['t_s'],3),
                                     waiting_at_release=ready<=last_release[slot]['t_s'],
                                     ready_to_admit_delay_s=round(e['t_s']-max(ready,last_release[slot]['t_s']),3)))
        elif kind=='goal_sent' and column_mode:
            if e['phase']=='staging':issues.append('dispersed waiting goal in column queue: '+n)
            if e['phase'] in ('convoy','deliver') and (e['row']!=1 or e['col']!=last_kind.get(n)):
                issues.append('column queue changed delivery destination: '+n)
        elif kind=='bay_released':
            slot=e['slot']
            if bay_owner.get(slot)!=n:issues.append('non owner released bay: '+str(slot))
            if e['distance_m']<=.45:issues.append('delivery bay released before safe clearance')
            bay_owner.pop(slot,None);last_release[slot]=e
        if kind=='delivery_arrived':
            slot=e['slot']
            mode=next(a.get('delivery_mode','serial') for a in events if a['event']=='scene_ready')
            if mode=='staged' and bay_owner.get(slot)!=n:issues.append('unadmitted delivery arrival: '+n)
            arrivals.setdefault(slot,[]).append(e)
    for n,ids in picked.items():
        expected=[f'{n}_box_{i+1}' for i in range(len(ids))]
        if ids!=expected:issues.append('non FIFO pickup: '+n)
        if delivered.get(n,[])!=ids[:len(delivered.get(n,[]))]:issues.append('non FIFO delivery: '+n)
    complete=any(e['event']=='mission_complete' for e in events)
    queues=next(e['queues'] for e in events if e['event']=='scene_ready')
    if complete:
        for n,order in queues.items():
            if len(delivered.get(n,[]))!=len(order):issues.append('missing deliveries: '+n)
            advances=sum(e['event']=='feed_started' and e.get('robot')==n for e in events)
            if advances!=len(order)-1:issues.append('wrong number of feeder advances: '+n)
    failed=any(e['event']=='mission_failed' for e in events)
    rounds=[e for e in events if e['event']=='round_ready']
    completed_rounds=[]
    for e in rounds:
        if not all(any(s['event']=='delivery_success' and s.get('robot')==n and
                       s.get('package')==package and s['slot']==e['kinds'][n]
                       for s in events) for n,package in e['packages'].items()):break
        completed_rounds.append(e['kinds'])
    coverage=round_coverage(completed_rounds) if rounds else None
    synced=next(e.get('sync_rounds',False) for e in events if e['event']=='scene_ready')
    if synced:
        for i,e in enumerate(rounds,1):
            if e['round']!=i:issues.append('non consecutive round number')
            if i>1:
                previous=rounds[i-2]
                if any(not any(s['event']=='delivery_success' and s.get('package')==package
                               and s['t_s']<e['t_s'] for s in events)
                       for package in previous['packages'].values()):
                    issues.append('new round admitted before previous deliveries completed')
            if complete:
                for n,package in e['packages'].items():
                    if not any(s['event']=='delivery_success' and s.get('robot')==n and
                               s.get('package')==package and s['slot']==e['kinds'][n]
                               for s in events):issues.append('round package not delivered: '+package)
        if complete and len(rounds)!=len(next(iter(queues.values()))):
            issues.append('missing synchronized rounds')
    start=next((e['t_s'] for e in events if e['event']=='mission_started'),None)
    finish=next((e['t_s'] for e in events if e['event']=='mission_complete'),None)
    last_delivery=max((e['t_s'] for e in events if e['event']=='delivery_success'),default=None)
    downtime=[]
    for slot,visits in arrivals.items():
        for previous,nxt in zip(visits,visits[1:]):
            success=next(e for e in events if e['event']=='delivery_success' and
                         e.get('package')==previous['package'])
            downtime.append(dict(slot=slot,previous=previous['robot'],next=nxt['robot'],
                                 seconds=round(nxt['t_s']-success['t_s'],3)))
    audit=dict(complete=complete,failed=failed,invariant_issues=issues,
               mission_duration_s=round(finish-start,3) if finish is not None and start is not None else None,
               picked=picked,delivered=delivered,
               synchronized_round_coverage=coverage,
               delivery_mode=next(e.get('delivery_mode','serial') for e in events if e['event']=='scene_ready'),
               time_to_last_delivery_s=round(last_delivery-start,3) if last_delivery is not None and start is not None else None,
               handoffs=handoffs,bay_turnaround=downtime,
               measurement_count=sum(e['event']=='vision_locked' for e in events),
               feeder_advances=sum(e['event']=='feed_started' for e in events))
    audit['outbound_cleared_count']=len(cleared)
    audit['synchronized_rounds']=synced
    audit['approach_stop_goal_count']=sum(e['event']=='goal_sent' and e.get('phase')=='approach' for e in events)
    audit['column_queues']=column_mode
    audit['column_join_count']=sum(e['event']=='column_join' for e in events)
    colors={'agv1':'#2674e5','agv2':'#e88a16','agv3':'#279c50'}
    with open(folder/'trajectories.csv') as f:rows=list(csv.DictReader(f))
    board=Board.load(folder/'grid.yaml')
    # Odom samples measure the travel and turning actually executed by the
    # simulated fleet, including avoidance maneuvers and final alignment.
    motion={}
    for n in ('agv1','agv2','agv3'):
        samples=[r for r in rows if r['robot']==n and
                 (start is None or float(r['t_s'])>=start) and
                 (finish is None or float(r['t_s'])<=finish)]
        distance=rotation=0.
        for previous,current in zip(samples,samples[1:]):
            distance+=math.hypot(float(current['x_m'])-float(previous['x_m']),
                                 float(current['y_m'])-float(previous['y_m']))
            change=float(current['yaw_rad'])-float(previous['yaw_rad'])
            rotation+=abs(math.atan2(math.sin(change),math.cos(change)))
        motion[n]=dict(distance_m=round(distance,3),
                       absolute_rotation_deg=round(math.degrees(rotation),1))
    audit['robot_motion']=motion
    audit['intermediate_exit_goal_count']=sum(e['event']=='goal_sent' and e.get('phase')=='exiting' for e in events)
    audit['planning_time_weights']=board.cfg.get('route_time_costs')
    exit_samples=[]
    for n in colors:
        samples=[r for r in rows if r['robot']==n]
        times=[float(r['t_s']) for r in samples]
        for e in events:
            if e['event']!='bay_released' or e.get('robot')!=n or not times:continue
            i=bisect.bisect_right(times,e['t_s'])-1
            if i<0:continue
            p=samples[i];x,y=board.xy(0,e['slot']-1)
            dx,dy=float(p['x_m'])-x,float(p['y_m'])-y
            along=dx*np.cos(board.yaw)+dy*np.sin(board.yaw)
            north=-dx*np.sin(board.yaw)+dy*np.cos(board.yaw)
            if np.hypot(dx,dy)<.25:continue
            direction=('east' if along>0 else 'west') if abs(along)>=abs(north) else ('north' if north>0 else 'south')
            exit_samples.append(dict(robot=n,slot=e['slot'],release_t_s=e['t_s'],
                                     sample_t_s=float(p['t_s']),direction=direction,x_m=float(p['x_m']),y_m=float(p['y_m'])))
    audit['departure_directions']={d:sum(e['direction']==d for e in exit_samples) for d in ('east','west','north','south')}
    audit['departure_samples']=exit_samples
    (folder/'audit.json').write_text(json.dumps(audit,indent=2))
    fig,(ax,timeline)=plt.subplots(1,2,figsize=(13,5.5),layout='constrained',
                                  gridspec_kw={'width_ratios':[1,1.3]})
    for a,b in board.edges():
        x,y=board.xy(*a);xx,yy=board.xy(*b)
        ax.plot([x,xx],[y,yy],color='.85',lw=1,zorder=0)
    for r,c in board.nodes():
        x,y=board.xy(r,c);ax.text(x,y,f'{r+1},{c+1}',fontsize=7,ha='center',va='bottom')
    for i,(n,color) in enumerate(colors.items()):
        samples=[r for r in rows if r['robot']==n]
        if samples:
            xy=np.array([[float(r['x_m']),float(r['y_m'])] for r in samples])
            ax.plot(xy[:,0],xy[:,1],color=color,lw=1.4,alpha=.8,label=n.upper())
            ax.plot(*xy[0],'o',color=color);ax.plot(*xy[-1],'s',color=color)
        ev=[e for e in events if e.get('robot')==n]
        for j,e in enumerate(ev):
            if e['event']!='load_started':continue
            end=next((a for a in ev[j+1:] if a['event']=='delivery_success'),None)
            if end:
                timeline.barh(i,end['t_s']-e['t_s'],left=e['t_s'],height=.42,color=color,alpha=.8)
                timeline.text((e['t_s']+end['t_s'])/2,i,
                              e['package'].rsplit('_',1)[-1],color='white',ha='center',va='center',fontsize=9)
        for e in ev:
            if e['event']=='feed_started':timeline.plot(e['t_s'],i-.3,'v',color=color,ms=5)
    for i in range(4):
        x,y=board.xy(0,i);ax.scatter(x,y,marker='s',s=130,color='#a5d6e8',zorder=0)
    for i,n in enumerate(colors):
        x,y=board.xy(i+1,4)
        center,length,_=conveyor_geometry(len(queues[n]))
        ax.add_patch(plt.Rectangle((x+center-length/2,y-.2),length,.4,fill=False,color='.3'))
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Quỹ đạo mô phỏng (odometry lý tưởng)')
    ax.set_aspect('equal');ax.legend(loc='upper left',fontsize=8)
    timeline.set(yticks=[0,1,2],yticklabels=['AGV1','AGV2','AGV3'],xlabel='Thời gian từ khi mở cảnh (s)',
                 title='Nạp → giao thành công; tam giác: băng tải tiến')
    timeline.grid(axis='x',alpha=.2)
    fig.savefig(folder/'trajectories_and_deliveries.png',dpi=180)
    plt.close(fig)
    if downtime:
        fig,ax=plt.subplots(figsize=(7,3.6),layout='constrained')
        ax.bar(range(len(downtime)),[x['seconds'] for x in downtime],color='#3279b8')
        ax.set(xticks=range(len(downtime)),
               xticklabels=[f"Ô (1,{x['slot']})\n{x['previous']} → {x['next']}" for x in downtime],
               ylabel='Thời gian (s)',title='Từ xe trước giao xong đến xe sau tới ô giao')
        ax.grid(axis='y',alpha=.2);fig.savefig(folder/'bay_turnaround.png',dpi=180);plt.close(fig)
    locks=[e for e in events if e['event']=='vision_locked']
    if locks:
        fig,axes=plt.subplots(1,2,figsize=(10,3.8),layout='constrained')
        x=np.arange(len(locks))
        for ax,dim in zip(axes,('width','height')):
            err=np.array([e[dim+'_m']-e['truth_'+dim+'_m'] for e in locks])*1000
            ax.bar(x,err,color=[colors[e['robot']] for e in locks])
            ax.axhline(0,color='.3',lw=.8)
            title='Chiều rộng' if dim=='width' else 'Chiều cao'
            ax.set(xlabel='Lần đo kiện hàng',ylabel='Sai số (mm)',title=title+' — so với cảnh tổng hợp')
            ax.grid(axis='y',alpha=.2)
        fig.savefig(folder/'synthetic_measurement_errors.png',dpi=180);plt.close(fig)
    print(json.dumps(audit,indent=2))
    return complete and not failed and not issues


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('folder');a=ap.parse_args()
    raise SystemExit(0 if report(a.folder) else 1)
