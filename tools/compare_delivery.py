#!/usr/bin/env python3
"""Compare two completed, identical-workload kinematic delivery runs."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from warehouse_report import report


def compare(serial,staged,out):
    serial,staged,out=Path(serial),Path(staged),Path(out)
    scenarios=[json.loads((folder/'scenario.json').read_text()) for folder in (serial,staged)]
    if scenarios[0]['delivery_mode']!='serial' or scenarios[1]['delivery_mode']!='staged':
        raise ValueError('Expected serial baseline, then staged candidate')
    for key in ('orders','height_boundaries_cm'):
        if scenarios[0][key]!=scenarios[1][key]:raise ValueError('Runs differ in '+key)
    audits=[]
    summaries=[]
    for folder in (serial,staged):
        if not report(folder):raise ValueError('Run incomplete, failed or violated invariants: '+str(folder))
        audits.append(json.loads((folder/'audit.json').read_text()))
        summaries.append(json.loads((folder/'summary.json').read_text()))
    old,new=[a['time_to_last_delivery_s'] for a in audits]
    gaps=[[gap['seconds'] for gap in a['bay_turnaround']] for a in audits]
    result=dict(scope='one pair of kinematic runs; not physical performance or a statistical estimate',
                workload=scenarios[0]['orders'],serial_log=str(serial),staged_log=str(staged),
                serial_time_to_last_delivery_s=old,staged_time_to_last_delivery_s=new,
                last_delivery_reduction_percent=round(100*(old-new)/old,2),
                serial_turnaround_s=gaps[0],staged_turnaround_s=gaps[1],
                staged_handoffs=audits[1]['handoffs'],
                minimum_center_separation_m=[s['min_robot_center_separation_m'] for s in summaries],
                proximity_violations=[s['proximity_violations'] for s in summaries])
    out.mkdir(parents=True,exist_ok=True)
    (out/'comparison.json').write_text(json.dumps(result,indent=2))
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    colors=['#9b9fa5','#267fbd']
    axes[0].bar(['Chờ tại ô lấy','Chờ gần ô giao'],[old,new],color=colors)
    for i,value in enumerate((old,new)):axes[0].text(i,value+1,f'{value:.1f} s',ha='center')
    axes[0].set(ylabel='Thời gian từ khi bắt đầu (s)',title='Giao xong kiện cuối')
    if len(gaps[0])==len(gaps[1]) and gaps[0]:
        x=np.arange(len(gaps[0]))
        axes[1].bar(x-.18,gaps[0],width=.36,label='Cách cũ',color=colors[0])
        axes[1].bar(x+.18,gaps[1],width=.36,label='Cách mới',color=colors[1])
        axes[1].set(xticks=x,xticklabels=[f'Lượt {i+1} → {i+2}' for i in x],ylabel='Thời gian (s)',
                    title='Xe trước giao xong → xe sau tới trạm')
        axes[1].legend()
    for ax in axes:ax.grid(axis='y',alpha=.2)
    fig.savefig(out/'comparison.png',dpi=180);plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('serial');ap.add_argument('staged');ap.add_argument('--out',required=True)
    args=ap.parse_args();compare(args.serial,args.staged,args.out)
