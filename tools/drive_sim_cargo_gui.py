#!/usr/bin/env python3
"""Click the existing cargo GUI during a ROS_DOMAIN_ID=87 simulation trial.

The GUI still runs the normal observation, measurement, pickup and delivery
logic. This helper simulates an operator waiting a fixed time before confirming
loading/unloading, then starts one more cycle if requested. Never use on cars.
"""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path


def xdotool(*args):
    return subprocess.run(['xdotool', *map(str, args)], check=True,
                          capture_output=True, text=True).stdout


def click(window, car):
    try:
        xdotool('windowactivate', '--sync', window)
    except subprocess.CalledProcessError:
        xdotool('windowfocus', '--sync', window)
    geom = dict(line.split('=', 1) for line in
                xdotool('getwindowgeometry', '--shell', window).splitlines()
                if '=' in line)
    x = int(geom['X']) + (140 if car == 'agv1' else 475)
    # Qt's expanded window adds a toolbar/status bar around the image.
    # This falls inside the bottom button with either Qt window style.
    y = int(geom['Y']) + int(geom['HEIGHT']) - 50
    xdotool('mousemove', '--sync', x, y)
    xdotool('click', '--clearmodifiers', 1)


def events(path):
    if not path.exists():
        return {}
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    return {(int(r['cycle']), r['car'], r['event']): r for r in rows
            if r.get('cycle') is not None}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('event_log')
    ap.add_argument('--cycles', type=int, default=2)
    ap.add_argument('--load-wait-s', type=float, default=2.0)
    ap.add_argument('--unload-wait-s', type=float, default=2.0)
    ap.add_argument('--return-wait-s', type=float, default=2.0)
    ap.add_argument('--timeout-s', type=float, default=400.0)
    ap.add_argument('--window-pid', type=int, help='target only this simulation GUI process')
    ap.add_argument('--screenshots', type=Path, help='save screenshots at workflow milestones')
    args = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        ap.error('Only ROS_DOMAIN_ID=87 simulation is allowed')
    if args.cycles < 1 or min(args.load_wait_s, args.unload_wait_s,
                              args.return_wait_s) < 0:
        ap.error('Cycles must be positive and waits nonnegative')
    path = Path(args.event_log)
    audit = path.with_name('sim_operator_clicks.jsonl')
    search = ['search', '--all', '--onlyvisible']
    if args.window_pid:
        proc = Path(f'/proc/{args.window_pid}')
        cmdline = (proc/'cmdline').read_bytes().split(b'\0')
        environment = (proc/'environ').read_bytes().split(b'\0')
        if b'--sim' not in cmdline or b'ROS_DOMAIN_ID=87' not in environment:
            ap.error('Target process must be cargo_live_view --sim in domain 87')
        search.extend(['--pid', args.window_pid])
    windows = xdotool(*search, '--name', 'AGV cargo').split()
    if not windows:
        ap.error('Open tools/cargo_live_view.py --sim first')
    window = windows[-1]
    xdotool('windowmove', window, 990, 60)
    time.sleep(.25)

    def act(cycle, car, action):
        click(window, car)
        row = {'t_unix': time.time(), 't_monotonic': time.monotonic(),
               'cycle': cycle, 'car': car, 'action': action}
        with audit.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        print(f"{car} cycle {cycle}: {action}", flush=True)

    for car in ('agv1', 'agv2'):
        act(1, car, 'start')
    acted = set()
    photographed = set()
    if args.screenshots:
        args.screenshots.mkdir(parents=True, exist_ok=True)

    def photo(cycle, car, event):
        if not args.screenshots:
            return
        from PIL import ImageGrab
        xdotool('key', '--window', window, 's')
        time.sleep(.15)
        name = f'cycle{cycle}_{car}_{event}.png'
        ImageGrab.grab(xdisplay=os.environ['DISPLAY']).save(args.screenshots/name)
        with (args.screenshots/'screenshots.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(file=name, t_unix=time.time(), cycle=cycle,
                                         car=car, event=event))+'\n')
    deadline = time.monotonic() + args.timeout_s
    while time.monotonic() < deadline:
        rows = events(path)
        for cycle in range(1, args.cycles + 1):
            for car in ('agv1', 'agv2'):
                for event in ('observation_arrived', 'measured', 'pickup_arrived',
                              'loaded_confirmed', 'delivery_arrived', 'unloaded_confirmed'):
                    key = (cycle, car, event)
                    if key in rows and key not in photographed:
                        photo(cycle, car, event)
                        photographed.add(key)
                observed = rows.get((cycle,car,'observation_arrived'))
                preview = (cycle,car,'measurement_preview')
                if (observed and (cycle,car,'measured') not in rows and
                        time.monotonic()-observed['t_monotonic'] >= 1.2 and
                        preview not in photographed):
                    photo(cycle,car,'measurement_preview')
                    photographed.add(preview)
                pickup = rows.get((cycle, car, 'pickup_arrived'))
                loaded = rows.get((cycle, car, 'loaded_confirmed'))
                if (pickup and not loaded and (cycle, car, 'load') not in acted and
                        time.monotonic() - pickup['t_monotonic'] >= args.load_wait_s):
                    act(cycle, car, 'load')
                    acted.add((cycle, car, 'load'))
                delivered = rows.get((cycle, car, 'delivery_arrived'))
                unloaded = rows.get((cycle, car, 'unloaded_confirmed'))
                if (delivered and not unloaded and (cycle, car, 'unload') not in acted and
                        time.monotonic() - delivered['t_monotonic'] >= args.unload_wait_s):
                    act(cycle, car, 'unload')
                    acted.add((cycle, car, 'unload'))
            if cycle < args.cycles:
                both = [rows.get((cycle, car, 'unloaded_confirmed'))
                        for car in ('agv1', 'agv2')]
                if (all(both) and (cycle, 'return') not in acted and
                        time.monotonic() - max(r['t_monotonic'] for r in both) >=
                        args.return_wait_s):
                    for car in ('agv1', 'agv2'):
                        act(cycle + 1, car, 'return')
                    acted.add((cycle, 'return'))
        if all((args.cycles, car, 'unloaded_confirmed') in rows
               for car in ('agv1', 'agv2')):
            photo(args.cycles, 'both', 'completed')
            print('Both cars completed all simulated cycles', flush=True)
            return 0
        time.sleep(.2)
    print('Timed out; see cargo_events.jsonl for incomplete cycles', flush=True)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
