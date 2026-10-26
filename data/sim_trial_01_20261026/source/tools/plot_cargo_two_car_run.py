#!/usr/bin/env python3
"""Draw traceable figures for a two-car pickup/delivery simulation run.

Input is the JSONL produced by cargo_live_view.py and record_fleet_trace.py.
The script does not interpolate missing event times or invent real-car results.
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grid_board import Board, config_path  # noqa: E402
from analyze_cargo_timing import read_events, summarize  # noqa: E402

CAR_COLORS = {'agv1': '#1764c0', 'agv2': '#e2781d'}
PHASES = (
    ('started', 'observation_arrived', 'Tới ô quan sát', '#4c78a8'),
    ('observation_arrived', 'measured', 'Đo 5 ảnh', '#79b6c9'),
    ('measured', 'pickup_arrived', 'Tới ô lấy', '#72a86a'),
    ('pickup_arrived', 'loaded_confirmed', 'Chờ xác nhận chất', '#c7a34a'),
    ('loaded_confirmed', 'delivery_arrived', 'Chạy tới ô giao', '#c96b56'),
)


def read_trace(path):
    by_car = defaultdict(list)
    commands = defaultdict(list)
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        name = row.get('name')
        if name not in CAR_COLORS:
            continue
        if row['kind'] == 'status' and row.get('x') is not None and row.get('y') is not None:
            by_car[name].append(row)
        elif row['kind'] == 'command':
            commands[name].append(row)
    for rows in by_car.values():
        rows.sort(key=lambda d: d['t_monotonic'])
    return by_car, commands


def grid_xy(board, x, y):
    dx, dy = x - board.center_x, y - board.center_y
    u = dx * board._cos + dy * board._sin
    v = -dx * board._sin + dy * board._cos
    return (u / board.spacing + board.center_col,
            v / board.spacing + board.center_row)


def stages(events):
    grouped = defaultdict(dict)
    for event in events:
        if event.get('cycle') is not None:
            grouped[(event['car'], int(event['cycle']))].setdefault(event['event'], event)
    return grouped


def clip_trace(rows, first, last):
    return [d for d in rows if first - 0.5 <= d['t_monotonic'] <= last + 0.5]


def path_length(rows):
    return sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
               for a, b in zip(rows, rows[1:])
               if b['t_monotonic'] - a['t_monotonic'] <= 1.5)


def distance_series(by_car, first, last):
    a = clip_trace(by_car['agv1'], first, last)
    b = clip_trace(by_car['agv2'], first, last)
    if not a or not b:
        return np.array([]), np.array([])
    ta = np.array([r['t_monotonic'] for r in a])
    tb = np.array([r['t_monotonic'] for r in b])
    times = np.union1d(ta, tb)
    times = times[(times >= max(ta[0], tb[0], first)) &
                  (times <= min(ta[-1], tb[-1], last))]
    if not len(times):
        return np.array([]), np.array([])
    xa = np.interp(times, ta, [r['x'] for r in a])
    ya = np.interp(times, ta, [r['y'] for r in a])
    xb = np.interp(times, tb, [r['x'] for r in b])
    yb = np.interp(times, tb, [r['y'] for r in b])
    return times - first, np.hypot(xa - xb, ya - yb)


def plot_routes(out, board, by_car, first, last, grouped, truth):
    fig, ax = plt.subplots(figsize=(8.6, 7.6), constrained_layout=True)
    ax.set_xlim(.55, 6.05)
    ax.set_ylim(.55, 5.45)
    ax.set_aspect('equal')
    ax.set_xticks(range(1, 6))
    ax.set_yticks(range(1, 6))
    ax.set_xlabel('Cột')
    ax.set_ylabel('Hàng')
    ax.grid(color='#dce3eb', lw=1)
    for r in range(1, 6):
        for c in range(1, 6):
            ax.plot(c, r, '.', color='#8298ad', ms=5, zorder=1)
            ax.text(c + .055, r + .065, f'({r},{c})', fontsize=7, color='#698097')
    ax.plot(5, 1, 's', ms=12, color='#444', label='Ô cấm (1,5)')
    for car, r in (('agv1',3), ('agv2',4)):
        w,h = truth[car]
        label = f'{w*100:g}×{h*100:g} cm'
        ax.plot(5.65, r, 's', ms=22, color='#bd7b42', alpha=.9)
        ax.text(5.38, r + .20, label, fontsize=8, color='#8b4e21')
    for car in ('agv1', 'agv2'):
        rows = clip_trace(by_car[car], first, last)
        xy = [grid_xy(board, d['x'], d['y']) for d in rows]
        if xy:
            ax.plot([v[0] for v in xy], [v[1] for v in xy],
                    color=CAR_COLORS[car], lw=2.3, alpha=.9,
                    label=f'Quỹ đạo {car}')
            ax.plot(xy[0][0], xy[0][1], 'o', ms=10, color=CAR_COLORS[car],
                    mec='white', mew=1.2)
            ax.plot(xy[-1][0], xy[-1][1], '*', ms=15, color=CAR_COLORS[car],
                    mec='white', mew=.7)
    for car, r in (('agv1', 3), ('agv2', 4)):
        ax.plot(4, r, 'D', ms=7, color=CAR_COLORS[car], mfc='white', mew=1.4)
        ax.plot(5, r, 's', ms=8, color=CAR_COLORS[car], mfc='white', mew=1.4)
    ax.set_title('Hai AGV: quỹ đạo ghi từ /fleet/status', fontweight='bold')
    ax.legend(loc='upper left', bbox_to_anchor=(0, -.08), ncol=3, frameon=False,
              fontsize=9)
    fig.savefig(out / 'quy_dao_2_xe.png', dpi=200)
    plt.close(fig)


def plot_timeline(out, grouped):
    keys = sorted(grouped, key=lambda k: (k[1], k[0]))
    fig, ax = plt.subplots(figsize=(10.5, max(3.5, 1.0 * len(keys) + 1.4)),
                           constrained_layout=True)
    for i, key in enumerate(keys):
        m = grouped[key]
        left = 0.0
        for start, end, label, color in PHASES:
            if start not in m or end not in m:
                continue
            duration = m[end]['t_monotonic'] - m[start]['t_monotonic']
            if duration < 0:
                continue
            ax.barh(i, duration, left=left, color=color, height=.62,
                    label=label if i == 0 else None)
            if duration >= 4:
                ax.text(left + duration / 2, i, f'{duration:.1f}s',
                        ha='center', va='center', fontsize=8, color='#17212b')
            left += duration
    ax.set_yticks(range(len(keys)), [f'Xe {car[-1]} · lượt {cycle}' for car, cycle in keys])
    ax.invert_yaxis()
    ax.set_xlabel('Giây kể từ khi bấm BẮT ĐẦU')
    ax.set_title('Thời gian từng bước lấy và giao hàng', fontweight='bold')
    ax.grid(axis='x', color='#e3e8ef')
    ax.set_axisbelow(True)
    ax.legend(loc='upper center', bbox_to_anchor=(.5, -.15), ncol=3,
              frameon=False, fontsize=8)
    fig.savefig(out / 'thoi_gian_lay_giao.png', dpi=200)
    plt.close(fig)


def plot_clearance(out, by_car, first, last):
    t, dist = distance_series(by_car, first, last)
    if not len(t):
        return None
    fig, ax = plt.subplots(figsize=(9.7, 4), constrained_layout=True)
    ax.plot(t, dist, color='#344f82', lw=1.8, label='Khoảng cách tâm (nội suy mẫu trạng thái)')
    ax.axhline(.26, color='#b93635', ls='--', lw=1.3,
               label='Tổng bán kính thân 2 xe: 0,26 m')
    ax.set(xlabel='Giây từ lúc bắt đầu lượt đầu', ylabel='Mét',
           title='Khoảng cách tâm hai xe trong lượt thử')
    ax.grid(color='#e3e8ef')
    ax.legend(frameon=False)
    fig.savefig(out / 'khoang_cach_2_xe.png', dpi=200)
    plt.close(fig)
    return float(dist.min())


def plot_measurements(out, grouped, truth):
    vals = []
    for car in ('agv1', 'agv2'):
        for cycle in sorted(c for n, c in grouped if n == car):
            row = grouped[(car, cycle)].get('measured')
            if row:
                vals.append((car, cycle, row['width_m'], row['height_m'], *truth[car]))
    if not vals:
        return
    fig, ax = plt.subplots(figsize=(8.5, 4.2), constrained_layout=True)
    x = np.arange(len(vals))
    width_error = [(r[2] - r[4]) * 100 for r in vals]
    height_error = [(r[3] - r[5]) * 100 for r in vals]
    ax.bar(x - .18, width_error, .36, color='#4c78a8', label='Rộng')
    ax.bar(x + .18, height_error, .36, color='#e2781d', label='Cao')
    ax.axhline(0, color='#444', lw=.8)
    ax.set_xticks(x, [f'Xe {r[0][-1]} · lượt {r[1]}' for r in vals])
    ax.set_ylabel('Sai số so với kích thước hộp ảo (cm)')
    ax.set_title('Sai số phép đo 5 ảnh trong mô phỏng', fontweight='bold')
    ax.grid(axis='y', color='#e3e8ef')
    ax.legend(frameon=False)
    fig.savefig(out / 'sai_so_do_hop_ao.png', dpi=200)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('events')
    ap.add_argument('trace')
    ap.add_argument('out_dir')
    ap.add_argument('--scenario', type=Path, help='scenario.json containing true box dimensions')
    args = ap.parse_args()
    truth = {'agv1': (.15,.32), 'agv2': (.17,.40)}
    if args.scenario:
        boxes = json.loads(args.scenario.read_text())['boxes']
        truth = {car:(b['width_m'],b['height_m']) for car,b in boxes.items()}
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    events = read_events(args.events)
    rows, _ = summarize(events, 'sim')
    grouped = stages(events)
    by_car, commands = read_trace(args.trace)
    if not grouped or any(not by_car[car] for car in CAR_COLORS):
        raise ValueError('Need cargo events and sampled status for both cars')
    first = min(v['started']['t_monotonic'] for v in grouped.values() if 'started' in v)
    last = max((v['delivery_arrived']['t_monotonic'] for v in grouped.values()
                if 'delivery_arrived' in v), default=max(d['t_monotonic'] for d in events))
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'figure.facecolor': 'white'})
    board = Board.load(config_path(True))
    plot_routes(out, board, by_car, first, last, grouped, truth)
    plot_timeline(out, grouped)
    min_distance = plot_clearance(out, by_car, first, last)
    plot_measurements(out, grouped, truth)
    summary = {
        'source_events': args.events,
        'source_trace': args.trace,
        'sampled_min_center_distance_m': round(min_distance, 3) if min_distance is not None else None,
        'status_sample_counts': {car: len(clip_trace(by_car[car], first, last))
                                 for car in CAR_COLORS},
        'path_length_m': {car: round(path_length(clip_trace(by_car[car], first, last)), 3)
                          for car in CAR_COLORS},
        'unique_route_ids': {car: len({d['command'].get('route_id') for d in commands[car]
                                       if d['command'].get('nodes')}) for car in CAR_COLORS},
        'cycles': rows,
    }
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2),
                                      encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
