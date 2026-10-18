#!/usr/bin/env python3
"""Turn recorded ROS fleet data into publication-ready, traceable figures."""
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grid_board import Board, config_path  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'docs/figures/mo_phong_nckh'
DATA = json.loads((DEST / 'du_lieu_mo_phong.json').read_text())
BOARD = Board.load(config_path(True))
COL = {'agv1': '#1464d2', 'agv2': '#e37514'}
SCENES = [
    ('doi_cho', 'A. Đổi chỗ, tránh đối đầu', {'agv1': (3, 2), 'agv2': (3, 4)},
     {'agv1': (3, 4), 'agv2': (3, 2)}),
    ('giao_cat', 'B. Giao cắt tại ô (3,3)', {'agv1': (2, 3), 'agv2': (3, 2)},
     {'agv1': (4, 3), 'agv2': (3, 4)}),
    ('vat_can_tinh', 'C. Vật cản tĩnh tại (3,3)', {'agv1': (3, 1)},
     {'agv1': (3, 5)}),
    ('vat_can_dong', 'D. Vật cản xuất hiện rồi rời đi', {'agv1': (3, 1)},
     {'agv1': (3, 5)}),
]


def grid_xy(x, y):
    dx, dy = x - BOARD.center_x, y - BOARD.center_y
    u = dx * BOARD._cos + dy * BOARD._sin
    v = -dx * BOARD._sin + dy * BOARD._cos
    return (u / BOARD.spacing + BOARD.center_col,
            v / BOARD.spacing + BOARD.center_row)


def base(ax):
    ax.set_xlim(0.55, 5.45)
    ax.set_ylim(0.55, 5.45)
    ax.set_aspect('equal')
    ax.set_xticks(range(1, 6))
    ax.set_yticks(range(1, 6))
    ax.set_xlabel('Cột')
    ax.set_ylabel('Hàng')
    ax.grid(color='#d9e2ec', linewidth=1.1)
    for r in range(1, 6):
        for c in range(1, 6):
            ax.plot(c, r, '.', color='#92a6b9', markersize=7)
    ax.plot(5, 1, 's', color='#333333', markersize=13, zorder=6)
    ax.text(4.9, 1.12, 'ô cấm', ha='right', fontsize=8, color='#333333')


def samples(scene, name):
    return [s for s in DATA['samples'] if s['scene'] == scene and s['name'] == name
            and s.get('x') is not None and s.get('y') is not None]


def actual_plot(ax, scene, name, start, goal):
    ss = samples(scene, name)
    if ss:
        xy = [grid_xy(s['x'], s['y']) for s in ss]
        ax.plot([p[0] for p in xy], [p[1] for p in xy], color=COL[name],
                linewidth=3, alpha=.92, zorder=4, label=f'Quỹ đạo {name}')
    ax.scatter([start[1]], [start[0]], s=145, c=COL[name], edgecolors='white',
               linewidth=1.5, zorder=8)
    ax.scatter([goal[1]], [goal[0]], s=180, marker='*', c=COL[name],
               edgecolors='white', linewidth=1, zorder=8)


def planned_plot(ax, scene, name):
    rows = [c for c in DATA['commands'] if c['scene'] == scene and c['name'] == name
            and len(c.get('nodes', [])) > 1]
    seen = set()
    for c in rows:
        nodes = tuple(map(tuple, c['nodes']))
        if nodes in seen:
            continue
        seen.add(nodes)
        xx = [node[1] + 1 for node in nodes]
        yy = [node[0] + 1 for node in nodes]
        ax.plot(xx, yy, '--', color=COL[name], linewidth=1.4, alpha=.6,
                zorder=3, label=f'Tuyến giao {name}' if len(seen) == 1 else None)


def scene_panel(ax, spec):
    scene, title, starts, goals = spec
    base(ax)
    ax.set_title(title, fontsize=12, fontweight='bold')
    if scene == 'giao_cat':
        ax.add_patch(Circle((3, 3), .18, facecolor='#fce7b5',
                            edgecolor='#d39618', linewidth=1.5, zorder=2))
    if scene in ('vat_can_tinh', 'vat_can_dong'):
        ax.plot(3, 3, 'X', color='#be242b', markersize=17, zorder=9)
        ax.annotate('Vật cản', (3, 3), (3.25, 3.37), fontsize=9,
                    color='#a91d26', arrowprops={'arrowstyle': '-', 'color': '#a91d26'})
    for name in starts:
        planned_plot(ax, scene, name)
        actual_plot(ax, scene, name, starts[name], goals[name])


def distance_series(scene):
    ss = sorted([s for s in DATA['samples'] if s['scene'] == scene], key=lambda s: s['t'])
    latest = {}
    out = []
    t0 = next((e['t'] for e in DATA['events'] if e['scene'] == scene and
               e['event'] == 'goal'), ss[0]['t'] if ss else 0)
    for s in ss:
        latest[s['name']] = s
        if len(latest) != 2:
            continue
        a, b = latest['agv1'], latest['agv2']
        if max(s['t'] - a['t'], s['t'] - b['t']) > .5:
            continue
        out.append((s['t']-t0, math.hypot(a['x']-b['x'], a['y']-b['y'])))
    return out


def summarize():
    stats = {}
    for scene, _, starts, goals in SCENES:
        a = [e for e in DATA['events'] if e['scene'] == scene]
        start = next((e['t'] for e in a if e['event'] == 'goal'), None)
        end = next((e['t'] for e in a if e['event'] == 'arrived'), None)
        dist = distance_series(scene)
        changes = {}
        for name in starts:
            ids = {c['route_id'] for c in DATA['commands'] if c['scene'] == scene
                   and c['name'] == name and len(c.get('nodes', [])) > 1}
            changes[name] = len(ids)
        stats[scene] = {'duration_s': round(end-start, 1) if start and end else None,
                        'arrived': end is not None,
                        'min_center_distance_m': round(min(d for _, d in dist), 3)
                        if dist else None,
                        'route_assignments': changes,
                        'sample_count': sum(len(samples(scene, n)) for n in starts),
                        'start': starts, 'goal': goals}
    return stats


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'figure.facecolor': 'white'})
    fig, axs = plt.subplots(2, 2, figsize=(12.2, 11.9))
    for ax, spec in zip(axs.flat, SCENES):
        scene_panel(ax, spec)
    handles, labels = axs.flat[0].get_legend_handles_labels()
    fig.subplots_adjust(top=.92, bottom=.11, hspace=.28, wspace=.2)
    fig.legend(handles, labels, loc='lower center', ncol=4, fontsize=9,
               bbox_to_anchor=(.5, .013))
    fig.text(.5, .07, '● vị trí xuất phát     ★ ô đích     ┄ tuyến được giao     ━ quỹ đạo ghi từ /fleet/status',
             ha='center', fontsize=9, color='#43566b')
    fig.suptitle('Điều phối hai AGV trên lưới 5×5, ô 0,60 m',
                 fontsize=16, fontweight='bold')
    fig.savefig(DEST / 'ket_qua_bon_tinh_huong.png', dpi=220)

    fig, ax = plt.subplots(figsize=(10, 4.3), constrained_layout=True)
    for scene, label, _, _ in SCENES[:2]:
        vals = distance_series(scene)
        if vals:
            ax.plot([t for t, d in vals], [d for t, d in vals], linewidth=2,
                    label=label.split('. ', 1)[-1])
    ax.axhline(.18, color='#b63434', linestyle='--', linewidth=1.2,
               label='Tổng bán kính thân 2 xe (0,18 m)')
    ax.set(xlabel='Thời gian sau khi giao đích (s)',
           ylabel='Khoảng cách giữa hai tâm xe (m)',
           title='Khoảng cách hai xe trong hai ca có xung đột')
    ax.legend(frameon=False)
    ax.grid(color='#e5eaf0')
    fig.savefig(DEST / 'khoang_cach_hai_xe.png', dpi=220, bbox_inches='tight')

    stats = summarize()
    (DEST / 'so_lieu_tong_hop.json').write_text(
        json.dumps(stats, ensure_ascii=False, indent=2))
    print(json.dumps(stats, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
