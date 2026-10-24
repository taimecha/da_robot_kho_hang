#!/usr/bin/env python3
"""Repeatable synthetic vision evaluation; no truth given to measure()."""
import argparse
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
cv2.setNumThreads(1)
from cargo_measure import measure
from warehouse_queue import Lane, SIZES, camera_config, classify_height, renderer, scan_boxes


def run(out):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    render = renderer()
    cases, tiles = [], []
    for front in SIZES:
        for rear in SIZES:
            for offset in (0., .025, -.025):
                lane = Lane('agv1', 2, (1.2, -.6), [front, rear])
                # Actual pose includes a small lateral positioning error.
                pose = (.6, -.6+offset, 0.)
                render.boxes = lane.items
                image = render.render(pose)
                scans = [scan_boxes(pose, lane.items)]*5
                row = dict(front=front, rear=rear, offset_m=offset)
                try:
                    result, vis = measure(image, scans, camera_config())
                    w, h, _ = SIZES[front]
                    row.update(width_m=result['width'], height_m=result['height'],
                               width_error_m=result['width']-w,
                               height_error_m=result['height']-h,
                               predicted=classify_height(result['height']))
                    row['passed'] = (row['predicted'] == front and
                                     abs(row['width_error_m']) < .015 and
                                     abs(row['height_error_m']) < .015)
                except ValueError as e:
                    row.update(passed=False, error=str(e)); vis = image
                cases.append(row)
                if offset == 0:
                    cv2.imwrite(str(out/f'front_{front}_rear_{rear}.png'), vis)
                    tile = cv2.resize(vis, (400, 300))
                    cv2.rectangle(tile, (0, 0), (400, 30), (25,25,25), -1)
                    cv2.putText(tile, f'Front T{front} / rear T{rear}  '+('PASS' if row['passed'] else 'REFUSE'),
                                (8,21), cv2.FONT_HERSHEY_SIMPLEX, .56, (80,240,120), 1)
                    tiles.append(tile)
    cv2.imwrite(str(out/'front_vs_rear_matrix.png'),
                np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,16,4)]))
    # No edge separating identical flat front faces: refusal is required.
    # Here a single larger silhouette conflicts with the nearer LiDAR width.
    render.boxes = Lane('agv1', 2, (1.2,-.6), [4]).items
    ambiguous = render.render((.6,-.6,0.))
    front_scans = [scan_boxes((.6,-.6,0.), Lane('agv1',2,(1.2,-.6),[1]).items)]*5
    try:
        measure(ambiguous, front_scans, camera_config())
        refused = False
    except ValueError:
        refused = True
    report = dict(kind='synthetic RGB + analytic LiDAR, NOT physical accuracy',
                  total=len(cases), passed=sum(c['passed'] for c in cases),
                  ambiguous_refused=refused, cases=cases)
    (out/'vision_results.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'cases'}, indent=2))
    return report['passed'] == len(cases) and refused


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--out', default='/tmp/warehouse_vision')
    args = ap.parse_args()
    raise SystemExit(0 if run(args.out) else 1)
