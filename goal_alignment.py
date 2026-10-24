"""Bounded straight correction of a small goal error, without another pivot.

Pure geometry used by fleet_agent; distances are metres. A differential-drive
robot cannot fix an arbitrary lateral error by moving straight. Reject that
case explicitly rather than silently widening the final position tolerance.
"""
import math
import numpy as np

POSITION_TOL = 0.02
ALIGN_STOP_RADIUS = 0.0185
MAX_CORRECTION = 0.08
MAX_LATERAL = 0.018
MAX_TRAVEL = 0.10
MAX_SPEED = 0.025
MIN_SPEED = 0.015


def body_error(pose, target):
    x, y, yaw = pose
    dx, dy = target[0]-x, target[1]-y
    return (dx*math.cos(yaw)+dy*math.sin(yaw),
            -dx*math.sin(yaw)+dy*math.cos(yaw))


def correction_plan(pose, target):
    """Return signed longitudinal distance and lateral error, or reject."""
    if not all(v is not None and math.isfinite(v) for v in (*pose, *target)):
        raise ValueError('khong co vi tri/goc hop le')
    along, across = body_error(pose, target)
    if math.hypot(along, across) > MAX_CORRECTION:
        raise ValueError('lech tam qua 8 cm - can dinh vi/di lai doan tiep can')
    if abs(across) > MAX_LATERAL:
        raise ValueError(f'lech ngang {abs(across)*100:.1f} cm - '
                         'khong the can tam bang tien/lui thang, khong xoay bu')
    return along, across


def swept_costmap_clear(msg, start, end, radius):
    """Require a fully known, obstacle-free swept footprint (either direction).

    Check raw occupancy, including unknown cells and other robots. Inflation
    values are not obstacles themselves; actual footprint + half-cell diagonal
    supplies the geometric clearance. No scan filtering or robot exemptions.
    """
    info = msg.info
    res, width, height = info.resolution, info.width, info.height
    if (msg.header.frame_id != 'map' or res <= 0 or width <= 0 or height <= 0
            or len(msg.data) != width*height):
        return False
    q = info.origin.orientation
    if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6:
        return False
    ox, oy = info.origin.position.x, info.origin.position.y
    pad = radius + res*math.sqrt(0.5)
    x0, x1 = min(start[0],end[0])-pad, max(start[0],end[0])+pad
    y0, y1 = min(start[1],end[1])-pad, max(start[1],end[1])+pad
    if x0 < ox or y0 < oy or x1 >= ox+width*res or y1 >= oy+height*res:
        return False
    cols = np.arange(math.floor((x0-ox)/res),math.floor((x1-ox)/res)+1)
    rows = np.arange(math.floor((y0-oy)/res),math.floor((y1-oy)/res)+1)
    xx, yy = np.meshgrid(ox+(cols+.5)*res,oy+(rows+.5)*res)
    vx, vy = end[0]-start[0], end[1]-start[1]
    length2 = vx*vx+vy*vy
    t = np.clip(((xx-start[0])*vx+(yy-start[1])*vy)/max(1e-12,length2),0.,1.)
    inside = (xx-start[0]-t*vx)**2+(yy-start[1]-t*vy)**2 <= pad*pad
    cost = np.asarray(msg.data).reshape(height,width)[np.ix_(rows,cols)]
    return not np.any(inside & ((cost < 0) | (cost >= 99)))
