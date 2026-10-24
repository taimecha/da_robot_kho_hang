"""Independent FIFO feeder mechanics. Metres, seconds; no ROS or image labels.

Ground truth belongs to the scene/evaluator. The detector receives RGB, scans
and calibration only, never this module's type IDs or package dimensions.
"""
import math
from dataclasses import dataclass, field

SIZES = {1: (.12, .27, .14), 2: (.185, .34, .18),
         3: (.24, .40, .22), 4: (.30, .47, .26)}  # width, height, depth
ORDERS = {'agv1': [1, 4, 2, 3], 'agv2': [2, 4, 1, 3],
          'agv3': [1, 3, 4, 2]}
DEMO_15 = {'agv1': [1, 4, 2, 3, 1], 'agv2': [1, 4, 2, 2, 2],
           'agv3': [1, 4, 3, 3, 2]}
# Rounds 6..10: three all-distinct cohorts and two same-class pairs.
DEMO_30 = {n: DEMO_15[n] + tail for n,tail in {
    'agv1': [2, 4, 3, 4, 1], 'agv2': [3, 1, 1, 2, 3],
    'agv3': [4, 2, 3, 1, 3]}.items()}
COLORS = {'agv1': '0.15 0.45 1.0', 'agv2': '1.0 0.5 0.0',
          'agv3': '0.2 0.75 0.3'}
DECK_Z, TRAY_Z, PITCH, FRONT_GAP = .06, .16, .47, .35
LOAD_SECONDS = 2.
PUSHER_HALF_X, PUSHER_HALF_Z = .0125, .0125
PUSHER_DOWN_Z = .015


def smooth_step(value):
    u=min(1.,max(0.,value))
    return u*u*(3-2*u)


def conveyor_geometry(count):
    """Rail and roller span enclosing all initially queued cartons."""
    end = max(1.9, FRONT_GAP + max(0, count-1)*PITCH +
              max(size[2] for size in SIZES.values()) + .10)
    rollers = math.ceil((end-.36)/.08)+1
    end = .36+(rollers-1)*.08+.04
    return (.30+end)/2, end-.30, rollers


def round_coverage(rounds):
    """Coverage of observed synchronized cohorts (or a proposed input plan)."""
    from itertools import combinations
    triples=[]; pairs={f'{a}+{b}':[] for a,b in combinations(COLORS,2)}
    for i, kinds in enumerate(rounds,1):
        if set(kinds)!=set(COLORS):
            raise ValueError('round must contain exactly agv1, agv2 and agv3')
        if len(set(kinds.values()))==1:triples.append(i)
        else:
            for a,b in combinations(COLORS,2):
                if kinds[a]==kinds[b]:pairs[f'{a}+{b}'].append(i)
    first=rounds[:5]
    prefix_ok=(len(first)==5 and sum(len(set(r.values()))==1 for r in first)==2 and
               all(any(r[a]==r[b] and len(set(r.values()))==2 for r in first)
                   for a,b in combinations(COLORS,2)))
    tail=rounds[5:]
    return dict(triple_rounds=triples, pair_rounds=pairs,
                distinct_rounds=[i for i,r in enumerate(rounds,1) if len(set(r.values()))==3],
                meets_demo_15=len(rounds)==5 and prefix_ok,
                meets_demo_30=(len(rounds)==10 and prefix_ok and
                               sum(len(set(r.values()))<3 for r in tail) in (1,2) and
                               sum(len(set(r.values()))==3 for r in tail)>=3))


def parse_orders(text, items=0):
    """Optional scenario input e.g. agv1:2;agv2:2;agv3:2 (never a detector input)."""
    orders={n:list(values) for n,values in ORDERS.items()}
    seen=set()
    for entry in text.split(';'):
        if not entry.strip():continue
        name,separator,values=entry.strip().partition(':')
        if not separator or name not in orders or name in seen:
            raise ValueError('orders format: agv1:1,4;agv2:2;agv3:3')
        try:values=[int(v.strip()) for v in values.split(',')]
        except ValueError:raise ValueError('package types must be integers 1..4') from None
        if not values or any(v not in SIZES for v in values):
            raise ValueError('package types must be integers 1..4')
        orders[name]=values;seen.add(name)
    return {n:values[:items] if items else values for n,values in orders.items()}


def classify_height(height):
    if not .22 <= height <= .50:
        raise ValueError('height outside calibrated simulation range')
    # Four equal 7 cm bins. Compare boundaries directly so 29/36/43 cm
    # enter the upper bin without floating-point division rounding down.
    return int(1+sum(height >= boundary for boundary in (.29, .36, .43)))


class DeliveryOutfeed:
    """Single-level receiving belts; packages leave via a covered collection bin.

    Kinematic transfer only. Archived packages retain IDs and delivery records,
    but their Gazebo models are parked below the scene after leaving the belt.
    """
    ENTRY, TRAVEL, SPEED = .48, 1.2, .25

    def __init__(self,board):
        self.board=board
        self.items=[]

    def entry(self,slot):
        x,y=self.board.xy(0,slot-1)
        return (x+self.ENTRY*math.sin(self.board.yaw),
                y-self.ENTRY*math.cos(self.board.yaw))

    def can_accept(self,slot,depth):
        x,y=self.entry(slot)
        return not any(b['slot']==slot and not b['outbound_cleared'] and
                       math.hypot(b['x']-x,b['y']-y)<(b['d']+depth)/2+.06
                       for b in self.items)

    def accept(self,item,slot,now):
        if item['kind']!=slot or not self.can_accept(slot,item['d']):
            raise ValueError('wrong delivery slot or receiving belt entrance occupied')
        item.update(slot=slot,accepted_at=now,outbound_cleared=False,z=DECK_Z,yaw=self.board.yaw)
        item['x'],item['y']=self.entry(slot)
        self.items.append(item)

    def update(self,now):
        cleared=[]
        for b in self.items:
            if b['outbound_cleared']:continue
            distance=max(0.,now-b['accepted_at'])*self.SPEED
            x,y=self.entry(b['slot'])
            b['x']=x+min(distance,self.TRAVEL)*math.sin(self.board.yaw)
            b['y']=y-min(distance,self.TRAVEL)*math.cos(self.board.yaw)
            if distance>=self.TRAVEL+b['d']/2:
                b['outbound_cleared']=True;b['z']=-5.
                cleared.append(b['id'])
        return cleared

    def shift_time(self,duration):
        for b in self.items:
            if not b['outbound_cleared']:b['accepted_at']+=duration


def scan_boxes(pose, boxes, rays=720):
    """Independent analytic nearest-return 2D sensor, used in vision checks.

    A larger rear box contributes side returns, never replaces a closer hit.
    """
    import numpy as np
    amin, inc = -math.pi, 2*math.pi/rays
    angles = amin+np.arange(rays)*inc+pose[2]
    out = np.full(rays, np.inf)
    for b in boxes:
        if not b.get('z', 0.) <= .19 <= b.get('z', 0.)+b['h']:
            continue
        c, s = math.cos(b['yaw']), math.sin(b['yaw'])
        px, py = pose[0]-b['x'], pose[1]-b['y']
        ox, oy = c*px+s*py, -s*px+c*py
        dx, dy = np.cos(angles-b['yaw']), np.sin(angles-b['yaw'])
        with np.errstate(divide='ignore', invalid='ignore'):
            ax, bx = (-b['w']/2-ox)/dx, (b['w']/2-ox)/dx
            ay, by = (-b['d']/2-oy)/dy, (b['d']/2-oy)/dy
        near = np.maximum(np.minimum(ax, bx), np.minimum(ay, by))
        far = np.minimum(np.maximum(ax, bx), np.maximum(ay, by))
        out = np.where((far >= near) & (near > .12), np.minimum(out, near), out)
    return amin, inc, out.tolist()


@dataclass
class Lane:
    name: str
    row: int
    pickup: tuple
    order: list = field(default_factory=list)

    def __post_init__(self):
        if not self.order:
            self.order = list(ORDERS[self.name])
        self.items = []
        for i, kind in enumerate(self.order):
            w, h, d = SIZES[kind]
            self.items.append(dict(id=f'{self.name}_box_{i+1}', kind=kind,
                                   x=self.pickup[0]+FRONT_GAP+d/2+i*PITCH,
                                   y=self.pickup[1], z=DECK_Z, yaw=-math.pi/2,
                                   w=w, h=h, d=d))
        self.payload = None
        self.stage = 'ready'
        self.started = 0.
        self.feed_start = {}
        self.transfer_start = None
        self.feed_count = 0

    @property
    def front(self):
        return self.items[0] if self.items else None

    def load(self, now):
        if self.stage != 'ready' or not self.front or self.payload:
            raise ValueError('feeder is not ready or robot still carries a package')
        self.payload = self.items.pop(0)
        self.payload['owner'] = self.name
        self.transfer_start = dict(self.payload)
        self.stage, self.started = 'loading', now

    def transfer_motion(self, now):
        """Lift first, then push; return below the deck before feeding again."""
        t=max(0.,now-self.started)
        return (smooth_step((t-.55)/.95), smooth_step((t-.25)/.30))

    def mechanism(self, now):
        """World positions shared by Gazebo and the swept-clearance checks.

        The head is behind the transferred carton, never inside the FIFO.
        During reset/feeding its complete swept volume is below DECK_Z.
        """
        x,y=self.pickup
        pose=dict(pusher=(x+.69,y,PUSHER_DOWN_Z),gate=(x+.33,y,.0775),
                  pusher_arm=(x+.69,y,PUSHER_DOWN_Z,0.,0.,math.pi/2),
                  lift=(x+.48,y,DECK_Z-.0125),
                  bridge=(x+.31,y,DECK_Z-.006,0.,math.pi/2,0.))
        if self.stage!='loading':return pose
        t=max(0.,now-self.started)
        _,lift=self.transfer_motion(now)
        reset=smooth_step((t-1.65)/.15)
        deployed=lift*(1-smooth_step((t-1.8)/.2))
        start=self.transfer_start
        # The package update already supplies its current x. Interpolate from
        # that actual rear face, not a fixed stroke through cartons of any size.
        head_x=self.payload['x']+start['d']/2+PUSHER_HALF_X+.003
        if t<.55:
            # Rise behind the lift table before approaching the carton. Rising
            # at the contact point would pass through the solid table itself.
            head_x=x+.69+(head_x-x-.69)*smooth_step((t-.25)/.30)
        if t>=1.8:
            head_x+=(x+.69-head_x)*smooth_step((t-1.8)/.2)
        raised=DECK_Z+lift*(TRAY_Z-DECK_Z)+.025
        head_z=PUSHER_DOWN_Z+(raised-PUSHER_DOWN_Z)*smooth_step(t/.25)
        head_z+=(PUSHER_DOWN_Z-head_z)*reset
        gate_down=smooth_step(t/.25)*(1-reset)
        # A hinged follower folds sideways above the bridge before lowering;
        # a rigid horizontal rod would cut through the bridge on the way down.
        arm_fold=max(1-smooth_step((t-.25)/.30),smooth_step((t-1.5)/.15))
        pose.update(pusher=(head_x,y,head_z),gate=(x+.33,y,.0775-.0575*gate_down),
                    pusher_arm=(head_x,y,head_z,0.,0.,math.pi/2*arm_fold),
                    lift=(x+.48,y,DECK_Z-.0125+(TRAY_Z-DECK_Z)*deployed),
                    bridge=(x+.31,y,DECK_Z-.006+(TRAY_Z-DECK_Z)*deployed,
                            0.,math.pi/2*(1-deployed),0.))
        return pose

    def update(self, now, robot_pose):
        """Returns one mechanical event, if a phase has just changed."""
        if self.payload:
            p = self.payload
            if self.stage == 'loading':
                push,lift = self.transfer_motion(now)
                a = self.transfer_start
                p['x'] = a['x']+(robot_pose[0]-a['x'])*push
                p['y'] = a['y']+(robot_pose[1]-a['y'])*push
                p['z'] = DECK_Z+(TRAY_Z-DECK_Z)*lift
                if now-self.started >= LOAD_SECONDS:
                    self.stage = 'wait_departure'
                    return 'loaded'
            else:
                p['x'], p['y'], p['z'] = robot_pose[0], robot_pose[1], TRAY_Z
                p['yaw'] = robot_pose[2]-math.pi/2
        if self.stage == 'wait_departure':
            if math.hypot(robot_pose[0]-self.pickup[0], robot_pose[1]-self.pickup[1]) > .42:
                if not self.items:
                    self.stage = 'empty'
                    return 'empty'
                self.feed_start = {b['id']: b['x'] for b in self.items}
                self.stage, self.started = 'feeding', now
                return 'feed_started'
        if self.stage == 'feeding':
            u = min(1., max(0., (now-self.started)/2.5))
            smooth = u*u*(3-2*u)
            for i, b in enumerate(self.items):
                target = self.pickup[0]+FRONT_GAP+b['d']/2+i*PITCH
                b['x'] = self.feed_start[b['id']]*(1-smooth)+target*smooth
            if u >= 1:
                self.feed_count += 1
                self.stage = 'ready'
                return 'feed_finished'
        return None

    def unload(self):
        if not self.payload or self.stage == 'loading':
            raise ValueError('no fully loaded package')
        item, self.payload = self.payload, None
        item.pop('owner', None)
        return item


def camera_config():
    return {'camera': dict(width=640, height=480, fx=492.8, fy=492.8,
                           cx=320., cy=240., dist=[-.12, .02, 0., 0., 0.],
                           x=.06, y=0., z=.19, yaw=0., pitch=0.),
            'lidar': dict(x=0., z=.19, half_fov_deg=35, max_range=2.4, scans=5),
            'support_z': DECK_Z, 'require_front_rectangle': True}


def renderer(seed=7, seams=True):
    """Reuse the calibrated synthetic RGB camera, without creating a ROS node."""
    from types import SimpleNamespace
    import numpy as np
    from sim_cargo import SimCargo
    cfg = camera_config()
    c = cfg['camera']
    obj = SimpleNamespace(K=np.array([[c['fx'], 0, c['cx']],
                                     [0, c['fy'], c['cy']], [0, 0, 1]]),
                          D=np.array(c['dist']), W=c['width'], H=c['height'],
                          cam=(c['x'], c['y'], c['z']), cam_yaw=0., cam_pitch=0.,
                          rng=np.random.default_rng(seed), boxes=[], carton_edges=seams)
    obj.to_cam = lambda pose, p: SimCargo.to_cam(obj, pose, p)
    obj.maps = SimCargo.distort_maps(obj)
    obj.render = lambda pose: SimCargo.render(obj, pose)
    return obj
