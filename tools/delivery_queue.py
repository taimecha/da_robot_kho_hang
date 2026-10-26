"""FIFO access to shared delivery bays, with independent physical waiting slots.

No ROS or simulated truth. Navigation/position freshness remain the caller's job.
Queue order follows the caller's registration; only the caller admits loaded cars.
"""
from collections import defaultdict, deque
import math


class DeliveryQueue:
    def __init__(self):
        self.waiting=defaultdict(deque)
        self.owners={}
        self.kind={}
        self.joined={}
        self.staging={}

    def enqueue(self, robot, kind, now):
        if robot in self.kind:
            if self.kind[robot]!=kind:
                raise ValueError('robot already has a different pending delivery')
            return False
        self.kind[robot]=kind
        self.joined[robot]=now
        self.waiting[kind].append(robot)
        return True

    def head(self, kind):
        q=self.waiting[kind]
        return q[0] if q else None

    def can_admit(self, robot):
        kind=self.kind[robot]
        return kind not in self.owners and self.head(kind)==robot

    def admit(self, robot):
        if not self.can_admit(robot):
            return False
        kind=self.kind[robot]
        self.waiting[kind].popleft()
        self.owners[kind]=robot
        return True

    def release(self, robot):
        kind=self.kind.get(robot)
        if self.owners.get(kind)!=robot:
            raise ValueError('only the current bay owner may release it')
        del self.owners[kind]
        del self.kind[robot]
        del self.joined[robot]

    def rank(self, robot):
        kind=self.kind.get(robot)
        if kind is None:return None
        if self.owners.get(kind)==robot:return 0
        return list(self.waiting[kind]).index(robot)+1

    def assign_staging(self, robot, board, unavailable=(), promote=False, pose=None):
        if robot in self.staging and not promote and self.staging[robot] not in unavailable:
            return self.staging[robot]
        kind=self.kind[robot]
        unavailable=set(unavailable) | {node for n,node in self.staging.items() if n!=robot}
        # Waiting cars stay off service row 1, pickup column 5 and camera
        # observation column 4. This preserves the exit and return corridors.
        candidates=[(r,c) for r in range(2,board.rows+1)
                    for c in range(1,min(3,board.cols)+1)
                    if (r,c) not in unavailable]
        if not candidates:return None
        col=min(kind,3)
        def score(node):
            r,c=node
            if pose is None:
                return (3*abs(c-col)+r-1,r,c)
            sr,sc=board.closest(*pose[:2]);sr+=1;sc+=1
            # Rank waiting positions by the complete trip (here -> waiting
            # position -> bay), rather than proximity to the bay alone.
            # A distant row-5 detour used to beat a nearby off-corridor slot.
            travel=abs(sr-r)+abs(sc-c)
            remaining=r-1+abs(c-kind)
            heading=round((pose[2]-board.yaw)/(math.pi/2))%4
            first=[]
            if c!=sc:first.append(0 if c>sc else 2)
            if r!=sr:first.append(1 if r>sr else 3)
            turns=min((min((d-heading)%4,(heading-d)%4) for d in first),default=0)
            turns+=int(r!=sr and c!=sc)
            return (travel+remaining,turns,remaining,travel,r,c)
        head=self.head(kind)
        if pose is None and head!=robot and head in self.staging:
            candidates=[n for n in candidates if score(n)>score(self.staging[head])]
        if not candidates:return None
        node=min(candidates,key=score)
        old=self.staging.get(robot)
        if old is not None and old not in unavailable and score(old)<=score(node):return old
        self.staging[robot]=node
        return node

    def release_staging_when_clear(self, robot, pose, board):
        node=self.staging.get(robot)
        if node is None:return False
        x,y=board.xy(node[0]-1,node[1]-1)
        if math.hypot(pose[0]-x,pose[1]-y)>board.spacing/2+board.robot_radius+.02:
            del self.staging[robot]
            return True


def bay_exit_clear(pose, board, kind):
    """Only release AFTER physically leaving the bay's reserved node envelope.

    Returning home is not required. Distance matches the traffic manager's
    half-cell + robot-radius occupancy, with 2 cm margin.
    """
    x,y=board.xy(0,kind-1)
    dx,dy=pose[0]-x,pose[1]-y
    if board.cfg.get('delivery_flow')=='flexible':
        return math.hypot(dx,dy)>board.spacing/2+board.robot_radius+.02
    along=dx*math.cos(board.yaw)+dy*math.sin(board.yaw)
    lateral=-dx*math.sin(board.yaw)+dy*math.cos(board.yaw)
    return along>board.spacing/2+board.robot_radius+.02 and abs(lateral)<.15


def column_controls(queue, board, poses, entries, entered):
    """One route per vehicle; progressively open its permitted queue row.

    A follower approaches at its nearby row, then waits outside the column
    until its predecessor clears the gate. Permissions advance with occupancy.
    No simulated parcel truth is used to grant movement.
    """
    result={}
    for kind in sorted(set(queue.waiting)|set(queue.owners)):
        owner=queue.owners.get(kind)
        members=([owner] if owner else [])+list(queue.waiting[kind])
        previous=None
        for i,n in enumerate(members):
            if n not in poses:continue
            pose=poses[n];x,y=board.xy(0,kind-1)
            dx,dy=pose[0]-x,pose[1]-y
            row=1+(-dx*math.sin(board.yaw)+dy*math.cos(board.yaw))/board.spacing
            lateral=dx*math.cos(board.yaw)+dy*math.sin(board.yaw)
            gate=i+1 if owner else i+2
            if previous:
                lead=result[previous];p=poses[previous]
                if previous not in entered:
                    gate=max(gate,lead['entry_row']+1)
                else:
                    lead_row=1+(-(p[0]-x)*math.sin(board.yaw)+(p[1]-y)*math.cos(board.yaw))/board.spacing
                    gap=(board.spacing/2+board.robot_radius+.02)/board.spacing
                    gate=max(gate,math.ceil(lead_row+gap-1e-6))
                gate=max(gate,lead['gate_row']+1)
            gate=min(board.rows+1,gate)
            # The gate is a permission limit, not a waypoint. A follower can
            # approach the column at its nearby row and wait outside until
            # the preceding car clears it. Forcing entry >= gate sent cars
            # around the perimeter even when the adjacent approach was free.
            entry=entries.get(n,min(board.rows,max(2,round(row))))
            if n in entries and abs(lateral)<.08 and row<=entry+.1:entered.add(n)
            result[n]=dict(slot=kind,entry_row=entry,gate_row=gate)
            previous=n
    return result
