"""3D kinematic mirror: conveyors, rollers, stop gates, transfer pushers, AGVs."""
import math
from pathlib import Path
from warehouse_queue import COLORS, DECK_Z, conveyor_geometry
from fleet_gazebo import box_visual, cyl_visual, robot_model
from warehouse_factory import factory_visuals
from warehouse_landmarks import LANDMARKS

WORLD = 'warehouse_queue'


def model(name, visuals, pose='0 0 0 0 0 0', static=False):
    return (f'<model name="{name}"><static>{str(static).lower()}</static><pose>{pose}</pose>'
            '<link name="link"><gravity>false</gravity>'
            '<inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>'
            + visuals+'</link></model>')


def build_world(board, lanes, path):
    right=max(4.7,max(l.pickup[0]+center+length/2+.5 for l in lanes.values()
                       for center,length,_ in [conveyor_geometry(len(l.order))]))
    bottom=min(-2.5,board.xy(0,0)[1]-2.1)
    fixed = [box_visual('floor', (right-3.3)/2, (bottom+2.5)/2, -.025,
                        right+3.3, 2.5-bottom, .05, '.83 .85 .87')]
    if board.cfg.get('lidar_map'):
        for i,(x,y,sx,sy) in enumerate(LANDMARKS):
            fixed.append(box_visual(f'lidar_landmark_{i}',x,y,.35,sx,sy,.7,'.38 .47 .55'))
    for i, (a, b) in enumerate(board.edges()):
        x,y = board.xy(*a); xx,yy = board.xy(*b)
        fixed.append(box_visual(f'line_{i}', (x+xx)/2, (y+yy)/2, .002,
                                math.hypot(xx-x, yy-y), .015, .003, '.2 .27 .32',
                                math.atan2(yy-y,xx-x)))
    for r,c in board.nodes():
        x,y=board.xy(r,c)
        fixed.append(cyl_visual(f'node_{r}_{c}',x,y,.004,.025,.004,'.25 .35 .45'))
    for i in range(4):
        x,y=board.xy(0,i)
        fixed.append(box_visual(f'drop_{i+1}',x,y,.004,.36,.36,.006,'.55 .78 .9'))
        # Receiving belt carries each delivered carton south into a covered
        # collection bin. It never builds a tower on the delivery node.
        def local_box(name,u,v,z,sx,sy,sz,rgb):
            return box_visual(name,x+u*math.cos(board.yaw)-v*math.sin(board.yaw),
                              y+u*math.sin(board.yaw)+v*math.cos(board.yaw),
                              z,sx,sy,sz,rgb,board.yaw)
        fixed.append(local_box(f'outfeed_{i+1}_deck',0,-1.03,.043,.38,1.5,.025,'.22 .27 .3'))
        for side in (-1,1):
            fixed.append(local_box(f'outfeed_{i+1}_rail_{side}',side*.21,-1.03,.065,
                                   .025,1.5,.065,'.4 .45 .5'))
            fixed.append(local_box(f'outfeed_{i+1}_bin_side_{side}',side*.22,-1.72,.3,
                                   .025,.4,.6,'.35 .43 .48'))
        fixed.append(local_box(f'outfeed_{i+1}_bin_roof',0,-1.72,.61,.46,.42,.025,'.35 .43 .48'))
        fixed.append(local_box(f'outfeed_{i+1}_bin_back',0,-1.92,.3,.46,.025,.6,'.35 .43 .48'))
        for j in range(18):
            fixed.append(local_box(f'outfeed_{i+1}_cleat_{j}',0,-.32-j*.08,.057,
                                   .36,.008,.006,'.48 .53 .58'))
        if board.cfg.get('forbidden_directions'):
            # Visible direction of the delivery exit corridor.
            x2,y2=board.xy(0,i+1)
            ax,ay=(x+x2)/2,(y+y2)/2
            yaw=board.yaw
            fixed.append(box_visual(f'flow_{i}',ax,ay,.009,.18,.012,.004,'.1 .42 .28',yaw))
            for sign in (-1,1):
                fixed.append(box_visual(f'flow_head_{i}_{sign}',
                                        ax+.055*math.cos(yaw)-sign*.022*math.sin(yaw),
                                        ay+.055*math.sin(yaw)+sign*.022*math.cos(yaw),
                                        .009,.075,.012,.004,'.1 .42 .28',yaw-sign*math.pi/6))
    dynamic=[]
    for lane in lanes.values():
        x,y=lane.pickup
        rgb=COLORS[lane.name]
        center, length, rollers = conveyor_geometry(len(lane.order))
        for side in (-1,1):
            fixed.append(box_visual(lane.name+f'_rail{side}', x+center,y+side*.2,.065,
                                    length,.025,.065,'.3 .34 .4'))
            for u in (.42,center+length/2-.08):
                fixed.append(box_visual(lane.name+f'_leg{side}_{u}',x+u,y+side*.18,.02,
                                        .025,.025,.04,'.3 .34 .4'))
        fixed.append(box_visual(lane.name+'_dock',x,y,.004,.34,.34,.005,rgb))
        for j in range(rollers):
            if .36+j*.08<.70:continue  # front station is a lift, not rollers
            # Cylinder default axis z; roll pi/2 aligns it with conveyor y.
            name=f'{lane.name}_roller_{j}'
            roller=cyl_visual('roller',0,0,0,.016,.36,'.65 .68 .72')
            roller+=box_visual('rotation_mark',.015,0,0,.003,.006,.35,'.15 .2 .25')
            dynamic.append(model(name,roller,
                                 f'{x+.36+j*.08} {y} .044 {math.pi/2} 0 0'))
        dynamic.append(model(lane.name+'_gate',box_visual('gate',0,0,0,.025,.35,.035,rgb),
                             f'{x+.33} {y} .0775 0 0 0'))
        dynamic.append(model(lane.name+'_pusher',box_visual('head',0,0,0,.025,.32,.025,'.85 .2 .15'),
                             f'{x+.69} {y} .015 0 0 0'))
        arm=box_visual('follower',.05,0,0,.10,.012,.01,'.6 .62 .65')
        arm+=cyl_visual('hinge',0,0,0,.009,.02,'.25 .3 .34')
        dynamic.append(model(lane.name+'_pusher_arm',arm,
                             f'{x+.69} {y} .015 0 0 {math.pi/2}'))
        dynamic.append(model(lane.name+'_lift',box_visual('lift_table',0,0,0,.34,.34,.025,'.5 .55 .6'),
                             f'{x+.48} {y} {DECK_Z-.0125} 0 0 0'))
        dynamic.append(model(lane.name+'_bridge',box_visual('transfer',-.075,0,0,.15,.34,.012,'.5 .55 .6'),
                             f'{x+.31} {y} {DECK_Z-.006} 0 {math.pi/2} 0'))
        for side in (-1,1):
            fixed.append(box_visual(lane.name+f'_lift_support_{side}',x+.48,y+side*.18,.017,
                                    .30,.015,.034,'.22 .28 .32'))
        dynamic.append(robot_model(lane.name,rgb))
        tray=box_visual('tray',0,0,0,.33,.33,.012,rgb)
        for a in (-1,1):
            for b in (-1,1):
                tray+=cyl_visual(f'tray_post_{a}_{b}',a*.065,b*.065,-.045,.006,.078,'.55 .58 .6')
        dynamic.append(model(lane.name+'_tray',tray,'0 0 -5 0 0 0'))
        for b in lane.items:
            # The same carton material for all four size classes.
            shape=box_visual('carton',0,0,b['h']/2,b['w'],b['d'],b['h'],'.72 .49 .27')
            # Vertical packing tape is decoration only; never fed to class rules.
            shape+=box_visual('tape',0,-b['d']/2-.001,b['h']/2,.018,.002,b['h'],'.84 .7 .47')
            dynamic.append(model(b['id'],shape,f"{b['x']} {b['y']} {b['z']} 0 0 {b['yaw']}"))
    from fleet_gazebo import GUI
    import xml.etree.ElementTree as ET
    gui=GUI.format(cx=-2.8,cy=-4.2)
    gui=gui.replace('2.4 0 0.95 1.5708','3.8 0 .64 .85')
    gui=gui.replace('-2.800 -4.200 3.8 0 .64 .85','-1.6 -2.6 3.4 0 .76 .736')
    if right>4.7:
        scale=(right+3.3)/8*1.2
        gui=gui.replace('-1.6 -2.6 3.4 0 .76 .736',
                        f'{-1.6*scale+(right-4.7)/2:.3f} {-2.6*scale:.3f} {3.4*scale:.3f} 0 .76 .736')
    root=ET.fromstring(gui)
    for plugin in list(root):
        if plugin.get('filename') in ('EntityTree','TransformControl','SelectEntities'):
            root.remove(plugin)
    gui=ET.tostring(root,encoding='unicode')
    # A dedicated GUI config avoids reusing layout state from an older world.
    # Gazebo's standalone .config uses top-level <window>/<plugin> elements,
    # unlike the SDF's enclosing <gui> element (see installed default config).
    plugins=gui[gui.index('>')+1:gui.rindex('</gui>')]
    window=('<window><width>1400</width><height>900</height>'
            '<dialog_on_exit>false</dialog_on_exit></window>')
    Path(path).with_name('gui.config').write_text('<?xml version="1.0"?>'+window+plugins)
    sdf=('<?xml version="1.0"?><sdf version="1.9"><world name="'+WORLD+'">'
         '<physics name="p" type="ignored"><max_step_size>0.01</max_step_size>'
         '<real_time_factor>1</real_time_factor></physics>'
         '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
         '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>'
         '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>'
         '<scene><ambient>.65 .65 .65 1</ambient><shadows>false</shadows></scene>'
         '<light name="sun" type="directional"><pose>0 0 6 0 0 0</pose>'
         '<diffuse>.8 .8 .8 1</diffuse><direction>-.3 -.5 -1</direction></light>'
         +gui+model('warehouse_floor', ''.join(fixed),static=True)
         +model('factory_surroundings',factory_visuals(right,bottom),static=True)
         +''.join(dynamic)+'</world></sdf>')
    Path(path).write_text(sdf)


class GazeboMirror:
    def __init__(self):
        from gz.transport13 import Node
        self.node=Node()
        self.ok=False

    def send(self, poses):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.boolean_pb2 import Boolean
        msg=Pose_V()
        for name,coords in poses.items():
            if len(coords)==5:x,y,z,roll,yaw=coords;pitch=0.
            else:x,y,z,roll,pitch,yaw=coords
            p=msg.pose.add(); p.name=name
            p.position.x,p.position.y,p.position.z=x,y,z
            cr,sr,cp,sp,cy,sy=math.cos(roll/2),math.sin(roll/2),math.cos(pitch/2),math.sin(pitch/2),math.cos(yaw/2),math.sin(yaw/2)
            p.orientation.x=sr*cp*cy-cr*sp*sy
            p.orientation.y=cr*sp*cy+sr*cp*sy
            p.orientation.z=cr*cp*sy-sr*sp*cy
            p.orientation.w=cr*cp*cy+sr*sp*sy
        ok,reply=self.node.request('/world/'+WORLD+'/set_pose_vector',msg,Pose_V,Boolean,50)
        self.ok=bool(ok and reply.data)
        return self.ok
