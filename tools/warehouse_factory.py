"""Procedural factory scenery outside the robot's mapped operating room."""
import math
from fleet_gazebo import box_visual, cyl_visual


def factory_visuals(right, bottom):
    v=[]
    def box(name,x,y,z,sx,sy,sz,color,yaw=0.):
        v.append(box_visual(name,x,y,z,sx,sy,sz,color,yaw))
    west,east,north,south=-4.5,right+2.6,4.3,bottom-1.8
    cx,cy=(west+east)/2,(north+south)/2
    box('factory_slab',cx,cy,-.065,east-west,north-south,.1,'.49 .53 .56')
    # Keep the foreground open for camera access; no opaque roof over the grid.
    box('back_wall',cx,north,1.4,east-west,.12,2.8,'.70 .75 .78')
    box('west_wall',west,cy,.55,.12,north-south,1.1,'.57 .64 .68')
    for i in range(math.ceil((east-west)/1.8)):
        x=west+.65+i*1.8
        if x>east-.3:break
        box(f'window_{i}',x,north-.067,1.85,1.3,.012,.65,'.25 .48 .59')
        box(f'window_bar_{i}',x,north-.08,1.85,.035,.018,.65,'.78 .83 .84')
    for i,x in enumerate((west+.15,(west+east)/2,east-.15)):
        for j,y in enumerate((north-.15,south+.15)):
            box(f'column_{i}_{j}',x,y,1.4,.16,.16,2.8,'.26 .32 .38')
            box(f'column_guard_{i}_{j}',x,y,.22,.20,.20,.44,'.96 .69 .08')
            for k in range(3):
                box(f'guard_band_{i}_{j}_{k}',x,y,.09+k*.12,.205,.205,.035,'.12 .15 .17')
        box(f'roof_tie_{i}',x,cy,2.8,.1,north-south,.12,'.34 .4 .44')
        box(f'light_{i}',x,north-1.25,2.69,.08,.95,.035,'.98 .98 .84')
    box('back_beam',cx,north-.15,2.8,east-west,.12,.14,'.26 .32 .38')

    # Shelves and pallets are beyond y=2.5, the existing sensor/map boundary.
    for rack in range(max(2,int((east-west-2)/1.5))):
        x=west+1.3+rack*1.5;y=3.25
        if x>east-.8:break
        for side in (-1,1):
            for depth in (-1,1):
                box(f'rack_{rack}_post_{side}_{depth}',x+side*.59,y+depth*.25,.78,
                    .045,.045,1.56,'.16 .34 .52')
        for shelf,z in enumerate((.16,.70,1.24)):
            box(f'rack_{rack}_shelf_{shelf}',x,y,z,1.24,.56,.045,'.89 .48 .12')
            for k in range(3):
                w=.26+.025*((rack+k)%3);h=.22+.06*((shelf+k)%3)
                bx=x+(k-1)*.37
                box(f'stock_{rack}_{shelf}_{k}',bx,y,z+.025+h/2,w,.36,h,'.65 .48 .31')
                box(f'stock_tape_{rack}_{shelf}_{k}',bx,y-.183,z+.025+h/2,.025,.004,h,'.83 .71 .48')

    # Parked forklift, west of the map's x=-2.5 boundary.
    fx,fy=-3.5,-.7
    box('forklift_body',fx,fy,.25,.65,.40,.34,'.94 .65 .06')
    box('forklift_rear',fx-.23,fy,.35,.20,.4,.34,'.87 .53 .04')
    box('forklift_seat',fx-.04,fy,.49,.22,.24,.10,'.13 .16 .18')
    box('forklift_seat_back',fx-.13,fy,.59,.06,.24,.24,'.13 .16 .18')
    for i,dx in enumerate((-.22,.23)):
        for side in (-1,1):
            v.append(cyl_visual(f'forklift_wheel_{i}_{side}',fx+dx,fy+side*.23,.12,
                                .12,.08,'.09 .10 .11',roll=math.pi/2))
            box(f'forklift_cage_{i}_{side}',fx+dx,fy+side*.19,.78,.025,.025,.70,'.22 .27 .3')
    box('forklift_canopy',fx,fy,1.15,.62,.49,.035,'.93 .63 .05')
    for side in (-1,1):
        box(f'forklift_mast_{side}',fx+.36,fy+side*.14,.57,.05,.045,1.08,'.24 .29 .32')
        box(f'forklift_fork_{side}',fx+.62,fy+side*.14,.07,.55,.055,.035,'.33 .38 .40')
    for i in range(3):
        x=east-1.05;y=1.25-i*.65
        box(f'pallet_{i}',x,y,.06,.8,.50,.12,'.57 .40 .23')
        for k in range(3):
            box(f'pallet_slat_{i}_{k}',x,y+(k-1)*.18,.125,.8,.13,.025,'.7 .53 .32')
        box(f'pallet_load_{i}',x,y,.37,.58,.4,.46,'.69 .51 .32')
    for i,y in enumerate((1.6,2.2)):
        box(f'electrical_cabinet_{i}',west+.3,y,.55,.38,.5,1.1,'.73 .77 .78')
        box(f'cabinet_handle_{i}',west+.5,y+.15,.6,.025,.025,.12,'.22 .27 .3')
        box(f'cabinet_signal_{i}',west+.5,y-.1,.83,.015,.06,.04,'.17 .65 .3')
    # Painted pedestrian route along the rear wall, outside AGV traffic.
    box('pedestrian_strip',cx,2.65,.003,east-west-.7,.035,.004,'.95 .75 .15')
    for i in range(7):
        box(f'crosswalk_{i}',west+.7,-2.4+i*.11,.003,.55,.06,.004,'.92 .92 .82')
    return ''.join(v)
