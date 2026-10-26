#!/usr/bin/env python3
"""Anchor the designed 5x5 board to walls observed in a saved LiDAR map.

Painted floor lines have no LiDAR return. The west/south wall offsets are
measured layout dimensions, not positions taken from the simulator's truth.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from grid_board import Board

WEST_WALL_TO_COL1_M=1.275
SOUTH_WALL_TO_ROW1_M=2.075


def place_grid(map_yaml,grid_yaml,west_offset=WEST_WALL_TO_COL1_M,
               south_offset=SOUTH_WALL_TO_ROW1_M):
    map_yaml=Path(map_yaml);grid_yaml=Path(grid_yaml)
    map_cfg=yaml.safe_load(map_yaml.read_text())
    image=np.asarray(Image.open(map_yaml.parent/map_cfg['image']))
    occupied=np.flipud(image)<90
    y,x=np.nonzero(occupied)
    if len(x)<80:raise ValueError('LiDAR map has too few occupied wall cells')
    res=float(map_cfg['resolution']);ox,oy=map_cfg['origin'][:2]
    west=ox+(float(np.percentile(x,2))+.5)*res
    south=oy+(float(np.percentile(y,2))+.5)*res
    cfg=yaml.safe_load(grid_yaml.read_text())
    cfg['center_x']=round(west+west_offset+(cfg['center_col']-1)*cfg['spacing'],3)
    cfg['center_y']=round(south+south_offset+(cfg['center_row']-1)*cfg['spacing'],3)
    cfg['yaw_deg']=0.0
    cfg['lidar_map']=True
    board=Board(cfg)
    cells=np.flipud(image)
    data=np.where(cells<90,100,np.where(cells>230,0,-1)).astype(np.int16)
    bad_nodes,bad_edges,_=board.check_map(data.ravel(),image.shape[1],image.shape[0],res,ox,oy)
    if bad_nodes or bad_edges:
        raise ValueError(f'mapped grid is not clear: {len(bad_nodes)} nodes, {len(bad_edges)} edges blocked')
    grid_yaml.write_text(yaml.safe_dump(cfg,sort_keys=False))
    audit=dict(west_wall_m=round(west,3),south_wall_m=round(south,3),
               west_wall_to_col1_m=west_offset,south_wall_to_row1_m=south_offset,
               center_x_m=cfg['center_x'],center_y_m=cfg['center_y'],
               blocked_nodes=0,blocked_edges=0)
    (grid_yaml.parent/'grid_alignment.json').write_text(json.dumps(audit,indent=2))
    return board,audit


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('map_yaml');ap.add_argument('grid_yaml')
    ap.add_argument('--west-offset',type=float,default=WEST_WALL_TO_COL1_M)
    ap.add_argument('--south-offset',type=float,default=SOUTH_WALL_TO_ROW1_M)
    a=ap.parse_args()
    _,audit=place_grid(a.map_yaml,a.grid_yaml,a.west_offset,a.south_offset)
    print(json.dumps(audit,indent=2))


if __name__=='__main__':main()
