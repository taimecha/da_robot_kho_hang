"""The driving grid is anchored to a scan map and rejected if it intersects obstacles."""
import tempfile
from pathlib import Path
import unittest

import numpy as np
from PIL import Image
import yaml

from grid_board import Board
from tools.warehouse_grid_from_map import place_grid


class LidarGridTests(unittest.TestCase):
    def make_room(self,folder,blocked_center=False):
        image=np.full((120,160),254,np.uint8)
        image[:2]=image[-2:]=0
        image[:,:2]=image[:,-2:]=0
        if blocked_center:image[63:67,48:52]=0
        Image.fromarray(image).save(folder/'map.pgm')
        (folder/'map.yaml').write_text(yaml.safe_dump(dict(
            image='map.pgm',resolution=.05,origin=[-2.5,-3.3,0.],
            occupied_thresh=.65,free_thresh=.196)))
        (folder/'grid.yaml').write_text(yaml.safe_dump(dict(
            rows=5,cols=5,spacing=.6,center_row=3,center_col=3,
            center_x=99.,center_y=99.,robot_radius=.13,
            node_clearance=.25,edge_clearance=.2)))

    def test_wall_anchor_places_clear_board_on_scan_map(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);self.make_room(folder)
            board,audit=place_grid(folder/'map.yaml',folder/'grid.yaml')
            self.assertLess(abs(board.center_x),.1)
            self.assertLess(abs(board.center_y),.1)
            self.assertTrue(board.cfg['lidar_map'])
            self.assertEqual((audit['blocked_nodes'],audit['blocked_edges']),(0,0))
            self.assertEqual(Board.load(folder/'grid.yaml').center_x,board.center_x)

    def test_obstacle_on_grid_prevents_dispatch(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);self.make_room(folder,blocked_center=True)
            with self.assertRaisesRegex(ValueError,'mapped grid is not clear'):
                place_grid(folder/'map.yaml',folder/'grid.yaml')


if __name__=='__main__':unittest.main()
