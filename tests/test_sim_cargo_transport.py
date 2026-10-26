"""Virtual cargo follows loading/unloading events without changing its dimensions."""
import json
import math
import sys
import unittest
from pathlib import Path
from types import MethodType, SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from sim_cargo import SimCargo
from grid_board import Board, config_path
from plot_cargo_two_car_run import grid_xy
from visualization_msgs.msg import Marker


class CargoTransportTest(unittest.TestCase):
    def node(self):
        class Publisher:
            def __init__(self): self.messages = []
            def publish(self, msg): self.messages.append(msg)
        n = SimpleNamespace(board=Board.load(config_path(True)), marker_count=0,
                            poses={'agv2': (.65,.635,0.)},
                            boxes=[dict(row=4,x=1.06,y=.635,yaw=-math.pi/2,
                                        w=.121,h=.24,d=.12)],
                            box_pub=Publisher(), mk_pub=Publisher())
        n.publish_boxes = MethodType(SimCargo.publish_boxes,n)
        return n

    def test_load_follow_unload_keeps_box_dimensions(self):
        n=self.node()
        SimCargo.cargo_event(n,SimpleNamespace(data=json.dumps(
            dict(car='agv2',event='loaded_confirmed',pickup_row=4))))
        b=n.boxes[0]
        self.assertEqual(b['owner'],'agv2')
        n.poses['agv2']=(-1.745,-1.165,-math.pi/2)
        n.publish_boxes()
        self.assertEqual((b['x'],b['y']),n.poses['agv2'][:2])
        SimCargo.cargo_event(n,SimpleNamespace(data=json.dumps(
            dict(car='agv2',event='unloaded_confirmed',deliver_row=1,deliver_col=1))))
        self.assertIsNone(b['owner'])
        self.assertTrue(b['delivered'])
        self.assertEqual((b['w'],b['h'],b['d']),(.121,.24,.12))
        self.assertEqual(b['z'],0.)
        self.assertFalse(any(m.action==Marker.DELETEALL for m in n.mk_pub.messages[-1].markers))

    def test_grid_center_is_not_shifted_by_one_cell(self):
        b=Board.load(config_path(True))
        self.assertEqual(grid_xy(b,*b.xy(2,2)),(3.,3.))


if __name__=='__main__': unittest.main()
