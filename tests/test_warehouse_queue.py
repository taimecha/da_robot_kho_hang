"""FIFO and departure interlocks, independent of ROS/Gazebo."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from warehouse_queue import (Lane, PITCH, DECK_Z, classify_height, DEMO_15, DEMO_30,
                             round_coverage, conveyor_geometry, TRAY_Z,
                             PUSHER_HALF_X, PUSHER_HALF_Z)


class FeederTests(unittest.TestCase):
    def test_pusher_swept_volume_clears_all_carton_pairs_and_feeding_path(self):
        import math
        import numpy as np
        # Include the head and rod, not just the head's centre. Both must be
        # below queued cartons for the complete return and indexing stroke.
        for first in range(1,5):
            for second in range(1,5):
                lane=Lane('agv1',2,(1.2,-.6),[first,second])
                lane.load(0.)
                for t in np.linspace(0,2,401):
                    lane.update(t,(1.2,-.6,0.))
                    m=lane.mechanism(t);hx,_,hz=m['pusher']
                    ax,_,az,_,_,yaw=m['pusher_arm']
                    arm_half_x=.05*abs(math.cos(yaw))+.006*abs(math.sin(yaw))
                    parts=[(hx,PUSHER_HALF_X,hz,PUSHER_HALF_Z),
                           (ax+.05*math.cos(yaw),arm_half_x,az,.005)]
                    gx,_,gz=m['gate'];lx,_,lz=m['lift']
                    bx,_,bz,_,pitch,_=m['bridge']
                    boxes=lane.items+([lane.payload] if lane.payload else [])
                    boxes+= [dict(id='lift',x=lx,d=.34,z=lz-.0125,h=.025),
                             dict(id='gate',x=gx,d=.025,z=gz-.0175,h=.035)]
                    bridge_half_x=.075*abs(math.cos(pitch))+.006*abs(math.sin(pitch))
                    bridge_half_z=.075*abs(math.sin(pitch))+.006*abs(math.cos(pitch))
                    boxes.append(dict(id='bridge',x=bx-.075*math.cos(pitch),d=2*bridge_half_x,
                                      z=bz+.075*math.sin(pitch)-bridge_half_z,h=2*bridge_half_z))
                    for b in boxes:
                        for px,half_x,pz,half_z in parts:
                            self.assertGreaterEqual(pz-half_z,0.)
                            overlap_x=min(px+half_x,b['x']+b['d']/2)-max(px-half_x,b['x']-b['d']/2)
                            overlap_z=min(pz+half_z,b['z']+b['h'])-max(pz-half_z,b['z'])
                            self.assertFalse(overlap_x>1e-6 and overlap_z>1e-6,
                                             (first,second,t,b['id'],'pusher intersects carton'))
                    if .56<t<1.5:
                        self.assertAlmostEqual(lane.payload['z'],TRAY_Z)
                        self.assertLess(m['gate'][2]+.0175,DECK_Z)
                self.assertEqual(lane.stage,'wait_departure')
                lane.update(3.,(.6,-.6,0.))
                for t in np.linspace(3,5.5,101):
                    lane.update(t,(.6,-.6,0.))
                    self.assertLess(lane.mechanism(t)['pusher'][2]+PUSHER_HALF_Z,DECK_Z)
                self.assertEqual(lane.stage,'ready')

    def test_demo30_keeps_first_fifteen_and_adds_three_distinct_two_pair_rounds(self):
        self.assertEqual(sum(map(len,DEMO_30.values())),30)
        for n,kinds in DEMO_30.items():
            self.assertEqual(len(kinds),10)
            self.assertEqual(kinds[:5],DEMO_15[n])
            self.assertEqual(set(kinds),{1,2,3,4})
        rounds=[{n:kinds[i] for n,kinds in DEMO_30.items()} for i in range(10)]
        coverage=round_coverage(rounds)
        self.assertTrue(coverage['meets_demo_30'])
        self.assertFalse(round_coverage(rounds[:-1])['meets_demo_30'])
        self.assertEqual(coverage['triple_rounds'],[1,2])
        self.assertEqual(coverage['distinct_rounds'],[6,7,9])
        self.assertEqual([i for times in coverage['pair_rounds'].values() for i in times if i>5],[8,10])

    def test_ten_cartons_fit_and_all_thirty_advance_in_fifo_order(self):
        for n,order in DEMO_30.items():
            lane=Lane(n,2,(1.2,-.6),order)
            center,length,_=conveyor_geometry(10)
            self.assertTrue(all(b['x']-1.2+b['d']/2<=center+length/2 for b in lane.items))
            for i,kind in enumerate(order):
                now=i*10
                self.assertEqual(lane.front['kind'],kind)
                lane.load(now);lane.update(now+2,(1.2,-.6,0))
                lane.update(now+3,(.6,-.6,0));lane.update(now+6,(.6,-.6,0))
                self.assertEqual(lane.unload()['id'],f'{n}_box_{i+1}')
            self.assertEqual(lane.feed_count,9)
            self.assertIsNone(lane.front)

    def test_demo15_has_two_triples_and_each_distinct_pair(self):
        self.assertEqual(sum(map(len,DEMO_15.values())),15)
        rounds=[{n:kinds[i] for n,kinds in DEMO_15.items()} for i in range(5)]
        coverage=round_coverage(rounds)
        self.assertTrue(coverage['meets_demo_15'])
        self.assertEqual(coverage['triple_rounds'],[1,2])
        self.assertEqual(coverage['pair_rounds'],{
            'agv1+agv2':[3], 'agv1+agv3':[4], 'agv2+agv3':[5]})
        self.assertFalse(round_coverage(rounds[:4])['meets_demo_15'])

    def test_five_cartons_fit_on_conveyor_and_advance_in_fifo_order(self):
        for n,order in DEMO_15.items():
            lane=Lane(n,2,(1.2,-.6),order)
            center,length,rollers=conveyor_geometry(len(order))
            self.assertGreater(rollers,19)
            end=center+length/2
            self.assertTrue(all(b['x']-lane.pickup[0]+b['d']/2<=end for b in lane.items))
            delivered=[]
            for i in range(5):
                start=i*10
                lane.load(start)
                lane.update(start+2,(1.2,-.6,0))
                lane.update(start+3,(.6,-.6,0))
                lane.update(start+6,(.6,-.6,0))
                delivered.append(lane.unload()['kind'])
            self.assertEqual(delivered,order)
            self.assertIsNone(lane.front)

    def test_only_head_can_load_and_other_lanes_do_not_advance(self):
        a = Lane('agv1', 2, (1.2, -.6))
        b = Lane('agv2', 3, (1.2, 0.))
        original = [dict(item) for item in b.items]
        first = a.front['id']
        rear_x = a.items[1]['x']
        a.load(0)
        self.assertEqual(a.payload['id'], first)
        with self.assertRaises(ValueError):
            a.load(1)
        a.update(3, (1.2, -.6, 0))
        for t in range(4, 12):
            a.update(t, (1.2, -.6, 0))
        self.assertEqual(a.items[0]['x'], rear_x)
        self.assertEqual(a.stage, 'wait_departure')
        self.assertEqual(b.items, original)

    def test_feed_requires_loaded_and_departed_and_moves_continuously(self):
        a = Lane('agv1', 2, (1.2, -.6))
        old = a.items[1]['x']
        a.load(0)
        self.assertIsNone(a.update(1, (.5, -.6, 0)))
        self.assertEqual(a.stage, 'loading')
        self.assertEqual(a.update(2, (1.2, -.6, 0)), 'loaded')
        self.assertEqual(a.update(3, (.7, -.6, 0)), 'feed_started')
        self.assertEqual(a.items[0]['x'], old)
        a.update(4.25, (.7, -.6, 0))
        self.assertAlmostEqual(a.items[0]['x'], old-PITCH/2)
        self.assertEqual(a.update(5.5, (.7, -.6, 0)), 'feed_finished')
        self.assertAlmostEqual(a.items[0]['x'], old-PITCH)
        self.assertEqual(a.front['kind'], 4)
        self.assertEqual(a.front['z'], DECK_Z)

    def test_empty_lane_does_not_feed_and_unload_required_before_next_pick(self):
        a = Lane('agv1', 2, (1.2, -.6), [1])
        a.load(0); a.update(2, (1.2, -.6, 0))
        self.assertEqual(a.update(3, (.6, -.6, 0)), 'empty')
        self.assertEqual(a.unload()['kind'], 1)
        with self.assertRaises(ValueError):
            a.unload()

    def test_four_height_bins_and_out_of_range_rejection(self):
        import json
        import numpy as np
        for i, height in enumerate((.27, .34, .40, .47), 1):
            self.assertEqual(classify_height(height), i)
        for height, expected in ((.22, 1), (.239, 1), (.289999, 1), (.29, 2),
                                 (.359999, 2), (.36, 3), (.429999, 3),
                                 (.43, 4), (.50, 4)):
            with self.subTest(height=height):
                self.assertEqual(classify_height(height), expected)
                self.assertEqual(json.dumps({'kind':classify_height(np.float64(height))}),
                                 json.dumps({'kind':expected}))
        for height in (.219999, .500001):
            with self.assertRaises(ValueError): classify_height(height)


if __name__ == '__main__': unittest.main()
