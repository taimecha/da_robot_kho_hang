"""Jitter, controlled recovery, frozen task timers, and lasting data loss."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from warehouse_telemetry import TelemetryGuard
from warehouse_scene import Warehouse


def ages(odom=.02,status=.2):
    return {n:(odom,status) for n in ('agv1','agv2','agv3')}


class GuardTests(unittest.TestCase):
    def test_original_101_second_gap_only_warns_then_recovers(self):
        g=TelemetryGuard()
        self.assertEqual(g.step(10,ages(1.01,1.13)),'warning')
        self.assertFalse(g.paused)
        self.assertEqual(g.step(10.1,ages()),'healthy')
        self.assertFalse(g.failed)

    def test_resume_requires_continuous_fresh_data_and_only_occurs_once(self):
        g=TelemetryGuard()
        self.assertEqual(g.step(10,ages(2.1,2.1)),'paused')
        self.assertEqual(g.step(10.2,ages()),'recovering')
        self.assertEqual(g.step(11.3,ages(1.,1.)),'paused')
        self.assertEqual(g.step(11.4,ages()),'recovering')
        self.assertEqual(g.step(12.5,ages()),'resumed')
        self.assertFalse(g.paused)
        self.assertEqual(g.step(12.6,ages()),'healthy')

    def test_real_loss_latches_failure_and_intermittent_data_cannot_wait_forever(self):
        g=TelemetryGuard()
        g.step(10,ages(2.1,2.1))
        self.assertEqual(g.step(18,ages(10.1,10.1)),'failed')
        self.assertEqual(g.step(19,ages()),'failed')
        g=TelemetryGuard();g.step(10,ages(2.1,2.1))
        self.assertEqual(g.step(25.1,ages(1.,1.)),'failed')

    def test_status_gap_and_invalid_data_also_stop(self):
        g=TelemetryGuard()
        self.assertEqual(g.step(10,ages(.02,3.1)),'paused')
        self.assertEqual(g.step(11,ages(float('nan'),.2)),'failed')
        invalid=ages();invalid['agv2']=(float('nan'),.2)
        self.assertEqual(TelemetryGuard().step(11,invalid),'failed')

    def test_pause_keeps_goal_and_payload_and_resumes_without_changing_deadlines_budget(self):
        n='agv1';lane=SimpleNamespace(stage='loading',started=8,payload={'id':'box1'})
        checkpoint=dict(command='agv1 2 5 d',old_route=7,ack=True,t=8)
        w=SimpleNamespace(started=True,failed=False,done=False,lanes={n:lane},
            pose_t={n:7.9},status_t={n:7.9},telemetry=TelemetryGuard(),telemetry_state='healthy',
            event=Mock(),fail=Mock(),outfeed=Mock(),pause_started=None,frame_epoch=0,samples={n:[1,2]},
            goal_pub=Mock(),deadlines={n:100},pending_goals={n:checkpoint},
            status={n:{'route_id':8}},last_goal_send={n:8})
        Warehouse.supervise_telemetry(w,10)
        self.assertEqual(w.goal_pub.publish.call_args.args[0].data,'stop')
        self.assertEqual(w.pending_goals[n]['command'],'agv1 2 5 d')
        self.assertEqual(w.samples[n],[])
        w.pose_t[n]=w.status_t[n]=10.2
        Warehouse.supervise_telemetry(w,10.2)
        w.pose_t[n]=w.status_t[n]=11.3
        Warehouse.supervise_telemetry(w,11.3)
        self.assertAlmostEqual(w.deadlines[n],101.3)
        self.assertAlmostEqual(lane.started,9.3)
        self.assertEqual(lane.payload,{'id':'box1'})
        self.assertFalse(checkpoint['ack'])
        self.assertEqual(checkpoint['old_route'],8)
        self.assertEqual(w.goal_pub.publish.call_args.args[0].data,'agv1 2 5 d')
        self.assertEqual(w.goal_pub.publish.call_count,2)
        Warehouse.supervise_telemetry(w,11.31)
        self.assertEqual(w.goal_pub.publish.call_count,2)
        w.fail.assert_not_called()

    def test_old_source_stamp_is_not_mistaken_for_a_fresh_packet(self):
        from unittest.mock import patch
        from nav_msgs.msg import Odometry
        msg=Odometry();msg.pose.pose.orientation.w=1.
        msg.header.stamp.sec=100
        w=SimpleNamespace(pose={},vel={},pose_t={},
                          get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=103_000_000_000)))
        with patch('warehouse_scene.time.monotonic',return_value=10):
            Warehouse.on_pose(w,'agv2',msg)
        self.assertEqual(w.pose_t['agv2'],7)

    def test_slow_gazebo_updates_do_not_block_or_build_a_backlog(self):
        from concurrent.futures import ThreadPoolExecutor
        import threading
        from warehouse_queue import Lane
        entered=threading.Event();release=threading.Event()
        def slow_send(poses):
            entered.set();release.wait(2)
        lane=Lane('agv1',2,(1.2,-.6),[1]*10)
        w=SimpleNamespace(lanes={'agv1':lane},pose={'agv1':(0.,-.6,0.)},
            telemetry=TelemetryGuard(),pause_started=None,failed=False,event=Mock(),dropped=[],box_pub=Mock(),
            phase={'agv1':'waiting'},outfeed=SimpleNamespace(update=lambda now:[]),
            mirror=SimpleNamespace(send=Mock(side_effect=slow_send)),mirror_future=None,
            mirror_worker=ThreadPoolExecutor(max_workers=1),last_pose_send=0)
        try:
            Warehouse.mechanics(w,10)
            self.assertTrue(entered.wait(1))
            self.assertFalse(w.mirror_future.done())
            Warehouse.mechanics(w,10.5)
            self.assertEqual(w.mirror.send.call_count,1)
            self.assertEqual(w.last_pose_send,10)
        finally:
            release.set();w.mirror_worker.shutdown(wait=True)

    def test_thirty_package_world_extends_floor_and_map(self):
        import tempfile
        import xml.etree.ElementTree as ET
        from warehouse_launch import prepare
        from warehouse_queue import DEMO_30
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            prepare(p,0,DEMO_30,sync_rounds=True)
            world=ET.parse(p/'warehouse.sdf')
            names=[m.attrib['name'] for m in world.findall('.//world/model')]
            self.assertEqual(len([n for n in names if '_box_' in n]),30)
            width,height=map(int,(p/'map.pgm').read_bytes().splitlines()[1].split())
            self.assertGreater(width,160)
            self.assertGreaterEqual(height,116)
            floor=world.find('.//visual[@name="floor"]')
            center=float(floor.find('pose').text.split()[0])
            length=float(floor.find('.//size').text.split()[0])
            self.assertGreater(center+length/2,6.)


if __name__=='__main__':unittest.main()
