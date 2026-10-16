import importlib.util
import math
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'src/my_bot_bringup/scripts/scan_filter.py'
spec = importlib.util.spec_from_file_location('scan_filter', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FilterTests(unittest.TestCase):
    def run_filter(self, values, increment=0.02):
        return module.filter_ranges(values, 0.1, 8.0, increment)

    def test_isolated_near_and_far(self):
        for spike in (0.13, 0.33, 7.0):
            source = [2.0, 2.0, spike, 2.0, 2.0]
            result, removed = self.run_filter(source)
            self.assertEqual(removed, 1)
            self.assertTrue(math.isnan(result[2]))
            self.assertEqual(source[2], spike)

    def test_new_box_and_wall_edges_survive(self):
        values = [2., 2., .85, .86, 2., 2., 3., 3.]
        self.assertEqual(self.run_filter(values), (values, 0))

    def test_invalids_are_not_free_space(self):
        values = [0., float('nan'), .3, float('inf'), 9.]
        result, count = self.run_filter(values)
        self.assertEqual(count, 1)
        self.assertTrue(math.isnan(result[2]))
        self.assertEqual(result[0], 0.)
        self.assertEqual(result[3], float('inf'))

    def test_full_scan_seam_only(self):
        values = [.8, 0., 0., .8]
        self.assertEqual(self.run_filter(values, math.pi / 2), (values, 0))
        self.assertEqual(self.run_filter(values, .1)[1], 2)

    def test_persistent_far_single_is_removed(self):
        f = module.ScanHistoryFilter(history_size=5)
        wall = [2.] * 16
        first = wall.copy();first[6] = .8
        out, removed = f.process(first,.1,8.,0.,math.pi/8,now=0.)
        self.assertEqual(removed, 1)
        self.assertTrue(math.isnan(out[6]))
        second = wall.copy();second[7] = .81
        out, removed = f.process(second,.1,8.,0.,math.pi/8,now=.12)
        self.assertEqual(removed, 1)
        self.assertTrue(math.isnan(out[7]))

    def test_first_scan_keeps_near_return_but_removes_far_single(self):
        f = module.ScanHistoryFilter()
        source = [2.] * 16
        source[4] = .28
        source[9] = 1.1
        result, removed = f.process(source,.1,8.,0.,math.pi/8,now=0.)
        self.assertEqual(removed,1)
        self.assertEqual(result[4],.28)
        self.assertTrue(math.isnan(result[9]))

    def test_narrow_object_needs_three_scans(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 16
        f.process(wall,.1,8.,0.,math.pi/8,now=0.,pose=(0.,0.,0.))
        spike = wall.copy();spike[6] = .3
        out, removed = f.process(spike,.1,8.,0.,math.pi/8,now=.12,
                                 pose=(0.,0.,0.))
        self.assertEqual(removed,1)
        self.assertTrue(math.isnan(out[6]))
        out, removed = f.process(spike,.1,8.,0.,math.pi/8,moving=True,now=.24,
                                 pose=(0.,0.,0.))
        self.assertEqual(removed, 1)
        self.assertTrue(math.isnan(out[6]))
        out, removed = f.process(spike,.1,8.,0.,math.pi/8,moving=True,now=.36,
                                 pose=(0.,0.,0.))
        self.assertEqual((out,removed),(spike,0))

    def test_transient_close_cluster_removed_during_motion(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 32
        f.process(wall,.1,8.,-math.pi,math.pi/16,now=0.,pose=(0.,0.,0.))
        noisy = wall.copy()
        noisy[10:16] = [.22] * 6
        out, removed = f.process(noisy,.1,8.,-math.pi,math.pi/16,
                                 moving=True,now=.12,pose=(.02,0.,0.))
        self.assertEqual(removed, 6)
        self.assertTrue(all(math.isnan(v) for v in out[10:16]))
        self.assertEqual(out[9], 2.)

    def test_real_narrow_obstacle_survives_translation_and_rotation(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 32
        first = wall.copy(); first[16] = .30
        f.process(first,.1,8.,-math.pi,math.pi/16,now=0.,pose=(0.,0.,0.))
        second = wall.copy(); second[15] = .28
        out, removed = f.process(second,.1,8.,-math.pi,math.pi/16,
                                 moving=True,now=.12,pose=(.02,0.,math.pi/16))
        self.assertEqual(removed, 1)
        third = wall.copy(); third[14] = .26
        out, removed = f.process(third,.1,8.,-math.pi,math.pi/16,
                                 moving=True,now=.24,pose=(.04,0.,math.pi/8))
        self.assertEqual(removed, 0)
        self.assertEqual(out[14], .26)

    def test_broad_near_obstacle_needs_only_two_scans(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 32
        f.process(wall,.1,8.,-math.pi,math.pi/16,now=0.,pose=(0.,0.,0.))
        obstacle = wall.copy(); obstacle[10:16] = [.28] * 6
        out, removed = f.process(obstacle,.1,8.,-math.pi,math.pi/16,
                                 now=.12,pose=(0.,0.,0.))
        self.assertEqual(removed, 6)
        out, removed = f.process(obstacle,.1,8.,-math.pi,math.pi/16,
                                 now=.24,pose=(0.,0.,0.))
        self.assertEqual((out, removed), (obstacle, 0))

    def test_large_far_cluster_preserved_on_first_appearance(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 32
        f.process(wall,.1,8.,-math.pi,math.pi/16,now=0.,pose=(0.,0.,0.))
        obstacle = wall.copy(); obstacle[15:18] = [.8,.81,.80]
        out, removed = f.process(obstacle,.1,8.,-math.pi,math.pi/16,
                                 moving=True,now=.12,pose=(.02,0.,0.))
        self.assertEqual((out, removed), (obstacle, 0))

    def test_far_two_ray_flash_removed_then_real_pair_confirmed(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 32
        f.process(wall,.1,8.,-math.pi,math.pi/16,now=0.,pose=(0.,0.,0.))
        pair = wall.copy(); pair[15:17] = [.8,.81]
        out, removed = f.process(pair,.1,8.,-math.pi,math.pi/16,
                                 moving=True,now=.12,pose=(.02,0.,0.))
        self.assertEqual(removed,2)
        self.assertTrue(all(math.isnan(v) for v in out[15:17]))
        out, removed = f.process(pair,.1,8.,-math.pi,math.pi/16,
                                 moving=True,now=.24,pose=(.04,0.,0.))
        self.assertEqual((out,removed),(pair,0))

    def test_far_single_cannot_confirm_a_new_pair(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 32
        f.process(wall,.1,8.,-math.pi,math.pi/16,now=0.,pose=(0.,0.,0.))
        single = wall.copy(); single[15] = .8
        f.process(single,.1,8.,-math.pi,math.pi/16,now=.12,pose=(0.,0.,0.))
        pair = wall.copy(); pair[15:17] = [.8,.81]
        out, removed = f.process(pair,.1,8.,-math.pi,math.pi/16,
                                 now=.24,pose=(0.,0.,0.))
        self.assertEqual(removed,2)
        self.assertTrue(all(math.isnan(v) for v in out[15:17]))

    def test_far_pair_survives_robot_rotation(self):
        f = module.ScanHistoryFilter()
        first = [2.] * 32
        first[16:18] = [.8,.81]
        f.process(first,.1,8.,-math.pi,math.pi/16,now=0.,pose=(0.,0.,0.))
        second = [2.] * 32
        second[15:17] = [.8,.81]
        out, removed = f.process(second,.1,8.,-math.pi,math.pi/16,
                                 moving=True,now=.12,pose=(.02,0.,math.pi/16))
        self.assertEqual((out,removed),(second,0))

    def test_clusters_do_not_join_across_depth_edges_but_wrap_at_360(self):
        valid = [True] * 5
        sizes = module.cluster_sizes([.8,.81,2.,2.01,2.02],valid,.07,.015,False)
        self.assertEqual(list(sizes),[2,2,3,3,3])
        sizes = module.cluster_sizes([.8,2.,2.,2.,.81],valid,.07,.015,True)
        self.assertEqual(list(sizes),[2,3,3,3,2])

    def test_history_expires_after_gap(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 16
        f.process(wall,.1,8.,0.,math.pi/8,now=0.,pose=(0.,0.,0.))
        obstacle = wall.copy(); obstacle[6] = .3
        out, removed = f.process(obstacle,.1,8.,0.,math.pi/8,
                                 now=.6,pose=(0.,0.,0.))
        self.assertEqual((out, removed), (obstacle, 0))

    def test_history_resets_if_scan_time_goes_backward(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 16
        f.process(wall,.1,8.,0.,math.pi/8,now=10.,pose=(0.,0.,0.))
        obstacle = wall.copy(); obstacle[6] = .3
        self.assertEqual(f.process(obstacle,.1,8.,0.,math.pi/8,
                                   now=9.9,pose=(0.,0.,0.)), (obstacle,0))

    def test_scan_uses_odom_from_acquisition_time(self):
        samples = [(10.00, 0., 0., 0.), (10.06, .06, 0., .2),
                   (10.12, .12, 0., .4)]
        self.assertEqual(module.closest_pose(samples, 10.055), (.06, 0., .2))
        self.assertIsNone(module.closest_pose(samples, 10.3))

    def test_missing_odom_during_motion_passes_raw_scan(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 16
        f.process(wall,.1,8.,0.,math.pi/8,now=0.)
        fresh = wall.copy(); fresh[5] = .28; fresh[8] = .82
        self.assertEqual(f.process(fresh,.1,8.,0.,math.pi/8,
                                   moving=True,now=.12), (fresh,0))


if __name__ == '__main__':
    unittest.main()
