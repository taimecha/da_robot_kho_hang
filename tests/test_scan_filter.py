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

    def test_persistent_narrow_robot_survives(self):
        f = module.ScanHistoryFilter(history_size=5)
        wall = [2.] * 16
        first = wall.copy();first[6] = .8
        out, removed = f.process(first,.1,8.,0.,math.pi/8,now=0.)
        self.assertEqual((out,removed),(first,0))
        second = wall.copy();second[7] = .81
        out, removed = f.process(second,.1,8.,0.,math.pi/8,now=.12)
        self.assertEqual((out,removed),(second,0))

    def test_transient_spike_and_motion(self):
        f = module.ScanHistoryFilter()
        wall = [2.] * 16
        f.process(wall,.1,8.,0.,math.pi/8,now=0.)
        spike = wall.copy();spike[6] = .3
        out, removed = f.process(spike,.1,8.,0.,math.pi/8,now=.12)
        self.assertEqual(removed,1)
        self.assertTrue(math.isnan(out[6]))
        out, removed = f.process(spike,.1,8.,0.,math.pi/8,moving=True,now=.24)
        self.assertEqual((out,removed),(spike,0))


if __name__ == '__main__':
    unittest.main()
