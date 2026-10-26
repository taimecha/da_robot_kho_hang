"""Timing values must be computed from ordered milestones, not invented for missing runs."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from analyze_cargo_timing import summarize  # noqa: E402


class CargoTimingTest(unittest.TestCase):
    def test_pickup_delivery_total_and_incomplete_cycle(self):
        names = ('started', 'observation_arrived', 'measured', 'pickup_arrived',
                 'loaded_confirmed', 'delivery_arrived', 'unloaded_confirmed')
        moments = (10, 15, 20, 30, 38, 62, 67)
        records = [dict(run_id='r1', car='agv1', cycle=1, event=name,
                        t_monotonic=moment, deliver_col=2)
                   for name, moment in zip(names, moments)]
        records.append(dict(run_id='r1', car='agv2', cycle=1,
                            event='started', t_monotonic=11))
        rows, _ = summarize(records, 'sim')
        good, incomplete = rows
        self.assertEqual(good['status'], 'delivered')
        self.assertEqual(good['T_pickup_s'], 28)
        self.assertEqual(good['T_wait_load_s'], 8)
        self.assertEqual(good['T_delivery_s'], 24)
        self.assertEqual(good['T_total_s'], 52)
        self.assertEqual(good['T_to_unload_confirmed_s'], 57)
        self.assertEqual(incomplete['status'], 'incomplete_or_invalid')
        self.assertIsNone(incomplete['T_total_s'])


if __name__ == '__main__':
    unittest.main()
