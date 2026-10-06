import unittest
from pathlib import Path
import numpy as np
from navigation_base.diagnostics.coverage_monitor import reference,measure

class CoverageTests(unittest.TestCase):
    def test_fixed_free_area(self):
        path=Path(__file__).resolve().parents[1]/'worlds/indoor_mapping.sdf'
        points,area=reference(path)
        self.assertAlmostEqual(area,465.5)
        self.assertEqual(len(points),186200)

    def test_unknown_and_outside_do_not_count(self):
        grid={'origin':[0,0,0],'resolution':1.,'data':np.array([[0,-1],[100,0]])}
        points=np.array([[.5,.5],[1.5,.5],[.5,1.5],[1.5,1.5],[3.,3.]])
        self.assertEqual(measure(grid,points)['observed_free_fraction'],.4)
        larger={**grid,'data':np.pad(grid['data'],((0,2),(0,2)),constant_values=-1)}
        self.assertEqual(measure(grid,points),measure(larger,points))
