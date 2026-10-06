"""Direct exploration measurement and protocol regressions, without robot motion."""
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from pydantic import ValidationError
from scan_drive_geometry import scan_rays,half_range_goal,direction_summary,wrap


class ScanDriveTests(unittest.TestCase):
    def test_finite_and_no_return_ranges_use_half_measured_distance(self):
        rays=scan_rays([10.,math.inf,math.nan,-math.inf,.1],0.,.1,.2,12.,0.)
        self.assertEqual(len(rays),2)
        self.assertEqual([r['half_range_m'] for r in rays],[5.,6.])
        self.assertTrue(rays[1]['no_return_within_range'])

    def test_heading_selection_wraps_across_pi(self):
        rays=[{'heading_map_rad':math.pi-.01,'half_range_m':3.,'visible_range_m':6.},
              {'heading_map_rad':0.,'half_range_m':1.,'visible_range_m':2.}]
        goal=half_range_goal(rays,-math.pi+.01,[2.,1.,0.])
        self.assertEqual(goal['distance_m'],3.)
        self.assertAlmostEqual(math.dist(goal['target_map_xy'],[2.,1.]),3.)

    def test_summary_does_not_restrict_arbitrary_heading(self):
        rays=scan_rays([2.,8.,4.],0.,.01,.2,12.,0.)
        self.assertEqual(max(r['visible_range_m'] for r in direction_summary(rays)),8.)
        self.assertEqual(half_range_goal(rays,.02,[0,0,0])['distance_m'],2.)
        with self.assertRaises(ValueError):half_range_goal([],0.,[0,0,0])

    def test_direct_mcp_has_only_scan_drive_stop_and_explicit_heading(self):
        root=Path(__file__).resolve().parents[2];log=root/'workspace/log';log.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=log) as directory:
            Path(directory,'config.json').write_text(json.dumps({'deadline_unix_s':0}))
            spec=importlib.util.spec_from_file_location('scan_drive_protocol_test',Path(__file__).with_name('mcp_scan_drive_server.py'))
            module=importlib.util.module_from_spec(spec)
            with patch.dict(os.environ,{'SCAN_DRIVE_RUN_DIRECTORY':directory}):spec.loader.exec_module(module)
            tools={t.name:t for t in module.registered}
            self.assertEqual(set(tools),{'scan_surroundings','drive_half_visible_range','stop_exploration'})
            tool=tools['drive_half_visible_range'];self.assertEqual(set(tool.parameters['required']),{'heading_map_rad','rationale'})
            with self.assertRaises(ValidationError):tool.fn_metadata.arg_model.model_validate({'rationale':'explore'})
            with self.assertRaises(ValidationError):tool.fn_metadata.arg_model.model_validate({'heading_map_rad':0.,'rationale':'explore','speed':5.})
            with patch.object(module.subprocess,'run') as launch:
                self.assertEqual(module.action('drive',heading_map_rad=0.)['reason'],'run_wall_budget')
                launch.assert_not_called()


if __name__=='__main__':unittest.main()
