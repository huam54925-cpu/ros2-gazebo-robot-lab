"""V2 mount selection and optional lidar gate, no ROS or motion."""
import builtins
import math
import unittest
from unittest.mock import patch
from safety_mode import resolve,runtime_policy,load_mount


class SafetyModeTests(unittest.TestCase):
    def test_off_never_imports_optional_mount(self):
        original=builtins.__import__
        def forbid(name,*args,**kwargs):
            if name.startswith('mounts'):raise AssertionError('off imported a gate')
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=forbid):
            self.assertIsNone(load_mount(resolve('off')))

    def test_auto_resolves_from_environment_and_explicit_modes_win(self):
        self.assertFalse(resolve('auto','gazebo')['enabled'])
        self.assertTrue(resolve('auto','unknown')['enabled'])
        self.assertFalse(resolve('off','unknown')['enabled'])
        self.assertTrue(resolve('on','gazebo')['enabled'])
        with self.assertRaises(ValueError):resolve('false')

    def test_runtime_is_bound_to_explicit_v2_gazebo_launch(self):
        with self.assertRaises(RuntimeError):runtime_policy({})
        with self.assertRaises(RuntimeError):runtime_policy({'ROBOT_RUNTIME':'scan-drive-v2','ROBOT_ENVIRONMENT':'physical'})
        self.assertFalse(runtime_policy({'ROBOT_RUNTIME':'scan-drive-v2','ROBOT_ENVIRONMENT':'gazebo'})['enabled'])

    def test_on_mount_rejects_body_and_future_forward_hit(self):
        mount=load_mount(resolve('on'))
        self.assertFalse(mount.check([[0.,0.]],0.,0.)['allowed'])
        self.assertTrue(mount.check([[.55,0.]],0.,0.)['allowed'])
        self.assertFalse(mount.check([[.55,0.]],.6,0.)['allowed'])
        self.assertTrue(mount.check([[.55,0.]],-.6,0.)['allowed'])
        self.assertTrue(mount.check([[5.,5.]],.6,.8)['allowed'])

    def test_on_mount_checks_rotation_and_invalid_samples(self):
        mount=load_mount(resolve('on'))
        self.assertTrue(mount.check([[-1.5,-.95]],0.,0.)['allowed'])
        self.assertFalse(mount.check([[-1.5,-.95]],0.,.8)['allowed'])
        self.assertFalse(mount.check([[math.nan,0.]],0.,0.)['allowed'])

    def test_renderer_has_no_old_explorer_dependency(self):
        import numpy as np
        from map_view import render
        grid={'data':np.array([[-1,-1,-1],[-1,0,100],[-1,-1,-1]]),
              'origin':[0.,0.,0.],'resolution':1.,'frame':'map','stamp_sim_s':1.}
        view=render(grid,[1.,1.,0.],'v2','test',[],'off')
        self.assertTrue(view['unknown_regions'])
        self.assertEqual(view['source'],'online_slam_only')


if __name__=='__main__':unittest.main()
