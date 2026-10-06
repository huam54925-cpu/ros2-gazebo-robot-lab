"""Contract regressions; no ROS processes."""
import copy
import json
from pathlib import Path
import sys
import unittest
import yaml
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parents[1])]
from navigation_base.robot_contract import CONTRACT,nav2_parameters
from robot_skills.investigation_worker import passage_crossed


class ContractTests(unittest.TestCase):
    def test_both_costmaps_and_controller_derive_from_one_contract(self):
        config=yaml.safe_load((Path(__file__).resolve().parents[1]/'navigation/nav2.yaml').read_text())
        original=copy.deepcopy(config);derived=nav2_parameters(config,'footprint_075')
        self.assertEqual(config,original)
        for name in ('local_costmap','global_costmap'):
            params=derived[name][name]['ros__parameters']
            self.assertEqual(json.loads(params['footprint']),CONTRACT['footprint'])
            self.assertEqual(params['footprint_padding'],CONTRACT['body_padding_m'])
            self.assertTrue(params['track_unknown_space'])
        follow=derived['controller_server']['ros__parameters']['FollowPath']
        self.assertLessEqual(follow['max_linear_vel'],CONTRACT['profiles']['footprint_075']['max_linear_m_s'])
        self.assertTrue(follow['use_collision_detection'])
        behavior=derived['behavior_server']['ros__parameters']
        for name in ('backup','drive_on_heading'):
            self.assertEqual(behavior[name+'.minimum_speed'],CONTRACT['short_motion']['speed_m_s'])
            self.assertEqual(behavior[name+'.deceleration_limit'],-CONTRACT['linear_deceleration_m_s2'])
        self.assertEqual(behavior['local_frame'],CONTRACT['odom_frame'])
        self.assertEqual(behavior['max_rotational_vel'],CONTRACT['profiles']['footprint_075']['max_angular_rad_s'])

    def test_center_past_entry_is_not_whole_body_through(self):
        passage={'a':[0,-2],'b':[0,2],'destination_side':-1}
        self.assertFalse(passage_crossed([[-3,0,0],[.5,0,0]],passage))
        self.assertTrue(passage_crossed([[-3,0,0],[2,0,0]],passage))
        self.assertFalse(passage_crossed([[-3,4,0],[2,4,0]],passage))


if __name__=='__main__':unittest.main()
