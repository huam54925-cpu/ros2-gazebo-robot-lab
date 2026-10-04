import math
import unittest
from observation_rules import exclusion

class ObservationRuleTests(unittest.TestCase):
    def task(self,angle,point=(0,0,0),gain='FEEDBACK_INCOMPLETE'):
        return {'kind':'perform_observation','accepted':True,'candidate':{'type':'rotate','start_pose':point,'signed_angle_rad':angle},
                'result':{'observation_metrics':{'gain_status':gain}}}
    def option(self,angle,point=(0,0,0)):
        return {'type':'rotate','start_pose':point,'signed_angle_rad':angle}
    def test_new_id_or_map_version_cannot_repeat_same_rotation(self):
        self.assertEqual(exclusion(self.option(math.pi/2),[self.task(math.pi/2)]),'rotation_already_attempted_nearby')
    def test_absolute_angle_budget_counts_opposite_directions(self):
        tasks=[self.task(math.pi/2),self.task(-math.pi/2)]
        self.assertEqual(exclusion(self.option(math.pi/4),tasks),'local_rotation_budget')
    def test_two_confirmed_low_gains_block_but_missing_evidence_does_not(self):
        tasks=[self.task(math.pi/4,gain='LOW_GAIN'),self.task(-math.pi/4,gain='LOW_GAIN')]
        self.assertEqual(exclusion(self.option(math.pi/2),tasks),'repeated_low_gain')
        tasks=[self.task(math.pi/4),self.task(-math.pi/4)]
        self.assertIsNone(exclusion(self.option(math.pi/2),tasks))

if __name__=='__main__': unittest.main()
