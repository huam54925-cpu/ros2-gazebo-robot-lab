import json
import unittest
from pathlib import Path
import gymnasium as gym
import numpy as np
from training.train_vehicle import create_sac

class ShapeEnv(gym.Env):
    observation_space=gym.spaces.Box(-1,1,(295,),dtype=np.float32)
    action_space=gym.spaces.Box(-1,1,(2,),dtype=np.float32)

class ConfigTests(unittest.TestCase):
    def test_sb3_constructor_does_not_mutate_checkpoint_configuration(self):
        cfg=json.loads((Path(__file__).resolve().parents[1]/'training/configs/sac_local.json').read_text())['model']
        original=json.loads(json.dumps(cfg))
        cfg['buffer_size']=64
        original['buffer_size']=64
        model=create_sac(ShapeEnv(),7,'cpu',cfg)
        self.assertEqual(cfg,original)
        self.assertEqual(json.loads(json.dumps(cfg)),original)
        self.assertFalse(model.policy_kwargs['use_sde'])
        self.assertNotIn('use_sde',cfg['policy_kwargs'])

if __name__=='__main__':unittest.main()
