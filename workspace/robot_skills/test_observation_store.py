import sys,time,tempfile,unittest,uuid,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_skills.store import Store

class ObservationStoreTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup); self.store=Store(tmp.name)
        self.catalog()
    def catalog(self,version='one'):
        self.store.set_meta('observation_catalog',{'expires_unix_s':time.time()+120,'options':[
          {'option_id':'OBS_'+version,'type':'rotate','start_pose':[0,0,0],'x':0,'y':0,'yaw':math.pi/2,
           'signed_angle_rad':math.pi/2,'map_version':version}]})
    def submit(self,rid=None,option='OBS_one'):
        return self.store.submit('perform_observation',{'option_id':option},rid or str(uuid.uuid4()))
    def test_unknown_and_expired_observations_are_rejected(self):
        task,launch=self.submit(option='OBS_missing'); self.assertFalse(launch)
        catalog=self.store.meta('observation_catalog'); catalog['expires_unix_s']=0; self.store.set_meta('observation_catalog',catalog)
        task,launch=self.submit(); self.assertFalse(launch); self.assertEqual(task['reason'],'observation_catalog_expired')
    def test_same_request_replays_same_task(self):
        rid=str(uuid.uuid4()); a,launch=self.submit(rid); b,again=self.submit(rid)
        self.assertTrue(launch); self.assertFalse(again); self.assertEqual(a['task_id'],b['task_id'])
    def test_new_catalog_cannot_erase_local_repeat_exclusion(self):
        task,_=self.submit(); self.store.update(task['task_id'],status='succeeded',stopped=True)
        self.catalog('two'); task,launch=self.submit(option='OBS_two')
        self.assertFalse(launch); self.assertEqual(task['reason'],'rotation_already_attempted_nearby')
    def test_observation_and_frontier_share_active_task_arbitration(self):
        self.submit(); task,launch=self.store.submit('fixed_step',{},str(uuid.uuid4()))
        self.assertFalse(launch); self.assertEqual(task['reason'],'another_task_active')

if __name__=='__main__': unittest.main()
