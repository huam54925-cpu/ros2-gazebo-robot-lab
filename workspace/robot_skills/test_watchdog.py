import sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_skills.api import RobotSkills
from robot_skills.store import Store
from robot_skills import mission,mission_watchdog

class WatchdogTests(unittest.TestCase):
    def test_lost_client_finishes_with_stop_and_keeps_latch(self):
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory); skills=RobotSkills(store)
            state={'status':'available','stop_latched':False,'sources':{'clock':{'fresh':True,'sim_time_s':1},
                   'odometry':{'fresh':True,'velocity':{'x':0,'z':0}}}}
            m=mission.start(store,mission.limits(),state)
            mission.update(store,m['mission_id'],heartbeat_monotonic_s=time.monotonic()-16)
            def stop(rid):
                task,_=store.submit('stop_robot',{},rid)
                return store.update(task['task_id'],status='succeeded',stopped=True)
            with patch('robot_skills.mission_watchdog.RobotSkills',return_value=skills), \
                    patch.object(skills,'get_robot_state',return_value=state),patch.object(skills,'stop_robot',side_effect=stop):
                mission_watchdog.run(m['mission_id'])
            finished=store.meta('mission:'+m['mission_id'])
            self.assertEqual(finished['stop_reason'],'client_heartbeat_lost')
            self.assertTrue(finished['stopped']); self.assertEqual(finished['status'],'finished')
            self.assertTrue(store.meta('stop_latched')); self.assertIsNone(store.meta('active_mission'))

if __name__=='__main__': unittest.main()
