"""Host scheduler regression tests: no Docker daemon or GPU used here."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPT=Path(__file__).resolve().parents[1]/'scripts'/'long_train.py'
spec=importlib.util.spec_from_file_location('long_train',SCRIPT)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class SchedulerTests(unittest.TestCase):
    def setUp(self):m.STOP_REQUESTED=False
    def test_cli_timeout(self):
        with patch.object(m.subprocess,'run') as run:
            m.docker(['info']);self.assertEqual(run.call_args.kwargs['timeout'],30)
    def test_cleanup_after_start_log_failure(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)
            with patch.object(m,'docker'),patch.object(m,'event',side_effect=OSError('disk full')),patch.object(m,'cleanup_owned') as clean:
                with self.assertRaises(OSError):
                    m.run_container(['run'],out=out,run_id='a'*32,name='test',log_path=out/'log',deadline=time.monotonic()+10,operation_seconds=10)
                clean.assert_called_once_with('a'*32)
    def test_cleanup_when_log_open_fails(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)
            with patch.object(m,'docker',side_effect=subprocess.TimeoutExpired('docker',30)),patch.object(m,'cleanup_owned') as clean:
                with self.assertRaises(OSError):
                    m.run_container(['run'],out=out,run_id='a'*32,name='test',log_path=out/'missing'/'log',deadline=0,operation_seconds=1)
                clean.assert_called_once()
    def test_cleanup_fails_closed(self):
        with patch.object(m,'owned_containers',side_effect=subprocess.TimeoutExpired('docker',30)):
            with self.assertRaises(m.OwnershipUncertain):m.cleanup_owned('a'*32)
    def test_cleanup_checks_identity(self):
        with patch.object(m,'owned_containers',return_value=['cid']),patch.object(m,'docker') as docker:
            docker.return_value.stdout=json.dumps([{'Config':{'Labels':{m.LABEL:'b'*32}}}])
            with self.assertRaises(m.OwnershipUncertain):m.cleanup_owned('a'*32)
            self.assertEqual(docker.call_count,1)
    def test_cleanup_rescans_late_children(self):
        with patch.object(m,'owned_containers',side_effect=[['parent'],['child'],[]]),patch.object(m,'docker') as docker:
            docker.return_value.stdout=json.dumps([{'Config':{'Labels':{m.LABEL:'a'*32}}}])
            m.cleanup_owned('a'*32)
            self.assertEqual([c.args[0] for c in docker.call_args_list if c.args[0][0]=='rm'],[['rm','-f','parent'],['rm','-f','child']])
    def test_stop_is_not_eval_error(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d);(out/'STOP').touch();n=0
            def docker(args,**kw):
                nonlocal n
                if args[0]=='inspect':
                    n+=1
                    return subprocess.CompletedProcess(args,0,json.dumps([{'State':{'Running':n==1,'ExitCode':0}}]),'')
                return subprocess.CompletedProcess(args,0,'','')
            with patch.object(m,'docker',side_effect=docker),patch.object(m,'cleanup_owned'):
                self.assertEqual(m.run_container(['run'],out=out,run_id='a'*32,name='eval',log_path=out/'log',deadline=time.monotonic()+100,operation_seconds=10),(0,True))
    def test_watchdog_cleans_up(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)
            def docker(args,**kw):
                return subprocess.CompletedProcess(args,0,json.dumps([{'State':{'Running':True}}]),'')
            with patch.object(m,'docker',side_effect=docker),patch.object(m,'cleanup_owned') as clean:
                with self.assertRaisesRegex(RuntimeError,'watchdog'):
                    m.run_container(['run'],out=out,run_id='a'*32,name='t',log_path=out/'log',deadline=time.monotonic()+100,operation_seconds=-1)
                clean.assert_called_once()
    def test_formal_checkpoint_validation(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);leaf=root/'step-10-x';leaf.mkdir();(root/'latest.txt').write_text(leaf.name)
            for f in ('model.zip','replay.pkl'):(leaf/f).write_bytes(b'test')
            data={'schema':'local-drive-v2','environment':'LocalDriveEnv','profile':'formal','num_timesteps':10,
                  'files':{f:m.sha256(leaf/f) for f in ('model.zip','replay.pkl')}}
            (leaf/'manifest.json').write_text(json.dumps(data));self.assertEqual(m.verify_latest(root)[0],10)
            (leaf/'replay.pkl').write_bytes(b'corrupt')
            with self.assertRaisesRegex(RuntimeError,'hash mismatch'):m.verify_latest(root)
    def test_sim_label_and_pin(self):
        args=m.common_args(Path('/data'),'a'*32,'x','trainer','/run/docker.sock','sha256:sim')
        self.assertIn('LOCAL_DRIVE_RUN_ID='+'a'*32,args);self.assertIn('LOCAL_DRIVE_SIM_IMAGE=sha256:sim',args)
    def test_context_uses_selected_context(self):
        def docker(args,**kw):
            return subprocess.CompletedProcess(args,0,'selected' if args==['context','show'] else 'unix:///run/mock.sock','')
        with patch.dict(m.os.environ,{},clear=True),patch.object(m,'docker',side_effect=docker),patch.object(Path,'exists',return_value=True):
            self.assertEqual(m.resolve_context(),('selected','/run/mock.sock'))
    def test_atomic_json(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'state.json';m.atomic_json(p,{'phase':'stopped'})
            self.assertEqual(json.loads(p.read_text()),{'phase':'stopped'});self.assertEqual(len(list(Path(d).iterdir())),1)

if __name__=='__main__':unittest.main()
