from pathlib import Path
import sys
import unittest
import json
import tempfile
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from exploration_support.frontier_identity import associate,candidate_identity
from exploration_support.memory import Event,EventStore
from dataclasses import asdict


class IdentityTests(unittest.TestCase):
    def test_move_split_merge_retains_lineage_across_new_candidate_ids(self):
        points=np.column_stack((np.arange(20)*.2,np.zeros(20)))
        first=associate([points]);identity=first['clusters'][0]['id']
        moved=associate([points+[0,.2]],first)
        self.assertEqual(moved['clusters'][0]['id'],identity)
        split=associate([points[:8],points[12:]],moved)
        self.assertEqual(len({c['id'] for c in split['clusters']}),2)
        self.assertTrue(all(identity in c['lineage'] and c['relation']=='split' for c in split['clusters']))
        merged=associate([points],split)
        self.assertEqual(merged['clusters'][0]['relation'],'merge')
        self.assertTrue({c['id'] for c in split['clusters']}<=set(merged['clusters'][0]['lineage']))
        self.assertEqual(candidate_identity({'x':2,'y':0,'id':'old'},merged),candidate_identity({'x':2,'y':0,'id':'new'},merged))

    def test_far_boundary_or_fresh_epoch_does_not_inherit_observations(self):
        points=np.array([[0,0],[.2,0],[.4,0]])
        first=associate([points]);second=associate([points+[10,0]],first)
        self.assertNotIn(first['clusters'][0]['id'],second['clusters'][0]['lineage'])
        self.assertEqual(associate([points])['clusters'][0]['parents'],[])

    def test_old_event_schema_replay_is_idempotent_without_rewriting(self):
        with tempfile.TemporaryDirectory() as directory:
            store=EventStore(Path(directory)/'events.sqlite3')
            event=Event('terminal:old','mission','epoch',1,(0.,0.,0.),'succeeded',True)
            old=asdict(event);old.pop('frontier_cluster_id');old.pop('frontier_lineage')
            payload=json.dumps(old)
            with store.connection() as con:
                con.execute('INSERT INTO exploration_events VALUES(?,?,?,?,?)',('terminal:old','mission','epoch',1,payload))
            self.assertFalse(store.append(event))
            self.assertEqual(store.events('mission','epoch'),[event])
            with store.connection() as con:
                self.assertEqual(con.execute('SELECT payload FROM exploration_events').fetchone()[0],payload)
