from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import tempfile
import time
import unittest
import uuid
import gc
import os

from store import Store


class StoreTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.store = Store(Path(temp.name))
        self.store.set_meta('catalog', {'expires_unix_s': time.time()+120, 'candidates': [
            {'frontier_id': 'F_test_1', 'x': 2, 'y': 3, 'yaw': 0, 'map_version': 'v1'},
            {'frontier_id': 'F_test_2', 'x': 4, 'y': 3, 'yaw': 0, 'map_version': 'v1'}]})

    def submit(self, request=None, frontier='F_test_1'):
        return self.store.submit('execute_frontier', {'frontier_id': frontier}, request or str(uuid.uuid4()))

    def test_idempotency_and_conflicting_reuse(self):
        rid = str(uuid.uuid4()); first, launch = self.submit(rid)
        second, again = self.submit(rid)
        self.assertTrue(launch); self.assertFalse(again)
        self.assertEqual(first['task_id'], second['task_id'])
        with self.assertRaisesRegex(ValueError, 'request_id_conflict'):
            self.submit(rid, 'F_test_2')

    def test_concurrent_admission_allows_only_one(self):
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda _: self.submit(), range(4)))
        self.assertEqual(sum(launch for _, launch in results), 1)

    def test_unknown_or_expired_candidate_never_admitted(self):
        task, launch = self.submit(frontier='F_missing')
        self.assertFalse(launch); self.assertEqual(task['reason'], 'unknown_frontier_id')
        catalog = self.store.meta('catalog'); catalog['expires_unix_s'] = 0
        self.store.set_meta('catalog', catalog)
        task, launch = self.submit()
        self.assertFalse(launch); self.assertEqual(task['reason'], 'frontier_catalog_expired')

    def test_stop_has_priority_and_blocks_future_requests(self):
        move, _ = self.submit()
        stop, launch = self.store.submit('stop_robot', {}, str(uuid.uuid4()))
        self.assertTrue(launch)
        self.assertTrue(self.store.should_cancel(move['task_id']))
        rejected, launch = self.submit(frontier='F_test_2')
        self.assertFalse(launch); self.assertEqual(rejected['reason'], 'stop_latched')
        with self.assertRaises(ValueError):
            self.store.clear_stop_after_verified_resume()

    def test_terminal_state_cannot_be_resurrected(self):
        task, _ = self.submit()
        self.store.update(task['task_id'], status='succeeded', stopped=True)
        after = self.store.update(task['task_id'], status='running')
        self.assertEqual(after['status'], 'succeeded')
        self.assertFalse(after['running'])

    def test_failed_candidate_cannot_be_resent_with_new_id(self):
        task, _ = self.submit()
        self.store.update(task['task_id'], status='aborted', stopped=True)
        task, launch = self.submit()
        self.assertFalse(launch); self.assertEqual(task['reason'], 'frontier_already_attempted')

    def test_unconfirmed_stop_blocks_other_candidate(self):
        task, _ = self.submit()
        self.store.update(task['task_id'], status='stop_unconfirmed')
        task, launch = self.submit(frontier='F_test_2')
        self.assertFalse(launch); self.assertEqual(task['reason'], 'previous_stop_unconfirmed')

    def test_uniform_schema_and_persistence(self):
        task, _ = self.submit()
        required = {'request_id','task_id','accepted','running','stopped','reason','sensor_fresh','map_version'}
        self.assertTrue(required <= task.keys())
        self.assertEqual(Store(self.store.directory).get(task['task_id']), task)

    def test_new_stop_cannot_be_cleared_by_an_older_resume(self):
        task, _ = self.store.submit('stop_robot', {}, str(uuid.uuid4()))
        self.store.update(task['task_id'], status='succeeded', stopped=True)
        with self.assertRaisesRegex(ValueError, 'stop_changed_during_resume'):
            self.store.clear_stop_after_verified_resume(0)
        self.assertTrue(self.store.meta('stop_latched'))
        self.store.clear_stop_after_verified_resume(1)
        self.assertFalse(self.store.meta('stop_latched'))

    def test_repeated_reads_close_file_descriptors_without_gc(self):
        before = len(os.listdir('/proc/self/fd'))
        gc.disable()
        try:
            for _ in range(1200):
                self.store.meta('stop_latched')
            self.assertLessEqual(len(os.listdir('/proc/self/fd')), before+2)
        finally:
            gc.enable()

    def test_recovery_preserves_failure_and_stop_latch(self):
        task, _ = self.submit()
        self.store.update(task['task_id'], status='indeterminate', reason='worker_failure')
        self.store.set_meta('stop_latched', True)
        recovered = self.store.record_stop_recovery(task['task_id'], {'verified': True})
        self.assertEqual(recovered['status'], 'indeterminate')
        self.assertEqual(recovered['reason'], 'worker_failure')
        self.assertTrue(recovered['stopped'] and recovered['resolved'])
        self.assertTrue(self.store.meta('stop_latched'))


if __name__ == '__main__':
    unittest.main()
