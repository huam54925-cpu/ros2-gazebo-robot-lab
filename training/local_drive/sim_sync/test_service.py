"""Transport ambiguity checks; execute with native Gazebo Python bindings."""
import unittest
from unittest.mock import Mock, patch
from worker import Worker, ServiceUnavailable

class ServiceTests(unittest.TestCase):
    def setUp(self):self.worker=Worker.__new__(Worker)
    def test_read_only_empty_response_retries(self):
        with patch.object(self.worker,'service',side_effect=[ServiceUnavailable('no reply'),ServiceUnavailable('empty'),'{"action_id":7}']) as service,patch('worker.time.sleep'):
            self.assertEqual(self.worker.status(),{'action_id':7});self.assertEqual(service.call_count,3)
    def test_empty_response_retry_is_bounded(self):
        with patch.object(self.worker,'service',side_effect=ServiceUnavailable('no reply')) as service,patch('worker.time.sleep'):
            with self.assertRaises(ServiceUnavailable):self.worker.status()
            self.assertEqual(service.call_count,3)
    def test_mutation_is_never_replayed(self):
        self.worker.client=Mock();self.worker.client.stdout.readline.return_value='{"ok":false}'
        for name,kind,body in (('/local_drive/command','gz.msgs.StringMsg','data: "x"'),('/world/demo/control','gz.msgs.WorldControl','pause: true multi_step: 200')):
            self.worker.client.reset_mock()
            with patch('worker.selectors.DefaultSelector') as selector:
                selector.return_value.__enter__.return_value.select.return_value=[True]
                with self.assertRaises(ServiceUnavailable):self.worker.service(name,kind,'Boolean',body)
            self.assertEqual(self.worker.client.stdin.write.call_count,1)
    def test_malformed_nonempty_ack_fails(self):
        with patch.object(self.worker,'service',return_value='not json') as service:
            with self.assertRaises(ValueError):self.worker.status()
            self.assertEqual(service.call_count,1)
if __name__=='__main__':unittest.main()
