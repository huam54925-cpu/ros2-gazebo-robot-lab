import asyncio
import json
from pathlib import Path
import tempfile
import unittest

from robot_status import get_robot_status
from run_readonly import read_mcp, safe_error


class ReadonlyTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'status.json'
        self.data = {'schema_version': 1, 'generated_monotonic_s': 100,
                     'sources': {'clock': {'sim_time_s': 20, 'received_monotonic_s': 100},
                                 'scan': {'stamp_sim_s': 19.9, 'received_monotonic_s': 100}}}

    def read(self, now=100.2):
        self.path.write_text(json.dumps(self.data))
        return get_robot_status(self.path, now)

    def test_missing_and_old_snapshot_unavailable(self):
        self.assertEqual(get_robot_status(self.path, 100)['status'], 'unavailable')
        self.assertEqual(self.read(104)['status'], 'unavailable')
        self.assertEqual(self.read(99)['status'], 'unavailable')

    def test_recent_delivery_does_not_hide_old_sensor_stamp(self):
        self.data['sources']['scan']['stamp_sim_s'] = 10
        result = self.read()
        self.assertEqual(result['status'], 'available')
        self.assertFalse(result['sources']['scan']['fresh'])

    def test_wall_clock_timeout_even_when_sim_time_paused(self):
        self.data['generated_monotonic_s'] = 106
        result = self.read(106)
        self.assertFalse(result['sources']['scan']['fresh'])
        self.assertFalse(result['sources']['clock']['fresh'])

    def test_recent_status_and_no_motion(self):
        result = self.read()
        self.assertTrue(result['sources']['scan']['fresh'])
        self.assertFalse(result['motion_tools_enabled'])
        self.assertNotIn('navigation', result['sources'])

    def test_arbitrary_tool_or_arguments_never_dispatched(self):
        class Session:
            async def call_tool(self, *args, **kwargs):
                raise AssertionError('Must not call MCP for invalid tools')
        for name, arguments in [('navigate_to', {}), ('get_robot_status', {'command': 'move'})]:
            with self.assertRaises(ValueError):
                asyncio.run(read_mcp(Session(), name, arguments))

    def test_exception_group_does_not_expose_secrets(self):
        error = ExceptionGroup('private-key', [RuntimeError('private-key')])
        self.assertNotIn('private-key', json.dumps(safe_error(error)))


if __name__ == '__main__':
    unittest.main()
