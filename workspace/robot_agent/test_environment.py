"""Verify no requests without keys, official endpoint restriction, and redaction."""
import json
from pathlib import Path
import tempfile
import unittest

import httpx2

from check_environment import OFFICIAL_URL, check_api, configuration


class EnvironmentTests(unittest.TestCase):
    def test_configuration_precedence_and_official_endpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            env = Path(directory) / '.env'
            env.write_text('OPENAI_API_KEY=file-key\nOPENAI_BASE_URL=https://api.openai.com/v1\n')
            self.assertEqual(configuration(env, {})[0], 'file-key')
            self.assertEqual(configuration(env, {'OPENAI_API_KEY': 'process-key'})[0],
                             'process-key')
            self.assertEqual(configuration(env, {'OPENAI_API_KEY': ''})[0], '')
            with self.assertRaises(ValueError):
                configuration(env, {'OPENAI_BASE_URL': 'https://other.example/v1'})

    def test_missing_key_never_calls_network(self):
        def forbidden(request):
            self.fail('A missing API key must not cause network access')
        result, code = check_api('', OFFICIAL_URL, httpx2.MockTransport(forbidden))
        self.assertEqual(code, 2)
        self.assertFalse(result['authenticated'])

    def test_read_only_models_call(self):
        def respond(request):
            self.assertEqual(request.method, 'GET')
            self.assertEqual(str(request.url), OFFICIAL_URL + '/models')
            self.assertEqual(request.headers['Authorization'], 'Bearer test-key')
            return httpx2.Response(200, json={'object': 'list', 'data': []})
        result, code = check_api('test-key', OFFICIAL_URL, httpx2.MockTransport(respond))
        self.assertEqual(code, 0)
        self.assertTrue(result['authenticated'])

    def test_http_error_does_not_echo_secret(self):
        def respond(request):
            return httpx2.Response(401, json={'error': {'message': 'private-test-key'}})
        result, code = check_api('private-test-key', OFFICIAL_URL,
                                 httpx2.MockTransport(respond))
        self.assertEqual(code, 3)
        self.assertEqual(result['http_status'], 401)
        self.assertNotIn('private-test-key', json.dumps(result))


if __name__ == '__main__':
    unittest.main()
