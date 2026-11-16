"""Exercise the local HTTP boundary with a simulated fleet, never ROS."""

from http.client import HTTPConnection
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from leap_fleet.web_console import Console, ConsoleServer, DemoMonitor
from leap_fleet.demo_connections import DemoConnections


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        config = Path(self.temp.name) / 'fleet.json'
        config.write_text(json.dumps({'cars': [{'name': 'car01', 'domain_id': 61}]}))
        self.monitor = DemoMonitor(config)
        self.console = Console(self.monitor, self.monitor.fleet, mode='demo')
        self.server = ConsoleServer(self.console, port=0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close)

    def close(self):
        self.server.shutdown()
        self.thread.join(2)
        self.server.server_close()
        self.console.close()

    def request(self, method, path, body=None, headers=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=4)
        try:
            connection.request(method, path, json.dumps(body) if body is not None else None,
                               headers or {})
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    def headers(self):
        return {'X-CSRF-Token': self.server.csrf_token, 'Content-Type': 'application/json'}

    def test_observation_is_read_only_and_returns_mode_and_token(self):
        status, data = self.request('GET', '/api/state')
        state = json.loads(data)
        self.assertEqual(status, 200)
        self.assertEqual(state['mode'], 'demo')
        self.assertEqual(state['csrf_token'], self.server.csrf_token)
        self.assertIsNone(state['job'])

    def test_other_origins_and_missing_token_cannot_send_commands(self):
        body = {'commands': {'car01': {'v': 0.05, 'w': 0}}, 'duration': 1}
        for headers in ({'Content-Type': 'application/json'},
                        {**self.headers(), 'Origin': 'https://example.com'},
                        {**self.headers(), 'Host': 'malicious.example'}):
            status, _ = self.request('POST', '/api/run', body, headers)
            self.assertEqual(status, 403)
            self.assertIsNone(self.console.snapshot()['job'])

    def test_finite_motion_via_http_and_stop(self):
        body = {'commands': {'car01': {'v': 0.05, 'w': 0.1}}, 'duration': 10}
        status, data = self.request('POST', '/api/run', body, self.headers())
        self.assertEqual(status, 200, data)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and self.monitor.snapshot()[0]['measured_v'] == 0:
            time.sleep(0.01)
        self.assertEqual(self.monitor.snapshot()[0]['measured_v'], 0.05)
        status, data = self.request('POST', '/api/stop', {'scope': 'job'}, self.headers())
        self.assertEqual(status, 200, data)
        self.assertEqual(json.loads(data)['job']['status'], 'stopping')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and self.console.snapshot()['job']['status'] == 'stopping':
            time.sleep(0.01)
        self.assertEqual(self.console.snapshot()['job']['status'], 'stopped')
        self.assertEqual(self.monitor.snapshot()[0]['measured_v'], 0)

    def test_invalid_or_unconnected_targets_never_start(self):
        self.monitor.cars['car01']['status'] = 'offline'
        for body in ({'commands': {'car01': {'v': 0.05, 'w': 0}}, 'duration': 1},
                     {'commands': {}, 'duration': 1},
                     {'commands': {'car01': {'v': 0, 'w': 0}}, 'duration': 1, 'typo': 1}):
            status, _ = self.request('POST', '/api/run', body, self.headers())
            self.assertEqual(status, 400)
            self.assertIsNone(self.console.snapshot()['job'])

    def test_assets_served_but_unlisted_paths_are_not(self):
        for path in ('/', '/app.js', '/style.css'):
            status, data = self.request('GET', path)
            self.assertEqual(status, 200)
            self.assertGreater(len(data), 0)
        self.assertEqual(self.request('GET', '/../../config/fleet.json')[0], 404)

    def test_connection_actions_require_csrf_and_valid_exact_payload(self):
        self.console.connections = DemoConnections(self.monitor)
        body = {'action': 'connect', 'cars': ['car01']}
        for headers in ({'Content-Type': 'application/json'},
                        {**self.headers(), 'Origin': 'https://example.com'}):
            status, _ = self.request('POST', '/api/connections', body, headers)
            self.assertEqual(status, 403)
        for invalid in ({**body, 'command': 'anything'},
                        {'action': 'connect', 'cars': ['unknown']},
                        {'action': 'connect', 'cars': ['car01', 'car01']},
                        {'action': 'connect', 'cars': 'car01'},
                        {'action': 'connect', 'cars': []},
                        {'action': 'run', 'cars': ['car01']},
                        {'action': 'connect'}):
            status, _ = self.request('POST', '/api/connections', invalid, self.headers())
            self.assertEqual(status, 400, invalid)
        self.assertFalse(self.console.connections.busy_for(['car01']))
        self.assertIsNone(self.console.snapshot()['job'])

    def test_connect_and_cancel_demo_never_create_motion_job(self):
        self.monitor.cars['car01'].update(status='offline', measured_v=None,
                                           measured_w=None, cmd_subscribers=0)
        self.console.connections = DemoConnections(self.monitor)
        body = {'action': 'connect', 'cars': ['car01']}
        status, data = self.request('POST', '/api/connections', body, self.headers())
        self.assertEqual(status, 200, data)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and self.console.connections.busy_for(['car01']):
            time.sleep(0.01)
        state = self.console.snapshot()
        self.assertIsNone(state['job'])
        self.assertEqual(state['cars'][0]['status'], 'connected')
        self.assertTrue(state['connections']['cars'][0]['owned'])
        self.assertEqual(state['cars'][0]['measured_v'], 0)
        status, data = self.request('POST', '/api/connections',
                                    {'action': 'cancel', 'cars': ['car01']}, self.headers())
        self.assertEqual(status, 200, data)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and self.console.connections.busy_for(['car01']):
            time.sleep(0.01)
        self.assertEqual(self.monitor.snapshot()[0]['status'], 'offline')
        self.assertIsNone(self.console.snapshot()['job'])


if __name__ == '__main__':
    unittest.main()
