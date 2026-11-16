import json
from pathlib import Path
import tempfile
import threading
import unittest

from leap_fleet.firmware_session import collect_car, record_firmware, SessionLogger


class FirmwareSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.car = {'name': 'car11', 'ip': '10.1.1.236'}
        self.chassis = {
            'device_name': 'Maturo_80456B56A820',
            'firmware': {'version': '2026.11.16-runtime1', 'project_name': 'chassis',
                         'app_sha256': 'a' * 64, 'password': 'SECRET'},
            'camera': {'ip': '10.1.1.233', 'name': 'Maturo_CAM_1CDBD4575FD4'},
            'runtime': {'wifi_password': 'SECRET'}, 'wifi': {'power_save': 0, 'password': 'SECRET'}}
        self.camera = {'name': 'Maturo_CAM_1CDBD4575FD4',
                       'firmware': {'version': '2026.11.16-runtime1', 'project': 'gc0308_http_stream'},
                       'time_sync': {'server': '10.1.1.249', 'synced': False, 'last_sync_unix_us': 0},
                       'wifi': {'power_save': False, 'password': 'SECRET'}, 'settings': {'key': 'SECRET'}}

    def fetch(self, ip, timeout):
        return self.chassis if ip == '10.1.1.236' else self.camera

    def test_both_versions_and_no_credentials(self):
        result = collect_car(self.car, fetcher=self.fetch)
        self.assertEqual(result['chassis']['status'], 'ok')
        self.assertEqual(result['camera']['status'], 'ok')
        self.assertEqual(result['camera']['mac'], '1c:db:d4:57:5f:d4')
        self.assertIs(result['camera']['time_sync']['synced'], False)
        self.assertNotIn('SECRET', json.dumps(result))
        self.assertNotIn('settings', result['camera'])

    def test_missing_version_remains_unknown(self):
        self.camera.pop('firmware')
        result = collect_car(self.car, fetcher=self.fetch)
        self.assertEqual(result['camera']['status'], 'version_unavailable')

    def test_cjson_scientific_notation_preserves_exact_sync_timestamp(self):
        self.camera['time_sync']['last_sync_unix_us'] = json.loads('1.79000000000001e15')
        result = collect_car(self.car, fetcher=self.fetch)
        self.assertEqual(result['camera']['time_sync']['last_sync_unix_us'], 1790000000000010)
        for invalid in (True, float('nan'), float('inf'), -1, 1.5, 2**53):
            self.camera['time_sync']['last_sync_unix_us'] = invalid
            self.assertNotIn('last_sync_unix_us', collect_car(self.car, fetcher=self.fetch)['camera']['time_sync'])

    def test_old_firmware_404_is_explicit_and_error_body_not_logged(self):
        def unavailable(ip, timeout):
            if ip == self.car['ip']:
                return self.chassis
            raise OSError('SECRET response body')
        result = collect_car(self.car, fetcher=unavailable)
        self.assertEqual(result['camera']['status'], 'unavailable')
        self.assertNotIn('SECRET', json.dumps(result))

    def test_missing_chassis_does_not_guess_camera(self):
        calls = []
        def unavailable(ip, timeout):
            calls.append(ip)
            raise TimeoutError()
        result = collect_car(self.car, fetcher=unavailable)
        self.assertEqual(calls, [self.car['ip']])
        self.assertEqual(result['camera']['status'], 'not_discovered')

    def test_camera_identity_mismatch_is_logged(self):
        self.camera['name'] = 'Maturo_CAM_000000000002'
        result = collect_car(self.car, fetcher=self.fetch)
        self.assertEqual(result['camera']['status'], 'identity_mismatch')

    def test_missing_camera_identity_never_reports_success(self):
        self.camera.pop('name')
        result = collect_car(self.car, fetcher=self.fetch)
        self.assertEqual(result['camera']['status'], 'identity_unverified')
        self.assertEqual(result['camera']['firmware']['version'], '2026.11.16-runtime1')
        self.camera['name'] = self.chassis['camera'].pop('name')
        self.assertEqual(collect_car(self.car, fetcher=self.fetch)['camera']['status'], 'identity_unverified')

    def test_untrusted_camera_address_not_requested(self):
        self.chassis['camera']['ip'] = '1.1.1.1'
        calls = []
        def fetched(ip, timeout):
            calls.append(ip)
            return self.chassis
        result = collect_car(self.car, fetcher=fetched)
        self.assertEqual(calls, ['10.1.1.236'])
        self.assertEqual(result['camera']['status'], 'unavailable')

    def test_async_writes_pending_without_waiting_for_network(self):
        entered, release = threading.Event(), threading.Event()
        def waiting(ip, timeout):
            entered.set()
            release.wait(2)
            return self.fetch(ip, timeout)
        with tempfile.TemporaryDirectory() as temp:
            path, worker = record_firmware([self.car], Path(temp) / 'session.json',
                                           asynchronous=True, fetcher=waiting)
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(json.loads(path.read_text())['collection_status'], 'pending')
            finally:
                release.set()
                worker.join(3)
            data = json.loads(path.read_text())
            self.assertEqual(data['collection_status'], 'complete')
            self.assertEqual(data['cars'][0]['chassis']['firmware']['version'], '2026.11.16-runtime1')

    def test_snapshot_never_overwrites_another_session(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'session.json'
            path.write_text('original')
            with self.assertRaises(FileExistsError):
                record_firmware([], path)
            self.assertEqual(path.read_text(), 'original')

    def test_session_logger_uses_selected_cars_and_new_files(self):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / 'fleet.json'
            # Missing IP prevents any network access and is an explicit error record.
            config.write_text(json.dumps({'cars': [{'name': 'car01'}, {'name': 'car02'}]}))
            logger = SessionLogger(config, log_dir=Path(temp) / 'logs')
            first = Path(logger.record(['car01'], session_id='job1', kind='velocity'))
            second = Path(logger.record(['car02'], session_id='job2', kind='trajectory'))
            logger.close()
            self.assertEqual(json.loads(first.read_text())['cars'][0]['car'], 'car01')
            self.assertEqual(json.loads(second.read_text())['cars'][0]['car'], 'car02')
            self.assertNotEqual(first, second)


if __name__ == '__main__':
    unittest.main()
