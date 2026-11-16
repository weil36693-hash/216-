"""Bringup lifecycle reporting with fake processes and no ROS/network traffic."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import fleet_bringup


class BringupStatusTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.status = self.root / 'status.json'
        self.config = self.root / 'fleet.json'
        self.config.write_text(json.dumps({'cars': [{
            'name': 'car04', 'ip': '198.51.100.4', 'domain_id': 4,
            'agent_port': 20004, 'pose_topic': '/vrpn_mocap/car_04/pose',
        }]}), encoding='utf-8')

    def read_status(self):
        return json.loads(self.status.read_text(encoding='utf-8'))

    def invoke(self, *extra, include_status=True):
        arguments = ['fleet_bringup.py', '--config', str(self.config), *extra]
        if include_status:
            arguments += ['--status-file', str(self.status)]
        with patch.object(sys, 'argv', arguments), redirect_stdout(io.StringIO()), \
                redirect_stderr(io.StringIO()):
            return fleet_bringup.main()

    def fake_environment(self, *, request_error=None, probe_code=0, cleanup_error=None):
        stack = ExitStack()
        self.addCleanup(stack.close)
        self.events, self.commands = [], []
        agent = self.root / 'install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent'
        library = self.root / 'vendor/install/lib/libmicroxrcedds_agent.so'
        agent.parent.mkdir(parents=True)
        library.parent.mkdir(parents=True)
        agent.write_bytes(b'fake executable; never launched')
        library.write_bytes(b'fake LEAP_AGENT_DOMAIN_ID library; never loaded')
        owner = self

        class Processes:
            def start(self, command, log, env=None):
                owner.commands.append(list(command))
                code = probe_code if any('fleet_probe.py' in str(part) for part in command) else None
                return SimpleNamespace(poll=lambda: code)

            def close(self):
                owner.events.append('cleanup')
                if cleanup_error:
                    raise cleanup_error

        original_update = fleet_bringup.StatusFile.update

        def update(status, phase, message, **options):
            owner.events.append(phase)
            return original_update(status, phase, message, **options)

        def sleep(seconds):
            if owner.status.exists() and owner.read_status()['phase'] == 'ready':
                raise KeyboardInterrupt

        request = MagicMock(return_value={
            'comm_mode': 'micro_ros', 'microros_agent_ip': '192.0.2.1',
            'microros_agent_port': 20004}, side_effect=request_error)
        original_open = open

        def retain_test_lock(*arguments, **options):
            output = original_open(*arguments, **options)
            self.addCleanup(output.close)
            return output

        stack.enter_context(patch.object(fleet_bringup, 'ROOT', self.root))
        stack.enter_context(patch.object(fleet_bringup, 'open', retain_test_lock, create=True))
        stack.enter_context(patch.object(fleet_bringup, 'Processes', Processes))
        stack.enter_context(patch.object(fleet_bringup, 'runtime_request', request))
        stack.enter_context(patch.object(fleet_bringup.socket, 'socket', MagicMock()))
        stack.enter_context(patch.object(fleet_bringup.subprocess, 'run', MagicMock()))
        stack.enter_context(patch.object(fleet_bringup.time, 'sleep', sleep))
        stack.enter_context(patch.object(fleet_bringup.StatusFile, 'update', update))
        stack.enter_context(patch.dict(os.environ))
        return request

    def test_atomic_replacement_is_complete_and_keeps_log_directory(self):
        reporter = fleet_bringup.StatusFile(self.status)
        reporter.update('checking', '检查中', log_dir=self.root / 'logs')
        replace = os.replace

        def inspect_replace(source, destination):
            self.assertEqual(Path(source).parent, self.status.parent)
            self.assertEqual(json.loads(Path(source).read_text())['phase'], 'waiting_raw')
            self.assertEqual(self.read_status()['phase'], 'checking')
            return replace(source, destination)

        with patch.object(fleet_bringup.os, 'replace', side_effect=inspect_replace):
            reporter.update('waiting_raw', '等待原始数据')
        result = self.read_status()
        self.assertEqual(set(result), {'phase', 'message', 'log_dir', 'error', 'updated_at'})
        self.assertEqual(result['phase'], 'waiting_raw')
        self.assertEqual(result['log_dir'], str(self.root / 'logs'))
        self.assertIsNone(result['error'])
        self.assertIsInstance(result['updated_at'], float)
        self.assertEqual(list(self.root.glob('.*.tmp')), [])

    def test_failed_atomic_replace_keeps_previous_json_and_removes_temporary(self):
        reporter = fleet_bringup.StatusFile(self.status)
        reporter.update('checking', '检查中')
        with patch.object(fleet_bringup.os, 'replace', side_effect=OSError('replace failed')):
            with self.assertRaisesRegex(OSError, 'replace failed'):
                reporter.update('ready', '就绪')
        self.assertEqual(self.read_status()['phase'], 'checking')
        self.assertEqual(list(self.root.glob('.*.tmp')), [])

    def test_failed_status_cannot_be_overwritten_by_stopped(self):
        reporter = fleet_bringup.StatusFile(self.status)
        reporter.update('failed', '启动失败', error='connection refused')
        original = self.read_status()
        reporter.update('stopped', '已清理')
        self.assertEqual(self.read_status(), original)

    def test_dry_run_writes_stopped_without_starting_or_configuring_anything(self):
        with patch.object(fleet_bringup, 'runtime_request') as request, \
                patch.object(fleet_bringup.subprocess, 'run') as run, \
                patch.object(fleet_bringup.socket, 'socket') as socket:
            self.assertEqual(self.invoke('--dry-run'), 0)
        self.assertEqual(self.read_status()['phase'], 'stopped')
        request.assert_not_called()
        run.assert_not_called()
        socket.assert_not_called()

    def test_without_status_option_dry_run_produces_no_files(self):
        before = set(self.root.iterdir())
        self.assertEqual(self.invoke('--dry-run', include_status=False), 0)
        self.assertEqual(set(self.root.iterdir()), before)

    def test_frontend_configuration_validation_failure_is_reported(self):
        with patch.object(fleet_bringup, 'validate', side_effect=ValueError('duplicate domain')):
            with self.assertRaisesRegex(ValueError, 'duplicate domain'):
                self.invoke('--dry-run')
        result = self.read_status()
        self.assertEqual(result['phase'], 'failed')
        self.assertIn('duplicate domain', result['error'])
        self.assertIsNone(result['log_dir'])

    def test_argument_parse_and_selection_errors_are_reported(self):
        for arguments in (('--timeout', 'not-a-number'), ('--cars', 'missing')):
            with self.subTest(arguments=arguments), self.assertRaises(SystemExit) as caught:
                self.invoke(*arguments)
            self.assertEqual(caught.exception.code, 2)
            self.assertEqual(self.read_status()['phase'], 'failed')

    def test_http_failure_is_failed_before_any_process_start(self):
        request = self.fake_environment(request_error=RuntimeError('offline'))
        with self.assertRaisesRegex(RuntimeError, 'offline'):
            self.invoke('--agent-ip', '192.0.2.1')
        result = self.read_status()
        self.assertEqual(result['phase'], 'failed')
        self.assertIn('offline', result['error'])
        self.assertTrue(result['log_dir'])
        self.assertEqual(self.commands, [])
        self.assertEqual(request.call_count, 1)

    def test_ready_requires_both_probes_and_interrupt_status_follows_cleanup(self):
        self.fake_environment()
        with self.assertRaises(KeyboardInterrupt):
            self.invoke('--agent-ip', '192.0.2.1')
        self.assertEqual(self.read_status()['phase'], 'stopped')
        self.assertEqual(self.events, ['checking', 'checking', 'waiting_raw', 'starting_base',
                                       'waiting_filtered', 'ready', 'cleanup', 'stopped'])
        probes = [command for command in self.commands
                  if any('fleet_probe.py' in str(part) for part in command)]
        self.assertEqual(len(probes), 2)
        self.assertNotIn('--filtered', probes[0])
        self.assertIn('--filtered', probes[1])

    def test_mismatched_network_without_configure_is_failed_without_writes(self):
        request = self.fake_environment()
        request.return_value['microros_agent_port'] = 8888
        with self.assertRaisesRegex(RuntimeError, '首次使用请加 --configure'):
            self.invoke('--agent-ip', '192.0.2.1')
        self.assertEqual(self.read_status()['phase'], 'failed')
        self.assertEqual(self.commands, [])
        self.assertEqual(request.call_count, 1)
        self.assertEqual(len(request.call_args.args), 1)

    def test_explicit_configure_reports_stage_and_retains_existing_semantics(self):
        request = self.fake_environment()
        request.return_value['microros_agent_port'] = 8888
        with self.assertRaises(KeyboardInterrupt):
            self.invoke('--agent-ip', '192.0.2.1', '--configure')
        self.assertIn('configuring', self.events)
        self.assertLess(self.events.index('configuring'), self.events.index('waiting_raw'))
        self.assertEqual(request.call_count, 2)
        self.assertEqual(request.call_args.args[1]['microros_agent_port'], 20004)

    def test_probe_failure_never_reports_ready_and_is_failed_after_cleanup(self):
        self.fake_environment(probe_code=1)
        with self.assertRaisesRegex(RuntimeError, 'raw'):
            self.invoke('--agent-ip', '192.0.2.1')
        self.assertEqual(self.read_status()['phase'], 'failed')
        self.assertNotIn('ready', self.events)
        self.assertEqual(self.events[-2:], ['cleanup', 'failed'])

    def test_cleanup_failure_is_failed_instead_of_normal_stopped(self):
        self.fake_environment(cleanup_error=RuntimeError('cleanup failed'))
        with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
            self.invoke('--agent-ip', '192.0.2.1')
        result = self.read_status()
        self.assertEqual(result['phase'], 'failed')
        self.assertIn('cleanup failed', result['error'])


if __name__ == '__main__':
    unittest.main()
