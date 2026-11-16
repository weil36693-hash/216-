"""Connection management checks with fake HTTP, routing, process and telemetry."""

from copy import deepcopy
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from leap_fleet.connections import ConnectionManager, _get_runtime as read_runtime


class FakeMonitor:
    def __init__(self):
        self.cars = {name: {'id': name, 'status': 'offline', 'cmd_publishers': 0,
                            'measured_v': None, 'measured_w': None}
                     for name in ('car01', 'car02')}

    def snapshot(self):
        return deepcopy(list(self.cars.values()))


class FakeProcess:
    def __init__(self, command, **kwargs):
        self.command, self.kwargs = command, kwargs
        self.returncode = None
        self.signals = []
        self.wait_timeouts = []
        self.hang = False
        self.status_file = Path(command[command.index('--status-file') + 1])

    def poll(self):
        return self.returncode

    def send_signal(self, value):
        self.signals.append(value)

    def wait(self, timeout):
        self.wait_timeouts.append(timeout)
        if self.hang:
            raise subprocess.TimeoutExpired(self.command, timeout)
        self.returncode = 130
        return self.returncode

    def status(self, phase, message='test status', error=None):
        self.status_file.write_text(json.dumps({'phase': phase, 'message': message,
                                               'log_dir': None, 'error': error,
                                               'updated_at': time.time()}), encoding='utf-8')


class ConnectionManagerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.workspace = Path(self.directory.name)
        (self.workspace / 'start_fleet.sh').write_text('# fake launcher\n', encoding='utf-8')
        (self.workspace / 'config').mkdir()
        self.config = self.workspace / 'config' / 'fleet.json'
        self.config.write_text(json.dumps({'cars': [
            {'name': 'car01', 'ip': '10.0.0.1', 'domain_id': 1, 'agent_port': 20001},
            {'name': 'car02', 'ip': '10.0.0.2', 'domain_id': 2, 'agent_port': 20002},
        ]}), encoding='utf-8')
        self.monitor = FakeMonitor()
        self.processes = []
        self.runtime_calls = []
        self.runtimes = {
            name: {'comm_mode': 'micro_ros', 'microros_agent_ip': '10.0.0.249',
                   'microros_agent_port': port}
            for name, port in (('car01', 20001), ('car02', 20002))}

        def runtime(car):
            self.runtime_calls.append(car['name'])
            return deepcopy(self.runtimes[car['name']])

        def popen(command, **kwargs):
            process = FakeProcess(command, **kwargs)
            self.processes.append(process)
            return process

        self.runtime = patch('leap_fleet.connections._get_runtime', side_effect=runtime).start()
        self.route = patch('leap_fleet.connections._host_ip', return_value='10.0.0.249').start()
        self.external = patch('leap_fleet.connections._external_busy', return_value=None).start()
        self.popen = patch('leap_fleet.connections.subprocess.Popen', side_effect=popen).start()
        self.addCleanup(patch.stopall)
        self.manager = ConnectionManager(self.config, self.monitor, self.workspace)
        self.addCleanup(self.cleanup_manager)

    def cleanup_manager(self):
        for process in self.processes:
            process.hang = False
        for car in self.monitor.cars.values():
            car.update(cmd_publishers=0, measured_v=0.0, measured_w=0.0)
        self.manager.close()

    def states(self):
        return {car['id']: car for car in self.manager.snapshot()['cars']}

    def wait_until(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            threading.Event().wait(0.005)
        self.fail(f'connection task did not complete: {self.manager.snapshot()}')

    def connect(self, name='car01'):
        before = len(self.processes)
        self.manager.request('connect', [name])
        self.wait_until(lambda: len(self.processes) > before)
        return self.processes[-1]

    def test_constructing_does_not_probe_network_or_start_processes(self):
        self.assertTrue(self.manager.snapshot()['available'])
        self.assertEqual(self.runtime_calls, [])
        self.assertEqual(self.processes, [])
        self.assertFalse(self.manager.busy_for(['car01', 'car02']))

    def test_missing_workspace_disables_actions_without_probing(self):
        manager = ConnectionManager(self.config, self.monitor, self.workspace / 'missing')
        self.assertFalse(manager.snapshot()['available'])
        self.assertTrue(manager.snapshot()['error'])
        with self.assertRaises(RuntimeError):
            manager.request('connect', ['car01'])
        manager.close()
        self.assertEqual(self.runtime_calls, [])
        self.assertEqual(self.processes, [])

    def test_unknown_or_duplicate_names_are_rejected_before_any_action(self):
        for names in (['car01', 'missing'], ['car01', 'car01'], [], [False]):
            with self.subTest(names=names), self.assertRaises(ValueError):
                self.manager.request('connect', names)
        self.assertEqual(self.runtime_calls, [])
        self.assertEqual(self.processes, [])

    def test_check_only_reads_and_presents_current_and_target(self):
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: not self.states()['car01']['busy'])
        state = self.states()['car01']
        self.assertEqual(state['phase'], 'idle')
        self.assertEqual(state['current'], state['target'])
        self.assertEqual(state['target']['microros_agent_port'], 20001)
        self.assertEqual(self.processes, [])

    def test_connect_command_is_single_car_absolute_and_never_configures_by_default(self):
        process = self.connect()
        self.assertNotIn('--configure', process.command)
        self.assertEqual(process.command[0], str(self.workspace / 'start_fleet.sh'))
        self.assertEqual(process.command[process.command.index('--config') + 1],
                         str(self.config.resolve()))
        self.assertEqual(process.command[process.command.index('--cars') + 1], 'car01')
        self.assertTrue(process.status_file.is_absolute())
        self.assertTrue(process.kwargs['start_new_session'])
        self.assertEqual(process.kwargs['env']['ROS_LOCALHOST_ONLY'], '0')
        self.assertTrue(self.states()['car01']['owned'])

    def test_one_vehicle_http_failure_does_not_cancel_the_other_launch(self):
        def runtime(car):
            if car['name'] == 'car01':
                raise TimeoutError('offline car')
            return deepcopy(self.runtimes['car02'])
        self.runtime.side_effect = runtime
        self.manager.request('connect', ['car01', 'car02'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'failed' and
                        self.states()['car02']['owned'])
        self.assertIn('offline car', self.states()['car01']['error'])
        self.assertEqual(len(self.processes), 1)
        self.assertIn('car02', self.processes[0].command)
        self.assertIsNone(self.processes[0].poll())

    def test_configuration_mismatch_is_previewed_without_a_process(self):
        self.runtimes['car01']['microros_agent_port'] = 8888
        self.manager.request('connect', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'needs_config')
        state = self.states()['car01']
        self.assertEqual(state['current']['microros_agent_port'], 8888)
        self.assertEqual(state['target']['microros_agent_port'], 20001)
        self.assertEqual(self.processes, [])

    def test_configure_requires_preview_and_stable_recheck_before_launch(self):
        with self.assertRaises(ValueError):
            self.manager.request('configure', ['car01'])
        self.runtimes['car01']['microros_agent_port'] = 8888
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'needs_config')
        self.runtimes['car01']['microros_agent_ip'] = '10.0.0.248'
        self.manager.request('configure', ['car01'])
        self.wait_until(lambda: not self.states()['car01']['busy'])
        self.assertEqual(self.states()['car01']['phase'], 'needs_config')
        self.assertEqual(self.states()['car01']['current']['microros_agent_ip'], '10.0.0.248')
        self.assertEqual(self.processes, [])
        self.manager.request('configure', ['car01'])
        self.wait_until(lambda: bool(self.processes))
        self.assertIn('--configure', self.processes[0].command)
        command = self.processes[0].command
        self.assertEqual(command[command.index('--agent-ip') + 1], '10.0.0.249')

    def test_repeated_connect_reuses_its_owned_process(self):
        process = self.connect()
        self.manager.request('connect', ['car01'])
        self.manager.request('connect', ['car01'])
        self.assertEqual(self.processes, [process])

    def test_check_cannot_clear_owned_startup_busy_without_a_valid_status_file(self):
        process = self.connect()
        self.wait_until(lambda: not self.manager._sessions['car01'].worker.is_alive())
        self.monitor.cars['car01']['status'] = 'connected'
        calls = len(self.runtime_calls)
        for contents in (None, '{"phase":'):
            with self.subTest(contents=contents):
                if contents is not None:
                    process.status_file.write_text(contents, encoding='utf-8')
                self.manager.request('check', ['car01'])
                state = self.states()['car01']
                self.assertEqual(state['phase'], 'starting')
                self.assertTrue(state['busy'])
                self.assertTrue(self.manager.busy_for(['car01']))
                self.assertEqual(len(self.runtime_calls), calls)

    def test_failed_thread_start_rolls_back_registration_and_other_owned_cleanup_still_runs(self):
        process = self.connect()
        self.wait_until(lambda: not self.manager._sessions['car01'].worker.is_alive())
        previous_count = len(self.manager._threads)
        with patch('leap_fleet.connections.threading.Thread.start',
                   side_effect=RuntimeError('thread unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'thread unavailable'):
                self.manager.request('connect', ['car02'])
        self.assertEqual(len(self.manager._threads), previous_count)
        self.assertIsNone(self.manager._sessions['car02'].worker)
        self.assertEqual(self.states()['car02']['phase'], 'failed')
        self.manager.close()
        self.assertEqual(process.signals, [signal.SIGINT])
        self.assertIsNotNone(process.poll())

    def test_close_thread_start_failure_does_not_skip_other_owned_processes(self):
        self.manager.request('connect', ['car01', 'car02'])
        self.wait_until(lambda: len(self.processes) == 2 and
                        all(not session.worker.is_alive()
                            for session in self.manager._sessions.values()))
        original = threading.Thread.start
        starts = 0

        def start(worker):
            nonlocal starts
            starts += 1
            if starts == 1:
                raise RuntimeError('one cancel thread unavailable')
            return original(worker)

        with patch('leap_fleet.connections.threading.Thread.start', new=start):
            with self.assertRaisesRegex(RuntimeError, '无法提交关闭任务'):
                self.manager.close()
        processes = {process.command[process.command.index('--cars') + 1]: process
                     for process in self.processes}
        self.assertEqual(processes['car01'].signals, [])
        self.assertEqual(processes['car02'].signals, [signal.SIGINT])
        self.manager.close()
        self.assertEqual(processes['car01'].signals, [signal.SIGINT])

    def test_duplicate_vehicle_ips_are_rejected_before_any_network_or_process_work(self):
        payload = json.loads(self.config.read_text(encoding='utf-8'))
        payload['cars'][1]['ip'] = payload['cars'][0]['ip']
        self.config.write_text(json.dumps(payload), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'IP addresses must be unique'):
            ConnectionManager(self.config, self.monitor, self.workspace)
        self.assertEqual(self.runtime_calls, [])
        self.assertEqual(self.processes, [])

    def test_existing_connected_car_is_external_and_does_not_probe_or_spawn(self):
        self.monitor.cars['car01']['status'] = 'connected'
        state = self.manager.request('connect', ['car01'])['cars'][0]
        self.assertEqual(state['phase'], 'external')
        self.assertFalse(state['owned'])
        self.assertEqual(self.runtime_calls, [])
        self.assertEqual(self.processes, [])
        self.manager.request('cancel', ['car01'])
        self.assertEqual(self.processes, [])

    def test_external_lock_or_port_is_not_replaced_even_during_configure(self):
        self.external.return_value = ('external', 'another terminal owns this domain')
        self.manager.request('connect', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'external')
        self.assertFalse(self.states()['car01']['owned'])
        self.assertEqual(self.processes, [])
        self.assertEqual(self.runtime_calls, [])
        self.external.return_value = None
        self.runtimes['car01']['microros_agent_port'] = 8888
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'needs_config')
        self.external.return_value = ('blocked', 'another Agent owns the port')
        self.manager.request('configure', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'blocked')
        self.assertEqual(self.processes, [])

    def test_ready_file_and_live_monitor_are_both_required_for_connected(self):
        process = self.connect()
        self.monitor.cars['car01']['status'] = 'connected'
        process.status('waiting_raw')
        self.wait_until(lambda: self.states()['car01']['phase'] == 'waiting_raw')
        self.assertTrue(self.manager.busy_for(['car01']))
        process.status('ready')
        self.wait_until(lambda: self.states()['car01']['phase'] == 'connected')
        self.assertFalse(self.manager.busy_for(['car01']))
        self.monitor.cars['car01']['status'] = 'stale'
        self.assertEqual(self.states()['car01']['phase'], 'waiting_data')
        self.assertFalse(self.states()['car01']['busy'])
        self.assertEqual(len(self.processes), 1)

    def test_missing_or_partial_status_files_keep_starting_and_logs_are_bounded(self):
        process = self.connect()
        self.assertEqual(self.states()['car01']['phase'], 'starting')
        process.status_file.write_text('{"phase":', encoding='utf-8')
        self.assertEqual(self.states()['car01']['phase'], 'starting')
        log_file = Path(self.states()['car01']['log_file'])
        log_file.write_text('x' * 20000 + 'tail', encoding='utf-8')
        state = self.states()['car01']
        self.assertLessEqual(len(state['log_tail']), self.manager.LOG_LIMIT)
        self.assertTrue(state['log_tail'].endswith('tail'))

    def test_cancel_uses_only_owned_wrapper_sigint_and_waits_for_cleanup(self):
        process = self.connect()
        self.manager.request('cancel', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'stopped')
        self.assertEqual(process.signals, [signal.SIGINT])
        self.assertEqual(process.wait_timeouts, [12.0])
        self.assertFalse(self.states()['car01']['owned'])
        self.assertEqual(len(self.processes), 1)

    def test_cancel_timeout_retains_ownership_and_can_be_retried_without_kill(self):
        process = self.connect()
        process.hang = True
        self.manager.request('cancel', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'failed')
        self.assertTrue(self.states()['car01']['owned'])
        self.assertIsNone(process.poll())
        process.hang = False
        self.manager.request('cancel', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'stopped')
        self.assertEqual(process.signals, [signal.SIGINT, signal.SIGINT])

    def test_cancel_failure_is_not_overwritten_by_a_previous_ready_status(self):
        process = self.connect()
        process.status('ready')
        self.monitor.cars['car01']['status'] = 'connected'
        self.wait_until(lambda: self.states()['car01']['phase'] == 'connected')
        process.hang = True
        self.manager.request('cancel', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'failed')
        for _ in range(3):
            state = self.states()['car01']
            self.assertEqual(state['phase'], 'failed')
            self.assertTrue(state['owned'])
            self.assertIn('12秒', state['error'])
        self.assertIsNone(process.poll())

    def test_lifecycle_close_releases_owned_process_despite_residual_odometry(self):
        process = self.connect()
        self.monitor.cars['car01']['measured_v'] = 0.01
        with self.assertRaises(RuntimeError):
            self.manager.request('cancel', ['car01'])
        self.manager.close()
        self.assertEqual(process.signals, [signal.SIGINT])
        self.assertIsNotNone(process.poll())
        self.assertFalse(self.states()['car01']['owned'])

    def test_cancel_during_http_check_prevents_late_spawn(self):
        entered, release = threading.Event(), threading.Event()

        def runtime(car):
            entered.set()
            if not release.wait(1.0):
                raise RuntimeError('test preflight not released')
            return deepcopy(self.runtimes[car['name']])

        self.runtime.side_effect = runtime
        self.manager.request('connect', ['car01'])
        self.assertTrue(entered.wait(0.5))
        self.manager.request('cancel', ['car01'])
        release.set()
        self.wait_until(lambda: self.states()['car01']['phase'] == 'stopped')
        self.assertEqual(self.processes, [])

    def test_read_only_preflight_limits_network_requests_to_six_at_once(self):
        entries = [{'name': f'car{index:02}', 'ip': f'10.0.0.{index}',
                    'domain_id': index, 'agent_port': 20000 + index}
                   for index in range(1, 9)]
        self.config.write_text(json.dumps({'cars': entries}), encoding='utf-8')
        self.manager = ConnectionManager(self.config, self.monitor, self.workspace)
        release, counter_lock = threading.Event(), threading.Lock()
        active = maximum = entered = 0

        def runtime(car):
            nonlocal active, maximum, entered
            with counter_lock:
                active += 1
                entered += 1
                maximum = max(maximum, active)
            try:
                if not release.wait(1.0):
                    raise RuntimeError('test preflight not released')
                return {'comm_mode': 'micro_ros', 'microros_agent_ip': '10.0.0.249',
                        'microros_agent_port': car['agent_port']}
            finally:
                with counter_lock:
                    active -= 1

        self.runtime.side_effect = runtime
        try:
            self.manager.request('check', [entry['name'] for entry in entries])
            self.wait_until(lambda: entered >= 6)
            self.assertEqual(entered, 6)
        finally:
            release.set()
        self.wait_until(lambda: all(not state['busy'] for state in self.states().values()))
        self.assertEqual(entered, 8)
        self.assertEqual(maximum, 6)
        self.assertEqual(self.processes, [])

    def test_close_during_preflight_prevents_any_process_submission_after_exit(self):
        entered, release = threading.Event(), threading.Event()
        failures = []

        def runtime(car):
            entered.set()
            if not release.wait(1.0):
                raise RuntimeError('test preflight not released')
            return deepcopy(self.runtimes[car['name']])

        def close():
            try:
                self.manager.close()
            except Exception as error:
                failures.append(error)

        self.runtime.side_effect = runtime
        self.manager.request('connect', ['car01'])
        self.assertTrue(entered.wait(0.5))
        closer = threading.Thread(target=close)
        closer.start()
        try:
            self.wait_until(lambda: self.manager._closed)
            release.set()
            closer.join(timeout=2.0)
            self.assertFalse(closer.is_alive())
            self.assertEqual(failures, [])
            self.assertEqual(self.processes, [])
        finally:
            release.set()
            closer.join(timeout=2.0)

    def test_configure_and_cancel_refuse_velocity_publishers_or_nonzero_measurements(self):
        process = self.connect()
        for key, value in (('cmd_publishers', 1), ('measured_v', 0.01), ('measured_w', -0.01)):
            with self.subTest(key=key):
                self.monitor.cars['car01'].update(cmd_publishers=0, measured_v=0.0, measured_w=0.0)
                self.monitor.cars['car01'][key] = value
                with self.assertRaises(RuntimeError):
                    self.manager.request('cancel', ['car01'])
                with self.assertRaises(RuntimeError):
                    self.manager.request('configure', ['car01'])
                self.assertEqual(process.signals, [])

    def test_child_exit_is_reported_without_restarting_or_touching_other_processes(self):
        process = self.connect()
        process.status('failed', error='MCU samples timed out')
        process.returncode = 1
        self.wait_until(lambda: not self.states()['car01']['owned'])
        self.assertEqual(self.states()['car01']['phase'], 'failed')
        self.assertIn('MCU samples timed out', self.states()['car01']['error'])
        self.assertEqual(len(self.processes), 1)

    def test_abnormal_wrapper_exit_preserves_logs_and_reports_unknown_child_cleanup(self):
        process = self.connect()
        process.status('ready')
        log_file = self.states()['car01']['log_file']
        process.returncode = -9
        self.wait_until(lambda: self.states()['car01']['phase'] == 'failed')
        state = self.states()['car01']
        self.assertIn('清理状态未知', state['error'])
        self.assertIn('残留', state['message'])
        self.assertEqual(state['log_file'], log_file)
        self.assertEqual(process.signals, [])
        self.assertEqual(len(self.processes), 1)
        with self.assertRaisesRegex(RuntimeError, '清理状态未知'):
            self.manager.close()
        # There are no real child processes in this fake; clear the deliberately
        # simulated unknown-cleanup condition for the fixture's final cleanup.
        self.manager._sessions['car01'].cleanup_unknown = False

    def test_terminal_status_distinguishes_proven_cleanup_from_abnormal_unknown_exit(self):
        process = self.connect()
        process.status('stopped', message='owned children cleaned')
        process.returncode = -9
        self.wait_until(lambda: not self.states()['car01']['owned'])
        self.assertNotIn('未知', self.states()['car01']['error'])
        self.assertFalse(self.manager._sessions['car01'].cleanup_unknown)

    def test_close_releases_both_owned_processes_and_rejects_future_requests(self):
        self.manager.request('connect', ['car01', 'car02'])
        self.wait_until(lambda: len(self.processes) == 2)
        self.manager.close()
        self.assertTrue(all(process.signals == [signal.SIGINT] for process in self.processes))
        self.assertTrue(all(process.poll() is not None for process in self.processes))
        with self.assertRaises(RuntimeError):
            self.manager.request('connect', ['car01'])
        self.manager.close()

    def test_snapshot_is_a_copy(self):
        state = self.manager.snapshot()
        state['cars'][0]['message'] = 'external mutation'
        self.assertNotEqual(self.states()['car01']['message'], 'external mutation')

    def test_legacy_status_is_a_successful_snapshot_without_watchdog_report(self):
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: not self.states()['car01']['busy'])
        state = self.states()['car01']
        status = state['vehicle_status']
        self.assertEqual(status['check_state'], 'ok')
        self.assertGreater(status['checked_at_unix'], 0)
        self.assertGreater(status['attempted_at_unix'], 0)
        self.assertIsNone(status['firmware'])
        self.assertIsNone(status['cmd_vel_watchdog'])
        self.assertEqual(state['current'], self.runtimes['car01'])
        self.assertEqual(self.processes, [])

    def test_metadata_changes_do_not_change_runtime_preview_or_configure_fields(self):
        self.runtimes['car01']['microros_agent_port'] = 8888
        revision = [1]

        def runtime(car):
            result = deepcopy(self.runtimes[car['name']])
            result['_status_metadata'] = {
                'firmware': {'version': '2.2.1-motion-safety', 'motion_safety_revision': 1},
                'cmd_vel_watchdog': {'enabled': True, 'timeout_ms': 500,
                                     'received_sequence': revision[0],
                                     'applied_sequence': revision[0], 'timed_out': False},
                'other_private_field': 'do not retain',
            }
            return result

        self.runtime.side_effect = runtime
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'needs_config')
        state = self.states()['car01']
        self.assertEqual(set(state['current']),
                         {'comm_mode', 'microros_agent_ip', 'microros_agent_port'})
        self.assertEqual(state['vehicle_status']['cmd_vel_watchdog']['timeout_ms'], 500)
        self.assertNotIn('other_private_field', state['vehicle_status'])
        revision[0] = 123
        self.manager.request('configure', ['car01'])
        self.wait_until(lambda: bool(self.processes))
        self.assertIn('--configure', self.processes[0].command)
        self.assertEqual(self.states()['car01']['vehicle_status'][
            'cmd_vel_watchdog']['received_sequence'], 123)
        self.assertNotIn('watchdog', ' '.join(self.processes[0].command))
        self.assertNotIn('firmware', ' '.join(self.processes[0].command))

    def test_failed_recheck_retains_old_metadata_with_original_time_and_failed_state(self):
        report = deepcopy(self.runtimes['car01'])
        report['_status_metadata'] = {
            'firmware': {'version': '2.2.1-motion-safety'},
            'cmd_vel_watchdog': {'enabled': True, 'timeout_ms': 500, 'timed_out': True}}
        self.runtime.side_effect = lambda car: deepcopy(report)
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: not self.states()['car01']['busy'])
        successful = self.states()['car01']['vehicle_status']
        self.assertEqual(successful['check_state'], 'ok')
        self.runtime.side_effect = TimeoutError('HTTP status timed out')
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: self.states()['car01']['phase'] == 'failed')
        failed = self.states()['car01']['vehicle_status']
        self.assertEqual(failed['check_state'], 'failed')
        self.assertIn('HTTP status timed out', failed['error'])
        self.assertEqual(failed['checked_at_unix'], successful['checked_at_unix'])
        self.assertGreaterEqual(failed['attempted_at_unix'], successful['attempted_at_unix'])
        self.assertEqual(failed['firmware'], successful['firmware'])
        self.assertEqual(failed['cmd_vel_watchdog'], successful['cmd_vel_watchdog'])
        self.assertEqual(self.runtime.call_count, 2)
        for _ in range(5):
            self.states()
        self.assertEqual(self.runtime.call_count, 2, 'snapshot must not poll vehicle HTTP')
        self.assertEqual(self.processes, [])

    def test_metadata_snapshot_stays_historical_during_recheck_and_after_cancel(self):
        self.manager.request('check', ['car01'])
        self.wait_until(lambda: not self.states()['car01']['busy'])
        previous = self.states()['car01']['vehicle_status']
        entered, release = threading.Event(), threading.Event()

        def runtime(car):
            entered.set()
            if not release.wait(1.0):
                raise RuntimeError('test HTTP status was not released')
            return deepcopy(self.runtimes[car['name']])

        self.runtime.side_effect = runtime
        try:
            self.manager.request('check', ['car01'])
            self.assertTrue(entered.wait(0.5))
            checking = self.states()['car01']['vehicle_status']
            self.assertEqual(checking['check_state'], 'checking')
            self.assertEqual(checking['checked_at_unix'], previous['checked_at_unix'])
            self.manager.request('cancel', ['car01'])
            self.assertEqual(self.states()['car01']['vehicle_status']['check_state'], 'cancelled')
        finally:
            release.set()
        self.wait_until(lambda: self.states()['car01']['phase'] == 'stopped')
        cancelled = self.states()['car01']['vehicle_status']
        self.assertEqual(cancelled['check_state'], 'cancelled')
        self.assertEqual(cancelled['checked_at_unix'], previous['checked_at_unix'])
        self.assertEqual(self.processes, [])


class VehicleStatusParsingTests(unittest.TestCase):
    def read(self, payload):
        response = MagicMock()
        response.__enter__.return_value = response
        response.read.return_value = json.dumps(payload).encode('utf-8')
        opener = MagicMock()
        opener.open.return_value = response
        with patch('leap_fleet.connections.build_opener', return_value=opener):
            result = read_runtime({'ip': '10.0.0.1'})
        opener.open.assert_called_once()
        request = opener.open.call_args.args[0]
        self.assertEqual(request.full_url, 'http://10.0.0.1/api/status')
        self.assertIsNone(request.data, 'status metadata must be read-only')
        self.assertEqual(opener.open.call_args.kwargs, {'timeout': 3.0})
        response.read.assert_called_once_with(1024 * 1024)
        return result

    @staticmethod
    def runtime():
        return {'comm_mode': 'micro_ros', 'microros_agent_ip': '10.0.0.249',
                'microros_agent_port': 20001, 'wifi_password': 'must not retain'}

    def test_old_firmware_without_metadata_is_supported(self):
        result = self.read({'runtime': self.runtime(), 'other_status': {'private': 'hidden'}})
        self.assertNotIn('wifi_password', result)
        self.assertEqual(result['_status_metadata'], {'firmware': None, 'cmd_vel_watchdog': None})
        self.assertEqual(set(result), {'comm_mode', 'microros_agent_ip',
                                       'microros_agent_port', '_status_metadata'})

    def test_new_firmware_watchdog_has_a_strict_read_only_metadata_whitelist(self):
        firmware = {'version': '2.2.1-motion-safety', 'motion_safety_revision': 1}
        watchdog = {'enabled': True, 'timeout_ms': 500, 'has_command': False,
                    'last_cmd_age_ms': None, 'timed_out': False,
                    'received_sequence': 0xffffffff, 'applied_sequence': 0}
        result = self.read({'runtime': self.runtime(),
                            'firmware': {**firmware, 'private': 'hidden'},
                            'cmd_vel_watchdog': {**watchdog, 'reset': True,
                                                 'runtime_override': {'timeout_ms': 0}},
                            'credentials': 'do not retain'})
        self.assertEqual(result['_status_metadata'],
                         {'firmware': firmware, 'cmd_vel_watchdog': watchdog})
        self.assertNotIn('credentials', result)
        self.assertNotIn('wifi_password', result)

    def test_malformed_optional_metadata_cannot_break_runtime_or_be_treated_as_enabled(self):
        result = self.read({'runtime': self.runtime(),
                            'firmware': {'version': 'x' * 500, 'motion_safety_revision': True},
                            'cmd_vel_watchdog': {'enabled': 'true', 'timeout_ms': -1,
                                                 'has_command': 1, 'last_cmd_age_ms': 0x100000000,
                                                 'timed_out': 'false', 'received_sequence': True,
                                                 'applied_sequence': 0x100000000}})
        metadata = result['_status_metadata']
        self.assertEqual(metadata['firmware'], {'version': 'x' * 128})
        self.assertIsNone(metadata['cmd_vel_watchdog'])
        self.assertEqual(result['microros_agent_port'], 20001)


if __name__ == '__main__':
    unittest.main()
