"""VRPN startup ownership tests; no ROS graph, network, or vehicle commands."""
from contextlib import ExitStack, redirect_stdout
import io
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import fleet_positions
from fleet_processes import Processes
from fleet_vrpn import VRPNStartLock, discover_vrpn


class VRPNTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.logs = self.root / 'log'
        self.config = self.root / 'config' / 'fleet.json'
        self.config.parent.mkdir()
        self.cars = [{'name': 'car01', 'ip': '192.0.2.1',
                      'pose_topic': '/vrpn_mocap/car_01/pose'}]
        self.config.write_text(json.dumps({'cars': self.cars}))
        self.args = SimpleNamespace(start_vrpn=True, server='192.0.2.2', port=3883)
        self.node = MagicMock()
        self.node.context.get_domain_id.return_value = 0
        self.ros = MagicMock()
        self.processes = MagicMock()
        self.reader = MagicMock()
        self.discovery = MagicMock(return_value=({}, False))
        self.messages = io.StringIO()
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        for target, replacement in [('DEFAULT_CONFIG', self.config),
                                    ('Processes', MagicMock(return_value=self.processes)),
                                    ('run_reader', self.reader), ('discover_vrpn', self.discovery)]:
            self.stack.enter_context(patch.object(fleet_positions, target, replacement))

    def run_positions(self):
        with redirect_stdout(self.messages):
            fleet_positions.read_positions(
                self.ros, MagicMock(return_value=self.node), object, object,
                self.args, self.cars)

    def assert_lock_available(self):
        lock = VRPNStartLock(self.logs, 0)
        try:
            self.assertTrue(lock.acquire())
        finally:
            lock.close()

    def test_existing_three_publishers_are_reused_and_reported(self):
        self.discovery.return_value = ({self.cars[0]['pose_topic']: 3}, True)
        self.run_positions()
        self.processes.start.assert_not_called()
        self.assertIn('单话题最多 3 个发布者', self.messages.getvalue())
        self.assertIsNone(self.reader.call_args.args[-1])
        self.assert_lock_available()

    def test_existing_client_without_active_pose_is_reused(self):
        self.discovery.return_value = ({}, True)
        self.run_positions()
        self.processes.start.assert_not_called()
        self.assertIn('暂未发现配置位姿发布者', self.messages.getvalue())

    def test_lock_busy_does_not_start_or_probe(self):
        lock = VRPNStartLock(self.logs, '00')
        self.assertTrue(lock.acquire())
        try:
            self.run_positions()
            self.processes.start.assert_not_called()
            self.discovery.assert_not_called()
            self.reader.assert_called_once()
            self.assertIn('持锁', self.messages.getvalue())
        finally:
            lock.close()

    def test_lock_held_until_owned_process_cleanup_completes(self):
        def check_lock():
            contender = VRPNStartLock(self.logs, 0)
            try:
                self.assertFalse(contender.acquire())
            finally:
                contender.close()
        self.reader.side_effect = lambda *args: check_lock()
        self.processes.close.side_effect = check_lock
        self.run_positions()
        self.processes.start.assert_called_once()
        self.assertEqual(len(self.processes.start.call_args.kwargs['pass_fds']), 1)
        self.assert_lock_available()

    def test_graph_error_never_starts_and_releases_lock(self):
        self.discovery.side_effect = RuntimeError('graph unavailable')
        with self.assertRaisesRegex(RuntimeError, 'graph unavailable'):
            self.run_positions()
        self.processes.start.assert_not_called()
        self.node.destroy_node.assert_called_once()
        self.ros.shutdown.assert_called_once()
        self.assert_lock_available()

    def test_start_failure_releases_resources(self):
        self.processes.start.side_effect = OSError('launch failed')
        with self.assertRaisesRegex(OSError, 'launch failed'):
            self.run_positions()
        self.processes.close.assert_called_once()
        self.assert_lock_available()

    def test_print_failure_after_start_still_cleans_up(self):
        with patch.object(fleet_positions, 'print', side_effect=BrokenPipeError):
            with self.assertRaises(BrokenPipeError):
                self.run_positions()
        self.processes.start.assert_called_once()
        self.processes.close.assert_called_once()
        self.assert_lock_available()

    def test_reader_setup_failure_still_cleans_up(self):
        self.reader.side_effect = OSError('csv cannot open')
        with self.assertRaisesRegex(OSError, 'csv cannot open'):
            self.run_positions()
        self.processes.close.assert_called_once()
        self.assert_lock_available()

    def test_hangup_and_termination_unwind_owned_processes(self):
        for signum in (signal.SIGHUP, signal.SIGTERM):
            with self.subTest(signum=signum):
                self.processes.close.reset_mock()
                self.reader.side_effect = lambda *args: fleet_positions.exit_on_signal(signum, None)
                with self.assertRaises(SystemExit) as caught:
                    self.run_positions()
                self.assertEqual(caught.exception.code, 128 + signum)
                self.processes.close.assert_called_once()
                self.assert_lock_available()

    def test_check_config_is_offline_even_when_start_requested(self):
        with patch.object(sys, 'argv', ['fleet_positions.py', '--check-config', '--start-vrpn',
                                       '--config', str(self.config)]), \
                patch.dict(sys.modules, {'rclpy': None}), redirect_stdout(self.messages):
            fleet_positions.main()
        self.discovery.assert_not_called()
        self.processes.start.assert_not_called()
        self.assertFalse(self.logs.exists())

    def test_different_domains_do_not_block_each_other(self):
        first, other = VRPNStartLock(self.logs, 0), VRPNStartLock(self.logs, 1)
        try:
            self.assertTrue(first.acquire())
            self.assertTrue(other.acquire())
        finally:
            first.close()
            other.close()

    def test_inherited_lock_survives_reader_descriptor_close(self):
        lock = VRPNStartLock(self.logs, 0)
        self.assertTrue(lock.acquire())
        processes = Processes()
        try:
            child = processes.start(
                [sys.executable, '-c', 'import time; time.sleep(30)'],
                self.root / 'child.log', pass_fds=(lock.fileno(),))
            lock.close()
            self.assertIsNone(child.poll())
            contender = VRPNStartLock(self.logs, 0)
            try:
                self.assertFalse(contender.acquire())
            finally:
                contender.close()
        finally:
            processes.close()
            lock.close()
        self.assert_lock_available()

    def test_two_simultaneous_processes_only_one_acquires_same_domain(self):
        code = ('import sys; from fleet_vrpn import VRPNStartLock; '
                'lock=VRPNStartLock(sys.argv[1], 0); '
                'print(int(lock.acquire()), flush=True); sys.stdin.readline(); lock.close()')
        children = [subprocess.Popen(
            [sys.executable, '-c', code, str(self.logs)],
            cwd=Path(fleet_positions.__file__).parent,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True) for _ in range(2)]
        try:
            self.assertEqual(sorted(child.stdout.readline().strip() for child in children), ['0', '1'])
        finally:
            for child in children:
                child.communicate('\n', timeout=5)
        self.assert_lock_available()

    def test_discovery_waits_for_delayed_publishers_and_counts_highest_seen(self):
        now = [0.0]
        node = MagicMock()
        node.count_publishers.side_effect = lambda topic: 3 if now[0] >= 0.2 else 0
        node.get_node_names_and_namespaces.return_value = []
        def spin(node, *, timeout_sec):
            now[0] += timeout_sec
        counts, found = discover_vrpn(node, ['/pose'], spin, timeout=0.5, clock=lambda: now[0])
        self.assertEqual(counts, {'/pose': 3})
        self.assertFalse(found)
        self.assertAlmostEqual(now[0], 0.5)

    def test_discovery_recognizes_idle_client_node(self):
        node = MagicMock()
        node.count_publishers.return_value = 0
        node.get_node_names_and_namespaces.return_value = [
            ('vrpn_mocap_client_node', '/vrpn_mocap')]
        counts, found = discover_vrpn(node, ['/pose'], MagicMock(), timeout=0)
        self.assertEqual(counts, {})
        self.assertTrue(found)


if __name__ == '__main__':
    unittest.main()
