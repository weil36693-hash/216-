"""CLI validation and command routing checks; never opens a ROS transport."""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from leap_fleet import cli


_DEFAULT_CONTROL = object()


class CliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.cars = (
            SimpleNamespace(name='car01', domain_id=1, cmd_vel_topic='/cmd_vel'),
            SimpleNamespace(name='car02', domain_id=2, cmd_vel_topic='/cmd_vel'),
        )
        self.loader = patch.object(cli, 'load_cars', side_effect=self.load_cars).start()
        self.addCleanup(patch.stopall)
        self.fleet = MagicMock()
        self.fleet.names = ('car01', 'car02')
        self.fleet.wait_for_subscribers.return_value = True
        self.factory = MagicMock()
        self.factory.return_value.__enter__.return_value = self.fleet
        self.velocity = lambda v, w: (v, w)
        self.control = SimpleNamespace(Fleet=self.factory, Velocity=self.velocity)
        self.logger_factory = patch('leap_fleet.firmware_session.SessionLogger').start()
        self.logger = self.logger_factory.return_value
        self.logger.directory = self.directory / 'logs'

    def load_cars(self, config=None, cars=None):
        if cars is None:
            return self.cars
        unknown = set(cars) - {car.name for car in self.cars}
        if unknown:
            raise ValueError('unknown vehicles: ' + ', '.join(sorted(unknown)))
        return tuple(car for car in self.cars if car.name in cars)

    def run_cli(self, arguments, control=_DEFAULT_CONTROL):
        stdout, stderr = io.StringIO(), io.StringIO()
        replacement = self.control if control is _DEFAULT_CONTROL else control
        with patch.dict(sys.modules, {'leap_fleet.control': replacement}):
            with redirect_stdout(stdout), redirect_stderr(stderr):
                status = cli.main(arguments)
        return status, stdout.getvalue(), stderr.getvalue()

    def write_batch(self, contents):
        path = self.directory / 'commands.json'
        path.write_text(json.dumps(contents), encoding='utf-8')
        return str(path)

    def test_list_works_without_importing_control_or_opening_ros(self):
        status, output, errors = self.run_cli(
            ['--config', '/tmp/fleet.json', '--cars', 'car02', 'list'], control=None)
        self.assertEqual(status, 0)
        self.assertIn('car02\t2\t/cmd_vel', output)
        self.assertNotIn('car01\t', output)
        self.assertEqual(errors, '')
        self.factory.assert_not_called()
        self.loader.assert_called_once_with(config=Path('/tmp/fleet.json'), cars=['car02'])

    def test_velocity_common_options_before_subcommand_and_default_rate(self):
        status, _, errors = self.run_cli(
            ['--cars', 'car01', 'car02', 'velocity', '--v', '0.05', '--w', '-0.1',
             '--duration', '2'])
        self.assertEqual((status, errors), (0, ''))
        self.factory.assert_called_once_with(config=None, cars=('car01', 'car02'))
        self.fleet.wait_for_subscribers.assert_called_once_with(timeout=5.0)
        self.fleet.run_for.assert_called_once_with(
            {'car01': (0.05, -0.1), 'car02': (0.05, -0.1)}, duration=2.0, rate=20.0)

    def test_single_vehicle_and_wait_zero_skip_discovery(self):
        status, _, _ = self.run_cli(
            ['velocity', '--cars', 'car02', '--w', '0.1', '--duration', '1',
             '--rate', '10', '--wait', '0'])
        self.assertEqual(status, 0)
        self.fleet.wait_for_subscribers.assert_not_called()
        self.fleet.run_for.assert_called_once_with(
            {'car02': (0.0, 0.1)}, duration=1.0, rate=10.0)

    def test_config_file_named_like_command_is_not_treated_as_subcommand(self):
        status, _, _ = self.run_cli(['--config', 'velocity', 'list'])
        self.assertEqual(status, 0)
        self.loader.assert_called_once_with(config=Path('velocity'), cars=None)
        self.factory.assert_not_called()

    def test_batch_defaults_to_only_json_targets(self):
        path = self.write_batch({'car02': {'v': 0.03, 'w': -0.1}})
        status, _, errors = self.run_cli(['batch', path, '--duration', '2.5', '--wait', '0'])
        self.assertEqual((status, errors), (0, ''))
        self.loader.assert_called_once_with(config=None, cars=['car02'])
        self.factory.assert_called_once_with(config=None, cars=('car02',))
        self.fleet.run_for.assert_called_once_with(
            {'car02': (0.03, -0.1)}, duration=2.5, rate=20.0)

    def test_batch_can_use_explicit_selection(self):
        path = self.write_batch({'car01': {'v': 0.05, 'w': 0},
                                 'car02': {'v': 0, 'w': 0.1}})
        status, _, _ = self.run_cli(
            ['--cars', 'car01', 'car02', 'batch', path, '--duration', '1'])
        self.assertEqual(status, 0)
        self.fleet.run_for.assert_called_once_with(
            {'car01': (0.05, 0), 'car02': (0, 0.1)}, duration=1.0, rate=20.0)

    def test_batch_selection_superset_only_opens_json_targets(self):
        path = self.write_batch({'car02': {'v': 0.03, 'w': -0.1}})
        status, _, errors = self.run_cli(
            ['batch', path, '--cars', 'car01', 'car02', '--duration', '1'])
        self.assertEqual((status, errors), (0, ''))
        self.loader.assert_called_once_with(config=None, cars=['car01', 'car02'])
        self.factory.assert_called_once_with(config=None, cars=('car02',))
        self.fleet.run_for.assert_called_once_with(
            {'car02': (0.03, -0.1)}, duration=1.0, rate=20.0)

    def test_batch_rejects_target_outside_explicit_selection_before_open(self):
        path = self.write_batch({'car02': {'v': 0.05, 'w': 0}})
        status, _, errors = self.run_cli(
            ['--cars', 'car01', 'batch', path, '--duration', '1'])
        self.assertEqual(status, 1)
        self.assertIn('outside --cars', errors)
        self.factory.assert_not_called()

    def test_unknown_vehicle_rejected_before_open(self):
        status, _, errors = self.run_cli(
            ['velocity', '--cars', 'car99', '--v', '0.1', '--duration', '1'])
        self.assertEqual(status, 1)
        self.assertIn('unknown vehicles', errors)
        self.factory.assert_not_called()

    def test_invalid_numeric_arguments_rejected_before_loading_config(self):
        for options in (
                [], ['--duration', '0'], ['--duration', '-1'], ['--duration', 'nan'],
                ['--duration', 'inf'], ['--duration', '1', '--rate', '0'],
                ['--duration', '1', '--rate', 'nan'], ['--duration', '1', '--rate', 'inf'],
                ['--duration', '1', '--wait', '-1'], ['--duration', '1', '--wait', 'nan'],
                ['--duration', '1', '--v', 'nan'], ['--duration', '1', '--w', 'inf']):
            with self.subTest(options=options):
                status, _, errors = self.run_cli(['velocity', *options])
                self.assertEqual(status, 2)
                self.assertIn('error:', errors)
        self.loader.assert_not_called()
        self.factory.assert_not_called()

    def test_invalid_batch_payloads_rejected_before_open(self):
        for payload in (
                [], {}, {'car01': [0.1, 0]}, {'car01': {'v': 0.1}},
                {'car01': {'v': 0.1, 'w': 0, 'typo': 1}},
                {'car01': {'v': True, 'w': 0}}, {'car01': {'v': '0.1', 'w': 0}},
                {'car01': {'v': float('nan'), 'w': 0}},
                {'car01': {'v': 0, 'w': float('inf')}}):
            with self.subTest(payload=payload):
                status, _, errors = self.run_cli(
                    ['batch', self.write_batch(payload), '--duration', '1'])
                self.assertEqual(status, 1)
                self.assertTrue(errors)
        self.loader.assert_not_called()
        self.factory.assert_not_called()

    def test_duplicate_batch_keys_are_rejected(self):
        path = self.directory / 'duplicate.json'
        path.write_text('{"car01":{"v":0.1,"w":0},"car01":{"v":0,"w":0}}')
        status, _, errors = self.run_cli(['batch', str(path), '--duration', '1'])
        self.assertEqual(status, 1)
        self.assertIn('duplicate JSON key', errors)
        self.factory.assert_not_called()

    def test_missing_or_malformed_batch_file_is_reported(self):
        path = self.directory / 'missing.json'
        status, _, errors = self.run_cli(['batch', str(path), '--duration', '1'])
        self.assertEqual(status, 1)
        self.assertTrue(errors)
        path.write_text('{broken')
        status, _, errors = self.run_cli(['batch', str(path), '--duration', '1'])
        self.assertEqual(status, 1)
        self.assertTrue(errors)
        self.factory.assert_not_called()

    def test_subscriber_timeout_does_not_send_motion(self):
        self.fleet.wait_for_subscribers.return_value = False
        status, _, errors = self.run_cli(['velocity', '--v', '0.05', '--duration', '1'])
        self.assertEqual(status, 1)
        self.assertIn('timed out', errors)
        self.fleet.run_for.assert_not_called()
        self.factory.return_value.__exit__.assert_called_once()

    def test_stop_routes_to_selected_fleet(self):
        status, _, _ = self.run_cli(['stop', '--cars', 'car02', '--wait', '0'])
        self.assertEqual(status, 0)
        self.factory.assert_called_once_with(config=None, cars=('car02',))
        self.fleet.stop.assert_called_once_with()
        self.fleet.run_for.assert_not_called()

    def test_logging_failure_does_not_prevent_stop(self):
        self.logger_factory.side_effect = OSError('disk unavailable')
        status, _, errors = self.run_cli(['stop', '--cars', 'car02', '--wait', '0'])
        self.assertEqual(status, 0)
        self.assertIn('固件日志创建失败', errors)
        self.fleet.stop.assert_called_once_with()

    def test_metadata_wait_happens_after_stop(self):
        order = []
        self.fleet.stop.side_effect = lambda: order.append('stop')
        self.logger.close.side_effect = lambda: order.append('metadata_wait')
        status, _, _ = self.run_cli(['stop', '--cars', 'car02', '--wait', '0'])
        self.assertEqual(status, 0)
        self.assertEqual(order, ['stop', 'metadata_wait'])

    def test_keyboard_interrupt_closes_context_and_returns_130(self):
        self.fleet.run_for.side_effect = KeyboardInterrupt
        status, _, errors = self.run_cli(['velocity', '--v', '0.05', '--duration', '1'])
        self.assertEqual(status, 130)
        self.assertIn('interrupted', errors)
        self.factory.return_value.__exit__.assert_called_once()

    def test_sdk_failure_is_reported_and_context_closes(self):
        self.fleet.run_for.side_effect = RuntimeError('transport failed')
        status, _, errors = self.run_cli(['velocity', '--v', '0.05', '--duration', '1'])
        self.assertEqual(status, 1)
        self.assertIn('transport failed', errors)
        self.factory.return_value.__exit__.assert_called_once()

    def test_dds_ack_cleanup_failure_is_reported_instead_of_success(self):
        self.factory.return_value.__exit__.side_effect = RuntimeError(
            'car02: DDS ACK timed out (not a vehicle execution ACK)')
        status, _, errors = self.run_cli(['stop', '--cars', 'car02', '--wait', '0'])
        self.assertEqual(status, 1)
        self.assertIn('DDS ACK timed out', errors)
        self.assertIn('not a vehicle execution ACK', errors)


if __name__ == '__main__':
    unittest.main()
