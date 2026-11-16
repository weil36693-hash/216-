"""Operator flow and failure regressions; in-memory vehicles only, no ROS."""

import copy
import threading
import time
import unittest
from unittest.mock import MagicMock

from leap_fleet.motion_channels import MotionChannels
from leap_fleet.web_console import Console


class FakeMonitor:
    def __init__(self):
        self.cars = [{'id': name, 'name': name, 'status': 'connected',
                      'measured_v': 0.0, 'measured_w': 0.0, 'age_s': 0.0,
                      'odom_received_at': time.monotonic(), 'odom_sequence': 1}
                     for name in ('car01', 'car02', 'car03')]
        self.fresh = True
        self.closed = False

    def snapshot(self):
        for car in self.cars:
            if self.fresh:
                car.update(odom_received_at=time.monotonic(), age_s=0.0,
                           odom_sequence=car['odom_sequence'] + 1)
        return copy.deepcopy(self.cars)

    def close(self):
        self.closed = True


class FakeFleet:
    def __init__(self, cars, monitor):
        self.names = tuple(cars)
        self.monitor = monitor
        self.commands, self.stops = [], []
        self.open_entered = threading.Event()
        self.publish_entered = threading.Event()
        self.open_release = self.publish_release = None
        self.open_error = self.publish_error = self.stop_error = None
        self.ignore_stop = False
        self.closed = threading.Event()

    def open(self):
        self.open_entered.set()
        if self.open_release is not None:
            self.open_release.wait(3)
        if self.open_error:
            raise self.open_error

    def wait_for_subscribers(self, timeout=5):
        return True

    def set_velocities(self, commands):
        if all(v.v == 0 and v.w == 0 for v in commands.values()):
            self.stop()
            return
        self.publish_entered.set()
        if self.publish_release is not None:
            self.publish_release.wait(3)
        if self.publish_error:
            raise self.publish_error
        self.commands.append(dict(commands))
        for car in self.monitor.cars:
            if car['id'] in commands:
                velocity = commands[car['id']]
                car.update(measured_v=velocity.v, measured_w=velocity.w)

    def stop(self, cars=None):
        self.stops.append(time.monotonic())
        if self.stop_error:
            raise self.stop_error
        if not self.ignore_stop:
            for car in self.monitor.cars:
                if car['id'] in self.names:
                    car.update(measured_v=0.0, measured_w=0.0)

    def close(self, *, send_stop=True):
        self.closed.set()


class WebConsoleTests(unittest.TestCase):
    def setUp(self):
        self.monitor = FakeMonitor()
        self.fleets = []
        self.configure_fleet = None
        self.channels = MotionChannels(self.fleet_factory, stop_duration=0.12)
        self.console = Console(self.monitor, self.fleet_factory, channels=self.channels)
        self.console.STOP_TIMEOUT = 0.5
        self.console.STOP_STABLE_TIME = 0.06
        self.console.DISCOVERY_TIMEOUT = 0.5
        self.addCleanup(self.console.close)

    def fleet_factory(self, cars):
        fleet = FakeFleet(cars, self.monitor)
        if self.configure_fleet:
            self.configure_fleet(fleet)
        self.fleets.append(fleet)
        return fleet

    def wait_until(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.005)
        self.fail('expected state was not reached')

    def finished(self):
        self.wait_until(lambda: self.console.snapshot()['job']['status']
                        not in {'starting', 'running', 'stopping'})
        self.console._worker.join(0.2)
        return self.console.snapshot()['job']

    def start(self, names=('car01',), duration=0.1):
        return self.console.start({name: {'v': 0.05, 'w': 0.0} for name in names}, duration, 50)

    def test_observation_never_opens_publishers(self):
        self.assertIsNone(self.console.snapshot()['job'])
        self.assertEqual(self.fleets, [])

    def test_motion_job_has_its_own_firmware_log(self):
        logger = MagicMock()
        logger.record.return_value = '/tmp/session_firmware.json'
        self.console.firmware_logger = logger
        self.start()
        final = self.finished()
        self.assertEqual(final['firmware_log'], '/tmp/session_firmware.json')
        logger.record.assert_called_once_with(['car01'], session_id=final['id'], kind='velocity')
        self.assertEqual(final['status'], 'completed')

    def test_metadata_write_failure_does_not_publish_motion_or_prevent_stop(self):
        logger = MagicMock()
        logger.record.side_effect = OSError('log unavailable')
        self.console.firmware_logger = logger
        self.start()
        final = self.finished()
        self.assertEqual(final['status'], 'failed')
        self.assertIn('log unavailable', final['error'])
        self.assertTrue(all(not fleet.commands for fleet in self.fleets))

    def test_invalid_requests_do_not_open_any_car(self):
        valid = {'car01': {'v': 0.05, 'w': 0}}
        for commands, duration, rate in (({}, 1, 20), ({'missing': {'v': 0, 'w': 0}}, 1, 20),
                                        ({'car01': {'v': float('nan'), 'w': 0}}, 1, 20),
                                        (valid, 0.09, 20), (valid, 61, 20), (valid, 1, 0),
                                        (valid, 1, 51), (valid, float('inf'), 20)):
            with self.subTest(commands=commands, duration=duration, rate=rate):
                with self.assertRaises(ValueError):
                    self.console.start(commands, duration, rate)
        self.assertEqual(self.fleets, [])

    def test_every_target_must_be_connected(self):
        self.monitor.cars[1]['status'] = 'offline'
        with self.assertRaises(ValueError):
            self.start(('car01', 'car02'))
        self.assertEqual(self.fleets, [])

    def test_finite_job_repeats_stop_and_reuses_publisher(self):
        self.start()
        final = self.finished()
        self.assertEqual(final['status'], 'completed')
        self.assertEqual(final['car_results']['car01']['stop_status'], 'confirmed')
        fleet = self.fleets[0]
        self.assertGreaterEqual(len(fleet.commands), 2)
        self.assertGreaterEqual(len(fleet.stops), 3)
        self.assertFalse(fleet.closed.is_set())
        self.start()
        self.assertEqual(self.finished()['status'], 'completed')
        self.assertEqual(len(self.fleets), 1)

    def test_stop_request_is_nonblocking_and_limits_scope(self):
        self.start(('car02',), duration=10)
        self.wait_until(lambda: self.fleets and self.fleets[0].commands)
        before = time.monotonic()
        result = self.console.stop()
        self.assertLess(time.monotonic() - before, 0.15)
        self.assertEqual(result['job']['status'], 'stopping')
        self.assertEqual(self.finished()['status'], 'stopped')
        self.assertEqual([fleet.names for fleet in self.fleets], [('car02',)])

    def test_stop_can_be_sent_again_after_completed_job(self):
        self.start()
        self.finished()
        prior = len(self.fleets[0].stops)
        self.console.stop()
        self.assertEqual(self.finished()['status'], 'stopped')
        self.assertGreater(len(self.fleets[0].stops), prior + 1)
        self.assertEqual(len(self.fleets), 1)

    def test_cancel_during_open_never_sends_motion(self):
        release = threading.Event()
        self.configure_fleet = lambda fleet: setattr(fleet, 'open_release', release)
        self.start(duration=10)
        self.wait_until(lambda: self.fleets and self.fleets[0].open_entered.is_set())
        try:
            self.console.stop()
        finally:
            release.set()
        self.assertEqual(self.finished()['status'], 'stopped')
        self.assertEqual(self.fleets[0].commands, [])
        self.assertGreaterEqual(len(self.fleets[0].stops), 3)

    def test_blocked_car_cannot_delay_other_car_stop_or_state_api(self):
        release = threading.Event()
        self.configure_fleet = lambda fleet: setattr(
            fleet, 'publish_release', release if fleet.names == ('car01',) else None)
        self.start(('car01', 'car02'), duration=10)
        self.wait_until(lambda: len(self.fleets) == 2 and
                        all(fleet.publish_entered.is_set() for fleet in self.fleets))
        try:
            before = time.monotonic()
            self.console.stop()
            self.console.snapshot()
            self.assertLess(time.monotonic() - before, 0.15)
            healthy = next(f for f in self.fleets if f.names == ('car02',))
            self.wait_until(lambda: bool(healthy.stops), timeout=0.2)
            final = self.finished()
            self.assertEqual(final['car_results']['car02']['stop_status'], 'confirmed')
            self.assertEqual(final['car_results']['car01']['stop_status'], 'unconfirmed')
            with self.assertRaises(RuntimeError):
                self.start()
        finally:
            release.set()
        self.wait_until(lambda: all(not item['stop_pending'] for item in self.channels.snapshot().values()))
        self.console.stop()
        self.assertEqual(self.finished()['status'], 'stopped')

    def test_preexisting_zero_feedback_is_not_stop_confirmation(self):
        self.monitor.fresh = False
        self.start()
        final = self.finished()
        self.assertEqual(final['status'], 'unconfirmed')
        self.assertEqual(final['car_results']['car01']['stop_status'], 'unconfirmed')
        with self.assertRaises(RuntimeError):
            self.start()
        self.monitor.fresh = True
        self.console.stop()
        self.assertEqual(self.finished()['status'], 'stopped')

    def test_fresh_but_moving_feedback_is_not_confirmed(self):
        self.configure_fleet = lambda fleet: setattr(fleet, 'ignore_stop', True)
        self.start()
        final = self.finished()
        self.assertEqual(final['status'], 'unconfirmed')
        self.assertIn('仍报告运动', final['car_results']['car01']['reason'])

    def test_one_new_zero_sample_cannot_be_counted_repeatedly(self):
        now = time.monotonic()
        channel = {'first_zero_at': now - 1, 'stop_pending': False, 'zero_count': 20}
        state = {'status': 'connected', 'odom_received_at': now, 'odom_sequence': 7,
                 'age_s': 0, 'measured_v': 0, 'measured_w': 0}
        tracker = {}
        for _ in range(20):
            result = self.console._feedback_result('car01', channel, state, tracker)
        self.assertEqual(result['stop_status'], 'pending')
        self.assertEqual(tracker['count'], 1)

    def test_released_or_unconfirmed_channel_cannot_reuse_old_zero_history(self):
        now = time.monotonic()
        state = {'status': 'connected', 'odom_received_at': now, 'odom_sequence': 7,
                 'age_s': 0, 'measured_v': 0, 'measured_w': 0}
        for flag in ('closed', 'releasing', 'stop_unconfirmed'):
            with self.subTest(flag=flag):
                channel = {'first_zero_at': now - 1, 'stop_pending': False,
                           'zero_count': 20, flag: True}
                tracker = {'count': 10, 'since': now - 0.5}
                result = self.console._feedback_result('car01', channel, state, tracker)
                self.assertEqual(result['stop_status'], 'pending')
                self.assertEqual(tracker, {})

    def test_offline_stop_all_keeps_independent_results(self):
        self.monitor.cars[2]['status'] = 'offline'
        self.console.stop('all')
        final = self.finished()
        self.assertEqual(final['status'], 'unconfirmed')
        self.assertEqual(len(self.fleets), 3)
        self.assertTrue(all(fleet.stops and not fleet.commands for fleet in self.fleets))
        self.assertEqual(final['car_results']['car01']['stop_status'], 'confirmed')
        self.assertEqual(final['car_results']['car03']['stop_status'], 'unconfirmed')
        self.start(('car01',))
        self.assertEqual(self.finished()['status'], 'completed')

    def test_stop_all_during_motion_extends_stop_targets(self):
        self.start(('car02',), duration=10)
        self.wait_until(lambda: self.fleets and self.fleets[0].commands)
        self.console.stop('all')
        final = self.finished()
        self.assertEqual(final['status'], 'stopped')
        self.assertEqual(set(final['car_results']), {'car01', 'car02', 'car03'})
        self.assertEqual(len(self.fleets), 3)
        self.assertTrue(all(not f.commands for f in self.fleets if f.names != ('car02',)))

    def test_open_failure_does_not_prevent_other_car_stop(self):
        self.configure_fleet = lambda fleet: setattr(
            fleet, 'open_error', RuntimeError('open failed') if fleet.names == ('car01',) else None)
        self.start(('car01', 'car02'))
        final = self.finished()
        self.assertEqual(final['status'], 'failed')
        self.assertEqual(final['car_results']['car02']['stop_status'], 'confirmed')
        self.assertTrue(all(not f.commands for f in self.fleets))

    def test_publish_error_is_visible_even_after_successful_zero(self):
        self.configure_fleet = lambda fleet: setattr(fleet, 'publish_error', RuntimeError('send failed'))
        self.start()
        final = self.finished()
        self.assertEqual(final['status'], 'failed')
        self.assertIn('send failed', final['error'])
        self.assertTrue(self.fleets[0].stops)

    def test_second_start_rejected_until_stop_verification_finishes(self):
        self.start(duration=10)
        with self.assertRaises(RuntimeError):
            self.start(('car02',))
        self.console.stop()
        with self.assertRaises(RuntimeError):
            self.start(('car02',))
        self.finished()

    def test_close_stops_workers_before_monitor_and_rejects_new_jobs(self):
        self.start(duration=10)
        self.wait_until(lambda: self.fleets and self.fleets[0].commands)
        self.console.close()
        self.assertTrue(self.monitor.closed)
        self.assertTrue(self.fleets[0].closed.is_set())
        self.assertTrue(self.fleets[0].stops)
        with self.assertRaises(RuntimeError):
            self.start()

    def test_repeated_stop_cannot_confirm_using_preceding_cached_window(self):
        self.start()
        self.assertEqual(self.finished()['status'], 'completed')
        snapshot_ready, release_snapshot = threading.Event(), threading.Event()
        stop_entered, stop_returned = threading.Event(), threading.Event()
        original_snapshot = self.channels.snapshot
        snapshot_worker = [None]
        errors = []

        def held_snapshot():
            states = original_snapshot()
            channel = states.get('car01', {})
            if (threading.current_thread() is snapshot_worker[0]
                    and channel.get('first_zero_at') is not None
                    and not channel.get('stop_pending')
                    and not channel.get('publishing') and not snapshot_ready.is_set()):
                # Pause after reading a finished zero window. A simultaneous
                # stop must either precede this reservation or start a new one;
                # it cannot reset the channel behind this cached confirmation.
                snapshot_ready.set()
                if not release_snapshot.wait(2.0):
                    raise RuntimeError('test snapshot barrier was not released')
            return states

        def stop_again():
            stop_entered.set()
            try:
                self.console.stop()
            except BaseException as error:
                errors.append(error)
            finally:
                stop_returned.set()

        self.channels.snapshot = held_snapshot
        self.addCleanup(setattr, self.channels, 'snapshot', original_snapshot)
        self.addCleanup(release_snapshot.set)
        stopper = None
        try:
            self.console.stop()
            snapshot_worker[0] = self.console._worker
            self.assertTrue(snapshot_ready.wait(2.0), 'finished zero snapshot was not captured')
            stopper = threading.Thread(target=stop_again, daemon=True)
            stopper.start()
            self.assertTrue(stop_entered.wait(1.0))
            # On the old unlocked-snapshot implementation this returns at once
            # and resets stop_pending before the old result is committed.
            stop_returned.wait(0.2)
            release_snapshot.set()
            stopper.join(timeout=2.0)
            self.assertFalse(stopper.is_alive(), 'repeated stop did not return')
            self.assertEqual(errors, [])
            final = self.finished()
            self.assertEqual(final['status'], 'stopped')
            self.assertEqual(final['car_results']['car01']['stop_status'], 'confirmed')
            channel = original_snapshot()['car01']
            self.assertFalse(channel['stop_pending'],
                             'job confirmed the preceding cache while a newer stop was pending')
            self.assertGreaterEqual(channel['zero_count'], 2)
        finally:
            release_snapshot.set()
            if stopper is not None:
                stopper.join(timeout=2.0)


if __name__ == '__main__':
    unittest.main()
