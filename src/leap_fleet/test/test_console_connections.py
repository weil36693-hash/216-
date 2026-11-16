"""Console/connection reservation races using fakes only; no ROS or network."""

import copy
import threading
import time
import unittest

from leap_fleet.web_console import Console
from leap_fleet.motion_channels import MotionChannels


class FakeMonitor:
    def __init__(self, events):
        self.events, self.closed = events, False
        self.sequence = 0

    def snapshot(self):
        self.sequence += 1
        return [{'id': name, 'status': 'connected', 'measured_v': 0.0, 'measured_w': 0.0,
                 'odom_received_at': time.monotonic(), 'odom_sequence': self.sequence, 'age_s': 0.0}
                for name in ('car01', 'car02')]

    def close(self):
        if not self.closed:
            self.events.append('monitor_close')
            self.closed = True


class FakeConnections:
    def __init__(self, events):
        self.events = events
        self.busy = set()
        self.requests = []
        self.closed = False
        self.request_error = self.close_error = None
        self.entered = threading.Event()
        self.release = None

    def snapshot(self):
        return {'busy': sorted(self.busy), 'requests': copy.deepcopy(self.requests)}

    def busy_for(self, cars):
        return bool(self.busy.intersection(cars))

    def request(self, action, cars):
        if self.request_error:
            raise self.request_error
        self.busy.update(cars)
        self.requests.append((action, list(cars)))
        self.entered.set()
        if self.release is not None and not self.release.wait(2.0):
            raise RuntimeError('fake request reservation was not released')
        return self.snapshot()

    def close(self):
        if self.closed:
            return
        self.events.append('connections_close')
        self.closed = True
        if self.close_error:
            raise self.close_error


class FakeFleet:
    def __init__(self, cars, events):
        self.names, self.events = tuple(cars), events
        self.open_entered = threading.Event()
        self.sent = threading.Event()
        self.closed = threading.Event()
        self.open_release = None
        self.stop_count = 0

    def open(self):
        self.events.append('fleet_open')
        self.open_entered.set()
        if self.open_release is not None and not self.open_release.wait(2.0):
            raise RuntimeError('fake fleet open was not released')

    def wait_for_subscribers(self, timeout=1.0):
        return True

    def set_velocities(self, commands):
        if all(v.v == 0 and v.w == 0 for v in commands.values()):
            self.stop()
        else:
            self.sent.set()

    def stop(self):
        self.events.append('fleet_stop')
        self.stop_count += 1

    def close(self, *, send_stop=True):
        self.events.append('fleet_close')
        self.closed.set()


class ConsoleConnectionsTests(unittest.TestCase):
    def setUp(self):
        self.events, self.fleets = [], []
        self.open_release = None
        self.monitor = FakeMonitor(self.events)
        self.connections = FakeConnections(self.events)
        self.console = Console(self.monitor, self.fleet_factory, connections=self.connections,
                               channels=MotionChannels(self.fleet_factory, stop_duration=0.12))
        self.console.STOP_STABLE_TIME = 0.06
        self.console.STOP_TIMEOUT = 0.5
        self.addCleanup(self.console.close)

    def fleet_factory(self, cars):
        fleet = FakeFleet(cars, self.events)
        fleet.open_release = self.open_release
        self.fleets.append(fleet)
        return fleet

    def wait_until(self, predicate):
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if predicate():
                return
            threading.Event().wait(0.002)
        self.fail('expected fake console state was not reached')

    def start_motion(self, car='car01'):
        return self.console.start({car: {'v': 0.05, 'w': 0.0}}, duration=10.0)

    def test_snapshot_includes_connection_state_and_routes_action(self):
        self.assertEqual(self.console.snapshot()['connections']['busy'], [])
        result = self.console.connection_action('connect', ['car02'])
        self.assertEqual(self.connections.requests, [('connect', ['car02'])])
        self.assertEqual(result['connections']['busy'], ['car02'])
        self.assertEqual(self.console.snapshot()['connections']['busy'], ['car02'])

    def test_same_car_motion_is_rejected_after_connection_is_reserved(self):
        self.console.connection_action('connect', ['car01'])
        with self.assertRaises(RuntimeError):
            self.start_motion('car01')
        self.assertEqual(self.fleets, [])
        self.assertIsNone(self.console.snapshot()['job'])

    def test_unrelated_car_can_run_during_reserved_connection(self):
        self.console.connection_action('connect', ['car02'])
        self.start_motion('car01')
        self.wait_until(lambda: bool(self.fleets) and self.fleets[0].sent.is_set())
        self.assertEqual(self.fleets[0].names, ('car01',))

    def test_connection_change_is_rejected_while_motion_is_starting(self):
        self.open_release = threading.Event()
        self.addCleanup(self.open_release.set)
        self.start_motion()
        self.wait_until(lambda: bool(self.fleets))
        self.assertTrue(self.fleets[0].open_entered.wait(0.5))
        with self.assertRaises(RuntimeError):
            self.console.connection_action('connect', ['car01'])
        self.assertEqual(self.connections.requests, [])

    def test_read_only_connection_check_is_allowed_during_motion(self):
        self.start_motion()
        self.wait_until(lambda: bool(self.fleets) and self.fleets[0].sent.is_set())
        result = self.console.connection_action('check', ['car01'])
        self.assertIn('connections', result)
        self.assertEqual(self.connections.requests, [('check', ['car01'])])
        self.assertEqual(self.console.snapshot()['job']['status'], 'running')

    def test_motion_and_connection_race_cannot_both_reserve_same_car(self):
        barrier = threading.Barrier(3)
        accepted, rejected = [], []

        def attempt(name, action):
            barrier.wait()
            try:
                action()
                accepted.append(name)
            except RuntimeError:
                rejected.append(name)

        connection = threading.Thread(target=attempt, args=(
            'connection', lambda: self.console.connection_action('connect', ['car01'])))
        motion = threading.Thread(target=attempt, args=('motion', self.start_motion))
        connection.start()
        motion.start()
        barrier.wait()
        connection.join(1.0)
        motion.join(1.0)
        self.assertFalse(connection.is_alive())
        self.assertFalse(motion.is_alive())
        self.assertEqual(len(accepted), 1)
        self.assertEqual(len(rejected), 1)
        self.assertNotEqual(accepted[0], rejected[0])

    def test_request_and_motion_share_lock_until_busy_reservation_is_visible(self):
        release = threading.Event()
        self.connections.release = release
        self.addCleanup(release.set)
        request_errors, motion_errors = [], []

        def connect():
            try:
                self.console.connection_action('connect', ['car01'])
            except Exception as error:
                request_errors.append(error)

        def move():
            try:
                self.start_motion()
            except RuntimeError as error:
                motion_errors.append(error)

        connection = threading.Thread(target=connect)
        connection.start()
        self.assertTrue(self.connections.entered.wait(0.5))
        motion = threading.Thread(target=move)
        motion.start()
        release.set()
        connection.join(1.0)
        motion.join(1.0)
        self.assertEqual(request_errors, [])
        self.assertEqual(len(motion_errors), 1)
        self.assertEqual(self.fleets, [])

    def test_stop_job_remains_available_while_connections_are_busy(self):
        self.start_motion()
        self.wait_until(lambda: bool(self.fleets) and self.fleets[0].sent.is_set())
        self.connections.busy.add('car02')
        result = self.console.stop(scope='job')
        self.assertEqual(result['job']['status'], 'stopping')
        self.wait_until(lambda: self.console.snapshot()['job']['status'] == 'stopped')
        self.assertGreater(self.fleets[0].stop_count, 0)
        self.assertFalse(self.fleets[0].closed.is_set())

    def test_stop_all_remains_available_during_connection_work(self):
        self.connections.busy.update(('car01', 'car02'))
        result = self.console.stop(scope='all')
        self.assertEqual(result['job']['status'], 'stopping')
        self.wait_until(lambda: self.console.snapshot()['job']['status'] == 'stopped')
        self.assertEqual({f.names for f in self.fleets}, {('car01',), ('car02',)})
        self.assertGreater(self.fleets[0].stop_count, 0)

    def test_manager_request_error_does_not_leave_console_motion_locked(self):
        self.connections.request_error = ValueError('invalid connection request')
        with self.assertRaisesRegex(ValueError, 'invalid connection request'):
            self.console.connection_action('connect', ['car01'])
        self.start_motion()
        self.wait_until(lambda: bool(self.fleets) and self.fleets[0].sent.is_set())

    def test_close_waits_for_motion_cleanup_before_connections_and_monitor(self):
        self.start_motion()
        self.wait_until(lambda: bool(self.fleets) and self.fleets[0].sent.is_set())
        self.console.close()
        self.assertLess(self.events.index('fleet_stop'), self.events.index('fleet_close'))
        self.assertLess(self.events.index('fleet_close'), self.events.index('connections_close'))
        self.assertLess(self.events.index('connections_close'), self.events.index('monitor_close'))
        with self.assertRaises(RuntimeError):
            self.console.connection_action('connect', ['car01'])
        with self.assertRaises(RuntimeError):
            self.start_motion()

    def test_manager_close_error_cannot_skip_monitor_cleanup(self):
        self.connections.close_error = RuntimeError('manager cleanup failed')
        with self.assertRaisesRegex(RuntimeError, 'manager cleanup failed'):
            self.console.close()
        self.assertTrue(self.monitor.closed)
        self.assertLess(self.events.index('connections_close'), self.events.index('monitor_close'))


if __name__ == '__main__':
    unittest.main()
