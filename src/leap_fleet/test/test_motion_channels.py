"""Independent motion senders with in-memory Fleet fakes; never opens ROS."""

import threading
import time
import unittest
from unittest import mock

from leap_fleet import Velocity
from leap_fleet.motion_channels import MotionChannels


ZERO = Velocity(0, 0)
MOVE = Velocity(0.05, 0)


class FakeFleet:
    def __init__(self, name):
        self.name = name
        self.open_entered = threading.Event()
        self.discovery_entered = threading.Event()
        self.motion_entered = threading.Event()
        self.close_entered = threading.Event()
        self.closed = threading.Event()
        self.open_release = self.motion_release = self.close_release = None
        self.discovery_release = None
        self.discovery_error = None
        self.matched = True
        self.open_error = self.close_error = None
        self.motion_errors = self.zero_errors = 0
        self.drop_first_zero = False
        self.attempts, self.delivered, self.close_options = [], [], []
        self.probes = 0

    @staticmethod
    def wait(release):
        if release is not None and not release.wait(2.0):
            raise RuntimeError('fake I/O was not released')

    def open(self):
        self.open_entered.set()
        self.wait(self.open_release)
        if self.open_error:
            raise self.open_error

    def wait_for_subscribers(self, timeout):
        self.probes += 1
        self.discovery_entered.set()
        self.wait(self.discovery_release)
        if self.discovery_error:
            raise self.discovery_error
        return self.matched

    def set_velocities(self, commands):
        if set(commands) != {self.name}:
            raise AssertionError('Each worker must send to exactly its own car')
        velocity = commands[self.name]
        self.attempts.append((time.monotonic(), velocity))
        if velocity != ZERO:
            self.motion_entered.set()
            self.wait(self.motion_release)
            if self.motion_errors:
                self.motion_errors -= 1
                raise RuntimeError('motion write failed')
        else:
            if self.zero_errors:
                self.zero_errors -= 1
                raise RuntimeError('zero write failed')
            if self.drop_first_zero:
                self.drop_first_zero = False
                return
        self.delivered.append((time.monotonic(), velocity))

    def close(self, *, send_stop=True):
        self.close_options.append(send_stop)
        self.close_entered.set()
        self.wait(self.close_release)
        self.closed.set()
        if self.close_error:
            raise self.close_error


class MotionChannelsTests(unittest.TestCase):
    def setUp(self):
        self.fleets = {}
        self.created = []
        self.configure = lambda fleet: None
        self.pool = MotionChannels(self.factory, stop_duration=0.06, stop_rate=100)
        self.addCleanup(self.cleanup)

    def factory(self, cars):
        self.assertEqual(len(cars), 1)
        fleet = FakeFleet(cars[0])
        self.configure(fleet)
        self.fleets[cars[0]] = fleet
        self.created.append(fleet)
        return fleet

    def cleanup(self):
        for fleet in self.created:
            fleet.matched = True
            fleet.zero_errors = 0
            fleet.discovery_error = None
            for release in (fleet.open_release, fleet.motion_release,
                            fleet.discovery_release, fleet.close_release):
                if release is not None:
                    release.set()
        try:
            self.pool.close(timeout=1.0)
        except RuntimeError:
            # Tests explicitly exercise and assert close failures before cleanup.
            pass

    def wait_until(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            threading.Event().wait(0.002)
        self.fail(f'expected channel state not reached: {self.pool.snapshot()}')

    def ready(self, *names):
        self.pool.ensure(names)
        self.wait_until(lambda: all(self.pool.snapshot().get(name, {}).get('ready')
                                   for name in names))

    def parked(self, *names):
        self.wait_until(lambda: all(not self.pool.snapshot()[name]['stop_pending']
                                   and not self.pool.snapshot()[name]['active']
                                   and not self.pool.snapshot()[name]['publishing'] for name in names))

    def test_ensure_is_nonblocking_and_opening_one_car_does_not_block_another(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'open_release', release) if fleet.name == 'a' else None
        before = time.monotonic()
        self.pool.ensure(['a', 'b'])
        self.assertLess(time.monotonic() - before, 0.1)
        self.wait_until(lambda: 'a' in self.fleets and self.pool.snapshot()['b']['ready'])
        self.assertTrue(self.fleets['a'].open_entered.is_set())
        self.assertFalse(self.pool.snapshot()['a']['ready'])
        self.pool.start({'b': MOVE}, duration=0.05, rate=50)
        self.wait_until(lambda: self.pool.snapshot()['b']['command_count'] > 0)
        self.assertFalse(release.is_set())

    def test_shared_deadline_expires_into_repeated_zero_without_closing_fleets(self):
        self.ready('a', 'b')
        deadline = self.pool.start({'a': MOVE, 'b': Velocity(0, 0.1)}, 0.06, 100)
        self.assertGreater(deadline, time.monotonic())
        self.wait_until(lambda: all(self.pool.snapshot()[name]['zero_count'] >= 2 for name in ('a', 'b')))
        self.parked('a', 'b')
        for name in ('a', 'b'):
            state = self.pool.snapshot()[name]
            self.assertGreater(state['command_count'], 0)
            self.assertGreaterEqual(state['zero_count'], 2)
            self.assertGreaterEqual(state['first_zero_at'], deadline)
            self.assertFalse(self.fleets[name].closed.is_set())
        self.pool.ensure(['a'])
        self.wait_until(lambda: self.pool.snapshot()['a']['ready'])
        self.pool.start({'a': MOVE}, 0.03, 50)
        self.assertEqual(len(self.created), 2)

    def test_batch_is_validated_before_any_car_begins(self):
        self.ready('a')
        with self.assertRaises(RuntimeError):
            self.pool.start({'a': MOVE, 'missing': MOVE}, 1, 20)
        self.assertEqual(self.fleets['a'].attempts, [])
        for value in (float('nan'), -1, 0):
            with self.subTest(duration=value), self.assertRaises(ValueError):
                self.pool.start({'a': MOVE}, value, 20)
        with self.assertRaises(ValueError):
            self.pool.start({'a': (0.05, 0)}, 1, 20)
        self.assertEqual(self.fleets['a'].attempts, [])

    def test_blocked_motion_does_not_block_stop_or_another_car(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release) if fleet.name == 'a' else None
        self.ready('a', 'b')
        self.pool.start({'a': MOVE, 'b': MOVE}, 10, 100)
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        before = time.monotonic()
        self.pool.stop(['a', 'b'])
        self.assertLess(time.monotonic() - before, 0.1)
        self.wait_until(lambda: self.pool.snapshot()['b']['zero_count'] >= 2)
        self.assertTrue(self.pool.snapshot()['a']['publishing'])
        self.assertEqual(self.pool.snapshot()['a']['zero_count'], 0)
        with self.assertRaises(RuntimeError):
            self.pool.start({'a': MOVE}, 0.1, 20)
        release.set()
        self.parked('a', 'b')
        attempts = [velocity for _, velocity in self.fleets['a'].attempts]
        self.assertEqual(attempts[0], MOVE)
        self.assertTrue(all(value == ZERO for value in attempts[1:]))

    def test_stop_during_late_open_and_discovery_is_not_lost(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'open_release', release)
        self.pool.stop(['a'])
        self.wait_until(lambda: 'a' in self.fleets)
        self.fleets['a'].matched = False
        release.set()
        self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] > 0)
        self.assertTrue(self.pool.snapshot()['a']['stop_pending'])
        self.fleets['a'].matched = True
        self.parked('a')
        self.assertTrue(all(velocity == ZERO for _, velocity in self.fleets['a'].attempts))

    def test_lost_first_zero_is_followed_by_more_zero_publications(self):
        self.ready('a')
        self.fleets['a'].drop_first_zero = True
        self.pool.stop(['a'])
        self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] >= 2)
        self.parked('a')
        self.assertGreaterEqual(len(self.fleets['a'].delivered), 1)
        self.assertTrue(all(velocity == ZERO for _, velocity in self.fleets['a'].delivered))

    def test_failing_first_zero_keeps_bounded_retry_waits(self):
        self.ready('a')
        self.fleets['a'].zero_errors = 1000
        channel = self.pool._channels['a']
        original_wait = channel.wake.wait
        zero_waits = []

        def wait(timeout=None):
            if timeout is not None and timeout <= 0:
                zero_waits.append(timeout)
            return original_wait(timeout)

        channel.wake.wait = wait
        with self.pool._lock:
            channel.next_probe = 0.0
            self.pool.stop(['a'])
        threading.Event().wait(0.04)
        self.assertEqual(zero_waits, [])
        self.assertLessEqual(len(self.fleets['a'].attempts), 8)
        self.fleets['a'].zero_errors = 0
        self.parked('a')

    def test_repeated_stop_preserves_window_counts_and_prevents_new_motion(self):
        self.ready('a')
        self.pool.stop(['a'])
        self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] >= 1)
        first = self.pool.snapshot()['a']['first_zero_at']
        count = self.pool.snapshot()['a']['zero_count']
        self.pool.stop(['a'])
        self.assertEqual(self.pool.snapshot()['a']['first_zero_at'], first)
        self.assertGreaterEqual(self.pool.snapshot()['a']['zero_count'], count)
        with self.assertRaises(RuntimeError):
            self.pool.start({'a': MOVE}, 0.1, 20)
        self.parked('a')

    def test_errors_remain_visible_after_zero_recovery_until_new_start(self):
        self.ready('a')
        self.fleets['a'].motion_errors = 1
        self.fleets['a'].zero_errors = 1
        self.pool.start({'a': MOVE}, 0.5, 50)
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is not None)
        self.parked('a')
        self.assertIn('zero write failed', self.pool.snapshot()['a']['error'])
        self.pool.start({'a': MOVE}, 0.03, 50)
        self.assertIsNone(self.pool.snapshot()['a']['error'])

    def test_expired_motion_block_does_not_send_more_motion_after_return(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release)
        self.ready('a')
        self.pool.start({'a': MOVE}, 0.03, 100)
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        threading.Event().wait(0.05)
        release.set()
        self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] > 0)
        self.parked('a')
        self.assertEqual(sum(velocity != ZERO for _, velocity in self.fleets['a'].attempts), 1)

    def test_probe_returning_after_deadline_cannot_send_expired_motion(self):
        self.ready('a')
        fleet = self.fleets['a']
        release = threading.Event()
        fleet.discovery_release = release
        fleet.discovery_entered.clear()
        with self.pool._lock:
            self.pool._channels['a'].next_probe = 0.0
            self.pool.start({'a': MOVE}, 0.03, 100)
        self.assertTrue(fleet.discovery_entered.wait(0.5))
        threading.Event().wait(0.05)
        release.set()
        self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] > 0)
        self.parked('a')
        self.assertTrue(all(velocity == ZERO for _, velocity in fleet.attempts))

    def test_new_idle_stop_clears_old_error_but_in_progress_stop_keeps_it(self):
        self.ready('a')
        self.fleets['a'].motion_errors = 1
        self.pool.start({'a': MOVE}, 0.5, 100)
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is not None)
        original = self.pool.snapshot()['a']['error']
        self.pool.stop(['a'])
        self.assertEqual(self.pool.snapshot()['a']['error'], original)
        self.parked('a')
        self.pool.stop(['a'])
        self.assertIsNone(self.pool.snapshot()['a']['error'])
        self.assertEqual(self.pool.snapshot()['a']['last_error'], original)
        self.parked('a')

    def test_idle_match_refresh_and_loss_during_motion(self):
        self.ready('a')
        self.fleets['a'].matched = False
        self.wait_until(lambda: not self.pool.snapshot()['a']['ready'])
        with self.assertRaises(RuntimeError):
            self.pool.start({'a': MOVE}, 1, 20)
        self.fleets['a'].matched = True
        self.ready('a')
        self.pool.start({'a': MOVE}, 10, 100)
        self.wait_until(lambda: self.pool.snapshot()['a']['command_count'] > 0)
        self.fleets['a'].matched = False
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is not None)
        self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] >= 2)
        self.assertTrue(self.pool.snapshot()['a']['stop_pending'])
        self.fleets['a'].matched = True
        self.parked('a')

    def test_ensure_refreshes_match_before_reusing_idle_connection(self):
        self.ready('a')
        probes = self.fleets['a'].probes
        release = threading.Event()
        self.fleets['a'].discovery_release = release
        self.pool.ensure(['a'])
        self.wait_until(lambda: self.fleets['a'].probes > probes)
        self.assertFalse(self.pool.snapshot()['a']['ready'])
        with self.assertRaises(RuntimeError):
            self.pool.start({'a': MOVE}, 1, 20)
        release.set()
        self.wait_until(lambda: self.pool.snapshot()['a']['ready'])

    def test_close_is_bounded_and_does_not_destroy_blocked_fleet(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release)
        self.ready('a')
        self.pool.start({'a': MOVE}, 10, 100)
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        before = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'a'):
            self.pool.close(timeout=0.03)
        self.assertLess(time.monotonic() - before, 0.15)
        self.assertFalse(self.fleets['a'].closed.is_set())
        release.set()
        self.pool.close(timeout=0.5)
        self.assertEqual(self.fleets['a'].close_options, [False])

    def test_close_releases_never_moving_unmatched_and_failed_open_channels(self):
        def configure(fleet):
            if fleet.name == 'a':
                fleet.matched = False
            else:
                fleet.open_error = RuntimeError('open failed')
        self.configure = configure
        self.pool.stop(['a', 'b'])
        self.wait_until(lambda: all(name in self.fleets for name in ('a', 'b')))
        self.wait_until(lambda: self.pool.snapshot()['b']['closed'])
        self.pool.close(timeout=0.5)
        self.assertTrue(all(fleet.closed.is_set() for fleet in self.created))
        self.assertTrue(self.pool.snapshot()['a']['stop_unconfirmed'])
        self.assertIn('open failed', self.pool.snapshot()['b']['error'])

    def test_release_only_idle_does_not_send_zero_and_removes_channel(self):
        self.ready('a')
        fleet = self.fleets['a']
        self.pool.release(['missing', 'a'], timeout=0.5)
        self.assertEqual(fleet.attempts, [])
        self.assertEqual(fleet.close_options, [False])
        self.assertEqual(self.pool.snapshot(), {})
        self.ready('a')
        self.assertEqual(len(self.created), 2)
        self.pool.start({'a': MOVE}, 0.03, 50)
        with self.assertRaises(RuntimeError):
            self.pool.release(['a'])

    def test_releasing_and_failed_cleanup_targets_do_not_block_healthy_stop(self):
        self.ready('releasing', 'failed', 'healthy')
        release = threading.Event()
        self.fleets['releasing'].close_release = release
        with self.assertRaises(RuntimeError):
            self.pool.release(['releasing'], timeout=0.01)
        self.fleets['failed'].close_error = RuntimeError('cleanup failed')
        with self.assertRaises(RuntimeError):
            self.pool.release(['failed'], timeout=0.5)
        self.pool.stop(['releasing', 'failed', 'healthy'])
        self.wait_until(lambda: self.pool.snapshot()['healthy']['zero_count'] >= 2)
        self.assertTrue(self.pool.snapshot()['releasing']['stop_unconfirmed'])
        self.assertTrue(self.pool.snapshot()['failed']['error'])
        release.set()

    def test_stream_validates_all_targets_and_intervals_before_reserving_any(self):
        self.ready('a')
        invalid = [dict(duration=0), dict(duration=float('nan')), dict(duration=1, ttl=0),
                   dict(duration=1, ttl=True), dict(duration=1, rate=0),
                   dict(duration=1, rate=float('inf')),
                   dict(duration=1, durations={}), dict(duration=1, durations={'a': 2}),
                   dict(duration=1, durations={'a': True}),
                   dict(duration=1, durations={'a': 0.5, 'b': 0.5})]
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.pool.start_stream(['a'], **options)
        with self.assertRaises(RuntimeError):
            self.pool.start_stream(['a', 'missing'], duration=1)
        self.assertFalse(self.pool.snapshot()['a']['active'])
        self.assertEqual(self.fleets['a'].attempts, [])
        self.assertEqual(self.pool._streams, {})

    def test_stream_has_common_clock_and_guarantees_initial_zero_before_immediate_update(self):
        self.ready('a', 'b')
        # Keep both workers out until the first update has already replaced the mailbox.
        with self.pool._lock:
            session = self.pool.start_stream(['a', 'b'], duration=0.5, rate=100, ttl=0.3)
            self.pool.update_stream(session['session_id'], {'a': MOVE, 'b': Velocity(0, 0.2)})
            snapshot = self.pool.snapshot()
        self.assertIsInstance(session['session_id'], str)
        self.assertAlmostEqual(session['deadline'] - session['started_at'], 0.5)
        for name in ('a', 'b'):
            self.assertEqual(snapshot[name]['stream_id'], session['session_id'])
            self.assertEqual(snapshot[name]['stream_started_at'], session['started_at'])
            self.assertEqual(snapshot[name]['stream_deadline'], session['deadline'])
        self.wait_until(lambda: all(self.fleets[name].motion_entered.is_set() for name in ('a', 'b')))
        for name in ('a', 'b'):
            self.assertEqual(self.fleets[name].attempts[0][1], ZERO)
        self.pool.stop(['a', 'b'])
        self.parked('a', 'b')

    def test_stream_batch_update_rejects_everything_before_modifying_a_valid_car(self):
        self.ready('a', 'b')
        session = self.pool.start_stream(['a', 'b'], duration=1, ttl=0.5)
        token = session['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        with self.pool._lock:
            old_expiry = self.pool._channels['a'].stream_expires_at
            for commands in ({'a': Velocity(0.2, 0), 'unknown': MOVE},
                             {'a': Velocity(0.2, 0), 'b': (0.1, 0)}):
                with self.assertRaises((RuntimeError, ValueError)):
                    self.pool.update_stream(token, commands)
                self.assertEqual(self.pool._channels['a'].velocity, MOVE)
                self.assertEqual(self.pool._channels['a'].stream_expires_at, old_expiry)
            for bad_token in ('unknown-session', '', None, 12):
                with self.assertRaises((RuntimeError, ValueError)):
                    self.pool.update_stream(bad_token, {'a': Velocity(0.2, 0)})
            with self.assertRaises(ValueError):
                self.pool.update_stream(token, {})
            self.assertEqual(self.pool._channels['a'].velocity, MOVE)

    def test_stream_ttl_failure_latches_and_sends_repeated_zeros(self):
        self.ready('a')
        session = self.pool.start_stream(['a'], duration=0.5, rate=100, ttl=0.04)
        self.pool.update_stream(session['session_id'], {'a': MOVE})
        self.wait_until(lambda: self.pool.snapshot()['a']['stream_expired'])
        self.parked('a')
        state = self.pool.snapshot()['a']
        self.assertIn('TTL expired', state['error'])
        self.assertIn('TTL expired', state['stop_reason'])
        self.assertGreaterEqual(state['zero_count'], 2)
        self.assertEqual(state['stream_id'], session['session_id'])
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(session['session_id'], {'a': MOVE})

    def test_stream_stopped_member_rejects_atomic_batch_while_other_member_continues(self):
        self.ready('a', 'b')
        token = self.pool.start_stream(['a', 'b'], duration=1, rate=100, ttl=0.5)['session_id']
        self.pool.update_stream(token, {'a': MOVE, 'b': MOVE})
        self.wait_until(lambda: self.fleets['b'].motion_entered.is_set())
        self.pool.stop(['a'])
        with self.pool._lock:
            with self.assertRaises(RuntimeError):
                self.pool.update_stream(token, {'b': Velocity(0.2, 0), 'a': MOVE})
            self.assertEqual(self.pool._channels['b'].velocity, MOVE)
        self.pool.update_stream(token, {'b': Velocity(0.08, 0.1)})
        self.wait_until(lambda: any(value == Velocity(0.08, 0.1)
                                   for _, value in self.fleets['b'].attempts))
        self.assertTrue(self.pool.snapshot()['b']['active'])
        self.pool.stop(['b'])
        self.parked('a', 'b')

    def test_stream_old_token_cannot_affect_a_new_session(self):
        self.ready('a')
        old = self.pool.start_stream(['a'], duration=1)['session_id']
        self.pool.stop(['a'])
        self.parked('a')
        self.ready('a')
        current = self.pool.start_stream(['a'], duration=1)['session_id']
        self.assertNotEqual(old, current)
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(old, {'a': MOVE})
        self.assertEqual(self.pool._channels['a'].velocity, ZERO)
        self.pool.update_stream(current, {'a': MOVE})
        self.pool.stop(['a'])
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(current, {'a': MOVE})

    def test_stream_blocked_car_expires_without_blocking_updates_or_other_car(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release) if fleet.name == 'a' else None
        self.ready('a', 'b')
        token = self.pool.start_stream(['a', 'b'], duration=1, rate=100, ttl=0.12)['session_id']
        self.pool.update_stream(token, {'a': MOVE, 'b': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        until = time.monotonic() + 0.16
        while time.monotonic() < until:
            before = time.monotonic()
            self.pool.update_stream(token, {'b': Velocity(0.08, 0)})
            self.assertLess(time.monotonic() - before, 0.05)
            threading.Event().wait(0.015)
        state = self.pool.snapshot()
        self.assertTrue(state['a']['stream_expired'])
        self.assertTrue(state['a']['publishing'])
        self.assertEqual(state['a']['zero_count'], 0)
        self.assertTrue(state['b']['active'])
        self.assertGreater(len(self.fleets['b'].delivered), 3)
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})
        self.pool.update_stream(token, {'b': MOVE})
        release.set()
        self.parked('a')
        self.assertEqual(sum(value != ZERO for _, value in self.fleets['a'].attempts), 1)
        self.pool.stop(['b'])
        self.parked('b')

    def test_stream_late_update_itself_expires_a_worker_blocked_in_io(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release)
        self.ready('a')
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=0.04)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        threading.Event().wait(0.06)
        # No snapshot or worker loop can have expired the record while this publish blocks.
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': Velocity(0.2, 0)})
        self.assertTrue(self.pool.snapshot()['a']['stream_expired'])
        release.set()
        self.parked('a')
        self.assertEqual(sum(value != ZERO for _, value in self.fleets['a'].attempts), 1)

    def test_stream_probe_returning_after_ttl_cannot_publish_nonzero(self):
        self.ready('a')
        fleet = self.fleets['a']
        release = threading.Event()
        fleet.discovery_release = release
        fleet.discovery_entered.clear()
        with self.pool._lock:
            self.pool._channels['a'].next_probe = 0.0
            token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=0.04)['session_id']
            self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(fleet.discovery_entered.wait(0.5))
        threading.Event().wait(0.06)
        release.set()
        self.wait_until(lambda: self.pool.snapshot()['a']['stream_expired'])
        self.parked('a')
        self.assertTrue(all(value == ZERO for _, value in fleet.attempts))

    def test_stream_total_duration_expires_during_blocked_io_without_ttl_error(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release)
        self.ready('a')
        session = self.pool.start_stream(['a'], duration=0.04, rate=100, ttl=0.5)
        self.pool.update_stream(session['session_id'], {'a': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        threading.Event().wait(0.06)
        state = self.pool.snapshot()['a']
        self.assertFalse(state['active'])
        self.assertFalse(state['stream_expired'])
        self.assertIsNone(state['error'])
        self.assertEqual(state['stop_reason'], 'stream duration complete')
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(session['session_id'], {'a': MOVE})
        release.set()
        self.parked('a')

    def test_stream_each_car_has_its_own_duration_and_long_car_keeps_running(self):
        self.ready('a', 'b')
        session = self.pool.start_stream(['a', 'b'], duration=0.4, rate=100, ttl=1,
                                         durations={'a': 0.04, 'b': 0.4})
        token = session['session_id']
        self.pool.update_stream(token, {'a': MOVE, 'b': MOVE})
        self.wait_until(lambda: not self.pool.snapshot()['a']['active'])
        state = self.pool.snapshot()
        self.assertAlmostEqual(state['a']['stream_deadline'] - session['started_at'], 0.04)
        self.assertEqual(state['b']['stream_deadline'], session['deadline'])
        self.assertFalse(state['a']['stream_expired'])
        self.assertIsNone(state['a']['error'])
        self.assertEqual(state['a']['stop_reason'], 'stream duration complete')
        self.assertTrue(state['b']['active'])
        self.pool.update_stream(token, {'b': Velocity(0.08, 0)})
        self.wait_until(lambda: any(value == Velocity(0.08, 0) for _, value in self.fleets['b'].attempts))
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})
        self.pool.stop(['b'])
        self.parked('a', 'b')

    def test_stream_stop_wins_over_update_already_validating_in_another_thread(self):
        self.ready('a')
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=0.5)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        validated, release = threading.Event(), threading.Event()
        original = self.pool._commands
        errors = []

        def delayed_validation(commands):
            result = original(commands)
            validated.set()
            if not release.wait(0.5):
                raise RuntimeError('test did not release validation')
            return result

        def update():
            try:
                self.pool.update_stream(token, {'a': Velocity(0.2, 0)})
            except Exception as error:
                errors.append(error)

        with mock.patch.object(self.pool, '_commands', delayed_validation):
            worker = threading.Thread(target=update)
            worker.start()
            try:
                self.assertTrue(validated.wait(0.5))
                self.pool.stop(['a'])
                self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] >= 1)
            finally:
                release.set()
                worker.join(0.5)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], RuntimeError)
        self.parked('a')
        self.assertFalse(self.pool.snapshot()['a']['active'])
        values = [value for _, value in self.fleets['a'].attempts]
        last_nonzero = max(index for index, value in enumerate(values) if value != ZERO)
        self.assertTrue(all(value == ZERO for value in values[last_nonzero + 1:]))
        self.assertNotIn(Velocity(0.2, 0), values)

    def test_stream_worker_failure_revokes_session_before_blocking_resource_close(self):
        self.ready('a')
        release = threading.Event()
        fleet = self.fleets['a']
        fleet.close_release = release
        channel = self.pool._channels['a']

        def fail_wait(timeout=None):
            raise RuntimeError('worker loop failed')

        with self.pool._lock:
            channel.wake.wait = fail_wait
            token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=0.5)['session_id']
        self.assertTrue(fleet.close_entered.wait(0.5))
        self.assertFalse(fleet.closed.is_set())
        self.assertFalse(self.pool.snapshot()['a']['active'])
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})
        with self.assertRaises(RuntimeError):
            self.pool.start_stream(['a'], duration=1)
        release.set()
        self.wait_until(lambda: self.pool.snapshot()['a']['closed'])

    def test_stream_error_in_one_car_does_not_stop_healthy_member(self):
        self.ready('a', 'b')
        self.fleets['a'].motion_errors = 1
        token = self.pool.start_stream(['a', 'b'], duration=1, rate=100, ttl=0.5)['session_id']
        self.pool.update_stream(token, {'a': MOVE, 'b': MOVE})
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is not None)
        self.assertTrue(self.pool.snapshot()['b']['active'])
        self.pool.update_stream(token, {'b': Velocity(0.08, 0)})
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})
        self.parked('a')
        self.assertIn('motion write failed', self.pool.snapshot()['a']['error'])
        self.pool.stop(['b'])
        self.parked('b')

    def test_stream_expiration_reason_uses_first_deadline_even_after_both_have_passed(self):
        self.ready('a')
        fleet = self.fleets['a']
        release = threading.Event()
        fleet.discovery_release = release
        fleet.discovery_entered.clear()
        with self.pool._lock:
            self.pool._channels['a'].next_probe = 0
            self.pool.start_stream(['a'], duration=0.05, ttl=0.02)
        self.assertTrue(fleet.discovery_entered.wait(0.5))
        threading.Event().wait(0.07)
        self.assertTrue(self.pool.snapshot()['a']['stream_expired'])
        release.set()
        self.parked('a')
        self.ready('a')
        # Equal lease and run deadlines are a normal completed duration.
        self.pool.start_stream(['a'], duration=0.04, ttl=0.04)
        self.wait_until(lambda: not self.pool.snapshot()['a']['active'])
        state = self.pool.snapshot()['a']
        self.assertFalse(state['stream_expired'])
        self.assertIsNone(state['error'])
        self.assertEqual(state['stop_reason'], 'stream duration complete')

    def test_fixed_run_after_stream_resets_stream_state_and_rejects_old_updates(self):
        self.ready('a')
        token = self.pool.start_stream(['a'], duration=0.04, ttl=1)['session_id']
        self.parked('a')
        self.ready('a')
        self.pool.start({'a': MOVE}, duration=0.04, rate=100)
        state = self.pool.snapshot()['a']
        self.assertIsNone(state['stream_id'])
        self.assertIsNone(state['stream_started_at'])
        self.assertIsNone(state['stream_deadline'])
        self.assertFalse(state['stream_expired'])
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': Velocity(0.2, 0)})
        self.parked('a')

    def test_continue_policy_requires_bool_and_none_ttl_keeps_mailbox_active(self):
        self.ready('a')
        for value in (None, 1, 'yes'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.pool.start_stream(['a'], duration=1, continue_on_error=value)
        session = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                         continue_on_error=True)
        self.pool.update_stream(session['session_id'], {'a': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        threading.Event().wait(0.33)  # Longer than the default receive lease, with no updates.
        state = self.pool.snapshot()['a']
        self.assertTrue(state['active'])
        self.assertTrue(state['continue_on_error'])
        self.assertFalse(state['stream_expired'])
        self.assertFalse(state['stop_pending'])
        self.assertIsNone(self.pool._channels['a'].stream_expires_at)
        self.pool.update_stream(session['session_id'], {'a': Velocity(0.08, 0)})
        self.pool.stop(['a'])
        self.parked('a')

    def test_continuing_stream_retries_motion_errors_and_clears_current_warning_on_recovery(self):
        self.ready('a')
        fleet = self.fleets['a']
        fleet.motion_errors = 1000
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.wait_until(lambda: self.pool.snapshot()['a']['transport_warning'] is not None)
        state = self.pool.snapshot()['a']
        self.assertTrue(state['active'])
        self.assertFalse(state['stop_pending'])
        self.assertFalse(state['stop_unconfirmed'])
        self.assertEqual(state['zero_count'], 0)
        self.assertIn('motion write failed', state['error'])
        self.wait_until(lambda: sum(value != ZERO for _, value in fleet.attempts) >= 3)
        self.pool.update_stream(token, {'a': Velocity(0.08, 0)})
        fleet.motion_errors = 0
        self.wait_until(lambda: any(value == Velocity(0.08, 0) for _, value in fleet.delivered)
                        and self.pool.snapshot()['a']['error'] is None)
        state = self.pool.snapshot()['a']
        self.assertIsNone(state['transport_warning'])
        self.assertIn('motion write failed', state['last_error'])
        self.assertTrue(state['active'])
        self.pool.stop(['a'])
        self.parked('a')

    def test_continuing_stream_keeps_publishing_while_subscriber_is_temporarily_missing(self):
        self.ready('a', 'b')
        token = self.pool.start_stream(['a', 'b'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        self.pool.update_stream(token, {'a': MOVE, 'b': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        self.fleets['a'].matched = False
        self.wait_until(lambda: self.pool.snapshot()['a']['transport_warning'] is not None)
        state = self.pool.snapshot()
        self.assertTrue(state['a']['active'])
        self.assertFalse(state['a']['stop_pending'])
        self.assertFalse(state['a']['stop_unconfirmed'])
        self.assertFalse(state['a']['ready'])
        self.assertTrue(state['b']['active'])
        self.pool.update_stream(token, {'a': Velocity(0.08, 0), 'b': Velocity(0.09, 0)})
        self.wait_until(lambda: any(value == Velocity(0.08, 0) for _, value in self.fleets['a'].delivered))
        self.assertIsNotNone(self.pool.snapshot()['a']['error'])
        self.fleets['a'].matched = True
        self.wait_until(lambda: self.pool.snapshot()['a']['ready']
                        and self.pool.snapshot()['a']['error'] is None)
        self.assertIsNone(self.pool.snapshot()['a']['transport_warning'])
        self.assertIsNotNone(self.pool.snapshot()['a']['last_error'])
        self.pool.stop(['a', 'b'])
        self.parked('a', 'b')

    def test_continuing_stream_probe_exception_recovers_without_revoking_session(self):
        self.ready('a')
        fleet = self.fleets['a']
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        fleet.discovery_error = RuntimeError('temporary graph failure')
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is not None)
        state = self.pool.snapshot()['a']
        self.assertIn('temporary graph failure', state['error'])
        self.assertTrue(state['active'])
        self.assertFalse(state['stop_unconfirmed'])
        self.pool.update_stream(token, {'a': Velocity(0.08, 0)})
        fleet.discovery_error = None
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is None)
        self.assertIn('temporary graph failure', self.pool.snapshot()['a']['last_error'])
        self.pool.stop(['a'])
        self.parked('a')

    def test_continuing_stream_recovers_probe_and_publish_errors_independently(self):
        self.ready('a')
        fleet = self.fleets['a']
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        fleet.matched = False
        fleet.motion_errors = 1000
        self.pool.update_stream(token, {'a': MOVE})
        self.wait_until(lambda: 'subscriber unavailable' in (self.pool.snapshot()['a']['error'] or '')
                        and 'motion write failed' in (self.pool.snapshot()['a']['error'] or ''))
        fleet.motion_errors = 0
        self.wait_until(lambda: any(value == MOVE for _, value in fleet.delivered)
                        and 'motion write failed' not in (self.pool.snapshot()['a']['error'] or ''))
        self.assertIn('subscriber unavailable', self.pool.snapshot()['a']['error'])
        fleet.matched = True
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is None)
        self.assertTrue(self.pool.snapshot()['a']['active'])
        self.pool.stop(['a'])
        self.parked('a')

    def test_continuing_stream_manual_stop_beats_late_failed_publish_and_clears_only_motion_error(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release)
        self.ready('a')
        self.fleets['a'].motion_errors = 1
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        self.pool.stop(['a'])
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': Velocity(0.2, 0)})
        release.set()
        self.parked('a')
        state = self.pool.snapshot()['a']
        self.assertIsNone(state['error'])
        self.assertIsNone(state['transport_warning'])
        self.assertIn('motion write failed', state['last_error'])
        self.assertTrue(state['continue_on_error'])
        values = [value for _, value in self.fleets['a'].attempts]
        self.assertEqual(sum(value != ZERO for value in values), 1)
        self.assertGreaterEqual(state['zero_count'], 2)
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})

    def test_continuing_stream_stop_during_blocked_probe_cannot_resume_on_recovery(self):
        self.ready('a')
        fleet = self.fleets['a']
        release = threading.Event()
        fleet.discovery_release = release
        fleet.discovery_entered.clear()
        with self.pool._lock:
            self.pool._channels['a'].next_probe = 0
            token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                          continue_on_error=True)['session_id']
            self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(fleet.discovery_entered.wait(0.5))
        self.pool.stop(['a'])
        fleet.discovery_error = RuntimeError('late graph failure')
        release.set()
        self.wait_until(lambda: self.pool.snapshot()['a']['stop_unconfirmed'])
        self.assertFalse(self.pool.snapshot()['a']['active'])
        self.assertTrue(self.pool.snapshot()['a']['stop_pending'])
        fleet.discovery_error = None
        self.parked('a')
        self.assertTrue(all(value == ZERO for _, value in fleet.attempts))
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})

    def test_continuing_stream_zero_failure_is_not_erased_by_later_recovery(self):
        self.ready('a')
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        self.fleets['a'].zero_errors = 1
        self.pool.stop(['a'])
        self.parked('a')
        state = self.pool.snapshot()['a']
        self.assertIn('zero write failed', state['error'])
        self.assertIn('zero write failed', state['last_error'])
        self.assertIsNone(state['transport_warning'])
        self.assertGreaterEqual(state['zero_count'], 2)
        self.pool.stop(['a'])  # A fresh explicit stop can recover the old failure.
        self.parked('a')
        self.assertIsNone(self.pool.snapshot()['a']['error'])

    def test_continuing_stream_still_obeys_individual_deadlines_and_optional_ttl(self):
        self.ready('a', 'b')
        session = self.pool.start_stream(['a', 'b'], duration=0.4, rate=100, ttl=None,
                                         durations={'a': 0.04, 'b': 0.4}, continue_on_error=True)
        self.pool.update_stream(session['session_id'], {'a': MOVE, 'b': MOVE})
        self.wait_until(lambda: not self.pool.snapshot()['a']['active'])
        state = self.pool.snapshot()
        self.assertTrue(state['b']['active'])
        self.assertEqual(state['a']['stop_reason'], 'stream duration complete')
        self.assertFalse(state['a']['stream_expired'])
        self.pool.stop(['b'])
        self.parked('a', 'b')
        self.ready('a')
        self.pool.start_stream(['a'], duration=0.4, rate=100, ttl=0.04, continue_on_error=True)
        self.wait_until(lambda: self.pool.snapshot()['a']['stream_expired'])
        self.parked('a')
        self.assertIn('TTL expired', self.pool.snapshot()['a']['error'])

    def test_continuing_stream_close_invalidates_token_and_never_revives_late_motion(self):
        release = threading.Event()
        self.configure = lambda fleet: setattr(fleet, 'motion_release', release)
        self.ready('a')
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(self.fleets['a'].motion_entered.wait(0.5))
        with self.assertRaises(RuntimeError):
            self.pool.close(timeout=0.01)
        self.assertEqual(self.pool._streams, {})
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})
        release.set()
        self.pool.close(timeout=0.5)
        self.assertTrue(self.fleets['a'].closed.is_set())
        self.assertEqual(sum(value != ZERO for _, value in self.fleets['a'].attempts), 1)

    def test_none_ttl_does_not_change_default_error_policy_and_fixed_start_resets_continuation(self):
        self.ready('a')
        self.pool.start_stream(['a'], duration=1, ttl=None, continue_on_error=True)
        self.pool.stop(['a'])
        self.parked('a')
        self.ready('a')
        self.pool.start({'a': MOVE}, duration=1, rate=100)
        self.assertFalse(self.pool.snapshot()['a']['continue_on_error'])
        self.assertIsNone(self.pool.snapshot()['a']['transport_warning'])
        self.fleets['a'].motion_errors = 1
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is not None)
        self.assertFalse(self.pool.snapshot()['a']['active'])
        self.parked('a')
        self.ready('a')
        self.fleets['a'].motion_errors = 1
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.wait_until(lambda: not self.pool.snapshot()['a']['active'])
        self.assertIn('motion write failed', self.pool.snapshot()['a']['error'])
        with self.assertRaises(RuntimeError):
            self.pool.update_stream(token, {'a': MOVE})

    def test_new_idle_stop_resets_old_zero_error_caches_without_clearing_in_progress_failure(self):
        self.pool.stop_duration = 0.12
        self.ready('a')
        fleet = self.fleets['a']
        token = self.pool.start_stream(['a'], duration=1, rate=100, ttl=None,
                                      continue_on_error=True)['session_id']
        self.pool.update_stream(token, {'a': MOVE})
        self.assertTrue(fleet.motion_entered.wait(0.5))
        self.pool.stop(['a'])
        self.wait_until(lambda: self.pool.snapshot()['a']['zero_count'] >= 2)
        fleet.zero_errors = 1000
        self.wait_until(lambda: self.pool.snapshot()['a']['error'] is not None)
        self.pool.stop(['a'])  # Repeated in-progress stop must retain this failure.
        self.assertIn('zero write failed', self.pool.snapshot()['a']['error'])
        self.parked('a')
        channel = self.pool._channels['a']
        self.assertIsNotNone(channel.publish_error)
        self.assertIsNotNone(channel.stop_error)
        fleet.zero_errors = 0
        probes = fleet.probes
        with self.pool._lock:
            self.pool.stop(['a'])
            self.assertIsNone(channel.publish_error)
            self.assertIsNone(channel.stop_error)
            self.assertIsNone(channel.state['error'])
            self.assertIsNone(channel.state['transport_warning'])
            channel.next_probe = 0
        self.parked('a')
        self.wait_until(lambda: fleet.probes > probes)
        self.assertIsNone(self.pool.snapshot()['a']['error'])
        self.assertIn('zero write failed', self.pool.snapshot()['a']['last_error'])


if __name__ == '__main__':
    unittest.main()
