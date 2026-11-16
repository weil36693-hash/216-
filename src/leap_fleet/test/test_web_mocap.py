"""Mocap telemetry never changes the chassis control readiness contract."""

from copy import deepcopy
import threading
import time
import unittest

from leap_fleet.web_console import Console
from leap_fleet.web_monitor import ConsoleMonitor


class FakeChassis:
    def __init__(self, events):
        self.events = events
        self.cars = [
            {'id': 'car04', 'name': 'car04', 'ip': '10.1.1.214', 'domain_id': 4,
             'status': 'connected', 'age_s': 0.01, 'measured_v': 0.0,
             'measured_w': 0.0, 'cmd_subscribers': 1, 'cmd_publishers': 0, 'error': None},
            {'id': 'car09', 'name': 'car09', 'ip': '10.1.1.229', 'domain_id': 9,
             'status': 'offline', 'age_s': None, 'measured_v': None,
             'measured_w': None, 'cmd_subscribers': 0, 'cmd_publishers': 0, 'error': None}]

    def open(self):
        self.events.append('chassis.open')
        return self

    def snapshot(self):
        for car in self.cars:
            if car['status'] == 'connected':
                car.update(odom_received_at=time.monotonic(),
                           odom_sequence=car.get('odom_sequence', 0) + 1)
        return deepcopy(self.cars)

    def close(self):
        self.events.append('chassis.close')


class FakeMocap:
    def __init__(self, events):
        self.events = events
        self.open_error = self.close_error = None
        self.poses = {
            'car04': {'status': 'missing', 'position': None, 'orientation': None,
                      'yaw_rad': None, 'frame_id': None, 'age_s': None,
                      'topic': '/test/car04/pose', 'domain_id': 0,
                      'publishers': 0, 'error': None, 'binding_confirmed': True},
            'car09': {'status': 'fresh', 'position': {'x': 1.0, 'y': 2.0, 'z': 0.1},
                      'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                      'yaw_rad': 0.0, 'frame_id': 'world', 'age_s': 0.01,
                      'topic': '/test/car09/pose', 'domain_id': 0,
                      'publishers': 1, 'error': None, 'binding_confirmed': False}}

    def open(self):
        self.events.append('mocap.open')
        if self.open_error is not None:
            raise self.open_error
        return self

    def snapshot(self):
        return deepcopy(self.poses)

    def close(self):
        self.events.append('mocap.close')
        if self.close_error is not None:
            raise self.close_error


class FakeFleet:
    def __init__(self, cars):
        self.names = tuple(cars)
        self.commands = []
        self.stops = 0
        self.closed = threading.Event()

    def open(self):
        return self

    def wait_for_subscribers(self, timeout=1.0):
        return True

    def set_velocities(self, commands):
        if all(v.v == 0 and v.w == 0 for v in commands.values()):
            self.stop()
        else:
            self.commands.append(dict(commands))

    def stop(self):
        self.stops += 1

    def close(self, *, send_stop=True):
        self.closed.set()


class WebMocapTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.chassis = FakeChassis(self.events)
        self.mocap = FakeMocap(self.events)
        self.monitor = ConsoleMonitor(chassis=self.chassis, mocap=self.mocap)

    def test_status_and_domain_are_kept_separate_for_both_sources(self):
        self.monitor.open()
        for status in ('fresh', 'stale', 'missing', 'invalid', 'error'):
            with self.subTest(mocap_status=status):
                self.mocap.poses['car04']['status'] = status
                cars = self.monitor.snapshot()
                self.assertEqual(cars[0]['status'], 'connected')
                self.assertEqual(cars[0]['domain_id'], 4)
                self.assertEqual(cars[0]['mocap']['status'], status)
                self.assertEqual(cars[0]['mocap']['domain_id'], 0)
                self.assertTrue(cars[0]['mocap']['binding_confirmed'])
                self.assertEqual(cars[1]['status'], 'offline')
                self.assertEqual(cars[1]['mocap']['status'], 'fresh')
                self.assertFalse(cars[1]['mocap']['binding_confirmed'])

    def test_mocap_startup_failure_is_displayed_without_breaking_chassis(self):
        self.mocap.open_error = RuntimeError('mocap domain unavailable')
        self.assertIs(self.monitor.open(), self.monitor)
        self.assertEqual(self.events, ['chassis.open', 'mocap.open'])
        cars = self.monitor.snapshot()
        self.assertEqual(cars[0]['status'], 'connected')
        self.assertEqual(cars[1]['status'], 'offline')
        for car in cars:
            self.assertEqual(car['mocap']['status'], 'error')
            self.assertIn('mocap domain unavailable', car['mocap']['error'])
        self.assertEqual(self.mocap.poses['car04']['status'], 'missing')
        self.assertIsNone(self.mocap.poses['car04']['error'])

    def test_failed_mocap_does_not_block_a_connected_chassis_command(self):
        self.mocap.open_error = RuntimeError('mocap offline')
        self.monitor.open()
        fleets = []

        def factory(cars):
            fleet = FakeFleet(cars)
            fleets.append(fleet)
            return fleet

        console = Console(self.monitor, factory)
        try:
            console.start({'car04': {'v': 0.01, 'w': -0.02}}, duration=0.1)
            worker = console._worker
            worker.join(timeout=2.0)
            self.assertFalse(worker.is_alive())
            self.assertEqual(console.snapshot()['job']['status'], 'completed')
            self.assertEqual(len(fleets), 1)
            self.assertEqual(fleets[0].names, ('car04',))
            self.assertTrue(fleets[0].commands)
            self.assertGreater(fleets[0].stops, 0)
            self.assertFalse(fleets[0].closed.is_set())
        finally:
            console.close()

    def test_fresh_mocap_cannot_make_an_offline_chassis_controllable(self):
        self.monitor.open()
        fleets = []
        console = Console(self.monitor, lambda cars: fleets.append(cars))
        try:
            with self.assertRaises(ValueError):
                console.start({'car09': {'v': 0.01, 'w': 0.0}}, duration=0.1)
            self.assertEqual(fleets, [])
        finally:
            console.close()

    def test_snapshot_mutations_cannot_change_component_caches_or_later_results(self):
        first = self.monitor.snapshot()
        first[0]['status'] = 'offline'
        first[1]['mocap']['position']['x'] = -999.0
        first[1]['mocap']['orientation']['w'] = -999.0
        second = self.monitor.snapshot()
        self.assertEqual(second[0]['status'], 'connected')
        self.assertEqual(second[1]['mocap']['position']['x'], 1.0)
        self.assertEqual(second[1]['mocap']['orientation']['w'], 1.0)
        self.assertNotIn('mocap', self.chassis.cars[0])
        self.assertEqual(self.mocap.poses['car09']['position']['x'], 1.0)

    def test_close_releases_chassis_even_when_mocap_cleanup_raises(self):
        self.mocap.close_error = RuntimeError('mocap cleanup failure')
        with self.assertRaisesRegex(RuntimeError, 'mocap cleanup failure'):
            self.monitor.close()
        self.assertEqual(self.events, ['mocap.close', 'chassis.close'])

    def test_close_releases_chassis_even_when_mocap_cleanup_is_interrupted(self):
        self.mocap.close_error = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            self.monitor.close()
        self.assertEqual(self.events, ['mocap.close', 'chassis.close'])


if __name__ == '__main__':
    unittest.main()
