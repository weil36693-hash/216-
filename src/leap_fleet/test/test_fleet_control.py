"""Offline SDK tests: fake transports never create ROS nodes or move a car."""

from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from leap_fleet import Fleet, Velocity


class FakeTransport:
    def __init__(self, cars):
        self.cars = tuple(cars)
        self.open_calls = 0
        self.close_calls = 0
        self.published = []
        self.attempted = []
        self.counts = {car.name: 1 for car in self.cars}
        self.open_error = None
        self.fail_next_publish = None
        self.always_fail = set()

    def open(self):
        self.open_calls += 1
        if self.open_error is not None:
            raise self.open_error

    def publish(self, car_id, velocity):
        self.attempted.append((car_id, velocity))
        if self.fail_next_publish == car_id or car_id in self.always_fail:
            self.fail_next_publish = None
            raise RuntimeError("simulated publish failure")
        self.published.append((car_id, velocity))

    def subscriber_count(self, car_id):
        return self.counts[car_id]

    def close(self):
        self.close_calls += 1


class FakeClock:
    """Advance without waiting, including loops that do not call sleep."""

    def __init__(self):
        self.now = 100.0
        self.sleeps = []

    def monotonic(self):
        self.now += 0.001
        return self.now

    def sleep(self, duration):
        if duration < 0:
            raise AssertionError("negative sleep")
        self.sleeps.append(duration)
        self.now += duration


class VelocityTests(unittest.TestCase):
    def test_velocity_is_frozen_and_does_not_impose_a_speed_limit(self):
        velocity = Velocity(1000.0, -1000.0)
        self.assertEqual((velocity.v, velocity.w), (1000.0, -1000.0))
        with self.assertRaises(FrozenInstanceError):
            velocity.v = 0.0

    def test_nonfinite_velocity_is_rejected_at_construction(self):
        for bad in (float("nan"), float("inf"), -float("inf")):
            for values in ((bad, 0.0), (0.0, bad)):
                with self.subTest(values=values):
                    with self.assertRaises(ValueError):
                        Velocity(*values)


class FleetControlTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config = Path(self.directory.name) / "fleet.json"
        self.config.write_text(json.dumps({"cars": [
            {"name": "car01", "domain_id": 1},
            {"name": "car02", "domain_id": 2,
             "cmd_vel_topic": "/drive/cmd_vel"},
            {"name": "car03", "domain_id": 25},
        ]}), encoding="utf-8")
        self.transports = []

    def transport_factory(self, cars):
        transport = FakeTransport(cars)
        self.transports.append(transport)
        return transport

    def fleet(self, cars=None):
        fleet = Fleet(config=self.config, cars=cars,
                      transport_factory=self.transport_factory)
        self.addCleanup(lambda: fleet.close(send_stop=False))
        return fleet

    def test_selected_car_keeps_its_domain_and_command_topic(self):
        fleet = self.fleet(cars=["car02"])
        self.assertEqual(fleet.names, ("car02",))
        self.assertIsInstance(fleet.configs, tuple)
        fleet.open()
        self.assertEqual(len(self.transports), 1)
        car, = self.transports[0].cars
        self.assertEqual((car.name, car.domain_id, car.cmd_vel_topic),
                         ("car02", 2, "/drive/cmd_vel"))
        fleet["car02"].set_velocity(0.1, -0.2)
        self.assertEqual(self.transports[0].published,
                         [("car02", Velocity(0.1, -0.2))])

    def test_default_topics_and_distinct_domains_reach_transport(self):
        fleet = self.fleet()
        self.assertEqual(fleet.names, ("car01", "car02", "car03"))
        fleet.open()
        self.assertEqual(
            [(c.name, c.domain_id, c.cmd_vel_topic)
             for c in self.transports[0].cars],
            [("car01", 1, "/cmd_vel"), ("car02", 2, "/drive/cmd_vel"),
             ("car03", 25, "/cmd_vel")])

    def test_unopened_fleet_rejects_motion_and_waits(self):
        fleet = self.fleet()
        for action in (
                lambda: fleet.set_velocity("car01", 0.1, 0.0),
                lambda: fleet["car01"].set_velocity(0.1, 0.0),
                lambda: fleet.set_velocities({"car01": (0.1, 0.0)}),
                lambda: fleet.stop(),
                lambda: fleet.stop_repeated(),
                lambda: fleet.wait_for_subscribers()):
            with self.subTest(action=action):
                with self.assertRaises(RuntimeError):
                    action()

    def test_close_before_open_is_a_noop(self):
        fleet = self.fleet()
        fleet.close()
        fleet.close()
        self.assertEqual(self.transports, [])

    def test_instant_command_publishes_once_and_car_stop_targets_one_car(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        fleet.set_velocity("car01", -1.0, 2.0)
        self.assertEqual(transport.published, [("car01", Velocity(-1.0, 2.0))])
        fleet["car02"].stop()
        self.assertEqual(transport.published[-1], ("car02", Velocity(0.0, 0.0)))
        self.assertEqual(len(transport.published), 2)

    def test_batch_accepts_velocity_and_pair_values(self):
        fleet = self.fleet()
        fleet.open()
        fleet.set_velocities({"car01": (0.1, 0.2),
                              "car03": Velocity(-0.3, -0.4)})
        self.assertEqual(self.transports[0].published,
                         [("car01", Velocity(0.1, 0.2)),
                          ("car03", Velocity(-0.3, -0.4))])

    def test_batch_publish_failures_do_not_skip_later_vehicles(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        transport.always_fail = {'car01', 'car03'}
        commands = {name: (0.1, 0.0) for name in fleet.names}
        with self.assertRaises(RuntimeError) as caught:
            fleet.set_velocities(commands)
        self.assertIn('car01', str(caught.exception))
        self.assertIn('car03', str(caught.exception))
        self.assertEqual([name for name, _ in transport.attempted], list(fleet.names))
        self.assertEqual(transport.published, [('car02', Velocity(0.1, 0.0))])

    def test_unknown_car_in_batch_prevents_every_publication(self):
        fleet = self.fleet()
        fleet.open()
        with self.assertRaises(KeyError):
            fleet.set_velocities({"car01": (0.1, 0.0),
                                  "missing": (0.2, 0.0)})
        self.assertEqual(self.transports[0].attempted, [])

    def test_invalid_velocity_in_batch_prevents_every_publication(self):
        fleet = self.fleet()
        fleet.open()
        for invalid in ((float("nan"), 0.0), (0.0, float("inf")),
                        (1.0,), (1.0, 2.0, 3.0)):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    fleet.set_velocities({"car01": (0.1, 0.0),
                                          "car02": invalid})
                self.assertEqual(self.transports[0].attempted, [])

    def test_invalid_single_velocity_does_not_publish(self):
        fleet = self.fleet()
        fleet.open()
        with self.assertRaises(ValueError):
            fleet.set_velocity("car01", 0.0, float("nan"))
        with self.assertRaises(KeyError):
            fleet.set_velocity("missing", 0.0, 0.0)
        self.assertEqual(self.transports[0].attempted, [])

    def test_stop_selected_cars_and_all_cars(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        fleet.stop(cars=["car02"])
        self.assertEqual(transport.published, [("car02", Velocity(0.0, 0.0))])
        transport.published.clear()
        fleet.stop()
        self.assertEqual(transport.published,
                         [(name, Velocity(0.0, 0.0)) for name in fleet.names])

    def test_stop_prevalidates_selected_car_names(self):
        fleet = self.fleet()
        fleet.open()
        with self.assertRaises(KeyError):
            fleet.stop(cars=["car01", "missing"])
        self.assertEqual(self.transports[0].attempted, [])

    def test_repeated_stop_keeps_retrying_failed_rounds_and_other_vehicles(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        transport.always_fail = {'car01'}
        clock = FakeClock()
        with patch('leap_fleet.control.time.monotonic', clock.monotonic), \
                patch('leap_fleet.control.time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'car01'):
                fleet.stop_repeated(['car01', 'car02'], duration=0.2, rate=20)
        attempted = [name for name, _ in transport.attempted]
        self.assertGreaterEqual(attempted.count('car01'), 3)
        self.assertEqual(attempted.count('car01'), attempted.count('car02'))
        self.assertNotIn('car03', attempted)
        self.assertTrue(all(velocity == Velocity(0.0, 0.0)
                            for _, velocity in transport.attempted))
        self.assertGreaterEqual(clock.now, 100.2)

    def test_repeated_stop_reports_transient_failures_after_finishing_the_window(self):
        fleet = self.fleet(cars=['car01'])
        fleet.open()
        transport = self.transports[0]
        transport.fail_next_publish = 'car01'
        clock = FakeClock()
        with patch('leap_fleet.control.time.monotonic', clock.monotonic), \
                patch('leap_fleet.control.time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, '1 rounds'):
                fleet.stop_repeated(duration=0.2)
        self.assertGreater(len(transport.published), 1)
        self.assertGreaterEqual(clock.now, 100.2)

    def test_repeated_stop_validates_all_arguments_before_publishing(self):
        fleet = self.fleet()
        fleet.open()
        for cars, arguments in ((['car01', 'missing'], {}),
                                (None, {'duration': 0}), (None, {'duration': float('nan')}),
                                (None, {'rate': 0}), (None, {'rate': float('inf')})):
            with self.subTest(cars=cars, arguments=arguments), \
                    self.assertRaises((KeyError, ValueError)):
                fleet.stop_repeated(cars, **arguments)
        self.assertEqual(self.transports[0].attempted, [])

    def test_wait_for_subscribers_succeeds_only_when_every_car_has_one(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        self.assertTrue(fleet.wait_for_subscribers(timeout=0.0))
        transport.counts["car02"] = 0
        self.assertFalse(fleet.wait_for_subscribers(timeout=0.0))

    def test_wait_for_subscribers_times_out_without_real_sleep(self):
        fleet = self.fleet()
        fleet.open()
        self.transports[0].counts["car03"] = 0
        clock = FakeClock()
        with patch("leap_fleet.control.time.monotonic", clock.monotonic), \
                patch("leap_fleet.control.time.sleep", clock.sleep):
            self.assertFalse(fleet.wait_for_subscribers(timeout=0.2))

    def test_run_for_repeats_and_finally_stops_only_its_commands(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        clock = FakeClock()
        commands = {"car01": (0.1, 0.0), "car02": Velocity(0.0, -0.2)}
        with patch("leap_fleet.control.time.monotonic", clock.monotonic), \
                patch("leap_fleet.control.time.sleep", clock.sleep):
            fleet.run_for(commands, duration=0.3, rate=20.0)
        self.assertEqual(transport.published[-2:],
                         [("car01", Velocity(0.0, 0.0)),
                          ("car02", Velocity(0.0, 0.0))])
        self.assertGreaterEqual(transport.published.count(
            ("car01", Velocity(0.1, 0.0))), 2)
        self.assertGreaterEqual(transport.published.count(
            ("car02", Velocity(0.0, -0.2))), 2)
        self.assertNotIn("car03", [name for name, _ in transport.attempted])
        self.assertTrue(clock.sleeps)

    def test_run_for_publish_failure_still_stops_all_task_cars(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        transport.fail_next_publish = "car02"
        clock = FakeClock()
        with patch('leap_fleet.control.time.monotonic', clock.monotonic), \
                patch('leap_fleet.control.time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, "simulated publish failure"):
                fleet.run_for({"car01": (0.1, 0.0), "car02": (0.1, 0.0)},
                              duration=1.0)
        self.assertEqual(transport.published[-2:],
                         [("car01", Velocity(0.0, 0.0)),
                          ("car02", Velocity(0.0, 0.0))])
        self.assertNotIn("car03", [name for name, _ in transport.attempted])

    def test_run_for_keyboard_interrupt_still_stops_task_car(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        with patch("leap_fleet.control.time.sleep", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                fleet.run_for({"car02": (0.1, 0.0)}, duration=1.0)
        self.assertEqual(transport.published[-1], ("car02", Velocity(0.0, 0.0)))
        self.assertNotIn("car01", [name for name, _ in transport.attempted])
        self.assertNotIn("car03", [name for name, _ in transport.attempted])

    def test_run_for_invalid_arguments_publish_nothing(self):
        fleet = self.fleet()
        fleet.open()
        for arguments in ({"duration": -1.0}, {"duration": float("inf")},
                          {"duration": 1.0, "rate": 0.0},
                          {"duration": 1.0, "rate": float("nan")}):
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    fleet.run_for({"car01": (0.1, 0.0)}, **arguments)
                self.assertEqual(self.transports[0].attempted, [])

    def test_run_for_prevalidates_every_command_before_starting(self):
        fleet = self.fleet()
        fleet.open()
        with self.assertRaises(KeyError):
            fleet.run_for({"car01": (0.1, 0.0), "missing": (0.1, 0.0)},
                          duration=1.0)
        self.assertEqual(self.transports[0].attempted, [])

    def test_context_manager_stops_and_closes_on_an_exception(self):
        fleet = self.fleet()
        with self.assertRaisesRegex(RuntimeError, "body failed"):
            with fleet:
                fleet.set_velocity("car01", 0.1, 0.0)
                raise RuntimeError("body failed")
        transport = self.transports[0]
        self.assertEqual(transport.published[-3:],
                         [(name, Velocity(0.0, 0.0)) for name in fleet.names])
        self.assertEqual(transport.close_calls, 1)
        with self.assertRaises(RuntimeError):
            fleet.set_velocity("car01", 0.1, 0.0)

    def test_open_and_close_are_idempotent_and_reopen_uses_new_transport(self):
        fleet = self.fleet()
        fleet.open()
        fleet.open()
        self.assertEqual(len(self.transports), 1)
        first = self.transports[0]
        self.assertEqual(first.open_calls, 1)
        fleet.close()
        publication_count = len(first.published)
        fleet.close()
        self.assertEqual(first.close_calls, 1)
        self.assertEqual(len(first.published), publication_count)
        with self.assertRaises(RuntimeError):
            fleet["car01"].set_velocity(0.1, 0.0)
        fleet.open()
        self.assertEqual(len(self.transports), 2)
        fleet.set_velocity("car01", 0.2, 0.0)
        self.assertEqual(self.transports[1].published,
                         [("car01", Velocity(0.2, 0.0))])
        self.assertEqual(len(first.published), publication_count)

    def test_close_failure_still_attempts_other_cars_and_releases_transport(self):
        fleet = self.fleet()
        fleet.open()
        transport = self.transports[0]
        transport.fail_next_publish = "car01"
        clock = FakeClock()
        with patch('leap_fleet.control.time.monotonic', clock.monotonic), \
                patch('leap_fleet.control.time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'car01'):
                fleet.close()
        for name in fleet.names:
            self.assertGreaterEqual(transport.attempted.count((name, Velocity(0.0, 0.0))), 3)
        self.assertEqual(transport.close_calls, 1)
        with self.assertRaises(RuntimeError):
            fleet.set_velocity("car02", 0.1, 0.0)

    def test_close_without_send_stop_only_releases_transport(self):
        fleet = self.fleet(cars=['car01'])
        fleet.open()
        fleet.set_velocity('car01', 0.1, 0.0)
        transport = self.transports[0]
        previous = list(transport.published)
        fleet.close(send_stop=False)
        self.assertEqual(transport.published, previous)
        self.assertEqual(transport.close_calls, 1)

    def test_default_close_sends_one_full_stop_window(self):
        fleet = self.fleet(cars=['car01'])
        fleet.open()
        clock = FakeClock()
        with patch('leap_fleet.control.time.monotonic', clock.monotonic), \
                patch('leap_fleet.control.time.sleep', clock.sleep):
            fleet.close()
        self.assertGreaterEqual(clock.now, 101.0)
        self.assertGreaterEqual(len(self.transports[0].published), 15)
        self.assertEqual(self.transports[0].close_calls, 1)

    def test_run_for_and_close_do_not_duplicate_a_completed_stop_window(self):
        fleet = self.fleet(cars=['car01'])
        fleet.open()
        clock = FakeClock()
        with patch('leap_fleet.control.time.monotonic', clock.monotonic), \
                patch('leap_fleet.control.time.sleep', clock.sleep):
            fleet.run_for({'car01': (0.1, 0.0)}, duration=0.1)
            count, finished_at = len(self.transports[0].published), clock.now
            fleet.close()
        self.assertEqual(len(self.transports[0].published), count)
        self.assertEqual(clock.now, finished_at)
        self.assertGreaterEqual(finished_at, 101.1)

    def test_new_command_invalidates_a_completed_stop_window(self):
        fleet = self.fleet(cars=['car01'])
        fleet.open()
        clock = FakeClock()
        with patch('leap_fleet.control.time.monotonic', clock.monotonic), \
                patch('leap_fleet.control.time.sleep', clock.sleep):
            fleet.stop_repeated()
            fleet.set_velocity('car01', 0.02, 0.0)
            before = len(self.transports[0].published)
            fleet.close()
        self.assertGreaterEqual(len(self.transports[0].published) - before, 15)

    def test_failed_open_releases_partial_transport_and_can_be_retried(self):
        def failing_factory(cars):
            transport = self.transport_factory(cars)
            if len(self.transports) == 1:
                transport.open_error = RuntimeError("simulated open failure")
            return transport

        fleet = Fleet(config=self.config, transport_factory=failing_factory)
        self.addCleanup(lambda: fleet.close(send_stop=False))
        with self.assertRaisesRegex(RuntimeError, "simulated open failure"):
            fleet.open()
        self.assertEqual(self.transports[0].close_calls, 1)
        with self.assertRaises(RuntimeError):
            fleet.set_velocity("car01", 0.1, 0.0)
        fleet.open()
        self.assertEqual(len(self.transports), 2)
        fleet.set_velocity("car01", 0.1, 0.0)
        self.assertEqual(self.transports[1].published,
                         [("car01", Velocity(0.1, 0.0))])


if __name__ == "__main__":
    unittest.main()
