"""Timed CSV and closed-loop unicycle simulations; no ROS or vehicle I/O."""

import csv
import io
import json
import math
import unittest

from leap_fleet.control import Velocity
from leap_fleet.trajectory import (
    Track, arc_csv, control_velocity, parse_csv, straight_csv,
)


KNOWN = ('car04', 'car08')


class CsvTests(unittest.TestCase):
    def parse(self, body):
        return parse_csv('car_id,t,x,y\n' + body, KNOWN)

    def test_bom_aliases_and_interleaved_cars_keep_individual_clocks(self):
        result = parse_csv('\ufeffcar_id,t,x,y\r\n4,0,1,2\r\n08,0,3,4\r\n'
                           'car04,2,2,2\r\ncar08,5,3,5\r\n', KNOWN)
        self.assertEqual(tuple(result.tracks), KNOWN)
        self.assertEqual(result.tracks['car04'].points, ((0., 1., 2.), (2., 2., 2.)))
        self.assertEqual(result.duration, 5.)
        self.assertEqual(result.sample_count, 4)
        self.assertTrue(result.tracks['car04'].sample(2).done)
        self.assertFalse(result.tracks['car08'].sample(2).done)

    def test_headers_and_column_counts_are_strict(self):
        for text in ('t,x,y,car_id\n', 'car_id,t,x,y,yaw\n',
                     'car_id,t,x,y\n4,0,0,0,0\n', 'car_id,t,x,y\n4,0,0\n',
                     'car_id,t,x,y\n4,0,"unterminated,0\n'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_csv(text, KNOWN)

    def test_unknown_car_and_duplicate_alias_timestamp_rejected(self):
        for body in ('5,0,0,0\n5,1,1,0\n', '4,0,0,0\n04,0,1,0\n',
                     'car04,0,0,0\ncar04,2,1,0\ncar04,1,2,0\n'):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.parse(body)

    def test_each_car_starts_at_zero_and_needs_two_points(self):
        for body in ('4,1,0,0\n4,2,1,0\n', '4,0,0,0\n',
                     '4,0,0,0\n4,1,1,0\n8,0,0,0\n', '', '\n'):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.parse(body)

    def test_nonfinite_numbers_limits_and_unrepresentable_speed(self):
        for body in ('4,0,NaN,0\n4,1,0,0\n', '4,0,0,0\n4,inf,0,0\n',
                     '4,-1,0,0\n4,1,0,0\n', '4,0,0,0\n4,3600.1,0,0\n',
                     '4,0,-100.1,0\n4,1,0,0\n', '4,0,0,0\n4,1,0,101\n',
                     '4,0,0,0\n4,5e-324,1,0\n'):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.parse(body)
        result = self.parse('4,0,-100,100\n4,3600,100,-100\n')
        self.assertEqual(result.duration, 3600)

    def test_high_speed_is_reported_for_execution_preflight_not_silently_retimed(self):
        track = self.parse('4,0,0,0\n4,0.1,10,0\n').tracks['car04']
        self.assertEqual(track.max_speed, 100)
        self.assertEqual(track.duration, .1)
        self.assertEqual(track.points[-1], (.1, 10., 0.))

    def test_global_point_limit_applies_before_any_simplification(self):
        text = 'car_id,t,x,y\n' + ''.join(f'4,{i/100},0,0\n' for i in range(100001))
        with self.assertRaisesRegex(ValueError, '100000'):
            parse_csv(text, KNOWN)

    def test_summary_preview_is_bounded_and_keeps_both_endpoints(self):
        text = straight_csv('car04', (0, 0), (1, 0), 30, step=.05)
        result = parse_csv(text, KNOWN)
        summary = result.summary()
        car = summary['cars'][0]
        self.assertEqual(car['sample_count'], 601)
        self.assertEqual(summary['sample_count'], 601)
        self.assertEqual(len(car['points']), 400)
        self.assertEqual(car['points'][0], {'t': 0., 'x': 0., 'y': 0.})
        self.assertEqual(car['points'][-1], {'t': 30., 'x': 1., 'y': 0.})
        json.dumps(summary, allow_nan=False)
        car['points'][0]['x'] = 99
        self.assertEqual(result.tracks['car04'].start, (0., 0.))


class ReferenceTests(unittest.TestCase):
    def test_linear_interpolation_uses_exact_time_not_arc_length(self):
        track = Track(((0, 0, 0), (1, 1, 0), (5, 1, 2)))
        reference = track.sample(3)
        self.assertEqual((reference.x, reference.y), (1., 1.))
        self.assertEqual((reference.vx, reference.vy, reference.speed), (0., .5, .5))
        self.assertAlmostEqual(reference.yaw, math.pi / 2)
        self.assertEqual((track.sample(1).vx, track.sample(1).vy), (0., .5))
        self.assertEqual(track.sample(-1).speed, 0)
        end = track.sample(99)
        self.assertEqual((end.t, end.x, end.y, end.speed, end.done), (5., 1., 2., 0., True))

    def test_dwell_keeps_previous_heading_and_initial_dwell_looks_forward(self):
        track = Track(((0, 0, 0), (1, 0, 0), (2, 1, 0),
                       (3, 1, 0), (4, 1, 1), (5, 1, 1)))
        for t in (.5, 2.5):
            self.assertEqual(track.sample(t).speed, 0)
            self.assertEqual(track.sample(t).yaw, 0)
        self.assertAlmostEqual(track.sample(4.5).yaw, math.pi / 2)
        self.assertAlmostEqual(track.sample(5).yaw, math.pi / 2)
        still = Track(((0, 2, 3), (2, 2, 3)))
        self.assertIsNone(still.initial_yaw)
        self.assertIsNone(still.sample(1).yaw)

    def test_forward_tangent_reverses_at_a_reversal_without_inventing_reverse_speed(self):
        track = Track(((0, 0, 0), (1, 1, 0), (2, 0, 0)))
        self.assertEqual(track.sample(.5).yaw, 0)
        self.assertAlmostEqual(abs(track.sample(1.5).yaw), math.pi)
        self.assertEqual(track.sample(1.5).speed, 1)


def simulate(track, *, disturbance=False):
    """Integrate actual unicycle motion under zero-order-held 20 Hz commands."""
    dt, x, y = .05, *track.start
    yaw = track.initial_yaw or 0.
    last = Velocity(0, 0)
    maximum_error, peak_v, peak_w = 0., 0., 0.
    for step in range(math.ceil(track.duration / dt)):
        elapsed = step * dt
        if disturbance and step == 100:
            y += .015
        reference = track.sample(elapsed)
        maximum_error = max(maximum_error, math.hypot(reference.x - x, reference.y - y))
        command = control_velocity(track, elapsed, x, y, yaw, last, dt)
        peak_v, peak_w = max(peak_v, command.v), max(peak_w, abs(command.w))
        if abs(command.w) < 1e-12:
            x += command.v * dt * math.cos(yaw)
            y += command.v * dt * math.sin(yaw)
        else:
            x += command.v / command.w * (math.sin(yaw + command.w * dt) - math.sin(yaw))
            y -= command.v / command.w * (math.cos(yaw + command.w * dt) - math.cos(yaw))
        yaw += command.w * dt
        last = command
    terminal = control_velocity(track, track.duration, x, y, yaw, last, dt)
    return maximum_error, math.dist((x, y), track.end), peak_v, peak_w, terminal


class ControllerSimulationTests(unittest.TestCase):
    def test_default_linear_speed_has_no_fixed_cap_and_keeps_acceleration_limit(self):
        track = Track(((0, 0, 0), (2, .6, 0)))
        command = control_velocity(track, 1, .3, 0, 0, Velocity(.2, 0), .05)
        self.assertGreater(command.v, .15)
        self.assertAlmostEqual(command.v, .205)
        self.assertLessEqual(command.v - .2, .1 * .05 + 1e-12)
        # Catch-up is also not secretly capped to the nominal reference speed.
        catch_up = control_velocity(track, 1, .2, 0, 0, Velocity(.31, 0), .05)
        self.assertGreater(catch_up.v, track.max_speed)
        self.assertLessEqual(catch_up.v - .31, .1 * .05 + 1e-12)

    def test_explicit_positive_linear_cap_remains_available(self):
        track = Track(((0, 0, 0), (2, .6, 0)))
        command = control_velocity(track, 1, .3, 0, 0, Velocity(.2, 0), .05, max_v=.15)
        self.assertEqual(command.v, .15)
        lower = control_velocity(track, 1, .3, 0, 0, Velocity(.1, 0), .05, max_v=.12)
        self.assertLessEqual(lower.v, .12)
        self.assertLessEqual(lower.v - .1, .1 * .05 + 1e-12)

    def test_faster_straight_simulation_can_exceed_old_cap_and_stops_on_time(self):
        track = parse_csv(straight_csv(4, (0, 0), (2, 0), 16), KNOWN).tracks['car04']
        maximum_error, final_error, peak_v, peak_w, terminal = simulate(track)
        self.assertGreater(track.max_speed, .2)
        self.assertGreater(peak_v, .2)
        self.assertLess(maximum_error, .02)
        self.assertLess(final_error, .01)
        self.assertLessEqual(peak_w, .6)
        self.assertEqual(terminal, Velocity(0, 0))

    def test_starting_ahead_waits_without_turning_then_resumes_on_same_clock(self):
        track = parse_csv(straight_csv(4, (0, 0), (.5, 0), 20), KNOWN).tracks['car04']
        for elapsed in (0, .5, 1, 2):
            command = control_velocity(track, elapsed, .03, 0, 0, Velocity(0, 0), .05)
            self.assertEqual(command, Velocity(0, 0))
        self.assertGreater(track.sample(6).x, .03)
        resumed = control_velocity(track, 6, .03, 0, 0, Velocity(0, 0), .05)
        self.assertGreater(resumed.v, 0)
        self.assertEqual(resumed.w, 0)
        self.assertEqual(track.duration, 20)
        self.assertEqual(control_velocity(track, 20, .03, 0, 0, resumed, .05), Velocity(0, 0))

    def test_ahead_waiting_is_in_reference_frame_and_lateral_correction_stays_bounded(self):
        yaw = math.pi / 3
        end = (.5 * math.cos(yaw), .5 * math.sin(yaw))
        track = parse_csv(straight_csv(4, (0, 0), end, 20), KNOWN).tracks['car04']
        x, y = .03 * math.cos(yaw), .03 * math.sin(yaw)
        command = control_velocity(track, 0, x, y, yaw, None, .05)
        self.assertEqual(command.v, 0)
        self.assertAlmostEqual(command.w, 0, places=9)
        lateral = control_velocity(track, 0, x - .01 * math.sin(yaw),
                                   y + .01 * math.cos(yaw), yaw, None, .05)
        self.assertEqual(lateral.v, 0)
        self.assertLess(lateral.w, 0)
        self.assertLessEqual(abs(lateral.w), 1.2 * .05)

    def test_lagging_vehicle_increases_forward_speed_and_corrects_lateral_error(self):
        track = parse_csv(straight_csv(4, (0, 0), (.5, 0), 20), KNOWN).tracks['car04']
        reference = track.sample(5)
        last = Velocity(.03, 0)
        baseline = control_velocity(track, 5, reference.x, reference.y, 0, last, .05)
        behind = control_velocity(track, 5, reference.x - .02, reference.y + .01,
                                  0, last, .05)
        self.assertGreater(behind.v, baseline.v)
        self.assertLess(behind.w, 0)
        self.assertLessEqual(behind.v, .15)
        self.assertLessEqual(abs(behind.w), .6)

    def test_generated_straight_runs_on_schedule_with_position_disturbance(self):
        track = parse_csv(straight_csv(4, (0, 0), (.5, 0), 20), KNOWN).tracks['car04']
        maximum_error, final_error, peak_v, peak_w, terminal = simulate(track, disturbance=True)
        self.assertLess(maximum_error, .025)
        self.assertLess(final_error, .01)
        self.assertLessEqual(peak_v, .15)
        self.assertLessEqual(peak_w, .6)
        self.assertEqual(terminal, Velocity(0, 0))

    def test_generated_arc_closes_loop_with_bounded_commands(self):
        track = parse_csv(arc_csv('04', (0, 0), .5, 0, math.pi, 30), KNOWN).tracks['car04']
        maximum_error, final_error, peak_v, peak_w, terminal = simulate(track)
        self.assertLess(maximum_error, .03)
        self.assertLess(final_error, .015)
        self.assertGreater(peak_w, .05)
        self.assertLessEqual(peak_v, .15)
        self.assertLessEqual(peak_w, .6)
        self.assertEqual(terminal, Velocity(0, 0))

    def test_late_controller_does_not_extend_time_or_keep_moving_after_end(self):
        track = Track(((0, 0, 0), (2, .1, 0)))
        self.assertEqual(control_velocity(track, 2, -1, 0, 0, Velocity(.1, .3), .05), Velocity(0, 0))
        self.assertEqual(control_velocity(track, 1, 0, 0, math.pi, Velocity(.1, 0), .05).v, 0)

    def test_acceleration_limits_and_scheduled_dwell(self):
        track = Track(((0, 0, 0), (1, .1, 0), (2, .1, 0), (3, .2, 0)))
        first = control_velocity(track, 0, 0, 0, .2, None, .05)
        self.assertLessEqual(first.v, .1 * .05)
        self.assertLessEqual(abs(first.w), 1.2 * .05)
        dwell = control_velocity(track, 1.5, 0, 0, 0, Velocity(.1, 0), .05)
        self.assertEqual(dwell, Velocity(0, 0))
        still = Track(((0, 1, 1), (5, 1, 1)))
        self.assertEqual(control_velocity(still, 1, 1, 1, 2, None, .05), Velocity(0, 0))

    def test_nonfinite_control_state_and_nonpositive_limits_rejected(self):
        track = Track(((0, 0, 0), (1, .1, 0)))
        for kwargs in ({'dt': 0}, {'max_v': math.inf}, {'max_v': math.nan},
                       {'max_v': 0}, {'max_v': -.1}, {'max_v': False},
                       {'max_w': 0}, {'x': math.nan}, {'y': math.inf}, {'yaw': math.nan}):
            args = dict(track=track, elapsed=0, x=0, y=0, yaw=0, last_velocity=None, dt=.05)
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                control_velocity(**args)


class GeneratorTests(unittest.TestCase):
    def test_generator_roundtrip_endpoints_and_ramps(self):
        for text, start, end in (
                (straight_csv('car04', (1, 2), (1, 2.5), 20), (1, 2), (1, 2.5)),
                (arc_csv(4, (0, 0), .5, 0, -math.pi / 2, 20), (.5, 0), (0, -.5))):
            with self.subTest(start=start):
                track = parse_csv(text, KNOWN).tracks['car04']
                self.assertEqual(track.start, start)
                self.assertAlmostEqual(track.end[0], end[0])
                self.assertAlmostEqual(track.end[1], end[1])
                self.assertEqual(track.duration, 20)
                self.assertLess(track.sample(0).speed, track.max_speed / 100)
                self.assertLess(track.sample(19.99).speed, track.max_speed / 100)
                self.assertEqual(list(csv.reader(io.StringIO(text)))[0], ['car_id', 't', 'x', 'y'])

    def test_generator_input_limits(self):
        for kwargs in ({'duration': 0}, {'duration': 3601}, {'step': 1e-10},
                       {'start': (math.nan, 0)}, {'end': (101, 0)}):
            args = dict(car_id='car04', start=(0, 0), end=(1, 0), duration=20)
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                straight_csv(**args)


if __name__ == '__main__':
    unittest.main()
