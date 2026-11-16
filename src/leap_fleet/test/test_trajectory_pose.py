"""Pose prediction tests use only in-memory snapshots and velocities."""

from copy import deepcopy
import math
import unittest

from leap_fleet.control import Velocity
from leap_fleet.trajectory import Track, control_velocity
from leap_fleet.trajectory_pose import PoseEstimate


def pose(x=0., y=0., yaw=0., *, received=10., age=0., status='fresh', stamp=None):
    return {'position': {'x': x, 'y': y, 'z': .4}, 'yaw_rad': yaw,
            'received_at_unix': received, 'source_stamp': stamp, 'age_s': age,
            'status': status, 'frame_id': 'world', 'topic': '/car/pose'}


class PoseEstimateTests(unittest.TestCase):
    def test_straight_gap_integrates_full_time_without_refreshing_anchor(self):
        estimate = PoseEstimate(pose(), 0.)
        first = estimate.update({'status': 'missing'}, Velocity(.2, 0), .25)
        self.assertAlmostEqual(first['x'], .05)
        last = estimate.update({'status': 'stale'}, Velocity(.2, 0), 10.)
        self.assertAlmostEqual(last['x'], 2.)
        self.assertEqual(last['mode'], 'estimated')
        self.assertEqual(last['gap_s'], 10.)

    def test_held_arc_and_small_angular_velocity_are_numerically_stable(self):
        estimate = PoseEstimate(pose(), 0.)
        result = estimate.update({}, Velocity(.2, .4), math.pi / .8)
        self.assertAlmostEqual(result['x'], .5, places=10)
        self.assertAlmostEqual(result['y'], .5, places=10)
        self.assertAlmostEqual(result['yaw'], math.pi / 2, places=10)
        tiny = PoseEstimate(pose(), 0.).update({}, Velocity(.2, 1e-14), 2.)
        self.assertAlmostEqual(tiny['x'], .4, places=12)
        self.assertAlmostEqual(tiny['y'], 0., places=12)

    def test_changed_previous_command_applies_to_each_complete_interval(self):
        estimate = PoseEstimate(pose(), 0.)
        estimate.update({}, Velocity(.2, 0), 1.)
        turned = estimate.update({}, Velocity(0, math.pi / 2), 2.)
        self.assertAlmostEqual(turned['x'], .2)
        self.assertAlmostEqual(turned['yaw'], math.pi / 2)
        final = estimate.update({}, Velocity(.1, 0), 3.)
        self.assertAlmostEqual(final['x'], .2)
        self.assertAlmostEqual(final['y'], .1)

    def test_duplicate_receive_timestamp_ignores_changed_coordinates_and_age_zero(self):
        estimate = PoseEstimate(pose(), 0.)
        for i in range(1, 101):
            result = estimate.update(pose(x=-100 + i, received=10., age=0.), Velocity(.2, 0), i * .05)
        self.assertAlmostEqual(result['x'], 1.)
        self.assertEqual(result['gap_s'], 5.)
        self.assertEqual(result['mode'], 'estimated')

    def test_repeated_fresh_frames_stay_mocap_until_anchor_expires_without_reanchoring(self):
        cached = pose()
        estimate = PoseEstimate(cached, 0.)
        for now in (.05, .1, .15, .2, .25, .3):
            result = estimate.update(cached, Velocity(.2, 0), now)
            self.assertEqual(result['mode'], 'mocap')
            self.assertAlmostEqual(result['x'], .2 * now)
            self.assertAlmostEqual(result['gap_s'], now)
        expired = estimate.update(cached, Velocity(.2, 0), .3001)
        self.assertEqual(expired['mode'], 'estimated')
        recovered = estimate.update(pose(x=.08, received=11.), Velocity(.2, 0), .4)
        self.assertEqual(recovered['mode'], 'mocap')
        self.assertEqual(recovered['gap_s'], 0.)
        invalid = estimate.update({'status': 'invalid'}, Velocity(.2, 0), .41)
        self.assertEqual(invalid['mode'], 'estimated')

    def test_republished_nonzero_source_stamp_does_not_renew_anchor(self):
        stamp = {'sec': 3, 'nanosec': 123}
        estimate = PoseEstimate(pose(stamp=stamp), 0.)
        result = estimate.update(pose(x=9., received=11., stamp=stamp), Velocity(.2, 0), 1.)
        self.assertAlmostEqual(result['x'], .2)
        self.assertEqual(result['mode'], 'estimated')
        recovered = estimate.update(pose(x=.25, received=12., stamp={'sec': 3, 'nanosec': 124}),
                                    Velocity(.2, 0), 1.1)
        self.assertEqual(recovered['mode'], 'mocap')
        self.assertAlmostEqual(recovered['x'], .25)

    def test_invalid_and_old_samples_do_not_reset_valid_age_or_raise(self):
        estimate = PoseEstimate(pose(age=.1), 0.)
        bad = [None, {}, {'status': 'invalid', 'age_s': None},
               pose(x=math.nan, received=11.), pose(yaw=math.inf, received=12.),
               pose(received=13., age=-.1), pose(received=14., age=math.nan),
               pose(received=15., age=.3001), pose(received=16., age=None),
               pose(received=17., status='stale'), pose(received=18., status='error')]
        for i, sample in enumerate(bad, 1):
            with self.subTest(sample=sample):
                result = estimate.update(sample, Velocity(.1, 0), float(i))
                self.assertEqual(result['mode'], 'estimated')
                self.assertAlmostEqual(result['x'], .1 * i)
                self.assertAlmostEqual(result['gap_s'], i + .1)

    def test_recovery_reanchors_at_receive_time_then_extrapolates_age(self):
        estimate = PoseEstimate(pose(), 0.)
        estimate.update({}, Velocity(.2, 0), 5.)
        fresh = pose(x=2., y=3., yaw=math.pi / 2, received=20., age=.2)
        original = deepcopy(fresh)
        recovered = estimate.update(fresh, Velocity(.2, 0), 6.)
        self.assertEqual(recovered['mode'], 'mocap')
        self.assertAlmostEqual(recovered['x'], 2.)
        self.assertAlmostEqual(recovered['y'], 3.04)
        self.assertAlmostEqual(recovered['gap_s'], .2)
        following = estimate.update(fresh, Velocity(.2, 0), 6.1)
        self.assertEqual(following['mode'], 'mocap')
        self.assertAlmostEqual(following['y'], 3.06)
        self.assertAlmostEqual(following['gap_s'], .3)
        self.assertEqual(fresh, original)

    def test_age_zero_is_accepted_but_older_recovery_cannot_rewind_newer_anchor(self):
        estimate = PoseEstimate(pose(), 10.)
        recovered = estimate.update(pose(x=1., received=11., age=0.), Velocity(.1, 0), 10.1)
        self.assertEqual(recovered['mode'], 'mocap')
        old = pose(x=-5., received=12., age=.3)
        result = estimate.update(old, Velocity(.1, 0), 10.2)
        self.assertEqual(result['mode'], 'mocap')
        self.assertAlmostEqual(result['x'], 1.01)
        self.assertAlmostEqual(result['gap_s'], .1)

    def test_no_timestamp_fallback_does_not_renew_frozen_age_zero_cache(self):
        initial = pose(received=None)
        estimate = PoseEstimate(initial, 0.)
        for i in range(1, 101):
            result = estimate.update(initial, Velocity(.2, 0), i * .1)
        self.assertEqual(result['mode'], 'estimated')
        self.assertAlmostEqual(result['x'], 2.)
        self.assertAlmostEqual(result['gap_s'], 10.)
        changed = estimate.update(pose(x=2.1, received=None), Velocity(.2, 0), 10.1)
        self.assertEqual(changed['mode'], 'mocap')
        self.assertAlmostEqual(changed['x'], 2.1)

    def test_no_timestamp_fallback_recognizes_an_observed_age_reset(self):
        estimate = PoseEstimate(pose(received=None), 0.)
        estimate.update(pose(received=None, age=.1), Velocity(.2, 0), .1)
        result = estimate.update(pose(received=None, age=0.), Velocity(.2, 0), .2)
        self.assertEqual(result['mode'], 'mocap')
        self.assertEqual(result['gap_s'], 0.)
        self.assertEqual(result['x'], 0.)

    def test_lost_updates_do_not_repeatedly_reset_to_stale_position(self):
        cached = pose()
        estimate = PoseEstimate(cached, 0.)
        # Reusing one sample at every 20 Hz tick must not leave x frozen at zero.
        for i in range(1, 2001):
            result = estimate.update(cached, Velocity(.25, 0), i * .05)
        self.assertAlmostEqual(result['x'], 25.)
        self.assertAlmostEqual(result['gap_s'], 100.)

    def test_long_stale_cache_does_not_make_position_feedback_accelerate_forever(self):
        cached = pose()
        estimate = PoseEstimate(cached, 0.)
        track = Track(((0., 0., 0.), (100., 20., 0.)))
        command, peak = Velocity(0., 0.), 0.
        for i in range(1, 2001):
            now = i * .05
            predicted = estimate.update(cached, command, now)
            command = control_velocity(track, now, predicted['x'], predicted['y'],
                                       predicted['yaw'], command, .05)
            peak = max(peak, command.v)
            if i == 1900:
                self.assertAlmostEqual(command.v, .2, places=4)
        # This is a scenario stability check, not an imposed speed cap. Reusing
        # cached x=0 as current position would cause continued catch-up acceleration.
        self.assertLess(peak, .5)
        self.assertEqual(command, Velocity(0., 0.))
        self.assertEqual(predicted['mode'], 'estimated')
        self.assertAlmostEqual(predicted['gap_s'], 100.)

    def test_initial_seed_and_clock_velocity_validation(self):
        for sample in ({}, pose(status='missing'), pose(x=math.nan), pose(age=.31)):
            with self.subTest(sample=sample), self.assertRaises(ValueError):
                PoseEstimate(sample, 0.)
        estimate = PoseEstimate(pose(), 1.)
        for velocity, now in ((Velocity(0, 0), math.nan), (Velocity(0, 0), .9), (None, 2.)):
            with self.subTest(now=now), self.assertRaises(ValueError):
                estimate.update({}, velocity, now)


if __name__ == '__main__':
    unittest.main()
