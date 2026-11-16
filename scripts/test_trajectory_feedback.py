"""Offline checks for obstacle-optional trajectory feedback and ownership."""

import math
from types import SimpleNamespace as Obj
import time
import unittest

try:
    from .vrpn_straight_goal import Feedback, command_velocity_valid, endpoints, gate_main
    from .vrpn_csv_trajectory import MotionGuard, steering
    from .recorded_path import PathGeometry
except ImportError:
    from vrpn_straight_goal import Feedback, command_velocity_valid, endpoints, gate_main
    from vrpn_csv_trajectory import MotionGuard, steering
    from recorded_path import PathGeometry


def message_pair():
    q = lambda: Obj(x=0.0, y=0.0, z=0.0, w=1.0)
    p = lambda: Obj(x=0.0, y=0.0, z=0.12)
    h = lambda frame: Obj(frame_id=frame, stamp=Obj(sec=1000, nanosec=0))
    pose = Obj(header=h('world'), pose=Obj(position=p(), orientation=q()))
    odom = Obj(header=h('odom'), child_frame_id='base_link',
               pose=Obj(pose=Obj(position=p(), orientation=q())),
               twist=Obj(twist=Obj(linear=Obj(x=0.0), angular=Obj(z=0.0))))
    return pose, odom


class VelocityTests(unittest.TestCase):
    def test_requested_speed_is_allowed_only_with_its_explicit_limit(self):
        self.assertTrue(command_velocity_valid(.125, 0.0, .125))
        self.assertFalse(command_velocity_valid(.125, 0.0))
        self.assertFalse(command_velocity_valid(.126, 0.0, .125))

    def test_nonfinite_and_unbounded_values_rejected(self):
        for values in ((math.nan, 0, .125), (0, math.inf, .125),
                       (0, 0, math.nan), (0, 0, .126), (0, 0, 0), (.125, .31, .125)):
            with self.subTest(values=values):
                self.assertFalse(command_velocity_valid(*values))

    def test_invalid_gate_limit_fails_before_ros_startup(self):
        with self.assertRaises(ValueError):
            gate_main(None, 'invalid_gate', .126)

    def test_straight_path_reaches_requested_speed(self):
        path = PathGeometry([(0,0),(2,0)])
        linear,angular = steering(path,0,0,0,0,.125)
        self.assertEqual(linear,.125)
        self.assertEqual(angular,0)

    def test_corner_and_endpoint_speeds_remain_lower(self):
        path = PathGeometry([(0,0),(2,0)])
        corner_speed,_ = steering(path,0,0,0,-.2,.125)
        end_speed,_ = steering(path,1.95,1.95,0,0,.125)
        self.assertLess(corner_speed,.125)
        self.assertLess(end_speed,.125)

    def test_rotation_speed_is_limited_and_ramped(self):
        path = PathGeometry([(0,0),(2,0)])
        linear,angular = steering(path,0,0,0,-math.pi/2,.125)
        self.assertEqual(linear,0)
        self.assertEqual(angular,.18)
        guard = MotionGuard()
        state = (0,0,0,0,0,0,None,None)
        previous = 0
        for step in range(10):
            linear,actual = guard.command(state,step*.05,0,0,angular)
            self.assertLessEqual(actual-previous,.03000001)
            self.assertLessEqual(actual,.18)
            previous = actual
        self.assertAlmostEqual(previous,.18)


class DirectionResponseTests(unittest.TestCase):
    def test_measured_response_delay_does_not_look_like_reversed_wiring(self):
        guard = MotionGuard()
        guard.last_w = -.18
        for step in range(24):
            now = step*.05
            yaw = min(now,.35)*.1 - max(0,now-.35)*.18
            guard.observe((0,0,yaw,0,0,yaw),now)

    def test_persistent_wrong_direction_still_stops(self):
        guard = MotionGuard()
        guard.last_w = .18
        with self.assertRaisesRegex(RuntimeError,'实际转向'):
            for step in range(30):
                now=step*.05
                guard.observe((0,0,-now*.18,0,0,-now*.18),now)

    def test_unresponsive_rotation_still_stops(self):
        guard = MotionGuard()
        guard.last_w = .18
        with self.assertRaisesRegex(RuntimeError,'转向没有响应'):
            for step in range(40):
                guard.observe((0,0,0,0,0,0),step*.05)


class FeedbackTests(unittest.TestCase):
    def feedback(self, use_laser=False):
        feedback = Feedback(use_ultrasonic=False, use_laser=use_laser)
        pose, odom = message_pair()
        feedback.receive('pose', pose)
        feedback.receive('odom', odom)
        return feedback, pose, odom

    def test_without_obstacles_needs_only_fresh_pose_and_odom(self):
        feedback, _, _ = self.feedback()
        self.assertEqual(set(feedback.limits), {'pose', 'odom'})
        self.assertEqual(feedback.validate(1000.0)[-2:], (None, None))

    def test_default_mode_still_requires_scan(self):
        feedback, _, _ = self.feedback(use_laser=True)
        with self.assertRaisesRegex(RuntimeError, 'scan'):
            feedback.validate(1000.0)

    def test_stale_pose_is_still_rejected(self):
        feedback, pose, _ = self.feedback()
        feedback.data['pose'] = pose, time.monotonic() - 0.5, 1000_000_000_000
        with self.assertRaisesRegex(RuntimeError, 'pose 数据过期'):
            feedback.validate(1000.0)

    def test_tilt_is_still_rejected(self):
        feedback, pose, _ = self.feedback()
        pose.pose.orientation.x = math.sin(0.5)
        pose.pose.orientation.w = math.cos(0.5)
        with self.assertRaisesRegex(RuntimeError, '倾斜'):
            feedback.validate(1000.0)

    def test_excessive_measured_speed_is_still_rejected(self):
        feedback, _, odom = self.feedback()
        odom.twist.twist.linear.x = 0.2
        with self.assertRaisesRegex(RuntimeError, '速度异常'):
            feedback.validate(1000.0)


class EndpointsTests(unittest.TestCase):
    def node(self, commands=(), sink_name='base'):
        info = lambda kind, name='base': Obj(node_namespace='/', node_name=name,
                                           topic_type=kind, endpoint_gid=(1,2,3))
        sources = {'/odom': [info('nav_msgs/msg/Odometry')], '/cmd_vel': list(commands)}
        sinks = [info('geometry_msgs/msg/Twist', sink_name)]
        return Obj(get_publishers_info_by_topic=lambda topic: sources.get(topic, []),
                   get_subscriptions_info_by_topic=lambda topic: sinks if topic == '/cmd_vel' else [])

    def test_can_check_unique_base_without_laser_or_ultrasound(self):
        self.assertEqual(endpoints(self.node(), use_ultrasonic=False, use_laser=False),
                         ((1,2,3), (1,2,3)))

    def test_default_mode_still_requires_laser(self):
        with self.assertRaisesRegex(RuntimeError, '/scan'):
            endpoints(self.node(), use_ultrasonic=False)

    def test_other_command_publishers_still_rejected(self):
        with self.assertRaisesRegex(RuntimeError, '已有 /cmd_vel'):
            endpoints(self.node(commands=[object()]), use_ultrasonic=False, use_laser=False)

    def test_wrong_command_sink_still_rejected(self):
        with self.assertRaisesRegex(RuntimeError, '不属于同一底盘'):
            endpoints(self.node(sink_name='other'), use_ultrasonic=False, use_laser=False)


if __name__ == '__main__':
    unittest.main()
