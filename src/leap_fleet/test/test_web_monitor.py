"""Read-only monitor behavior using fake ROS modules; no vehicle connections."""

from contextlib import contextmanager, nullcontext
import json
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from leap_fleet.web_monitor import RosMonitor


def odometry(v=0.0, w=0.0):
    return SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=v), angular=SimpleNamespace(z=w))))


@contextmanager
def fake_ros(*, fail_domain=None, failure=None, real_thread=False):
    contexts, nodes, executors, threads, events = [], [], [], [], []

    class Context:
        def __init__(self):
            self.domain = None
            contexts.append(self)

        def init(self, *, args, domain_id):
            self.domain = domain_id
            events.append(('init', domain_id, args))
            if domain_id == fail_domain:
                raise failure

        def try_shutdown(self):
            events.append(('context_close', self.domain))

    class Node:
        def __init__(self, name, *, context, **kwargs):
            self.domain = context.domain
            self.options = kwargs
            self.subscription = None
            self.subscribers = 0
            self.publishers = 0
            self.queue = []
            self.graph_error = None
            self.destroy_error = None
            nodes.append(self)

        def create_subscription(self, message_type, topic, callback, qos):
            self.subscription = SimpleNamespace(topic=topic, callback=callback, qos=qos)
            return self.subscription

        def create_publisher(self, *args, **kwargs):
            raise AssertionError('The monitor must not create command publishers')

        def count_subscribers(self, topic):
            if self.graph_error is not None:
                raise self.graph_error
            events.append(('count_subscribers', self.domain, topic))
            return self.subscribers

        def count_publishers(self, topic):
            events.append(('count_publishers', self.domain, topic))
            return self.publishers

        def destroy_node(self):
            events.append(('node_close', self.domain))
            if self.destroy_error is not None:
                raise self.destroy_error

    class Executor:
        def __init__(self, *, context):
            self.context = context
            self.node = None
            executors.append(self)

        def add_node(self, node):
            self.node = node

        def spin_once(self, *, timeout_sec):
            if timeout_sec != 0.0:
                raise AssertionError('Every domain must be polled without blocking')
            if self.node.queue:
                self.node.subscription.callback(self.node.queue.pop(0))

        def shutdown(self, *, timeout_sec):
            events.append(('executor_close', self.context.domain))
            return True

    class Thread:
        def __init__(self, *, target, name, daemon):
            self.target = target
            self.alive = False
            threads.append(self)

        def start(self):
            self.alive = True

        def join(self, *, timeout):
            events.append(('thread_join',))
            self.alive = False

        def is_alive(self):
            return self.alive

    def module(**attributes):
        result = ModuleType('fake_ros')
        result.__dict__.update(attributes)
        return result

    modules = {
        'nav_msgs': module(), 'nav_msgs.msg': module(Odometry=object),
        'rclpy': module(), 'rclpy.context': module(Context=Context),
        'rclpy.node': module(Node=Node),
        'rclpy.executors': module(SingleThreadedExecutor=Executor),
        'rclpy.qos': module(
            QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
            ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort'),
            DurabilityPolicy=SimpleNamespace(VOLATILE='volatile')),
    }
    thread_patch = (nullcontext() if real_thread else
                    patch('leap_fleet.web_monitor.threading.Thread', Thread))
    with patch.dict(sys.modules, modules), thread_patch:
        yield SimpleNamespace(contexts=contexts, nodes=nodes, executors=executors,
                              threads=threads, events=events)


class WebMonitorTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config = Path(self.directory.name) / 'fleet.json'
        self.config.write_text(json.dumps({'cars': [
            {'name': 'car01', 'domain_id': 1, 'ip': '10.1.1.202',
             'pose_topic': '/vrpn_mocap/car_01/pose'},
            {'name': 'car02', 'domain_id': 25, 'ip': '10.1.1.253',
             'cmd_vel_topic': '/test/command', 'odom_topic': '/test/odometry'},
        ]}), encoding='utf-8')

    def test_constructor_and_snapshot_work_without_ros(self):
        with patch.dict(sys.modules, {'rclpy': None, 'nav_msgs': None}):
            monitor = RosMonitor(self.config)
            states = monitor.snapshot()
            self.assertEqual([item['id'] for item in states], ['car01', 'car02'])
            self.assertEqual(states[0]['name'], states[0]['id'])
            self.assertEqual(states[0]['ip'], '10.1.1.202')
            self.assertEqual(monitor.metadata['car01']['pose_topic'],
                             '/vrpn_mocap/car_01/pose')
            self.assertTrue(all(item['status'] == 'offline' for item in states))
            self.assertTrue(all(item['age_s'] is None for item in states))
            self.assertTrue(all(item['measured_v'] is None for item in states))
            monitor.close()

    def test_per_domain_read_only_subscriptions_use_one_thread(self):
        with fake_ros() as ros:
            monitor = RosMonitor(self.config).open()
            monitor.open()
            self.assertEqual([context.domain for context in ros.contexts], [1, 25])
            self.assertEqual([node.subscription.topic for node in ros.nodes],
                             ['/odom', '/test/odometry'])
            self.assertEqual(len(ros.executors), 2)
            self.assertEqual(len(ros.threads), 1)
            self.assertTrue(all(not node.options['use_global_arguments']
                                for node in ros.nodes))
            self.assertTrue(all(node.subscription.qos.depth == 1 for node in ros.nodes))
            monitor._poll_once(update_graph=True)
            self.assertIn(('count_subscribers', 25, '/test/command'), ros.events)
            self.assertIn(('count_publishers', 1, '/cmd_vel'), ros.events)
            monitor.close()
            previous = list(ros.events)
            monitor.close()
            self.assertEqual(ros.events, previous)
            join = ros.events.index(('thread_join',))
            self.assertLess(join, ros.events.index(('executor_close', 25)))
            self.assertLess(ros.events.index(('executor_close', 25)),
                            ros.events.index(('node_close', 25)))
            self.assertLess(ros.events.index(('node_close', 25)),
                            ros.events.index(('context_close', 25)))

    def test_discovery_freshness_and_measured_speeds_are_independent_per_car(self):
        with fake_ros() as ros:
            monitor = RosMonitor(self.config).open()
            try:
                ros.nodes[0].subscribers = 1
                ros.nodes[0].publishers = 2
                monitor._poll_once(update_graph=True)
                self.assertEqual(monitor.snapshot()[0]['status'], 'discovered')
                ros.nodes[0].queue.append(odometry(0.12, -0.3))
                with patch('leap_fleet.web_monitor.time.monotonic', return_value=100.0):
                    monitor._poll_once(update_graph=False)
                    states = monitor.snapshot()
                self.assertEqual(states[0]['status'], 'connected')
                self.assertEqual(states[0]['age_s'], 0.0)
                self.assertEqual(states[0]['measured_v'], 0.12)
                self.assertEqual(states[0]['measured_w'], -0.3)
                self.assertEqual(states[0]['cmd_publishers'], 2)
                self.assertEqual(states[1]['status'], 'offline')
                with patch('leap_fleet.web_monitor.time.monotonic', return_value=102.0):
                    self.assertEqual(monitor.snapshot()[0]['status'], 'connected')
                with patch('leap_fleet.web_monitor.time.monotonic', return_value=102.001):
                    self.assertEqual(monitor.snapshot()[0]['status'], 'stale')
                states[0]['error'] = 'external mutation'
                self.assertIsNone(monitor.snapshot()[0]['error'])
            finally:
                monitor.close()

    def test_background_thread_consumes_both_domains_and_stops_before_teardown(self):
        with fake_ros(real_thread=True) as ros:
            monitor = RosMonitor(self.config).open()
            thread = monitor._thread
            try:
                for index, node in enumerate(ros.nodes):
                    node.subscribers = 1
                    node.queue.append(odometry(index + 0.1, index - 0.2))
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    states = monitor.snapshot()
                    if all(state['status'] == 'connected' for state in states):
                        break
                    time.sleep(0.01)
                self.assertEqual([state['status'] for state in states],
                                 ['connected', 'connected'])
                self.assertEqual([state['measured_v'] for state in states], [0.1, 1.1])
            finally:
                monitor.close()
            self.assertFalse(thread.is_alive())
            self.assertEqual([event[1] for event in ros.events
                              if event[0] == 'context_close'], [25, 1])

    def test_invalid_odometry_does_not_renew_receive_age(self):
        with fake_ros() as ros:
            monitor = RosMonitor(self.config).open()
            try:
                ros.nodes[0].subscribers = 1
                ros.nodes[0].queue.append(odometry(0.1, 0.2))
                with patch('leap_fleet.web_monitor.time.monotonic', return_value=10.0):
                    monitor._poll_once(update_graph=True)
                for message in (odometry(float('nan'), 0.0),
                                odometry(0.0, float('inf')), SimpleNamespace()):
                    ros.nodes[0].queue.append(message)
                    with patch('leap_fleet.web_monitor.time.monotonic', return_value=13.0):
                        monitor._poll_once(update_graph=False)
                        state = monitor.snapshot()[0]
                    self.assertEqual(state['status'], 'stale')
                    self.assertEqual(state['age_s'], 3.0)
                    self.assertEqual(state['measured_v'], 0.1)
            finally:
                monitor.close()

    def test_odometry_without_command_sink_is_discovered_not_connected(self):
        with fake_ros() as ros:
            monitor = RosMonitor(self.config).open()
            try:
                ros.nodes[0].queue.append(odometry())
                monitor._poll_once(update_graph=True)
                self.assertEqual(monitor.snapshot()[0]['status'], 'discovered')
                ros.nodes[1].publishers = 1
                monitor._poll_once(update_graph=True)
                self.assertEqual(monitor.snapshot()[1]['status'], 'offline')
            finally:
                monitor.close()

    def test_graph_failure_is_visible_and_recovers_on_successful_refresh(self):
        with fake_ros() as ros:
            monitor = RosMonitor(self.config).open()
            try:
                ros.nodes[0].graph_error = RuntimeError('discovery unavailable')
                ros.nodes[1].subscribers = 1
                monitor._poll_once(update_graph=True)
                states = monitor.snapshot()
                self.assertEqual(states[0]['status'], 'error')
                self.assertIn('discovery unavailable', states[0]['error'])
                self.assertEqual(states[1]['status'], 'discovered')
                ros.nodes[0].graph_error = None
                monitor._poll_once(update_graph=True)
                self.assertEqual(monitor.snapshot()[0]['status'], 'offline')
                self.assertIsNone(monitor.snapshot()[0]['error'])
            finally:
                monitor.close()

    def test_single_domain_init_failure_does_not_hide_other_vehicles(self):
        with fake_ros(fail_domain=25, failure=RuntimeError('domain unavailable')) as ros:
            monitor = RosMonitor(self.config).open()
            ros.nodes[0].subscribers = 1
            monitor._poll_once(update_graph=True)
            states = monitor.snapshot()
            self.assertEqual(states[0]['status'], 'discovered')
            self.assertEqual(states[1]['status'], 'error')
            self.assertIn('domain unavailable', states[1]['error'])
            self.assertIn(('context_close', 25), ros.events)
            monitor.close()
            self.assertIn(('context_close', 1), ros.events)

    def test_interrupted_open_releases_every_created_context(self):
        with fake_ros(fail_domain=25, failure=KeyboardInterrupt('interrupted')) as ros:
            monitor = RosMonitor(self.config)
            with self.assertRaises(KeyboardInterrupt):
                monitor.open()
            self.assertEqual([item for item in ros.events if item[0] == 'context_close'],
                             [('context_close', 25), ('context_close', 1)])
            monitor.close()

    def test_cleanup_failure_still_closes_remaining_contexts(self):
        with fake_ros() as ros:
            monitor = RosMonitor(self.config).open()
            ros.nodes[1].destroy_error = RuntimeError('node destroy failed')
            with self.assertRaisesRegex(RuntimeError, 'node destroy failed'):
                monitor.close()
            self.assertIn(('context_close', 25), ros.events)
            self.assertIn(('context_close', 1), ros.events)
            monitor.close()

    def test_invalid_odometry_topic_is_rejected_before_ros_startup(self):
        payload = json.loads(self.config.read_text(encoding='utf-8'))
        payload['cars'][1]['odom_topic'] = 'relative/topic'
        self.config.write_text(json.dumps(payload), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'odom_topic'):
            RosMonitor(self.config)


if __name__ == '__main__':
    unittest.main()
