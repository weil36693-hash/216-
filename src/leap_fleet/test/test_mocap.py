"""Mocap SDK validation and lifecycle tests; fake ROS never opens sockets."""

from contextlib import contextmanager, nullcontext
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from leap_fleet import Mocap


def pose(x=1.0, y=2.0, z=0.15, yaw=0.0, scale=1.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='world',
                               stamp=SimpleNamespace(sec=123, nanosec=456)),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0,
                                        z=scale * math.sin(yaw / 2),
                                        w=scale * math.cos(yaw / 2))))


@contextmanager
def fake_ros(*, fail_at=None, fail_topic=None, real_thread=False, cleanup_failure=False,
             failure=None):
    contexts, nodes, executors, threads, events = [], [], [], [], []

    def fail(stage):
        if stage == fail_at:
            if failure is not None:
                raise failure
            raise RuntimeError('simulated ' + stage + ' failure')

    class Context:
        def __init__(self):
            self.domain = None
            contexts.append(self)

        def init(self, *, args, domain_id):
            self.domain = domain_id
            events.append(('init', domain_id, args))
            fail('init')

        def try_shutdown(self):
            events.append(('context_close', self.domain))

    class Node:
        def __init__(self, name, *, context, **options):
            fail('node')
            self.context, self.options = context, options
            self.subscriptions, self.publishers, self.graph_errors = {}, {}, {}
            self.queue = []
            self.destroy_error = None
            nodes.append(self)

        def create_subscription(self, message_type, topic, callback, qos):
            if topic == fail_topic:
                raise RuntimeError('simulated subscription failure')
            subscription = SimpleNamespace(topic=topic, callback=callback, qos=qos)
            self.subscriptions[topic] = subscription
            return subscription

        def create_publisher(self, *args, **kwargs):
            raise AssertionError('Mocap must never create any publisher')

        def count_publishers(self, topic):
            events.append(('count_publishers', topic))
            if topic in self.graph_errors:
                raise self.graph_errors[topic]
            return self.publishers.get(topic, 0)

        def destroy_node(self):
            events.append(('node_close',))
            if self.destroy_error:
                raise self.destroy_error

    class Executor:
        def __init__(self, *, context):
            fail('executor')
            self.node = None
            self.spin_error = None
            self.shutdown_error = (RuntimeError('executor close failure')
                                   if cleanup_failure else None)
            executors.append(self)

        def add_node(self, node):
            self.node = node
            fail('add_node')

        def spin_once(self, *, timeout_sec):
            if timeout_sec != 0.02:
                raise AssertionError('Mocap should wait briefly only when no data is ready')
            if self.spin_error:
                raise self.spin_error
            if self.node.queue:
                topic, message = self.node.queue.pop(0)
                self.node.subscriptions[topic].callback(message)
            elif real_thread:
                time.sleep(0.001)

        def shutdown(self, *, timeout_sec):
            events.append(('executor_close',))
            if self.shutdown_error:
                raise self.shutdown_error
            return True

    class Thread:
        def __init__(self, *, target, name, daemon):
            self.alive = False
            self.stuck = False
            threads.append(self)

        def start(self):
            fail('thread')
            self.alive = True

        def is_alive(self):
            return self.alive

        def join(self, *, timeout):
            events.append(('thread_join',))
            if not self.stuck:
                self.alive = False

    def module(**attributes):
        result = ModuleType('fake_ros')
        result.__dict__.update(attributes)
        return result

    modules = {
        'geometry_msgs': module(), 'geometry_msgs.msg': module(PoseStamped=object),
        'rclpy': module(), 'rclpy.context': module(Context=Context),
        'rclpy.node': module(Node=Node),
        'rclpy.executors': module(SingleThreadedExecutor=Executor),
        'rclpy.qos': module(
            QoSProfile=lambda **options: SimpleNamespace(**options),
            ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort'),
            DurabilityPolicy=SimpleNamespace(VOLATILE='volatile')),
    }
    thread_patch = nullcontext() if real_thread else patch('leap_fleet.mocap.threading.Thread', Thread)
    with patch.dict(sys.modules, modules), thread_patch:
        yield SimpleNamespace(contexts=contexts, nodes=nodes, executors=executors,
                              threads=threads, events=events)


class MocapTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config = Path(self.directory.name) / 'fleet.json'
        self.payload = {'mocap_domain_id': 7, 'cars': [
            {'name': 'car01', 'domain_id': 1, 'pose_topic': '/mocap/car_01/pose',
             'mocap_binding_confirmed': True},
            {'name': 'car02', 'domain_id': 4, 'pose_topic': '/mocap/car_02/pose'},
            {'name': 'car03', 'domain_id': 25},
        ]}
        self.write_config()

    def write_config(self):
        self.config.write_text(json.dumps(self.payload), encoding='utf-8')

    def test_construction_and_copied_missing_states_do_not_import_ros(self):
        with patch.dict(sys.modules, {'rclpy': None, 'geometry_msgs': None}):
            mocap = Mocap(self.config)
            states = mocap.snapshot()
            self.assertEqual(tuple(states), ('car01', 'car02', 'car03'))
            self.assertTrue(all(state['status'] == 'missing' for state in states.values()))
            self.assertTrue(states['car01']['binding_confirmed'])
            self.assertFalse(states['car02']['binding_confirmed'])
            self.assertEqual(states['car01']['domain_id'], 7)
            self.assertIsNone(states['car03']['topic'])
            self.assertIsNone(states['car01']['age_s'])
            states['car01']['binding_confirmed'] = False
            self.assertTrue(mocap.get('car01')['binding_confirmed'])
            with self.assertRaises(KeyError):
                mocap.get('missing')
            mocap.close()

    def test_domain_default_override_and_selection(self):
        self.payload.pop('mocap_domain_id')
        self.write_config()
        self.assertEqual(Mocap(self.config).domain_id, 0)
        selected = Mocap(self.config, cars='car02', domain_id=81)
        self.assertEqual(selected.names, ('car02',))
        self.assertEqual(selected.get('car02')['domain_id'], 81)
        with self.assertRaises(KeyError):
            selected.get('car01')

    def test_invalid_configuration_is_rejected_before_ros(self):
        for domain in (True, -1, 100, 1.0, '0'):
            with self.subTest(domain=domain), self.assertRaises(ValueError):
                Mocap(self.config, domain_id=domain)
        for stale in (True, 0, -1, float('inf'), float('nan'), '1'):
            with self.subTest(stale=stale), self.assertRaises(ValueError):
                Mocap(self.config, stale_after=stale)
        for topic in ('relative/topic', '/', '/bad//topic', '/bad topic', 123):
            self.payload['cars'][0]['pose_topic'] = topic
            self.write_config()
            with self.subTest(topic=topic), self.assertRaises(ValueError):
                Mocap(self.config)
        self.payload['cars'][0]['pose_topic'] = '/mocap/car_01/pose'
        self.payload['cars'][0]['mocap_binding_confirmed'] = 'yes'
        self.write_config()
        with self.assertRaises(ValueError):
            Mocap(self.config)

    def test_all_missing_or_empty_topics_need_no_ros_resources(self):
        self.payload['cars'][0]['pose_topic'] = ''
        self.payload['cars'][1]['pose_topic'] = None
        self.write_config()
        with patch.dict(sys.modules, {'rclpy': None, 'geometry_msgs': None}):
            with Mocap(self.config) as mocap:
                self.assertTrue(all(state['status'] == 'missing'
                                    for state in mocap.snapshot().values()))

    def test_selected_pose_topics_must_be_unique(self):
        self.payload['cars'][1]['pose_topic'] = '/mocap/car_01/pose'
        self.write_config()
        with self.assertRaisesRegex(ValueError, 'pose_topic must be unique'):
            Mocap(self.config)
        self.assertEqual(Mocap(self.config, cars='car01').names, ('car01',))

    def test_one_context_subscribes_all_selected_topics_read_only(self):
        with fake_ros() as ros:
            mocap = Mocap(self.config).open()
            try:
                self.assertIs(mocap.open(), mocap)
                self.assertEqual([context.domain for context in ros.contexts], [7])
                self.assertEqual(len(ros.nodes), 1)
                self.assertEqual(len(ros.executors), 1)
                self.assertEqual(len(ros.threads), 1)
                node = ros.nodes[0]
                self.assertEqual(tuple(node.subscriptions),
                                 ('/mocap/car_01/pose', '/mocap/car_02/pose'))
                self.assertFalse(node.options['use_global_arguments'])
                self.assertFalse(node.options['start_parameter_services'])
                self.assertFalse(node.options['enable_rosout'])
                for subscription in node.subscriptions.values():
                    self.assertEqual(subscription.qos.depth, 1)
                    self.assertEqual(subscription.qos.reliability, 'best_effort')
                    self.assertEqual(subscription.qos.durability, 'volatile')
                node.publishers['/mocap/car_01/pose'] = 2
                node.queue.append(('/mocap/car_01/pose', pose(yaw=0.75, scale=1.02)))
                with patch('leap_fleet.mocap.time.monotonic', return_value=10.0), \
                        patch('leap_fleet.mocap.time.time', return_value=1000.0):
                    mocap._poll_once(update_graph=True)
                    state = mocap.get('car01')
                self.assertEqual(state['status'], 'fresh')
                self.assertEqual(state['publishers'], 2)
                self.assertEqual(state['position'], {'x': 1.0, 'y': 2.0, 'z': 0.15})
                self.assertEqual(state['frame_id'], 'world')
                self.assertEqual(state['source_stamp'], {'sec': 123, 'nanosec': 456})
                self.assertEqual(state['received_at_unix'], 1000.0)
                self.assertAlmostEqual(state['yaw_rad'], 0.75)
                self.assertAlmostEqual(math.hypot(*state['orientation'].values()), 1.0)
                self.assertEqual(mocap.get('car02')['status'], 'missing')
            finally:
                mocap.close()

    def test_freshness_and_routing_are_independent_per_car(self):
        with fake_ros() as ros:
            mocap = Mocap(self.config, stale_after=0.5).open()
            try:
                node = ros.nodes[0]
                node.queue.append(('/mocap/car_02/pose', pose(x=4.0, yaw=-1.25)))
                with patch('leap_fleet.mocap.time.monotonic', return_value=10.0):
                    mocap._poll_once(update_graph=False)
                with patch('leap_fleet.mocap.time.monotonic', return_value=10.5):
                    self.assertEqual(mocap.get('car02')['status'], 'fresh')
                with patch('leap_fleet.mocap.time.monotonic', return_value=10.501):
                    states = mocap.snapshot()
                self.assertEqual(states['car01']['status'], 'missing')
                self.assertEqual(states['car02']['status'], 'stale')
                self.assertAlmostEqual(states['car02']['age_s'], 0.501)
                self.assertAlmostEqual(states['car02']['yaw_rad'], -1.25)
                self.assertEqual(states['car02']['position']['x'], 4.0)
                self.assertEqual(states['car03']['status'], 'missing')
            finally:
                mocap.close()

    def test_nested_snapshots_and_get_results_cannot_mutate_cache(self):
        with fake_ros() as ros:
            mocap = Mocap(self.config).open()
            try:
                ros.nodes[0].queue.append(('/mocap/car_01/pose', pose()))
                mocap._poll_once(update_graph=False)
                result = mocap.get('car01')
                result['position']['x'] = 99
                result['orientation']['w'] = 0
                result['source_stamp']['sec'] = 0
                result['status'] = 'missing'
                self.assertEqual(mocap.get('car01')['position']['x'], 1.0)
                self.assertEqual(mocap.get('car01')['orientation']['w'], 1.0)
                self.assertEqual(mocap.get('car01')['source_stamp']['sec'], 123)
                self.assertEqual(mocap.get('car01')['status'], 'fresh')
            finally:
                mocap.close()

    def test_invalid_pose_clears_previous_values_and_valid_input_recovers(self):
        invalid = [pose(x=float('nan')), pose(y=float('inf')), pose(scale=0),
                   pose(scale=2), SimpleNamespace()]
        with fake_ros() as ros:
            mocap = Mocap(self.config).open()
            try:
                for message in invalid:
                    ros.nodes[0].queue.append(('/mocap/car_01/pose', pose()))
                    mocap._poll_once(update_graph=False)
                    ros.nodes[0].queue.append(('/mocap/car_01/pose', message))
                    mocap._poll_once(update_graph=True)
                    state = mocap.get('car01')
                    self.assertEqual(state['status'], 'invalid')
                    self.assertTrue(state['error'])
                    for field in ('position', 'orientation', 'yaw_rad', 'age_s',
                                  'received_at_unix', 'source_stamp'):
                        self.assertIsNone(state[field])
                    mocap._poll_once(update_graph=True)
                    self.assertEqual(mocap.get('car01')['status'], 'invalid')
                ros.nodes[0].queue.append(('/mocap/car_01/pose', pose()))
                mocap._poll_once(update_graph=False)
                self.assertEqual(mocap.get('car01')['status'], 'fresh')
                self.assertIsNone(mocap.get('car01')['error'])
            finally:
                mocap.close()

    def test_subscription_failure_is_isolated_to_its_car(self):
        with fake_ros(fail_topic='/mocap/car_01/pose') as ros:
            mocap = Mocap(self.config).open()
            try:
                ros.nodes[0].queue.append(('/mocap/car_02/pose', pose()))
                mocap._poll_once(update_graph=True)
                self.assertEqual(mocap.get('car01')['status'], 'error')
                self.assertIn('subscription', mocap.get('car01')['error'])
                self.assertEqual(mocap.get('car02')['status'], 'fresh')
                self.assertEqual(mocap.get('car03')['status'], 'missing')
            finally:
                mocap.close()

    def test_graph_and_executor_failures_are_visible_and_recover(self):
        with fake_ros() as ros:
            mocap = Mocap(self.config).open()
            try:
                node, executor = ros.nodes[0], ros.executors[0]
                node.queue.append(('/mocap/car_01/pose', pose()))
                node.graph_errors['/mocap/car_01/pose'] = RuntimeError('graph unavailable')
                mocap._poll_once(update_graph=True)
                self.assertEqual(mocap.get('car01')['status'], 'error')
                self.assertEqual(mocap.get('car02')['status'], 'missing')
                node.graph_errors.clear()
                mocap._poll_once(update_graph=True)
                self.assertEqual(mocap.get('car01')['status'], 'fresh')
                executor.spin_error = RuntimeError('executor unavailable')
                mocap._poll_once(update_graph=False)
                self.assertEqual(mocap.get('car01')['status'], 'error')
                self.assertEqual(mocap.get('car02')['status'], 'error')
                self.assertEqual(mocap.get('car03')['status'], 'missing')
                executor.spin_error = None
                mocap._poll_once(update_graph=False)
                self.assertEqual(mocap.get('car01')['status'], 'fresh')
            finally:
                mocap.close()

    def test_partial_initialization_failures_release_all_created_resources(self):
        for stage in ('init', 'node', 'executor', 'add_node', 'thread'):
            with self.subTest(stage=stage), fake_ros(fail_at=stage) as ros:
                mocap = Mocap(self.config)
                with self.assertRaisesRegex(RuntimeError, 'simulated ' + stage):
                    mocap.open()
                self.assertIn(('context_close', 7), ros.events)
                if ros.nodes:
                    self.assertIn(('node_close',), ros.events)
                if ros.executors:
                    self.assertIn(('executor_close',), ros.events)
                self.assertIsNone(mocap._context)
                previous = list(ros.events)
                mocap.close()
                self.assertEqual(ros.events, previous)

    def test_close_joins_before_disposal_is_idempotent_and_can_reopen(self):
        with fake_ros() as ros:
            mocap = Mocap(self.config).open()
            mocap.close()
            order = [entry[0] for entry in ros.events]
            self.assertLess(order.index('thread_join'), order.index('executor_close'))
            self.assertLess(order.index('executor_close'), order.index('node_close'))
            self.assertLess(order.index('node_close'), order.index('context_close'))
            previous = list(ros.events)
            mocap.close()
            self.assertEqual(ros.events, previous)
            mocap.open()
            self.assertEqual(len(ros.contexts), 2)
            self.assertEqual(mocap.get('car01')['status'], 'missing')
            mocap.close()

    def test_initialization_cleanup_failure_preserves_original_exception(self):
        with fake_ros(fail_at='add_node', cleanup_failure=True) as ros:
            mocap = Mocap(self.config)
            with self.assertRaisesRegex(RuntimeError, 'simulated add_node failure') as caught:
                mocap.open()
            self.assertIn('initialization cleanup failed', str(caught.exception.__cause__))
            self.assertIn(('node_close',), ros.events)
            self.assertIn(('context_close', 7), ros.events)

    def test_interrupted_initialization_releases_resources_and_propagates(self):
        with fake_ros(fail_at='add_node', failure=KeyboardInterrupt()) as ros:
            with self.assertRaises(KeyboardInterrupt):
                Mocap(self.config).open()
            self.assertIn(('executor_close',), ros.events)
            self.assertIn(('node_close',), ros.events)
            self.assertIn(('context_close', 7), ros.events)

    def test_cleanup_failure_cannot_skip_node_and_context_release(self):
        with fake_ros() as ros:
            mocap = Mocap(self.config).open()
            ros.executors[0].shutdown_error = RuntimeError('executor close failure')
            ros.nodes[0].destroy_error = RuntimeError('node close failure')
            with self.assertRaisesRegex(RuntimeError, 'Mocap cleanup failed'):
                mocap.close()
            self.assertIn(('node_close',), ros.events)
            self.assertIn(('context_close', 7), ros.events)
            self.assertIsNone(mocap._context)
            mocap.close()

    def test_stuck_thread_retains_resources_until_it_can_be_stopped(self):
        with fake_ros() as ros:
            mocap = Mocap(self.config).open()
            ros.threads[0].stuck = True
            with self.assertRaisesRegex(RuntimeError, 'resources retained'):
                mocap.close()
            self.assertNotIn(('node_close',), ros.events)
            self.assertIsNotNone(mocap._context)
            ros.threads[0].stuck = False
            mocap.close()
            self.assertIn(('context_close', 7), ros.events)

    def test_context_manager_releases_resources_when_body_raises(self):
        with fake_ros() as ros:
            with self.assertRaisesRegex(RuntimeError, 'body failed'):
                with Mocap(self.config):
                    raise RuntimeError('body failed')
            self.assertIn(('context_close', 7), ros.events)

    def test_real_background_thread_receives_both_cars_and_stops(self):
        with fake_ros(real_thread=True) as ros:
            mocap = Mocap(self.config).open()
            thread = mocap._thread
            try:
                node = ros.nodes[0]
                node.queue.extend([('/mocap/car_01/pose', pose(x=1.0)),
                                   ('/mocap/car_02/pose', pose(x=4.0))])
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if all(mocap.get(name)['status'] == 'fresh'
                           for name in ('car01', 'car02')):
                        break
                    time.sleep(0.01)
                self.assertEqual(mocap.get('car01')['position']['x'], 1.0)
                self.assertEqual(mocap.get('car02')['position']['x'], 4.0)
            finally:
                mocap.close()
            self.assertFalse(thread.is_alive())

    def test_background_drains_callback_backlog_without_per_message_sleep(self):
        with fake_ros(real_thread=True) as ros:
            mocap = Mocap(self.config).open()
            try:
                node = ros.nodes[0]
                node.queue.extend((f'/mocap/car_0{index % 2 + 1}/pose', pose(x=index))
                                  for index in range(100))
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    latest = mocap.get('car02')['position']
                    if not node.queue and latest is not None and latest['x'] == 99.0:
                        break
                    time.sleep(0.001)
                self.assertEqual(node.queue, [])
                self.assertEqual(mocap.get('car01')['position']['x'], 98.0)
                self.assertEqual(mocap.get('car02')['position']['x'], 99.0)
            finally:
                mocap.close()


if __name__ == '__main__':
    unittest.main()
