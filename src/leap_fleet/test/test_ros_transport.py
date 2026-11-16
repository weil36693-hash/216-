"""Transport lifecycle failures, using fake ROS modules and no DDS traffic."""

from contextlib import contextmanager
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from leap_fleet import Fleet
from leap_fleet.config import CarConfig
from leap_fleet.transport import RosTransport


@contextmanager
def fake_ros(*, fail_stage=None, failure=None):
    events, contexts, nodes, publishers = [], [], [], []

    class Context:
        def __init__(self):
            self.index = len(contexts)
            contexts.append(self)

        def init(self, *, args, domain_id):
            events.append(('init', self.index, args, domain_id))
            if self.index == 1 and fail_stage == 'init':
                raise failure

        def try_shutdown(self):
            events.append(('shutdown', self.index))

    class Publisher:
        def __init__(self, index):
            self.index = index
            self.ack_error = None
            self.acknowledged = True
            publishers.append(self)

        def wait_for_all_acked(self, duration):
            events.append(('ack', self.index))
            if self.ack_error is not None:
                raise self.ack_error
            return self.acknowledged

    class Node:
        def __init__(self, name, *, context, **kwargs):
            self.index = context.index
            self.destroy_error = None
            events.append(('node', self.index, kwargs))
            if self.index == 1 and fail_stage == 'node':
                raise failure
            nodes.append(self)

        def create_publisher(self, message_type, topic, qos):
            events.append(('publisher', self.index, topic))
            if self.index == 1 and fail_stage == 'publisher':
                raise failure
            return Publisher(self.index)

        def destroy_node(self):
            events.append(('destroy', self.index))
            if self.destroy_error is not None:
                raise self.destroy_error

    def module(**values):
        result = ModuleType('fake_ros')
        result.__dict__.update(values)
        return result

    modules = {
        'geometry_msgs': module(),
        'geometry_msgs.msg': module(Twist=object),
        'rclpy': module(),
        'rclpy.context': module(Context=Context),
        'rclpy.node': module(Node=Node),
        'rclpy.qos': module(
            QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
            ReliabilityPolicy=SimpleNamespace(RELIABLE='reliable'),
            DurabilityPolicy=SimpleNamespace(VOLATILE='volatile')),
        'rclpy.duration': module(Duration=lambda **kwargs: kwargs),
    }
    with patch.dict(sys.modules, modules):
        yield SimpleNamespace(events=events, contexts=contexts, nodes=nodes,
                              publishers=publishers)


class RosTransportLifecycleTests(unittest.TestCase):
    cars = (CarConfig('car01', 1), CarConfig('car02', 2), CarConfig('car03', 3))

    def test_partial_open_failure_releases_created_resources(self):
        for stage in ('init', 'node', 'publisher'):
            for error_type in (RuntimeError, KeyboardInterrupt):
                with self.subTest(stage=stage, error=error_type.__name__), \
                        tempfile.TemporaryDirectory() as directory:
                    config = Path(directory) / 'fleet.json'
                    config.write_text(json.dumps({'cars': [
                        {'name': car.name, 'domain_id': car.domain_id}
                        for car in self.cars]}), encoding='utf-8')
                    failure = error_type('open interrupted')
                    created = []

                    def factory(cars):
                        transport = RosTransport(cars)
                        created.append(transport)
                        return transport

                    with fake_ros(fail_stage=stage, failure=failure) as ros:
                        fleet = Fleet(config, transport_factory=factory)
                        with self.assertRaises(error_type) as caught:
                            fleet.open()
                        self.assertIs(caught.exception, failure)
                        self.assertEqual(
                            [item[1] for item in ros.events if item[0] == 'shutdown'],
                            [1, 0])
                        self.assertIn(('destroy', 0), ros.events)
                        if stage == 'publisher':
                            self.assertIn(('destroy', 1), ros.events)
                        self.assertEqual(created[0]._resources, [])
                        self.assertEqual(created[0]._publishers, {})
                        with self.assertRaises(RuntimeError):
                            fleet.set_velocity('car01', 0.0, 0.0)

    def test_cleanup_exceptions_do_not_skip_other_nodes_or_contexts(self):
        with fake_ros() as ros:
            transport = RosTransport(self.cars)
            transport.open()
            ros.publishers[0].ack_error = RuntimeError('ack failed')
            ros.nodes[2].destroy_error = RuntimeError('destroy failed')
            with self.assertRaisesRegex(RuntimeError, 'ROS cleanup failed'):
                transport.close()
            self.assertEqual(
                [item[1] for item in ros.events if item[0] == 'destroy'], [2, 1, 0])
            self.assertEqual(
                [item[1] for item in ros.events if item[0] == 'shutdown'], [2, 1, 0])
            self.assertEqual(transport._resources, [])
            self.assertEqual(transport._publishers, {})
            previous = list(ros.events)
            transport.close()
            self.assertEqual(ros.events, previous)

    def test_ack_keyboard_interrupt_still_releases_every_resource(self):
        with fake_ros() as ros:
            transport = RosTransport(self.cars)
            transport.open()
            interruption = KeyboardInterrupt('close interrupted')
            ros.publishers[0].ack_error = interruption
            with self.assertRaises(KeyboardInterrupt) as caught:
                transport.close()
            self.assertIs(caught.exception, interruption)
            self.assertEqual(
                [item[1] for item in ros.events if item[0] == 'destroy'], [2, 1, 0])
            self.assertEqual(
                [item[1] for item in ros.events if item[0] == 'shutdown'], [2, 1, 0])
            self.assertEqual(transport._resources, [])
            self.assertEqual(transport._publishers, {})

    def test_dds_ack_timeout_is_reported_after_all_resources_are_released(self):
        with fake_ros() as ros:
            transport = RosTransport(self.cars)
            transport.open()
            ros.publishers[0].acknowledged = False
            ros.publishers[2].acknowledged = False
            with self.assertRaises(RuntimeError) as caught:
                transport.close()
            message = str(caught.exception)
            self.assertIn('car01: DDS ACK timed out', message)
            self.assertIn('car03: DDS ACK timed out', message)
            self.assertIn('not a vehicle execution ACK', message)
            self.assertEqual([event[1] for event in ros.events if event[0] == 'ack'], [0, 1, 2])
            self.assertEqual([event[1] for event in ros.events if event[0] == 'destroy'], [2, 1, 0])
            self.assertEqual([event[1] for event in ros.events if event[0] == 'shutdown'], [2, 1, 0])
            self.assertEqual(transport._resources, [])
            self.assertEqual(transport._publishers, {})


if __name__ == '__main__':
    unittest.main()
