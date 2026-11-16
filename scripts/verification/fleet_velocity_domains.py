#!/usr/bin/env python3
"""Integration check with 25 local synthetic subscribers on a random topic.

Never publishes to /cmd_vel. Requires ROS_LOCALHOST_ONLY=1 and a sourced ROS
environment. No Agent, mocap, robot, or existing fleet configuration is used.
"""

import json
import os
from pathlib import Path
import tempfile
import time
import uuid

from geometry_msgs.msg import Twist
from leap_fleet import Fleet, Velocity
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('run with ROS_LOCALHOST_ONLY=1; this is a local simulation')
    topic = '/leap_velocity_test_' + uuid.uuid4().hex + '/commands'
    latest, received, resources = {}, {}, []
    cars = [{'name': f'car{i + 1:02}', 'domain_id': 61 + i,
             'cmd_vel_topic': topic} for i in range(25)]

    def receive(name, msg):
        assert (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y) == (0, 0, 0, 0)
        latest[name] = Velocity(msg.linear.x, msg.angular.z)
        received[name] = received.get(name, 0) + 1

    def pump_until(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for context, node, executor in resources:
                executor.spin_once(timeout_sec=0)
            if predicate():
                return
            time.sleep(0.005)
        raise AssertionError(f'timed out: latest={latest}')

    try:
        # Same random topic in every domain: only explicit domain routing can
        # deliver each car its distinct command. Domain 60 is a negative probe.
        for car in [*cars, {'name': 'wrong_domain', 'domain_id': 60}]:
            context = Context()
            resources.append([context, None, None])
            context.init(args=[], domain_id=car['domain_id'])
            node = Node('leap_test_receiver_' + car['name'], context=context,
                        use_global_arguments=False, enable_rosout=False,
                        start_parameter_services=False)
            resources[-1][1] = node
            node.create_subscription(
                Twist, topic, lambda msg, name=car['name']: receive(name, msg),
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
            executor = SingleThreadedExecutor(context=context)
            resources[-1][2] = executor
            executor.add_node(node)

        with tempfile.TemporaryDirectory(prefix='leap_fleet_domains_') as directory:
            config = Path(directory) / 'fleet.json'
            config.write_text(json.dumps({'cars': cars}), encoding='utf-8')
            expected = {car['name']: Velocity((i + 1) * 0.001, -(i + 1) * 0.002)
                        for i, car in enumerate(cars)}
            with Fleet(config=config) as fleet:
                assert fleet.wait_for_subscribers(20), 'DDS subscribers not matched'
                start = time.monotonic()
                fleet.set_velocities(expected)
                pump_until(lambda: all(latest.get(k) == v for k, v in expected.items()))
                routing_seconds = time.monotonic() - start
                assert 'wrong_domain' not in latest
                fleet['car01'].set_velocity(-0.02, 0.04)
                pump_until(lambda: latest['car01'] == Velocity(-0.02, 0.04))
                assert all(latest[k] == v for k, v in expected.items() if k != 'car01')

                fleet.stop(['car01'])
                pump_until(lambda: latest['car01'] == Velocity(0, 0))
                fleet.run_for({car['name']: (0.01, 0.02) for car in cars},
                              duration=0.25, rate=20)
                pump_until(lambda: all(latest.get(car['name']) == Velocity(0, 0)
                                       for car in cars))
                assert all(received[car['name']] >= 3 for car in cars)
                # Leave nonzero synthetic values pending, verifying close's
                # final zero publications with subscribers still alive.
                fleet.set_velocities(expected)
                pump_until(lambda: all(latest.get(k) == v for k, v in expected.items()))
            pump_until(lambda: all(latest.get(car['name']) == Velocity(0, 0)
                                   for car in cars))
            assert 'wrong_domain' not in latest
            print(json.dumps({'cars': 25, 'domains': [61, 85], 'topic': topic,
                              'domain_routing': 'passed', 'single_car': 'passed',
                              'run_for_and_stop': 'passed', 'close_zeros': 'passed',
                              'wrong_domain_received': False,
                              'first_batch_seconds': round(routing_seconds, 3)}, indent=2))
    finally:
        for context, node, executor in reversed(resources):
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            context.try_shutdown()


if __name__ == '__main__':
    main()
