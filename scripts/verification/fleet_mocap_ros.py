#!/usr/bin/env python3
"""Local ROS mocap SDK verification on random PoseStamped topics.

Run with ROS_LOCALHOST_ONLY=1 and a sourced ROS environment. Synthetic poses
use domains 81/82; no Agent, real VRPN topics, or velocity publisher is used.
"""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
import time
import uuid

from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from leap_fleet.mocap import Mocap


@dataclass
class Resource:
    context: object = None
    node: object = None


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('run with ROS_LOCALHOST_ONLY=1; this is a local simulation')
    prefix = '/leap_mocap_test_' + uuid.uuid4().hex
    topics = {name: f'{prefix}/{name}/pose' for name in ('car04', 'car09')}
    command_topic = prefix + '/commands'
    cars = [{'name': 'car04', 'domain_id': 41, 'pose_topic': topics['car04'],
             'cmd_vel_topic': command_topic, 'mocap_binding_confirmed': True},
            {'name': 'car09', 'domain_id': 42, 'pose_topic': topics['car09'],
             'cmd_vel_topic': command_topic}]
    expected = {'car04': ((1.25, -2.5, 0.11), math.pi / 2),
                'car09': ((-0.7, 2.1, 0.15), -math.pi / 3)}
    enabled = {name: True for name in topics}
    invalid = {name: False for name in topics}
    resources, publishers = [], {}
    reader = duplicate = None
    next_publish = 0.0

    def message(node, position, yaw, *, bad=False):
        pose = PoseStamped()
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.header.frame_id = 'synthetic_world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        if bad:
            pose.pose.orientation.w = 0.0
        else:
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def pump():
        nonlocal next_publish
        now = time.monotonic()
        if now < next_publish:
            return
        next_publish = now + 0.04
        for name, publisher in publishers.items():
            if enabled[name]:
                position, yaw = expected[name]
                publisher.publish(message(resources[0].node, position, yaw, bad=invalid[name]))
        # Same topic in a different domain must never overwrite car04's pose.
        negative_publisher.publish(message(resources[1].node, (99.0, 98.0, 97.0), 0.0))

    def wait_until(predicate, description, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pump()
            if predicate():
                return
            time.sleep(0.01)
        states = None if reader is None else reader.snapshot()
        raise AssertionError(f'timed out waiting for {description}: {states}')

    def assert_pose(name, state):
        position, yaw = expected[name]
        assert state['status'] == 'fresh', state
        assert state['domain_id'] == 81 and state['topic'] == topics[name], state
        assert state['frame_id'] == 'synthetic_world', state
        for axis, value in zip(('x', 'y', 'z'), position):
            assert math.isclose(state['position'][axis], value, abs_tol=1e-10), state
        error = math.atan2(math.sin(state['yaw_rad'] - yaw), math.cos(state['yaw_rad'] - yaw))
        assert abs(error) < 1e-10, state
        assert math.isclose(sum(value * value for value in state['orientation'].values()),
                            1.0, abs_tol=1e-10), state

    cleanup_errors = []
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for domain in (81, 82):
            resource = Resource()
            resources.append(resource)
            resource.context = Context()
            resource.context.init(args=[], domain_id=domain)
            resource.node = Node('leap_mocap_verifier_' + str(domain),
                                 context=resource.context, use_global_arguments=False,
                                 enable_rosout=False, start_parameter_services=False)
        primary = resources[0].node
        publishers = {name: primary.create_publisher(PoseStamped, topic, qos)
                      for name, topic in topics.items()}
        negative_publisher = resources[1].node.create_publisher(
            PoseStamped, topics['car04'], qos)
        previous_command_publishers = primary.count_publishers('/cmd_vel')
        assert primary.count_publishers(command_topic) == 0

        with tempfile.TemporaryDirectory(prefix='leap_mocap_ros_') as directory:
            config = Path(directory) / 'fleet.json'
            config.write_text(json.dumps({'mocap_domain_id': 0, 'cars': cars}), encoding='utf-8')
            reader = Mocap(config=config, domain_id=81, stale_after=0.25).open()
            wait_until(lambda: all(state['status'] == 'fresh' and state['publishers'] == 1
                                   for state in reader.snapshot().values()),
                       'two fresh, independently routed poses')
            for name, state in reader.snapshot().items():
                assert_pose(name, state)
            assert reader.get('car04')['binding_confirmed'] is True
            assert reader.get('car09')['binding_confirmed'] is False
            copied = reader.get('car04')
            copied['position']['x'] = -999.0
            assert reader.get('car04')['position']['x'] == expected['car04'][0][0]
            print('PASS: per-car topic/domain routing, quaternion yaw, metadata and copied snapshots',
                  flush=True)

            enabled['car09'] = False
            wait_until(lambda: reader.get('car09')['status'] == 'stale',
                       'only the stopped pose stream becoming stale', timeout=3.0)
            assert reader.get('car09')['age_s'] > 0.25
            assert reader.get('car04')['status'] == 'fresh'
            print('PASS: missing pose updates become stale independently per car', flush=True)

            invalid['car09'] = True
            enabled['car09'] = True
            wait_until(lambda: reader.get('car09')['status'] == 'invalid',
                       'invalid quaternion rejection')
            state = reader.get('car09')
            assert state['position'] is None and state['orientation'] is None
            assert state['yaw_rad'] is None and state['age_s'] is None
            assert state['error'], state
            invalid['car09'] = False
            wait_until(lambda: reader.get('car09')['status'] == 'fresh',
                       'valid pose recovery')
            assert_pose('car09', reader.get('car09'))
            print('PASS: zero quaternion is rejected and valid poses recover', flush=True)

            duplicate = primary.create_publisher(PoseStamped, topics['car04'], qos)
            wait_until(lambda: reader.get('car04')['publishers'] == 2,
                       'duplicate pose publisher discovery')
            primary.destroy_publisher(duplicate)
            duplicate = None
            wait_until(lambda: reader.get('car04')['publishers'] == 1,
                       'duplicate publisher removal')
            assert primary.count_publishers(command_topic) == 0
            assert primary.count_publishers('/cmd_vel') == previous_command_publishers
            print('PASS: duplicate pose sources are counted; no velocity publisher is created',
                  flush=True)

            reader.close()
            reader = None
            wait_until(lambda: all(publisher.get_subscription_count() == 0
                                   for publisher in publishers.values()),
                       'mocap subscriptions released')
            print(json.dumps({'result': 'PASS', 'mocap_domain': 81,
                              'negative_domain': 82, 'pose_topics': topics}), flush=True)
    finally:
        if reader is not None:
            try:
                reader.close()
            except BaseException as error:
                cleanup_errors.append(error)
        for resource in reversed(resources):
            operations = []
            if resource.node is not None:
                operations.append(resource.node.destroy_node)
            if resource.context is not None:
                operations.append(resource.context.try_shutdown)
            for operation in operations:
                try:
                    operation()
                except BaseException as error:
                    cleanup_errors.append(error)
        if cleanup_errors:
            raise RuntimeError('mocap integration cleanup failed: ' +
                               '; '.join(map(str, cleanup_errors)))
    print('PASS: SDK subscriptions and all synthetic contexts closed', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
