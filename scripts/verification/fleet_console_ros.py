#!/usr/bin/env python3
"""Verify the console against two local synthetic cars on random ROS topics.

Requires ROS_LOCALHOST_ONLY=1 and a sourced ROS environment. Uses domains 61
and 62 with temporary configuration; never publishes to the real /cmd_vel.
No vehicle, Agent, motion capture, or existing fleet configuration is used.
"""

from dataclasses import dataclass
from functools import partial
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import uuid

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from leap_fleet import Fleet
from leap_fleet.web_console import Console
from leap_fleet.web_monitor import RosMonitor


@dataclass
class SyntheticCar:
    name: str
    context: object = None
    node: object = None
    executor: object = None
    odom_publisher: object = None


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('run with ROS_LOCALHOST_ONLY=1; this is a local simulation')
    prefix = '/leap_console_test_' + uuid.uuid4().hex
    command_topic, odom_topic = prefix + '/commands', prefix + '/odometry'
    assert command_topic != '/cmd_vel' and odom_topic != '/odom'
    entries = [{'name': 'car01', 'domain_id': 61, 'ip': '127.0.0.1',
                'cmd_vel_topic': command_topic, 'odom_topic': odom_topic},
               {'name': 'car02', 'domain_id': 62, 'ip': '127.0.0.1',
                'cmd_vel_topic': command_topic, 'odom_topic': odom_topic}]
    telemetry = {'car01': (0.013, -0.021), 'car02': (-0.017, 0.023)}
    received = {entry['name']: [] for entry in entries}
    ignored_zeros = {entry['name']: [] for entry in entries}
    applied_zeros = {entry['name']: [] for entry in entries}
    odom_samples = {entry['name']: [] for entry in entries}
    drop_next_zero = {entry['name']: True for entry in entries}
    odom_enabled = {entry['name']: True for entry in entries}
    resources, pump_errors = [], []
    lock, pump_stop = threading.RLock(), threading.Event()
    pump = monitor = console = None

    def receive(name, message):
        assert (message.linear.y, message.linear.z,
                message.angular.x, message.angular.y) == (0.0, 0.0, 0.0, 0.0)
        with lock:
            now = time.monotonic()
            velocity = (message.linear.x, message.angular.z)
            received[name].append((now, *velocity))
            if velocity != (0.0, 0.0):
                telemetry[name] = velocity
                drop_next_zero[name] = True
            elif drop_next_zero[name]:
                # The message is received, but this synthetic chassis ignores
                # its first stop command. Only a later zero changes telemetry.
                ignored_zeros[name].append((now, *telemetry[name]))
                drop_next_zero[name] = False
            else:
                telemetry[name] = velocity
                applied_zeros[name].append(now)

    def pump_synthetic():
        next_odometry = 0.0
        try:
            while not pump_stop.is_set():
                now = time.monotonic()
                publish_odometry = now >= next_odometry
                if publish_odometry:
                    next_odometry = now + 0.05
                for resource in resources:
                    resource.executor.spin_once(timeout_sec=0.0)
                    if publish_odometry:
                        with lock:
                            enabled = odom_enabled[resource.name]
                            velocity = telemetry[resource.name]
                        if enabled:
                            message = Odometry()
                            message.header.stamp = resource.node.get_clock().now().to_msg()
                            message.header.frame_id = 'odom'
                            message.child_frame_id = 'base_link'
                            message.pose.pose.orientation.w = 1.0
                            message.twist.twist.linear.x = velocity[0]
                            message.twist.twist.angular.z = velocity[1]
                            resource.odom_publisher.publish(message)
                            with lock:
                                odom_samples[resource.name].append(
                                    (time.monotonic(), *velocity))
                pump_stop.wait(0.005)
        except BaseException as error:
            with lock:
                pump_errors.append(error)
            pump_stop.set()

    def wait_until(predicate, *, timeout=10.0, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with lock:
                if pump_errors:
                    raise AssertionError('synthetic ROS pump failed') from pump_errors[0]
            if predicate():
                return
            time.sleep(0.01)
        state = None if console is None else console.snapshot()
        raise AssertionError(f'timed out waiting for {description}: {state}')

    def terminal_job(expected):
        wait_until(lambda: console.snapshot()['job']['status'] not in
                   ('starting', 'running', 'stopping'),
                   description=f'{expected} console task')
        # The final state is assigned at the end of the worker's finally block.
        console._worker.join(timeout=2.0)
        assert not console._worker.is_alive(), 'console worker did not exit'
        job = console.snapshot()['job']
        assert job['status'] == expected, job
        assert job['error'] is None, job
        return job

    def sample_values(name, since):
        with lock:
            return [(v, w) for stamp, v, w in received[name] if stamp >= since]

    def has_velocity(name, since, velocity):
        return velocity in sample_values(name, since)

    def ends_with_zero(name, since):
        samples = sample_values(name, since)
        return bool(samples) and samples[-1] == (0.0, 0.0)

    def assert_stop_feedback(job, names, since):
        """Check retransmission and new telemetry, not just the last Twist."""
        for name in names:
            result = job['car_results'][name]
            assert result['stop_status'] == 'confirmed', (name, result)
            assert result['zero_count'] >= 3, (name, result)
            zeros = [value for value in sample_values(name, since) if value == (0.0, 0.0)]
            assert len(zeros) >= 3, (name, zeros)
            with lock:
                applied = [stamp for stamp in applied_zeros[name] if stamp >= since]
                assert applied, (name, applied_zeros[name])
                samples = [(stamp, v, w) for stamp, v, w in odom_samples[name]
                           if stamp >= applied[0] and (v, w) == (0.0, 0.0)]
            assert len(samples) >= console.STOP_MIN_SAMPLES, (name, samples)
            assert samples[-1][0] - samples[0][0] >= console.STOP_STABLE_TIME, (name, samples)

    cleanup_errors = []
    try:
        command_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        odom_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        for entry in entries:
            resource = SyntheticCar(entry['name'])
            resources.append(resource)
            resource.context = Context()
            resource.context.init(args=[], domain_id=entry['domain_id'])
            resource.node = Node('leap_console_synthetic_' + entry['name'],
                                 context=resource.context, use_global_arguments=False,
                                 enable_rosout=False, start_parameter_services=False)
            resource.node.create_subscription(
                Twist, command_topic,
                lambda message, name=entry['name']: receive(name, message), command_qos)
            resource.odom_publisher = resource.node.create_publisher(
                Odometry, odom_topic, odom_qos)
            resource.executor = SingleThreadedExecutor(context=resource.context)
            resource.executor.add_node(resource.node)

        pump = threading.Thread(target=pump_synthetic, name='synthetic-car-pump')
        pump.start()
        with tempfile.TemporaryDirectory(prefix='leap_console_ros_') as directory:
            config = Path(directory) / 'fleet.json'
            config.write_text(json.dumps({'cars': entries}), encoding='utf-8')
            monitor = RosMonitor(config).open()
            console = Console(monitor, partial(Fleet, config=config))
            wait_until(lambda: all(car['status'] == 'connected' for car in monitor.snapshot()),
                       description='two connected synthetic cars')
            states = {state['id']: state for state in monitor.snapshot()}
            for name, velocity in telemetry.items():
                assert (states[name]['measured_v'], states[name]['measured_w']) == velocity
                assert states[name]['cmd_subscribers'] == 1
                assert states[name]['cmd_publishers'] == 0
            print('PASS: per-domain read-only monitor and odometry velocities', flush=True)

            expected = {'car01': (0.041, -0.12), 'car02': (-0.034, 0.09)}
            began = time.monotonic()
            console.start({name: {'v': v, 'w': w} for name, (v, w) in expected.items()},
                          duration=0.25, rate=40)
            final = terminal_job('completed')
            wait_until(lambda: all(ends_with_zero(name, began) for name in expected),
                       description='final zero velocity in both domains')
            for name, velocity in expected.items():
                samples = sample_values(name, began)
                assert velocity in samples, (name, samples)
                assert all(value in (velocity, (0.0, 0.0)) for value in samples), (name, samples)
                with lock:
                    ignored = [(stamp, v, w) for stamp, v, w in ignored_zeros[name]
                               if stamp >= began]
                assert ignored and ignored[0][1:] == velocity, (name, ignored)
            assert_stop_feedback(final, expected, began)
            print('PASS: distinct domain commands stop after ignoring each first zero; '
                  'retransmissions and multiple fresh near-zero odometry samples confirmed',
                  flush=True)

            # A finished task must remain stoppable using the same persistent
            # writers. An extra stop is a new zero window, not a no-op.
            began = time.monotonic()
            with lock:
                for name in expected:
                    drop_next_zero[name] = True
            result = console.stop(scope='job')
            assert result['job']['status'] == 'stopping', result
            repeated = terminal_job('stopped')
            assert set(repeated['cars']) == set(expected), repeated
            assert_stop_feedback(repeated, expected, began)
            wait_until(lambda: all(state['cmd_publishers'] == 1 for state in monitor.snapshot()),
                       description='one reused persistent writer in each domain')
            print('PASS: a completed task can be stopped again with repeated zeros and '
                  'fresh per-car feedback; persistent writers are reused', flush=True)

            began = time.monotonic()
            cancelled = (-0.025, -0.07)
            console.start({'car01': {'v': cancelled[0], 'w': cancelled[1]}},
                          duration=10.0, rate=40)
            wait_until(lambda: has_velocity('car01', began, cancelled),
                       description='command before cancellation')
            console.stop(scope='job')
            cancelled_job = terminal_job('stopped')
            wait_until(lambda: ends_with_zero('car01', began),
                       description='zero velocity after cancellation')
            # DDS can still drain zero samples from the preceding completed
            # task. The unselected domain must receive no new motion command.
            assert all(value == (0.0, 0.0) for value in sample_values('car02', began)), \
                'unselected car received a motion command'
            assert_stop_feedback(cancelled_job, ['car01'], began)
            print('PASS: stopping a running task sends final zero only to its participant', flush=True)

            with lock:
                odom_enabled['car02'] = False
            wait_until(lambda: {state['id']: state for state in monitor.snapshot()}[
                'car02']['status'] == 'stale', timeout=5.0,
                description='stale odometry with the command sink still present')
            states = {state['id']: state for state in monitor.snapshot()}
            assert states['car02']['age_s'] > 2.0, states
            assert states['car02']['cmd_subscribers'] == 1, states
            assert states['car01']['status'] == 'connected', states
            try:
                console.start({'car02': {'v': 0.03, 'w': 0.0}}, duration=0.1)
            except ValueError:
                pass
            else:
                raise AssertionError('console accepted a new task for stale odometry')
            print('PASS: missing odometry becomes stale and rejects a new console task', flush=True)

            began = time.monotonic()
            result = console.stop(scope='all')
            assert result['job']['status'] == 'stopping', result
            all_stopped = terminal_job('unconfirmed')
            assert set(all_stopped['cars']) == set(expected), all_stopped
            results = all_stopped['car_results']
            assert results['car01']['stop_status'] == 'confirmed', results
            assert results['car02']['stop_status'] == 'unconfirmed', results
            assert all(item['zero_count'] >= 3 for item in results.values()), results
            assert_stop_feedback(all_stopped, ['car01'], began)
            assert console.snapshot()['stop_unconfirmed'] == ['car02']
            print('PASS: stop_all keeps healthy-car feedback confirmed and stale-car '
                  'feedback unconfirmed despite zero retransmissions to both', flush=True)

            with lock:
                summary = {name: len(samples) for name, samples in received.items()}
            print(json.dumps({'result': 'PASS', 'domains': [61, 62],
                              'command_topic': command_topic, 'odom_topic': odom_topic,
                              'received_commands': summary,
                              'stop_results': all_stopped['car_results'],
                              'ignored_first_zeros': {name: len(values)
                                                      for name, values in ignored_zeros.items()}},
                             ensure_ascii=False), flush=True)
    finally:
        if console is not None:
            try:
                console.close()
            except BaseException as error:
                cleanup_errors.append(error)
        elif monitor is not None:
            try:
                monitor.close()
            except BaseException as error:
                cleanup_errors.append(error)
        pump_stop.set()
        if pump is not None:
            pump.join(timeout=3.0)
            if pump.is_alive():
                cleanup_errors.append(RuntimeError('synthetic pump did not exit'))
        for resource in reversed(resources):
            operations = []
            if resource.executor is not None:
                operations.append(lambda resource=resource:
                                  resource.executor.shutdown(timeout_sec=1.0))
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
            raise RuntimeError('integration cleanup failed: ' +
                               '; '.join(map(str, cleanup_errors)))
    print('PASS: console, monitor, synthetic thread and all contexts closed', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
