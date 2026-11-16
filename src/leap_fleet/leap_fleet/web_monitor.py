"""Read-only chassis observation and independent motion-capture aggregation."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import threading
import time
import uuid

from .config import default_config, load_cars


@dataclass
class _Resource:
    name: str
    context: object = None
    node: object = None
    executor: object = None


class RosMonitor:
    """Observe all configured domains using one polling thread.

    ``snapshot()`` only reads a locked cache, so HTTP requests never spin ROS
    executors. ROS imports and communication are deferred until ``open()``.
    """

    STALE_AFTER = 2.0
    GRAPH_INTERVAL = 0.5
    POLL_INTERVAL = 0.02

    def __init__(self, config_path=None):
        path = default_config() if config_path is None else Path(config_path).expanduser()
        self.configs = load_cars(path)
        entries = json.loads(path.read_text(encoding='utf-8'))['cars']
        self.metadata = {entry['name']: dict(entry) for entry in entries}
        self._cmd_topics = {car.name: car.cmd_vel_topic for car in self.configs}
        self._odom_topics = {}
        for car in self.configs:
            topic = self.metadata[car.name].get('odom_topic', '/odom')
            if (not isinstance(topic, str) or
                    not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', topic)):
                raise ValueError(f'{car.name}: odom_topic must be an absolute ROS topic')
            self._odom_topics[car.name] = topic
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._stop = threading.Event()
        self._resources = []
        self._thread = None
        self._opened = False
        self._states = self._empty_states()

    def _empty_states(self):
        return {car.name: {'received_at': None, 'measured_v': None,
                           'odom_sequence': 0,
                           'measured_w': None, 'cmd_subscribers': 0,
                           'cmd_publishers': 0, 'error': None}
                for car in self.configs}

    def open(self):
        """Start read-only ROS observation; a failed vehicle does not hide others."""
        with self._lifecycle_lock:
            if self._opened:
                return self
            from nav_msgs.msg import Odometry
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            with self._lock:
                self._states = self._empty_states()
            self._stop.clear()
            suffix = uuid.uuid4().hex[:8]
            try:
                for car in self.configs:
                    resource = _Resource(car.name)
                    self._resources.append(resource)
                    try:
                        resource.context = Context()
                        resource.context.init(args=[], domain_id=car.domain_id)
                        resource.node = Node(
                            f'leap_monitor_{car.name}_{suffix}', context=resource.context,
                            use_global_arguments=False, start_parameter_services=False,
                            enable_rosout=False)
                        resource.node.create_subscription(
                            Odometry, self._odom_topics[car.name],
                            lambda message, name=car.name: self._receive(name, message), qos)
                        resource.executor = SingleThreadedExecutor(context=resource.context)
                        resource.executor.add_node(resource.node)
                    except Exception as error:
                        cleanup = self._dispose(resource)
                        interruption = next((item for item in cleanup
                                             if not isinstance(item, Exception)), None)
                        if interruption is not None:
                            raise interruption
                        self._set_error(car.name, error, cleanup)
                        self._resources.remove(resource)
                self._thread = threading.Thread(
                    target=self._run, name='leap-fleet-monitor', daemon=True)
                self._thread.start()
                self._opened = True
            except BaseException:
                self._stop.set()
                if self._thread is not None and self._thread.is_alive():
                    self._thread.join(timeout=2.0)
                for resource in reversed(self._resources):
                    self._dispose(resource)
                self._resources.clear()
                self._thread = None
                raise
        return self

    def _receive(self, name, message):
        try:
            v = float(message.twist.twist.linear.x)
            w = float(message.twist.twist.angular.z)
            if not math.isfinite(v) or not math.isfinite(w):
                return
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        with self._lock:
            self._states[name].update(received_at=time.monotonic(),
                                      measured_v=v, measured_w=w)
            self._states[name]['odom_sequence'] += 1

    def _set_error(self, name, error, cleanup=()):
        detail = f'{type(error).__name__}: {error}'
        if cleanup:
            detail += '; cleanup: ' + '; '.join(str(item) for item in cleanup)
        with self._lock:
            self._states[name]['error'] = detail

    def _poll_once(self, *, update_graph):
        for resource in self._resources:
            if self._stop.is_set():
                break
            try:
                resource.executor.spin_once(timeout_sec=0.0)
                if update_graph:
                    topic = self._cmd_topics[resource.name]
                    subscribers = resource.node.count_subscribers(topic)
                    publishers = resource.node.count_publishers(topic)
                    with self._lock:
                        self._states[resource.name].update(
                            cmd_subscribers=subscribers, cmd_publishers=publishers, error=None)
            except Exception as error:
                self._set_error(resource.name, error)

    def _run(self):
        next_graph = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            update_graph = now >= next_graph
            if update_graph:
                next_graph = now + self.GRAPH_INTERVAL
            self._poll_once(update_graph=update_graph)
            self._stop.wait(self.POLL_INTERVAL)

    def snapshot(self):
        """Return copied status records; ``age_s`` is local odometry receive age."""
        now = time.monotonic()
        result = []
        with self._lock:
            for car in self.configs:
                state = self._states[car.name]
                received = state['received_at']
                age = None if received is None else max(0.0, now - received)
                if state['error'] is not None:
                    status = 'error'
                elif age is not None and age > self.STALE_AFTER:
                    status = 'stale'
                elif age is not None and state['cmd_subscribers'] > 0:
                    status = 'connected'
                elif age is not None or state['cmd_subscribers'] > 0:
                    status = 'discovered'
                else:
                    status = 'offline'
                ip = self.metadata[car.name].get('ip', '')
                result.append({
                    'id': car.name, 'name': car.name,
                    'ip': ip if isinstance(ip, str) else '', 'domain_id': car.domain_id,
                    'status': status, 'age_s': age,
                    'odom_received_at': received,
                    'odom_sequence': state['odom_sequence'],
                    'measured_v': state['measured_v'], 'measured_w': state['measured_w'],
                    'cmd_subscribers': state['cmd_subscribers'],
                    'cmd_publishers': state['cmd_publishers'], 'error': state['error'],
                })
        return result

    @staticmethod
    def _dispose(resource):
        errors = []
        operations = []
        if resource.executor is not None:
            operations.append(lambda: resource.executor.shutdown(timeout_sec=0.5))
        if resource.node is not None:
            operations.append(resource.node.destroy_node)
        if resource.context is not None:
            operations.append(resource.context.try_shutdown)
        for operation in operations:
            try:
                operation()
            except BaseException as error:
                errors.append(error)
        resource.executor = resource.node = resource.context = None
        return errors

    def close(self):
        """Stop the polling thread before releasing its executors and contexts."""
        with self._lifecycle_lock:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
                if self._thread.is_alive():
                    raise RuntimeError('ROS monitor thread did not stop; resources retained')
            errors = []
            for resource in reversed(self._resources):
                errors.extend(self._dispose(resource))
            self._resources.clear()
            self._thread = None
            self._opened = False
            with self._lock:
                self._states = self._empty_states()
            interruption = next((error for error in errors
                                 if not isinstance(error, Exception)), None)
            if interruption is not None:
                raise interruption
            if errors:
                raise RuntimeError('ROS monitor cleanup failed: ' + '; '.join(map(str, errors)))


class ConsoleMonitor:
    """Combine chassis and mocap caches without coupling their readiness.

    A mocap initialization failure is displayed in telemetry; it does not disable
    the existing velocity interface. Neither component creates command publishers.
    """

    def __init__(self, config_path=None, *, mocap_domain=None, chassis=None, mocap=None):
        from .mocap import Mocap
        self.chassis = chassis if chassis is not None else RosMonitor(config_path)
        self.mocap = mocap if mocap is not None else Mocap(config_path, domain_id=mocap_domain)
        self._mocap_error = None

    def open(self):
        self.chassis.open()
        try:
            self.mocap.open()
            self._mocap_error = None
        except Exception as error:
            self._mocap_error = f'{type(error).__name__}: {error}'
        return self

    def snapshot(self):
        poses = self.mocap.snapshot()
        cars = self.chassis.snapshot()
        for car in cars:
            pose = poses.get(car['id'])
            if pose is not None:
                if self._mocap_error is not None:
                    pose.update(status='error', error=self._mocap_error)
                car['mocap'] = pose
        return cars

    def close(self):
        try:
            self.mocap.close()
        finally:
            self.chassis.close()
