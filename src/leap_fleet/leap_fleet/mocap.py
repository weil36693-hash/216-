"""Read-only motion-capture poses in one explicit ROS domain.

``fresh`` describes receipt of valid pose messages. It does not establish live
tracking, a vehicle/rigid-body binding, or alignment with the vehicle's forward
axis. Reported yaw is the rigid body's raw yaw; no calibration is applied.
"""

from copy import deepcopy
import json
import math
from pathlib import Path
import re
import threading
import time
import uuid
import warnings

from .config import default_config, load_cars


class Mocap:
    """Observe configured ``pose_topic`` values without publishing commands.

    Construction and snapshots do not import ROS. ``open()`` creates one
    Context, Node, executor, and background thread for the selected pose topics.
    Missing topics remain ``missing`` and do not prevent observing other cars.
    """

    GRAPH_INTERVAL = 0.5
    POLL_INTERVAL = 0.02

    def __init__(self, config=None, cars=None, domain_id=None, stale_after=1.0):
        path = default_config() if config is None else Path(config).expanduser()
        self.configs = load_cars(path, cars)
        self.names = tuple(car.name for car in self.configs)
        payload = json.loads(path.read_text(encoding='utf-8'))
        domain = payload.get('mocap_domain_id', 0) if domain_id is None else domain_id
        if type(domain) is not int or not 0 <= domain <= 99:
            raise ValueError('mocap domain_id must be an integer in 0..99')
        if (isinstance(stale_after, bool) or not isinstance(stale_after, (int, float))
                or not math.isfinite(stale_after) or stale_after <= 0):
            raise ValueError('stale_after must be a finite positive number')
        self.domain_id = domain
        self.stale_after = float(stale_after)
        metadata = {entry['name']: entry for entry in payload['cars']}
        self._topics, self._bindings = {}, {}
        topic_owners = {}
        for name in self.names:
            topic = metadata[name].get('pose_topic')
            if topic is None or topic == '':
                topic = None
            elif (not isinstance(topic, str) or
                  not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', topic)):
                raise ValueError(f'{name}: pose_topic must be an absolute ROS topic')
            if topic is not None:
                if topic in topic_owners:
                    raise ValueError(f'pose_topic must be unique: {topic} is used by '
                                     f'{topic_owners[topic]} and {name}')
                topic_owners[topic] = name
            confirmed = metadata[name].get('mocap_binding_confirmed', False)
            if type(confirmed) is not bool:
                raise ValueError(f'{name}: mocap_binding_confirmed must be a boolean')
            self._topics[name], self._bindings[name] = topic, confirmed
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._stop = threading.Event()
        self._context = self._node = self._executor = self._thread = None
        self._subscriptions = {}
        self._opened = False
        self._reset_states()

    def _reset_states(self):
        self._states = {
            name: {'status': 'missing', 'topic': self._topics[name],
                   'domain_id': self.domain_id, 'frame_id': None,
                   'position': None, 'orientation': None, 'yaw_rad': None,
                   'received_at_unix': None, 'source_stamp': None,
                   'publishers': 0, 'error': None,
                   'binding_confirmed': self._bindings[name], '_received_at': None}
            for name in self.names}
        self._subscription_errors = {}
        self._graph_errors = {}
        self._spin_error = None

    def open(self):
        """Start read-only subscriptions; repeated calls are harmless."""
        with self._lifecycle_lock:
            if self._opened:
                return self
            if self._context is not None:
                raise RuntimeError('Mocap has retained resources; close it before reopening')
            with self._lock:
                self._reset_states()
            if not any(self._topics.values()):
                self._opened = True
                return self
            from geometry_msgs.msg import PoseStamped
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            self._stop.clear()
            try:
                self._context = Context()
                self._context.init(args=[], domain_id=self.domain_id)
                self._node = Node(
                    'leap_mocap_' + uuid.uuid4().hex[:8], context=self._context,
                    use_global_arguments=False, start_parameter_services=False,
                    enable_rosout=False)
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                for name, topic in self._topics.items():
                    if topic is None:
                        continue
                    try:
                        self._subscriptions[name] = self._node.create_subscription(
                            PoseStamped, topic,
                            lambda message, car=name: self._receive(car, message), qos)
                    except Exception as error:
                        with self._lock:
                            self._subscription_errors[name] = self._error_text(error)
                self._executor = SingleThreadedExecutor(context=self._context)
                self._executor.add_node(self._node)
                self._thread = threading.Thread(
                    target=self._run, name='leap-mocap-reader', daemon=True)
                self._thread.start()
                self._opened = True
            except BaseException as error:
                self._stop.set()
                try:
                    self._join_thread()
                except BaseException as cleanup_error:
                    raise error from cleanup_error
                cleanup_errors = self._dispose()
                if cleanup_errors:
                    raise error from RuntimeError(
                        'Mocap initialization cleanup failed: '
                        + '; '.join(map(str, cleanup_errors)))
                raise
        return self

    @staticmethod
    def _error_text(error):
        return f'{type(error).__name__}: {error}'

    @staticmethod
    def _pose(message):
        p, q = message.pose.position, message.pose.orientation
        values = tuple(float(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('pose contains nonfinite values')
        x, y, z, qx, qy, qz, qw = values
        norm = math.hypot(qx, qy, qz, qw)
        if not 0.95 <= norm <= 1.05:
            raise ValueError('quaternion must have a near-unit, nonzero norm')
        qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
        frame = message.header.frame_id
        stamp = message.header.stamp
        if not isinstance(frame, str):
            raise ValueError('frame_id must be a string')
        if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
                or not 0 <= stamp.nanosec < 1_000_000_000):
            raise ValueError('source timestamp is invalid')
        return {'frame_id': frame, 'position': {'x': x, 'y': y, 'z': z},
                'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
                'yaw_rad': math.atan2(2 * (qw * qz + qx * qy),
                                      1 - 2 * (qy * qy + qz * qz)),
                'source_stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec}}

    def _receive(self, name, message):
        try:
            pose = self._pose(message)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            with self._lock:
                self._states[name].update(
                    status='invalid', frame_id=None, position=None,
                    orientation=None, yaw_rad=None, received_at_unix=None,
                    source_stamp=None, _received_at=None, error=self._error_text(error))
            return
        with self._lock:
            self._states[name].update(pose, status='fresh', error=None,
                                      _received_at=time.monotonic(),
                                      received_at_unix=time.time())

    def _poll_once(self, *, update_graph):
        success = True
        try:
            # Pending callbacks return immediately. Waiting only when there is
            # no work avoids limiting a 25-car pose stream to one callback per
            # polling interval.
            self._executor.spin_once(timeout_sec=self.POLL_INTERVAL)
            with self._lock:
                self._spin_error = None
        except Exception as error:
            success = False
            with self._lock:
                self._spin_error = self._error_text(error)
        if update_graph:
            for name in self._subscriptions:
                if self._stop.is_set():
                    break
                try:
                    count = self._node.count_publishers(self._topics[name])
                    with self._lock:
                        self._states[name]['publishers'] = count
                        self._graph_errors.pop(name, None)
                except Exception as error:
                    with self._lock:
                        self._graph_errors[name] = self._error_text(error)
        return success

    def _run(self):
        next_graph = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            update_graph = now >= next_graph
            if update_graph:
                next_graph = now + self.GRAPH_INTERVAL
            if not self._poll_once(update_graph=update_graph):
                self._stop.wait(self.POLL_INTERVAL)

    def snapshot(self):
        """Return independent pose records keyed by the configured car ID."""
        now = time.monotonic()
        result = {}
        with self._lock:
            for name, cached in self._states.items():
                state = deepcopy(cached)
                received = state.pop('_received_at')
                state['age_s'] = None if received is None else max(0.0, now - received)
                error = (self._subscription_errors.get(name) or self._graph_errors.get(name)
                         or (self._spin_error if self._topics[name] else None))
                if error:
                    state.update(status='error', error=error)
                elif state['status'] == 'fresh' and state['age_s'] > self.stale_after:
                    state['status'] = 'stale'
                result[name] = state
        return result

    def get(self, car_id):
        """Return a copied record; unknown or unselected IDs raise KeyError."""
        if car_id not in self.names:
            raise KeyError(f'car not selected: {car_id!r}')
        return self.snapshot()[car_id]

    def _join_thread(self):
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError('Mocap reader thread did not stop; resources retained')

    def _dispose(self):
        errors = []
        operations = []
        if self._executor is not None:
            operations.append(lambda: self._executor.shutdown(timeout_sec=0.5))
        if self._node is not None:
            operations.append(self._node.destroy_node)
        if self._context is not None:
            operations.append(self._context.try_shutdown)
        for operation in operations:
            try:
                operation()
            except BaseException as error:
                errors.append(error)
        self._thread = self._executor = self._node = self._context = None
        self._subscriptions.clear()
        return errors

    def close(self):
        """Release subscriptions and contexts; never publish a stop command."""
        with self._lifecycle_lock:
            self._stop.set()
            self._join_thread()
            errors = self._dispose()
            self._opened = False
            with self._lock:
                self._reset_states()
            interruption = next((error for error in errors
                                 if not isinstance(error, Exception)), None)
            if interruption is not None:
                raise interruption
            if errors:
                raise RuntimeError('Mocap cleanup failed: ' + '; '.join(map(str, errors)))

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as error:
            if exc_type is None:
                raise
            warnings.warn(f'Mocap cleanup failed: {error}', RuntimeWarning)
        return False
