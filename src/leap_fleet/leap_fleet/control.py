"""Direct differential-drive commands, routed to the existing per-car domains."""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from numbers import Real
import sys
import threading
import time
import warnings

from .config import load_cars


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


@dataclass(frozen=True)
class Velocity:
    """Body-frame forward speed (m/s) and yaw rate (rad/s); no lateral command."""

    v: float
    w: float

    def __post_init__(self):
        object.__setattr__(self, 'v', _finite(self.v, 'v'))
        object.__setattr__(self, 'w', _finite(self.w, 'w'))


class Car:
    """A selected vehicle; shares the owning Fleet's connection lifecycle."""

    def __init__(self, fleet, name):
        self._fleet = fleet
        self.name = name

    def set_velocity(self, v, w):
        """Publish one command. Continuous algorithms call this in their loop."""
        self._fleet.set_velocity(self.name, v, w)

    def stop(self):
        self._fleet.stop([self.name])


class Fleet:
    """One publisher per selected car, with no planners or sensor dependencies.

    Publishers use explicit ROS contexts and do not change ROS_DOMAIN_ID or
    initialize/shutdown the caller's default ROS context. Opening never sends
    velocity commands. Closing publishes zeros to all selected cars.
    """

    def __init__(self, config=None, cars=None, *, transport_factory=None):
        self.configs = load_cars(config, cars)
        self.names = tuple(car.name for car in self.configs)
        if transport_factory is None:
            from .transport import RosTransport
            transport_factory = RosTransport
        self._factory = transport_factory
        self._transport = None
        self._lock = threading.RLock()
        self._stopped_cars = set()
        self._command_counter = 0

    def open(self):
        with self._lock:
            if self._transport is not None:
                return self
            transport = self._factory(self.configs)
            try:
                transport.open()
                self._transport = transport
                self._stopped_cars.clear()
                self._command_counter += 1
            except BaseException:
                self._transport = None
                try:
                    transport.close()
                except Exception:
                    pass
                raise
        return self

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as error:
            if exc_type is None:
                raise
            warnings.warn(f'command cleanup failed: {error}', RuntimeWarning)
        return False

    def __getitem__(self, name):
        self._check_name(name)
        return Car(self, name)

    def _check_name(self, name):
        if name not in self.names:
            raise KeyError(f'car not selected: {name!r}')

    def _require_open(self):
        if self._transport is None:
            raise RuntimeError('Fleet is not open; use with Fleet(...) or open()')
        return self._transport

    def _commands(self, commands):
        if not isinstance(commands, Mapping) or not commands:
            raise ValueError('commands must be a nonempty mapping: car -> Velocity or (v, w)')
        result = {}
        for name, value in commands.items():
            self._check_name(name)
            if not isinstance(value, Velocity):
                if not isinstance(value, (tuple, list)) or len(value) != 2:
                    raise ValueError(f'{name}: expected Velocity or (v, w)')
                value = Velocity(*value)
            result[name] = value
        return result

    def set_velocity(self, car, v, w):
        """Publish exactly one Twist: linear.x=v, angular.z=w, other axes zero."""
        self.set_velocities({car: Velocity(v, w)})

    def set_velocities(self, commands):
        """Validate the entire batch, then publish in mapping order.

        This is a batch of individual ROS publications, not an atomic or
        hard-synchronized multi-car command. Transport errors can partially send.
        No speed clipping or background command repetition is performed.
        """
        batch = self._commands(commands)
        with self._lock:
            transport = self._require_open()
            self._stopped_cars.difference_update(batch)
            self._command_counter += 1
            errors = []
            for name, velocity in batch.items():
                try:
                    transport.publish(name, velocity)
                except Exception as error:
                    errors.append(f'{name}: {error}')
            if errors:
                raise RuntimeError('velocity publish failed: ' + '; '.join(errors))

    def stop(self, cars=None):
        """Send zero to selected cars; try every target even if one send fails."""
        targets = self.names if cars is None else ((cars,) if isinstance(cars, str) else tuple(cars))
        for name in targets:
            self._check_name(name)
        with self._lock:
            transport = self._require_open()
            errors = []
            for name in dict.fromkeys(targets):
                try:
                    transport.publish(name, Velocity(0.0, 0.0))
                except Exception as error:
                    errors.append(f'{name}: {error}')
            if errors:
                raise RuntimeError('stop publish failed: ' + '; '.join(errors))

    def stop_repeated(self, cars=None, duration=1.0, rate=20.0):
        """Repeat zero commands for a bounded window, then report all send failures.

        A failed car or round does not skip the others. This is a command send
        window, not acknowledgement that a vehicle physically stopped.
        """
        targets = self.names if cars is None else ((cars,) if isinstance(cars, str) else tuple(cars))
        for name in targets:
            self._check_name(name)
        targets = tuple(dict.fromkeys(targets))
        duration, rate = _finite(duration, 'duration'), _finite(rate, 'rate')
        if duration <= 0 or rate <= 0:
            raise ValueError('duration and rate must be positive')
        with self._lock:
            self._require_open()
            counter = self._command_counter
        if not targets:
            return
        failures = {}
        deadline = time.monotonic() + duration
        first = True
        while first or time.monotonic() < deadline:
            first = False
            sent = time.monotonic()
            try:
                self.stop(targets)
            except Exception as error:
                message = str(error)
                failures[message] = failures.get(message, 0) + 1
            remaining = min(sent + 1.0 / rate, deadline) - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        if failures:
            summary = '; '.join(f'{message} ({count} rounds)' for message, count in failures.items())
            raise RuntimeError('repeated stop publish failed: ' + summary)
        with self._lock:
            if self._command_counter == counter:
                self._stopped_cars.update(targets)

    def wait_for_subscribers(self, timeout=5.0):
        """Wait up to one total timeout for a matching subscriber per car.

        A match establishes DDS discovery, not acknowledgement of vehicle motion.
        This method does not send commands or require odometry or mocap.
        """
        timeout = _finite(timeout, 'timeout')
        if timeout < 0:
            raise ValueError('timeout must be nonnegative')
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                transport = self._require_open()
                if all(transport.subscriber_count(name) > 0 for name in self.names):
                    return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.05, remaining))

    def run_for(self, commands, duration, rate=20.0):
        """Repeat a fixed batch for duration seconds, then send zero to its cars.

        This blocking convenience method uses a monotonic clock. For a changing
        control algorithm use set_velocities() inside the algorithm's own loop.
        It does not create a task or a background thread.
        """
        batch = self._commands(commands)
        duration, rate = _finite(duration, 'duration'), _finite(rate, 'rate')
        if duration <= 0 or rate <= 0:
            raise ValueError('duration and rate must be positive')
        self._require_open()
        deadline = time.monotonic() + duration
        period = 1.0 / rate
        try:
            while time.monotonic() < deadline:
                sent = time.monotonic()
                self.set_velocities(batch)
                remaining = min(sent + period, deadline) - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            has_error = sys.exc_info()[0] is not None
            try:
                self.stop_repeated(batch)
            except Exception as error:
                if not has_error:
                    raise
                warnings.warn(f'stop publish failed during cleanup: {error}', RuntimeWarning)

    def close(self, *, send_stop=True):
        """Release connections, normally after one repeated-zero send window.

        ``send_stop=False`` only releases resources for a caller that already
        owns the stop procedure. A successful stop_repeated()/run_for() is not
        repeated unless another command was sent afterwards.
        """
        if type(send_stop) is not bool:
            raise ValueError('send_stop must be a boolean')
        with self._lock:
            if self._transport is None:
                return
            transport = self._transport
            try:
                pending = tuple(name for name in self.names if name not in self._stopped_cars)
                if send_stop and pending:
                    self.stop_repeated(pending)
            finally:
                self._transport = None
                self._stopped_cars.clear()
                transport.close()
