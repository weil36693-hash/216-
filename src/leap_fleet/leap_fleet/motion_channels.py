"""Persistent, independent single-car command senders for the web console.

Each worker owns its Fleet and all of its I/O. The shared lock only protects
small command/state records; it is never held while opening, publishing, or
closing a Fleet. A stop replaces the latest command instead of joining a queue.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
import math
import threading
import time
import uuid

from .control import Velocity


_ZERO = Velocity(0.0, 0.0)


def _number(value, name, *, zero=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (value < 0 if zero else value <= 0)):
        raise ValueError(f'{name} must be a finite {"nonnegative" if zero else "positive"} number')
    return float(value)


def _names(names):
    try:
        selected = [names] if isinstance(names, str) else list(names)
    except TypeError as error:
        raise ValueError('names must contain vehicle IDs') from error
    if not selected or any(not isinstance(name, str) or not name for name in selected):
        raise ValueError('names must contain vehicle IDs')
    return tuple(dict.fromkeys(selected))


@dataclass
class _Channel:
    name: str
    wake: threading.Event = field(default_factory=threading.Event)
    worker: object = None
    velocity: Velocity = _ZERO
    deadline: object = None
    next_at: float = 0.0
    period: float = 0.05
    stop_until: object = None
    release_requested: bool = False
    close_error: object = None
    matched: bool = False
    next_probe: float = 0.0
    probe_generation: int = 0
    checked_generation: int = -1
    motion_attempted: bool = False
    stream_ttl: object = None
    stream_expires_at: object = None
    stream_initial_zero: bool = False
    probe_error: object = None
    publish_error: object = None
    stop_error: object = None
    state: dict = field(default_factory=lambda: {
        'ready': False, 'opening': True, 'error': None, 'last_error': None,
        'publishing': False, 'command_count': 0, 'zero_count': 0,
        'first_zero_at': None, 'last_publish_at': None, 'stop_pending': False,
        'active': False, 'closed': False, 'releasing': False, 'checking': False,
        'stop_unconfirmed': False, 'stop_reason': None,
        'stream_id': None, 'stream_expired': False,
        'stream_started_at': None, 'stream_deadline': None,
        'continue_on_error': False, 'transport_warning': None,
    })


class MotionChannels:
    """Latest-command mailboxes with a separate persistent Fleet per vehicle."""

    def __init__(self, fleet_factory, *, stop_duration=1.0, stop_rate=20.0):
        self._factory = fleet_factory
        self.stop_duration = _number(stop_duration, 'stop_duration')
        self.stop_rate = _number(stop_rate, 'stop_rate')
        self._lock = threading.RLock()
        self._channels = {}
        self._streams = {}
        self._closed = False

    PROBE_INTERVAL = 0.1

    def _ensure_locked(self, names, *, refresh=False):
        for name in names:
            previous = self._channels.get(name)
            if previous is not None:
                if previous.worker is not None and previous.worker.is_alive():
                    if (refresh and not previous.state['active']
                            and not previous.state['stop_pending']
                            and not previous.release_requested):
                        previous.probe_generation += 1
                        previous.next_probe = 0.0
                        previous.state['ready'] = False
                        previous.wake.set()
                    continue
                if previous.close_error:
                    previous.state.update(ready=False, error=previous.close_error,
                                          last_error=previous.close_error)
                    continue
            channel = _Channel(name)
            if previous is not None:
                channel.state['error'] = previous.state['error']
                channel.state['last_error'] = previous.state['last_error']
            self._channels[name] = channel
            try:
                channel.worker = threading.Thread(
                    target=self._run, args=(channel,), name=f'fleet-motion-{name}', daemon=True)
                channel.worker.start()
            except Exception as error:
                channel.worker = None
                channel.state.update(opening=False, closed=True,
                                     error=str(error), last_error=str(error))

    def ensure(self, names):
        """Start any missing single-car workers without waiting for ROS I/O."""
        selected = _names(names)
        with self._lock:
            if self._closed:
                raise RuntimeError('MotionChannels is closed')
            self._ensure_locked(selected, refresh=True)

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            for channel in self._channels.values():
                self._expire_locked(channel, now)
            return {name: dict(channel.state) for name, channel in self._channels.items()}

    def _check_ready_locked(self, names):
        for name in names:
            channel = self._channels.get(name)
            if channel is None:
                raise RuntimeError(f'{name}: call ensure and wait for ready before start')
            state = channel.state
            if (not state['ready'] or state['opening'] or state['closed']
                    or state['publishing'] or state['stop_pending'] or state['active']
                    or state['checking'] or channel.checked_generation != channel.probe_generation
                    or channel.release_requested):
                raise RuntimeError(f'{name}: channel is not idle and ready')

    @staticmethod
    def _commands(commands):
        if not isinstance(commands, Mapping) or not commands:
            raise ValueError('commands must be a nonempty mapping of car IDs to Velocity')
        batch = dict(commands)
        _names(batch)
        if any(not isinstance(value, Velocity) for value in batch.values()):
            raise ValueError('each command must be a Velocity')
        return batch

    def start(self, commands, duration, rate=20.0):
        """Atomically accept one finite run; no I/O occurs in the caller."""
        duration, rate = _number(duration, 'duration'), _number(rate, 'rate')
        batch = self._commands(commands)
        with self._lock:
            if self._closed:
                raise RuntimeError('MotionChannels is closed')
            self._check_ready_locked(batch)
            now = time.monotonic()
            deadline = now + duration
            for name, velocity in batch.items():
                channel = self._channels[name]
                channel.velocity, channel.deadline = velocity, deadline
                channel.next_at, channel.period = now, 1.0 / rate
                channel.stop_until = None
                channel.stream_ttl = channel.stream_expires_at = None
                channel.stream_initial_zero = False
                channel.probe_error = channel.publish_error = channel.stop_error = None
                channel.state.update(active=True, error=None, last_error=None,
                                     command_count=0, zero_count=0, first_zero_at=None,
                                     last_publish_at=None, stop_unconfirmed=False, stop_reason=None,
                                     stream_id=None, stream_expired=False,
                                     stream_started_at=None, stream_deadline=None,
                                     continue_on_error=False, transport_warning=None)
                channel.wake.set()
            return deadline

    def start_stream(self, names, *, duration, rate=20.0, ttl=0.3, durations=None,
                     continue_on_error=False):
        """Reserve ready channels with a common clock and an expiring latest-command mailbox.

        Returns monotonic ``started_at``/``deadline`` and a unique ``session_id``.
        Each car publishes an initial zero before any supplied velocity. Updates
        may cover a subset; they renew only those cars' short receive leases.
        ``ttl=None`` disables that lease. ``continue_on_error=True`` retries
        discovery/publication failures while keeping the latest command active;
        it never overrides a manual stop, close, or a car's duration deadline.
        """
        selected = _names(names)
        duration = _number(duration, 'duration')
        rate = _number(rate, 'rate')
        ttl = None if ttl is None else _number(ttl, 'ttl')
        if type(continue_on_error) is not bool:
            raise ValueError('continue_on_error must be a bool')
        if durations is None:
            per_car = dict.fromkeys(selected, duration)
        else:
            if not isinstance(durations, Mapping) or set(durations) != set(selected):
                raise ValueError('durations must contain exactly the selected vehicle IDs')
            per_car = {name: _number(durations[name], f'{name} duration') for name in selected}
            if any(value > duration for value in per_car.values()):
                raise ValueError('each vehicle duration must be at most duration')
        if not math.isfinite(1.0 / rate):
            raise ValueError('rate produces a non-finite publication period')
        with self._lock:
            if self._closed:
                raise RuntimeError('MotionChannels is closed')
            self._check_ready_locked(selected)
            now = time.monotonic()
            deadline = now + duration
            session_id = uuid.uuid4().hex
            self._streams[session_id] = set(selected)
            for name in selected:
                channel = self._channels[name]
                channel.velocity, channel.deadline = _ZERO, now + per_car[name]
                channel.next_at, channel.period = now, 1.0 / rate
                channel.stop_until = None
                channel.stream_ttl = ttl
                channel.stream_expires_at = None if ttl is None else now + ttl
                channel.stream_initial_zero = True
                channel.probe_error = channel.publish_error = channel.stop_error = None
                channel.state.update(active=True, error=None, last_error=None,
                                     command_count=0, zero_count=0, first_zero_at=None,
                                     last_publish_at=None, stop_unconfirmed=False, stop_reason=None,
                                     stream_id=session_id, stream_expired=False,
                                     stream_started_at=now, stream_deadline=channel.deadline,
                                     continue_on_error=continue_on_error, transport_warning=None)
                channel.wake.set()
            return {'session_id': session_id, 'started_at': now, 'deadline': deadline}

    def update_stream(self, session_id, commands):
        """Atomically replace a nonempty active subset; stopped/expired tokens never resume."""
        if not isinstance(session_id, str) or not session_id:
            raise ValueError('session_id must be a nonempty string')
        batch = self._commands(commands)
        with self._lock:
            if self._closed:
                raise RuntimeError('MotionChannels is closed')
            members = self._streams.get(session_id)
            if members is None:
                raise RuntimeError('stream session is no longer active')
            now = time.monotonic()
            # Expiration is independent of a worker returning from blocked I/O.
            for name in tuple(members):
                self._expire_locked(self._channels[name], now)
            members = self._streams.get(session_id)
            if members is None:
                raise RuntimeError('stream session is no longer active')
            for name in batch:
                channel = self._channels.get(name)
                if (name not in members or channel is None or not channel.state['active']
                        or channel.state['stream_id'] != session_id):
                    raise RuntimeError(f'{name}: vehicle is not active in this stream session')
            for name, velocity in batch.items():
                channel = self._channels[name]
                channel.velocity = velocity
                channel.stream_expires_at = (None if channel.stream_ttl is None
                                             else now + channel.stream_ttl)
                channel.wake.set()

    def _end_stream_locked(self, channel):
        members = self._streams.get(channel.state['stream_id'])
        if members is not None:
            members.discard(channel.name)
            if not members:
                del self._streams[channel.state['stream_id']]
        channel.stream_ttl = channel.stream_expires_at = None
        channel.stream_initial_zero = False

    def _expire_locked(self, channel, now):
        state = channel.state
        if not state['active']:
            return
        duration_due = now >= channel.deadline
        ttl_due = (channel.stream_expires_at is not None
                   and now >= channel.stream_expires_at)
        if ttl_due and (not duration_due or channel.stream_expires_at < channel.deadline):
            message = 'stream command TTL expired'
            state.update(stream_expired=True, error=message, last_error=message,
                         stop_reason=message)
            self._stop_locked(channel)
        elif duration_due:
            if state['stream_id'] is not None:
                state['stop_reason'] = 'stream duration complete'
            self._stop_locked(channel)

    def _stop_locked(self, channel):
        self._end_stream_locked(channel)
        if channel.state['closed'] or channel.state['stop_pending']:
            return
        channel.stop_error = None
        channel.velocity, channel.deadline = _ZERO, None
        channel.next_at, channel.stop_until = time.monotonic(), None
        channel.state.update(active=False, stop_pending=True, first_zero_at=None, zero_count=0)
        channel.wake.set()

    def stop(self, names):
        """Replace motion with repeated zeros, including channels still opening."""
        selected = _names(names)
        with self._lock:
            if self._closed and any(name not in self._channels for name in selected):
                raise RuntimeError('MotionChannels is closed')
            if not self._closed:
                self._ensure_locked(selected)
            for name in selected:
                channel = self._channels[name]
                if channel.release_requested:
                    channel.state.update(stop_unconfirmed=True,
                                         stop_reason='idle channel is being released')
                    continue
                if channel.state['closed']:
                    channel.state.update(stop_unconfirmed=True,
                                         stop_reason='channel is closed; zero publication unconfirmed')
                    continue
                state = channel.state
                if not (state['active'] or state['stop_pending'] or state['publishing']
                        or state['opening']):
                    # A new explicit parking operation may recover from an old
                    # run's error. Stops already handling that error retain it.
                    channel.stop_error = channel.publish_error = None
                    state.update(error=None, transport_warning=None)
                self._stop_locked(channel)

    def _probe(self, channel, fleet):
        with self._lock:
            generation = channel.probe_generation
            channel.state.update(checking=True, ready=False)
        error = None
        try:
            matched = fleet.wait_for_subscribers(timeout=0.0)
        except Exception as failure:
            matched, error = False, str(failure)
        with self._lock:
            previous = channel.matched
            channel.matched = bool(matched)
            channel.checked_generation = generation
            channel.next_probe = (time.monotonic() + self.PROBE_INTERVAL
                                  if generation == channel.probe_generation else 0.0)
            state = channel.state
            state.update(checking=False, opening=False,
                         ready=bool(matched) and generation == channel.probe_generation)
            if state['continue_on_error']:
                channel.probe_error = (error or 'velocity subscriber unavailable; retrying'
                                       if not matched else None)
                self._transport_status_locked(channel)
            elif error is not None:
                state.update(error=error, last_error=error)
            if not matched and (state['active'] or (previous and state['stop_pending'])):
                message = error or 'velocity subscriber lost during motion/stop'
                if state['continue_on_error'] and state['active']:
                    return
                state.update(error=message, last_error=message, stop_unconfirmed=True)
                self._stop_locked(channel)
                channel.stop_until = None

    @staticmethod
    def _transport_status_locked(channel):
        message = '; '.join(str(value) for value in
                            (channel.probe_error, channel.publish_error) if value) or None
        channel.state['transport_warning'] = message
        # The optional receive lease remains a terminal stop rule. A successful
        # zero after its expiry must not erase that separate failure reason.
        if not channel.state['stream_expired']:
            channel.state['error'] = channel.stop_error or message
        if message is not None:
            channel.state['last_error'] = message

    def _run(self, channel):
        fleet = None
        try:
            fleet = self._factory(cars=[channel.name])
            fleet.open()
            while True:
                send = None
                wait = None
                with self._lock:
                    now = time.monotonic()
                    state = channel.state
                    if channel.release_requested:
                        break
                    if self._closed and not channel.matched and not channel.motion_attempted:
                        state.update(stop_unconfirmed=state['stop_pending'],
                                     stop_reason='no matching subscriber; resource closed without motion')
                        break
                    self._expire_locked(channel, now)
                    probe_due = (now >= channel.next_probe and not
                                 (state['stop_pending'] and state['zero_count'] == 0))
                if probe_due:
                    self._probe(channel, fleet)
                with self._lock:
                    now = time.monotonic()
                    state = channel.state
                    if channel.release_requested:
                        break
                    self._expire_locked(channel, now)
                    if state['stop_pending']:
                        if (channel.stop_until is not None and now >= channel.stop_until
                                and state['zero_count'] >= 2):
                            state['stop_pending'] = False
                            state['stop_unconfirmed'] = False
                            if self._closed:
                                break
                        elif now >= channel.next_at:
                            send = ('zero', _ZERO)
                            channel.next_at = now + 1.0 / self.stop_rate
                        else:
                            wait = channel.next_at - now
                            if channel.stop_until is not None and state['zero_count'] >= 2:
                                wait = min(wait, max(0.0, channel.stop_until - now))
                    elif state['active'] and (channel.matched or state['continue_on_error']):
                        if now >= channel.next_at:
                            send = (('stream_initial', _ZERO) if channel.stream_initial_zero
                                    else ('motion', channel.velocity))
                            channel.next_at = now + channel.period
                        else:
                            wait = min(channel.next_at, channel.deadline) - now
                            if channel.stream_expires_at is not None:
                                wait = min(wait, max(0.0, channel.stream_expires_at - now))
                    elif self._closed:
                        break
                    if send is None:
                        # The first zero has priority over a discovery probe.
                        # If that write keeps failing, an overdue suppressed
                        # probe must not turn the retry delay into a busy-spin.
                        if not (state['stop_pending'] and state['zero_count'] == 0):
                            until_probe = max(0.0, channel.next_probe - now)
                            wait = until_probe if wait is None else min(wait, until_probe)
                    if send:
                        state['publishing'] = True
                        if send[0] == 'motion' and (send[1].v != 0 or send[1].w != 0):
                            channel.motion_attempted = True
                if send is None:
                    channel.wake.wait(wait)
                    channel.wake.clear()
                    continue
                kind, velocity = send
                error = None
                try:
                    fleet.set_velocities({channel.name: velocity})
                except Exception as failure:
                    error = str(failure)
                with self._lock:
                    state = channel.state
                    state['publishing'] = False
                    if error is not None:
                        if state['continue_on_error']:
                            if kind == 'zero':
                                channel.stop_error = error
                            channel.publish_error = error
                            self._transport_status_locked(channel)
                        else:
                            state.update(error=error, last_error=error)
                        if not (state['continue_on_error'] and state['active']):
                            self._stop_locked(channel)
                    else:
                        if state['continue_on_error']:
                            channel.publish_error = None
                            self._transport_status_locked(channel)
                        finished = time.monotonic()
                        state['last_publish_at'] = finished
                        if kind == 'zero':
                            state['zero_count'] += 1
                            if state['first_zero_at'] is None:
                                state['first_zero_at'] = finished
                            if channel.matched and channel.stop_until is None:
                                channel.stop_until = finished + self.stop_duration
                        else:
                            state['command_count'] += 1
                            if kind == 'stream_initial':
                                channel.stream_initial_zero = False
        except Exception as error:
            with self._lock:
                channel.state.update(error=str(error), last_error=str(error))
        finally:
            with self._lock:
                # Closing I/O may itself block. Revoke updates and new starts
                # before entering it, while this worker still owns the Fleet.
                self._end_stream_locked(channel)
                channel.state.update(ready=False, active=False)
            if fleet is not None:
                try:
                    # This worker has completed its zero window, or never sent
                    # motion. Release must not introduce an extra command.
                    fleet.close(send_stop=False)
                except Exception as error:
                    with self._lock:
                        channel.close_error = str(error)
                        channel.state.update(error=str(error), last_error=str(error))
            with self._lock:
                channel.state.update(ready=False, opening=False, publishing=False,
                                     checking=False, active=False, closed=True)

    @staticmethod
    def _join(channels, timeout):
        deadline = time.monotonic() + timeout
        for channel in channels:
            if channel.worker is not None:
                channel.worker.join(timeout=max(0.0, deadline - time.monotonic()))

    @staticmethod
    def _pending(channels):
        return [channel.name for channel in channels
                if channel.worker is not None and channel.worker.is_alive()]

    def release(self, names, timeout=1.0):
        """Close and forget only idle channels, without publishing extra zeros."""
        selected, timeout = _names(names), _number(timeout, 'timeout', zero=True)
        with self._lock:
            channels = [self._channels[name] for name in selected if name in self._channels]
            for channel in channels:
                state = channel.state
                if (not state['closed'] and (state['active'] or state['opening']
                        or state['stop_pending'] or state['publishing'] or state['checking'])):
                    raise RuntimeError(f'{channel.name}: cannot release a busy channel')
            for channel in channels:
                channel.release_requested = True
                channel.state.update(ready=False, releasing=True)
                channel.wake.set()
        self._join(channels, timeout)
        with self._lock:
            pending = self._pending(channels)
            errors = [f'{channel.name}: {channel.close_error}' for channel in channels
                      if channel.close_error]
            for channel in channels:
                if channel.name not in pending and not channel.close_error:
                    if self._channels.get(channel.name) is channel:
                        del self._channels[channel.name]
        if pending or errors:
            raise RuntimeError('motion channel release incomplete: '
                               + '; '.join([*pending, *errors]))

    def close(self, timeout=3.0):
        """Park and close within one total budget; blocked workers remain owned."""
        timeout = _number(timeout, 'timeout', zero=True)
        with self._lock:
            self._closed = True
            channels = list(self._channels.values())
            for channel in channels:
                if not channel.release_requested:
                    self._stop_locked(channel)
                channel.wake.set()
        self._join(channels, timeout)
        with self._lock:
            pending = self._pending(channels)
            errors = [f'{channel.name}: {channel.close_error}' for channel in channels
                      if channel.close_error]
            unfinished = [channel.name for channel in channels
                          if channel.state['stop_pending'] and channel.name not in pending
                          and channel.motion_attempted]
        if pending or errors or unfinished:
            raise RuntimeError('motion channels not closed: '
                               + '; '.join([*pending, *errors, *unfinished]))
