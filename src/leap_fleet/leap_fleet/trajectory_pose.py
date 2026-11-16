"""Local pose estimates for timed control when mocap updates are unavailable.

Dead reckoning assumes the previous target velocity was executed. It is not
measured odometry and does not establish whether a command reached the vehicle.
No ROS, command publication, monitor mutation, or automatic stopping occurs here.
"""

from collections import deque
from collections.abc import Mapping
import math
from numbers import Real

from .control import Velocity


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, OverflowError):
        return False


def _time(value):
    if not _finite(value):
        raise ValueError('now must be a finite monotonic time')
    return float(value)


def _wrap(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _integrate(pose, velocity, dt):
    """Exact held-command unicycle step, stable for zero/small angular velocity."""
    x, y, yaw = pose
    half_turn = velocity.w * dt / 2.
    if not math.isfinite(half_turn):
        raise ValueError('pose integration overflow')
    sinc = (1. - half_turn * half_turn / 6. if abs(half_turn) < 1e-6
            else math.sin(half_turn) / half_turn)
    distance = velocity.v * dt * sinc
    result = (x + distance * math.cos(yaw + half_turn),
              y + distance * math.sin(yaw + half_turn),
              _wrap(yaw + 2. * half_turn))
    if not all(math.isfinite(value) for value in result):
        raise ValueError('pose integration overflow')
    return result


class PoseEstimate:
    """Integrate complete monotonic intervals and accept each mocap sample once.

    ``mode='mocap'`` means a valid fresh payload is available and the last accepted
    anchor is within ``max_age``. Re-reading that frame still integrates motion
    without re-anchoring; its mode does not flicker at a faster control rate.
    A new sample is extrapolated over its reported receive age using the supplied
    previous target velocity. This is a held-command approximation for that age.

    Receive timestamps identify samples; they never supply elapsed time. A
    nonzero source stamp also prevents a republished source frame from repeatedly
    anchoring the estimate. Recent identities are remembered in bounded caches.

    Without either timestamp, changed XY/yaw/source metadata is considered a new
    sample. An unchanged value is accepted again only after an observed age
    decrease advances its inferred sample time. A frozen same-value age=0 cache
    therefore cannot renew the anchor forever. This fallback cannot prove actual
    tracking freshness. Initial seeding assumes no motion during its receive age.
    """

    def __init__(self, pose, now, max_age=.3):
        now = _time(now)
        if not _finite(max_age) or max_age <= 0:
            raise ValueError('max_age must be finite and positive')
        self.max_age = float(max_age)
        self._received, self._source = deque(maxlen=256), deque(maxlen=256)
        self._fallback_value = self._fallback_age = None
        candidate = self._candidate(pose)
        if candidate is None:
            raise ValueError('initial pose must be fresh with finite XY/yaw and a valid age')
        values, age = candidate
        self._pose, self._at, self._anchor_at = values, now, now - age
        self._remember(pose, values, age)

    def _candidate(self, pose):
        if not isinstance(pose, Mapping) or pose.get('status') != 'fresh':
            return None
        position = pose.get('position')
        age = pose.get('age_s')
        if not isinstance(position, Mapping) or not _finite(age) or not 0 <= age <= self.max_age:
            return None
        values = position.get('x'), position.get('y'), pose.get('yaw_rad')
        if not all(_finite(value) for value in values):
            return None
        x, y, yaw = (float(value) for value in values)
        return (x, y, _wrap(yaw)), float(age)

    @staticmethod
    def _identities(pose):
        received = pose.get('received_at_unix')
        received = float(received) if _finite(received) else None
        stamp = pose.get('source_stamp')
        source = None
        if isinstance(stamp, Mapping):
            sec, nanosec = stamp.get('sec'), stamp.get('nanosec')
            if (type(sec) is int and type(nanosec) is int and
                    0 <= nanosec < 1_000_000_000 and (sec != 0 or nanosec != 0)):
                source = sec, nanosec
        return received, source

    @staticmethod
    def _fallback(pose, values):
        # Metadata is only a discriminator, never a coordinate transform.
        return (*values, *(pose.get(key) if isinstance(pose.get(key), str) else None
                           for key in ('frame_id', 'topic')))

    def _remember(self, pose, values, age):
        received, source = self._identities(pose)
        if received is not None:
            self._received.append(received)
        if source is not None:
            self._source.append(source)
        self._fallback_value = self._fallback(pose, values)
        self._fallback_age = age

    def update(self, pose, velocity, now):
        """Return a copied ``x,y,yaw,mode,gap_s`` estimate without stopping on gaps.

        ``velocity`` is the previous target, not an acknowledgment from hardware.
        Missing, invalid, stale, old and duplicate observations simply leave the
        anchor unchanged. Caller cancellation and CSV deadlines remain separate.
        """
        now = _time(now)
        if now < self._at:
            raise ValueError('monotonic time must not go backwards')
        if not isinstance(velocity, Velocity):
            raise ValueError('velocity must be a finite Velocity')
        predicted = _integrate(self._pose, velocity, now - self._at)
        candidate = self._candidate(pose)
        if candidate is not None:
            values, age = candidate
            sample_at = now - age
            received, source = self._identities(pose)
            if received is not None or source is not None:
                new = ((received is None or received not in self._received)
                       and (source is None or source not in self._source))
            else:
                fingerprint = self._fallback(pose, values)
                new = (fingerprint != self._fallback_value or
                       (self._fallback_age is not None and age < self._fallback_age - 1e-6
                        and sample_at > self._anchor_at + 1e-6))
                # Observe age changes without pretending they are valid anchors.
                self._fallback_value, self._fallback_age = fingerprint, age
            if new:
                self._remember(pose, values, age)
                # Do not replace a newer anchor with an out-of-order sample.
                if sample_at >= self._anchor_at:
                    predicted = _integrate(values, velocity, age)
                    self._anchor_at = sample_at
        self._pose, self._at = predicted, now
        gap = max(0., now - self._anchor_at)
        mode = ('mocap' if candidate is not None and gap <= self.max_age + 1e-9
                else 'estimated')
        return {'x': predicted[0], 'y': predicted[1], 'yaw': predicted[2],
                'mode': mode, 'gap_s': gap}
