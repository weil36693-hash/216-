"""Offline, timed world-XY references and forward differential-drive tracking.

CSV times are kept exactly: neither filtering nor time stretching is applied.
Positions are interpolated linearly; corners need not be dynamically feasible.
The controller limits acceleration and turning, not arrival accuracy.
This module never imports ROS or publishes vehicle commands.
"""

import bisect
import csv
from dataclasses import dataclass, field
import io
import math
import re

from .control import Velocity


MAX_POINTS = 100_000
MAX_DURATION = 3600.0
MAX_COORDINATE = 100.0


def _number(value, label):
    if isinstance(value, bool):
        raise ValueError(f'{label} 必须是有限数值')
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(f'{label} 必须是有限数值') from error
    if not math.isfinite(result):
        raise ValueError(f'{label} 必须是有限数值')
    return result


def _coordinate(value, label):
    value = _number(value, label)
    if abs(value) > MAX_COORDINATE:
        raise ValueError(f'{label} 的绝对值不能超过 100 米')
    return value


def _car_number(name):
    match = re.fullmatch(r'(?:car)?([0-9]+)', name)
    return int(match[1]) if match else None


def _aliases(known_cars):
    names = [known_cars] if isinstance(known_cars, str) else list(known_cars)
    if not names or any(not isinstance(name, str) or not name for name in names):
        raise ValueError('known_cars 必须包含配置中的车辆名')
    aliases = {}
    for name in names:
        number = _car_number(name)
        if number is not None:
            if number in aliases and aliases[number] != name:
                raise ValueError('配置中的车辆数字编号不唯一')
            aliases[number] = name
    return set(names), aliases


@dataclass(frozen=True)
class Reference:
    """A sample on the unchanged time axis; world velocities are metres/second.

    ``yaw`` is the forward tangent, not a recorded or calibrated body heading.
    A stationary interval holds the previous tangent (or the next at the start).
    A wholly stationary track has ``yaw=None``. At/after the end velocities are
    zero and ``done`` is true. Before time zero velocities are also zero.
    """

    t: float
    x: float
    y: float
    yaw: float | None
    vx: float
    vy: float
    speed: float
    done: bool


@dataclass(frozen=True)
class Track:
    points: tuple
    duration: float = field(init=False)
    start: tuple = field(init=False)
    end: tuple = field(init=False)
    initial_yaw: float | None = field(init=False)
    max_speed: float = field(init=False)
    _times: tuple = field(init=False, repr=False)
    _segments: tuple = field(init=False, repr=False)
    _last_yaw: float | None = field(init=False, repr=False)

    def __post_init__(self):
        points = []
        for row in self.points:
            if len(row) != 3:
                raise ValueError('轨迹点需要 t,x,y')
            t = _number(row[0], 't')
            if not 0 <= t <= MAX_DURATION:
                raise ValueError('每车时间必须在 0～3600 秒')
            if (not points and t != 0) or (points and t <= points[-1][0]):
                raise ValueError('每车 t 必须从 0 开始并严格递增，不能重复')
            points.append((t, _coordinate(row[1], 'x'), _coordinate(row[2], 'y')))
        if not 2 <= len(points) <= MAX_POINTS:
            raise ValueError('每车至少需要两个点，且不能超过 100000 个点')
        segments, first_yaw, previous_yaw = [], None, None
        for a, b in zip(points, points[1:]):
            dt, dx, dy = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy = dx / dt, dy / dt
            speed = math.hypot(vx, vy)
            if not all(math.isfinite(v) for v in (vx, vy, speed)):
                raise ValueError('采样间隔过小，计算出的速度不是有限数值')
            yaw = math.atan2(dy, dx) if dx != 0 or dy != 0 else None
            if yaw is not None:
                first_yaw = yaw if first_yaw is None else first_yaw
                previous_yaw = yaw
            segments.append((vx, vy, speed, previous_yaw))
        # Initial dwell has no preceding tangent; use the first future motion.
        segments = tuple((vx, vy, speed, first_yaw if yaw is None else yaw)
                         for vx, vy, speed, yaw in segments)
        object.__setattr__(self, 'points', tuple(points))
        object.__setattr__(self, '_times', tuple(p[0] for p in points))
        object.__setattr__(self, '_segments', segments)
        object.__setattr__(self, '_last_yaw', previous_yaw)
        object.__setattr__(self, 'duration', points[-1][0])
        object.__setattr__(self, 'start', points[0][1:])
        object.__setattr__(self, 'end', points[-1][1:])
        object.__setattr__(self, 'initial_yaw', first_yaw)
        object.__setattr__(self, 'max_speed', max(segment[2] for segment in segments))

    def sample(self, t):
        t = _number(t, '采样时间')
        if t >= self.duration:
            return Reference(self.duration, *self.end, self._last_yaw, 0., 0., 0., True)
        if t < 0:
            return Reference(0., *self.start, self.initial_yaw, 0., 0., 0., False)
        index = bisect.bisect_right(self._times, t) - 1
        a, b = self.points[index:index + 2]
        fraction = (t - a[0]) / (b[0] - a[0])
        vx, vy, speed, yaw = self._segments[index]
        return Reference(t, a[1] + fraction * (b[1] - a[1]),
                         a[2] + fraction * (b[2] - a[2]), yaw, vx, vy, speed, False)


@dataclass(frozen=True)
class TrajectorySet:
    tracks: dict

    @property
    def duration(self):
        return max(track.duration for track in self.tracks.values())

    @property
    def sample_count(self):
        return sum(len(track.points) for track in self.tracks.values())

    def summary(self):
        cars = []
        for name, track in self.tracks.items():
            count = min(400, len(track.points))
            indices = [i * (len(track.points) - 1) // (count - 1) for i in range(count)]
            cars.append({'car_id': name, 'sample_count': len(track.points),
                         'duration': track.duration,
                         'start': dict(zip(('x', 'y'), track.start)),
                         'end': dict(zip(('x', 'y'), track.end)),
                         'initial_yaw': track.initial_yaw, 'max_speed': track.max_speed,
                         'points': [dict(zip(('t', 'x', 'y'), track.points[i])) for i in indices]})
        return {'cars': cars, 'duration': self.duration, 'sample_count': self.sample_count}


def parse_csv(text, known_cars):
    """Parse ``car_id,t,x,y``; rows may interleave cars but each clock increases.

    ``4``, ``04`` and ``car04`` resolve to the configured numeric vehicle name.
    Blank lines and a UTF-8 BOM are accepted. No points are removed or retimed.
    """
    if not isinstance(text, str):
        raise ValueError('CSV 必须是 UTF-8 文本')
    names, aliases = _aliases(known_cars)
    rows = csv.reader(io.StringIO(text.removeprefix('\ufeff')), strict=True)
    grouped, count, header = {}, 0, False
    try:
        for row in rows:
            if not any(cell.strip() for cell in row):
                continue
            row = [cell.strip() for cell in row]
            if not header:
                if row != ['car_id', 't', 'x', 'y']:
                    raise ValueError('CSV 表头必须为 car_id,t,x,y 四列')
                header = True
                continue
            if len(row) != 4:
                raise ValueError(f'CSV 第 {rows.line_num} 行必须恰好四列')
            name = row[0]
            if name not in names:
                name = aliases.get(_car_number(name))
            if name is None or name not in names:
                raise ValueError(f'CSV 第 {rows.line_num} 行车号不在配置中：{row[0]}')
            count += 1
            if count > MAX_POINTS:
                raise ValueError('CSV 全文件最多 100000 个轨迹点')
            grouped.setdefault(name, []).append(tuple(_number(v, label) for v, label in
                                                     zip(row[1:], ('t', 'x', 'y'))))
    except csv.Error as error:
        raise ValueError(f'CSV 格式错误：{error}') from error
    if not grouped:
        raise ValueError('CSV 至少需要一辆车的轨迹数据')
    tracks = {}
    for name, points in grouped.items():
        try:
            tracks[name] = Track(tuple(points))
        except ValueError as error:
            raise ValueError(f'{name}: {error}') from error
    return TrajectorySet(tracks)


def _clip(value, limit):
    return max(-limit, min(limit, value))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def control_velocity(track, elapsed, x, y, yaw, last_velocity, dt, *,
                     max_v=None, max_w=.6, max_accel=.10, max_alpha=1.2):
    """Compute one forward-only command at the current, unmodified task time.

    Tangent speed provides feedforward; along-track error changes only forward
    speed, so being ahead cannot reverse the desired heading. Cross-track error
    provides a bounded heading correction. Large heading errors rotate first.
    Dwell intervals command no translation and retain their adjacent heading.
    ``max_v=None`` applies no fixed linear-speed cap; a finite positive value
    optionally limits linear speed. Acceleration and angular limits still apply.
    The caller owns pose freshness, cancellation, task identity and command TTL.
    Time expiry and the rotation gate may cut translation immediately; safety
    zeros take priority over the ordinary acceleration ramp.
    """
    elapsed, x, y, yaw, dt = (_number(value, label) for value, label in
                             zip((elapsed, x, y, yaw, dt), ('elapsed', 'x', 'y', 'yaw', 'dt')))
    max_v = None if max_v is None else _number(max_v, 'max_v')
    limits = [_number(value, label) for value, label in
              zip((max_w, max_accel, max_alpha), ('max_w', 'max_accel', 'max_alpha'))]
    if dt <= 0 or any(value <= 0 for value in limits) or (max_v is not None and max_v <= 0):
        raise ValueError('dt 和控制限值必须大于 0')
    max_w, max_accel, max_alpha = limits
    if last_velocity is None:
        last_velocity = Velocity(0., 0.)
    if not isinstance(last_velocity, Velocity):
        raise ValueError('last_velocity 必须为 Velocity 或 None')
    reference = track.sample(elapsed)
    if elapsed < 0 or reference.done or track.initial_yaw is None:
        return Velocity(0., 0.)
    if reference.speed == 0:
        # A scheduled dwell must not turn into delayed travel toward an old point.
        target_v = 0.
        heading_error = _wrap(reference.yaw - yaw)
    else:
        tx, ty = math.cos(reference.yaw), math.sin(reference.yaw)
        ex, ey = reference.x - x, reference.y - y
        along, across = ex * tx + ey * ty, -ex * ty + ey * tx
        forward = max(0., reference.speed + 1.5 * along)
        # A finite lookahead keeps small lateral errors from requesting a
        # quarter-turn when the scheduled speed approaches zero. Pure along-
        # track overshoot holds the reference heading and waits rather than
        # inventing a reverse tangent and turning back toward the old point.
        correction = _clip(math.atan2(1.5 * across, max(forward, .04)), .6)
        desired_yaw = reference.yaw + correction
        heading_error = _wrap(desired_yaw - yaw)
        target_v = forward * max(0., math.cos(heading_error))
        if max_v is not None:
            target_v = min(max_v, target_v)
        if abs(heading_error) > .65:
            target_v = 0.
    target_w = _clip(3.0 * heading_error, max_w)
    if target_v == 0:
        v = 0.
    else:
        v = max(0., last_velocity.v + _clip(target_v - last_velocity.v, max_accel * dt))
        if max_v is not None:
            v = min(max_v, v)
    w = _clip(last_velocity.w + _clip(target_w - last_velocity.w, max_alpha * dt), max_w)
    return Velocity(v, w)


def _generator(car_id, duration, step, position):
    if isinstance(car_id, bool) or not isinstance(car_id, (str, int)):
        raise ValueError('car_id 必须为车辆编号')
    name = str(car_id)
    number = _car_number(name)
    if number is None:
        raise ValueError('car_id 必须为数字或 car 加数字')
    name = f'car{number:02d}'
    duration, step = _number(duration, 'duration'), _number(step, 'step')
    if not 0 < duration <= MAX_DURATION or step <= 0:
        raise ValueError('duration 必须在 (0,3600] 秒，step 必须大于 0')
    ratio = duration / step
    if not math.isfinite(ratio) or ratio > MAX_POINTS - 1:
        raise ValueError('生成轨迹最多 100000 个点')
    intervals = max(1, math.ceil(ratio))
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator='\n')
    writer.writerow(('car_id', 't', 'x', 'y'))
    for i in range(intervals + 1):
        fraction = i / intervals
        progress = fraction ** 3 * (10 - 15 * fraction + 6 * fraction ** 2)
        x, y = position(progress)
        row = (duration * fraction, _coordinate(x, 'x'), _coordinate(y, 'y'))
        writer.writerow((name, *(format(value, '.15g') for value in row)))
    return buffer.getvalue()


def straight_csv(car_id, start, end, duration, *, step=.05):
    """Sample a quintic-progress straight line; duration includes start/stop.

    This generates samples of a smooth source curve. CSV reconstruction still
    uses the documented piecewise-linear interpolation, including tiny corners.
    """
    if len(start) != 2 or len(end) != 2:
        raise ValueError('start/end 必须为 XY 对')
    sx, sy = (_coordinate(value, 'start') for value in start)
    ex, ey = (_coordinate(value, 'end') for value in end)
    return _generator(car_id, duration, step,
                      lambda u: (sx + u * (ex - sx), sy + u * (ey - sy)))


def arc_csv(car_id, center, radius, start_angle, sweep_angle, duration, *, step=.05):
    """Sample a quintic-progress circular arc; angles use radians, CCW positive."""
    if len(center) != 2:
        raise ValueError('center 必须为 XY 对')
    cx, cy = (_coordinate(value, 'center') for value in center)
    radius = _number(radius, 'radius')
    start_angle, sweep_angle = _number(start_angle, 'start_angle'), _number(sweep_angle, 'sweep_angle')
    if radius <= 0:
        raise ValueError('radius 必须大于 0')
    if not math.isfinite(start_angle + sweep_angle):
        raise ValueError('圆弧角度溢出')
    return _generator(car_id, duration, step,
                      lambda u: (cx + radius * math.cos(start_angle + u * sweep_angle),
                                 cy + radius * math.sin(start_angle + u * sweep_angle)))
