"""Offline validation and geometry for the single-body HTR CSV export.

Returned positions use metres. Recorded velocities and Euler angles are never
used as commands. No ROS modules are imported by this module.
"""

import bisect
import csv
import math
from pathlib import Path
import statistics


META_HEADER = (
    'NumFrames', 'NumSegments', 'DataFrameRate', 'EulerRotationOrder',
    'BoneAxis', 'TranslationUnits')
DATA_HEADER = (
    '', 'Timestamp', 'XToGlobal1', 'YToGlobal1', 'ZToGlobal1',
    'VxToGlobal1', 'VyToGlobal1', 'VzToGlobal1', 'VrToGlobal1',
    'AxToGlobal1', 'AyToGlobal1', 'AzToGlobal1', 'ArToGlobal1',
    'QxToGlobal1', 'QyToGlobal1', 'QzToGlobal1', 'QwToGlobal1',
    'ExToGlobal1', 'EyToGlobal1', 'EzToGlobal1',
    'EVxToGlobal1', 'EVyToGlobal1', 'EVzToGlobal1',
    'EAxToGlobal1', 'EAyToGlobal1', 'EAzToGlobal1', 'Segmentlength1', '')


def _number(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{label}: invalid number') from error
    if not math.isfinite(result):
        raise ValueError(f'{label}: nonfinite number')
    return result


def _distance_to_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    squared = dx * dx + dy * dy
    u = (((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / squared
         if squared else 0.0)
    u = max(0.0, min(1.0, u))
    return math.hypot(point[0] - a[0] - u * dx, point[1] - a[1] - u * dy)


def _rdp_indices(points, epsilon):
    keep = {0, len(points) - 1}
    pending = [(0, len(points) - 1)]
    while pending:
        a, b = pending.pop()
        if b - a < 2:
            continue
        distance, index = max(
            (_distance_to_segment(points[j], points[a], points[b]), j)
            for j in range(a + 1, b))
        if distance > epsilon:
            keep.add(index)
            pending.extend(((a, index), (index, b)))
    return sorted(keep)


def load_recording(path):
    """Validate this export schema, remove short tracking failures and simplify.

    ``max_filter_deviation`` bounds the distance from each retained raw sample
    to its corresponding final polyline segment, including both filters.
    Both the raw valid route and the simplified route must be at most 30 m.
    """
    with Path(path).open(encoding='utf-8-sig', newline='') as handle:
        rows = [[cell.strip() for cell in row] for row in csv.reader(handle)]
    cursor = 0

    def take():
        nonlocal cursor
        while cursor < len(rows) and not any(rows[cursor]):
            cursor += 1
        if cursor >= len(rows):
            raise ValueError('CSV ended before required data')
        row = rows[cursor]
        cursor += 1
        return row

    def expect(expected):
        if tuple(take()) != tuple(expected):
            raise ValueError(f'Unexpected CSV header; expected {expected}')

    expect(('#Hierarchical Translation and Rotation (.csv) file',))
    expect(('[Head]',))
    expect(META_HEADER)
    metadata = take()
    if len(metadata) != len(META_HEADER):
        raise ValueError('Invalid metadata column count')
    metadata = dict(zip(META_HEADER, metadata))
    try:
        declared_frames = int(metadata['NumFrames'])
        segments = int(metadata['NumSegments'])
    except ValueError as error:
        raise ValueError('Invalid frame or segment count') from error
    if declared_frames < 2 or segments != 1:
        raise ValueError('Expected at least two frames and exactly one segment')
    if _number(metadata['DataFrameRate'], 'DataFrameRate') <= 0:
        raise ValueError('DataFrameRate must be positive')
    if (metadata['EulerRotationOrder'] != 'ZYX' or metadata['BoneAxis'] != 'X'
            or metadata['TranslationUnits'] != 'mm'):
        raise ValueError('Expected ZYX, X bone axis and mm translation units')
    expect(('[SegmentNames&Hierarchy]',))
    expect(('Segment', 'Parent'))
    expect(('Segment1', ''))
    expect(('[SegmentData]',))
    segment_header = take()
    if (segment_header[:3] != ['Frame#', '', 'Segment1']
            or any(segment_header[3:])):
        raise ValueError('Invalid SegmentData group header')
    expect(DATA_HEADER)

    good = []
    rejected = []
    timestamps = []
    bad_since = None
    frame_count = 0
    for row in rows[cursor:]:
        if not any(row):
            continue
        if len(row) != len(DATA_HEADER) or row[-1]:
            raise ValueError('Invalid data row column count')
        try:
            frame = int(row[0])
            timestamp = int(row[1])
        except ValueError as error:
            raise ValueError('Frame and Timestamp must be integers') from error
        frame_count += 1
        if frame != frame_count:
            raise ValueError('Frame numbers must be consecutive from 1')
        if timestamp < 0 or (timestamps and timestamp <= timestamps[-1]):
            raise ValueError('Timestamps must strictly increase')
        timestamps.append(timestamp)
        values = {}
        for label, value in zip(DATA_HEADER[2:-1], row[2:-1]):
            if value:
                values[label] = _number(value, label)
        required = ('XToGlobal1', 'YToGlobal1', 'ZToGlobal1',
                    'QxToGlobal1', 'QyToGlobal1', 'QzToGlobal1', 'QwToGlobal1')
        if any(label not in values for label in required):
            raise ValueError('Missing required position or quaternion')
        point = values['XToGlobal1'] / 1000.0, values['YToGlobal1'] / 1000.0
        if max(abs(point[0]), abs(point[1])) > 20.0:
            raise ValueError('Recorded XY exceeds the fixed 20 m coordinate limit')
        x, y, z, w = (values[label] for label in required[3:])
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not 0.95 < norm < 1.05:
            raise ValueError('Invalid quaternion norm')
        x, y, z, w = (v / norm for v in (x, y, z, w))
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        bad = max(abs(roll), abs(pitch)) > 0.35
        if bad:
            if bad_since is None:
                bad_since = timestamp
            rejected.append(frame)
        else:
            good.append((frame, point))
        if bad_since is not None and timestamp - bad_since > 250:
            raise ValueError('Tracking failure exceeds 0.25 seconds')
        if not bad:
            bad_since = None
    if frame_count != declared_frames:
        raise ValueError('NumFrames does not match data rows')
    if len(good) < 2 or good[0][0] != 1 or good[-1][0] != frame_count:
        raise ValueError('Recording endpoints must have valid tracking')
    raw = [point for _, point in good]
    raw_length = sum(math.dist(a, b) for a, b in zip(raw, raw[1:]))
    if raw_length > 30.0:
        raise ValueError('Recorded route exceeds the fixed 30 m length limit')
    smoothed = [
        tuple(statistics.median(p[k] for p in raw[max(0, j - 4):j + 5])
              for k in (0, 1))
        for j in range(len(raw))]
    smoothed[0], smoothed[-1] = raw[0], raw[-1]
    keep = _rdp_indices(smoothed, 0.008)
    points = [smoothed[j] for j in keep]
    geometry = PathGeometry(points)
    if not 0 < geometry.length <= 30.0:
        raise ValueError('Simplified route must have length in (0, 30] m')
    deviation = max(
        _distance_to_segment(raw[j], smoothed[a], smoothed[b])
        for a, b in zip(keep, keep[1:]) for j in range(a, b + 1))
    return {
        'points': points, 'length': geometry.length,
        'raw_frames': frame_count, 'rejected_frames': len(rejected),
        'rejected_frame_numbers': rejected,
        'recorded_seconds': (timestamps[-1] - timestamps[0]) / 1000.0,
        'units': 'm', 'source_units': 'mm',
        'max_filter_deviation': deviation,
        'median_filter_deviation': max(math.dist(a, b) for a, b in zip(raw, smoothed)),
    }


class PathGeometry:
    """Polyline with projections restricted to an explicit local arc window."""

    def __init__(self, points):
        self.points = []
        for point in points:
            if len(point) != 2:
                raise ValueError('Expected XY pairs')
            p = (_number(point[0], 'X'), _number(point[1], 'Y'))
            if not self.points or p != self.points[-1]:
                self.points.append(p)
        if not self.points:
            raise ValueError('Empty path')
        self.cumulative = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.dist(a, b))
        self.length = self.cumulative[-1]

    def point_at(self, s):
        s = max(0.0, min(self.length, _number(s, 'Arc length')))
        if len(self.points) == 1 or s >= self.length:
            return self.points[-1]
        i = max(0, bisect.bisect_right(self.cumulative, s) - 1)
        a, b = self.points[i], self.points[i + 1]
        u = (s - self.cumulative[i]) / (self.cumulative[i + 1] - self.cumulative[i])
        return a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])

    def project(self, x, y, progress, max_advance):
        x, y = _number(x, 'X'), _number(y, 'Y')
        progress = _number(progress, 'Progress')
        max_advance = _number(max_advance, 'Maximum advance')
        if not 0 <= progress <= self.length or max_advance < 0:
            raise ValueError('Invalid progress or negative advance')
        low = max(0.0, progress - 0.05)
        high = min(self.length, progress + max_advance)
        if len(self.points) == 1:
            return 0.0, math.hypot(x - self.points[0][0], y - self.points[0][1])
        best = None
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            lo, hi = max(low, self.cumulative[i]), min(high, self.cumulative[i + 1])
            if lo > hi:
                continue
            length = self.cumulative[i + 1] - self.cumulative[i]
            dx, dy = b[0] - a[0], b[1] - a[1]
            projected = self.cumulative[i] + ((x - a[0]) * dx + (y - a[1]) * dy) / length
            s = max(lo, min(hi, projected))
            u = (s - self.cumulative[i]) / length
            distance = math.hypot(x - a[0] - u * dx, y - a[1] - u * dy)
            candidate = (distance, abs(s - progress), s)
            if best is None or candidate < best:
                best = candidate
        return best[2], best[0]
