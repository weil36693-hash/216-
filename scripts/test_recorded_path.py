"""Offline data validation and local projection regression tests."""

import csv
import math
from pathlib import Path
import tempfile
import unittest

try:
    from .recorded_path import DATA_HEADER, META_HEADER, PathGeometry, load_recording
except ImportError:
    from recorded_path import DATA_HEADER, META_HEADER, PathGeometry, load_recording


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'test.csv'

    def fixture(self, points=None, units='mm', changes=None, timestamps=None):
        points = points or [(0, 0), (1000, 0), (2000, 0)]
        rows = [
            ['#Hierarchical Translation and Rotation (.csv) file'], ['[Head]'],
            list(META_HEADER), [str(len(points)), '1', '90', 'ZYX', 'X', units], [],
            ['[SegmentNames&Hierarchy]'], ['Segment', 'Parent'], ['Segment1', ''], [],
            ['[SegmentData]'], ['Frame#', '', 'Segment1'], list(DATA_HEADER)]
        for i, p in enumerate(points):
            values = {'Timestamp': str(timestamps[i] if timestamps else 1000 + i * 11),
                      'XToGlobal1': str(p[0]), 'YToGlobal1': str(p[1]),
                      'ZToGlobal1': '130', 'QxToGlobal1': '0', 'QyToGlobal1': '0',
                      'QzToGlobal1': '0', 'QwToGlobal1': '1'}
            if changes:
                values.update(changes.get(i, {}))
            rows.append([str(i + 1)] + [values.get(k, '') for k in DATA_HEADER[1:]])
        self.write(rows)
        return rows

    def write(self, rows):
        with self.path.open('w', newline='', encoding='utf-8') as handle:
            csv.writer(handle).writerows(rows)

    def test_mm_conversion_and_endpoints(self):
        self.fixture()
        result = load_recording(self.path)
        self.assertEqual(result['points'], [(0.0, 0.0), (2.0, 0.0)])
        self.assertEqual(result['length'], 2.0)
        self.assertEqual(result['units'], 'm')
        self.assertEqual(result['source_units'], 'mm')
        self.assertEqual(result['raw_frames'], 3)
        self.assertAlmostEqual(result['recorded_seconds'], 0.022)

    def test_wrong_units_and_headers_rejected(self):
        self.fixture(units='m')
        with self.assertRaises(ValueError):
            load_recording(self.path)
        rows = self.fixture()
        rows[11][2] = 'WrongX'
        self.write(rows)
        with self.assertRaises(ValueError):
            load_recording(self.path)

    def test_nonfinite_values_rejected_in_required_and_optional_columns(self):
        for field in ('XToGlobal1', 'QwToGlobal1', 'VxToGlobal1'):
            with self.subTest(field=field):
                self.fixture(changes={1: {field: 'NaN'}})
                with self.assertRaises(ValueError):
                    load_recording(self.path)

    def test_timestamp_must_increase(self):
        for times in ([1000, 1000, 1022], [1000, 999, 1022]):
            self.fixture(timestamps=times)
            with self.assertRaises(ValueError):
                load_recording(self.path)

    def test_coordinate_and_total_length_limits(self):
        for points in ([(0, 0), (21000, 0)], [(0, 0), (19000, 0), (0, 0)]):
            self.fixture(points=points)
            with self.assertRaises(ValueError):
                load_recording(self.path)

    def test_long_tracking_failure_rejected(self):
        quaternion = {'QxToGlobal1': str(math.sin(.5)), 'QwToGlobal1': str(math.cos(.5))}
        self.fixture(points=[(i * 10, 0) for i in range(30)],
                     changes={i: quaternion for i in range(1, 29)})
        with self.assertRaisesRegex(ValueError, '0.25'):
            load_recording(self.path)

    @unittest.skipUnless(Path('/home/ros-car/下载/car_test021-tb04.csv').is_file(),
                         'Actual recording is unavailable')
    def test_actual_recording_frame_rejection_and_simplification(self):
        result = load_recording('/home/ros-car/下载/car_test021-tb04.csv')
        self.assertEqual(result['raw_frames'], 6281)
        self.assertEqual(result['rejected_frames'], 16)
        self.assertEqual(result['rejected_frame_numbers'],
                         list(range(530, 538)) + list(range(5929, 5937)))
        self.assertEqual(len(result['points']), 32)
        self.assertAlmostEqual(result['length'], 10.244080831055019, places=7)
        self.assertAlmostEqual(result['recorded_seconds'], 69.771)
        self.assertEqual(result['points'][0], (-.9013265380000001, -1.578729736))
        self.assertEqual(result['points'][-1], (-.44109848, -1.631905518))
        self.assertLess(result['max_filter_deviation'], .008)


class GeometryTests(unittest.TestCase):
    def test_point_at_interpolates_and_clamps(self):
        path = PathGeometry([(0, 0), (1, 0), (1, 2)])
        self.assertEqual(path.length, 3.0)
        self.assertEqual(path.point_at(-1), (0.0, 0.0))
        self.assertEqual(path.point_at(.5), (.5, 0.0))
        self.assertEqual(path.point_at(2), (1.0, 1.0))
        self.assertEqual(path.point_at(10), (1.0, 2.0))

    def test_closed_endpoint_does_not_jump_forward(self):
        path = PathGeometry([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])
        self.assertEqual(path.project(0, 0, 0, .2), (0.0, 0.0))
        # Even with the full loop available, equal-distance ties prefer the
        # current progress rather than the coincident endpoint.
        self.assertEqual(path.project(0, 0, 0, path.length), (0.0, 0.0))
        s, distance = path.project(0, 0, 3.95, .1)
        self.assertEqual(s, 4.0)
        self.assertEqual(distance, 0.0)

    def test_projection_is_clipped_to_arc_window(self):
        path = PathGeometry([(0, 0), (1, 0), (1, 1)])
        s, distance = path.project(0, 0, 1.2, .2)
        self.assertAlmostEqual(s, 1.15)
        self.assertAlmostEqual(distance, math.hypot(1, .15))
        s, distance = path.project(1, 1, .2, .1)
        self.assertAlmostEqual(s, .3)
        self.assertAlmostEqual(distance, math.hypot(.7, 1))

    def test_invalid_inputs_and_duplicate_points(self):
        path = PathGeometry([(0, 0), (0, 0), (1, 0)])
        self.assertEqual(path.length, 1)
        with self.assertRaises(ValueError):
            PathGeometry([(math.nan, 0)])
        with self.assertRaises(ValueError):
            path.project(0, 0, .2, -.1)
        with self.assertRaises(ValueError):
            path.point_at(math.nan)


if __name__ == '__main__':
    unittest.main()
